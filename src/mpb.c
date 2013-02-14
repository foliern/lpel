/*
 * mpb.c
 *
 *  Created on: Feb 6, 2013
 *      Author: Simon
 */
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>

// for communication over MPB
#include <sys/mman.h>
#include <string.h>
#include <unistd.h>
//to use uint64_t
#include <stdint.h>
#include "SCC_API.h"
#include "scc.h"
#include "lpel.h"
#include "mpb.h"
#include "configuration.h"



#define PAGE_SIZE           (16*1024*1024)

/* open-only flags */
//#define	O_RDONLY	0x0000		/* open for reading only */
//#define	O_WRONLY	0x0001		/* open for writing only */
//#define	O_RDWR		0x0002		/* open for reading and writing */
//#define	O_ACCMODE	0x0003		/* mask for above modes */
//#define	O_SYNC		0x0080		/* synch I/O file integrity */

/*
 * Protections are chosen from these bits, or-ed together
 */
//#define	PROT_NONE	0x00	/* [MC2] no permissions */
//#define	PROT_READ	0x01	/* [MC2] pages can be read */
//#define	PROT_WRITE	0x02	/* [MC2] pages can be written */
//#define	PROT_EXEC	0x04	/* [MC2] pages can be executed */

/*
 * Flags contain sharing type and options.
 * Sharing types; choose one.
 */
#define	MAP_SHARED	0x0001		/* [MF|SHM] share changes */
#define	MAP_PRIVATE	0x0002		/* [MF|SHM] changes are private */
#if !defined(_POSIX_C_SOURCE) || defined(_DARWIN_C_SOURCE)
#define	MAP_COPY	MAP_PRIVATE	/* Obsolete */
#endif	/* (!_POSIX_C_SOURCE || _DARWIN_C_SOURCE) */

#define LOCAL_LUT   0x14
#define REMOTE_LUT  (LOCAL_LUT + local_pages)

//#define	NULL __DARWIN_NULL

int MPBDeviceFD; // File descriptor for message passing buffers.

typedef union block {
  struct {
    union block *next;
    size_t size;
  } hdr;
  uint32_t align;   // Forces proper allignment
} block_t;

typedef struct {
  unsigned char free;
  unsigned char size;
} lut_state_t;

void *remote;
unsigned char local_pages;

static void *local;
static int mem, cache;
static block_t *freeList;
static lut_state_t *lutState;
static unsigned char remote_pages;

/******************************************************************************/
/* Private functions                                                           */
/******************************************************************************/


//--------------------------------------------------------------------------------------
// FUNCTION: RC_cache_invalidate
//--------------------------------------------------------------------------------------
// invalidate (not flush!) lines in L1 that map to MPB lines
//--------------------------------------------------------------------------------------
void SCC_cache_invalidate()
{
	__asm__ volatile ( ".byte 0x0f; .byte 0x0a;\n" ); // CL1FLUSHMB
	return;
}


//--------------------------------------------------------------------------------------
// FUNCTION: memcpy_put
//--------------------------------------------------------------------------------------
// optimized memcpy for copying data from private memory to MPB
//--------------------------------------------------------------------------------------
inline static void *MPB_memcpy_put(void *dest, const void *src, size_t count)
{
	int i, j, k;

	asm volatile (
				"cld; rep ; movsl\n\t"
				"movl %4, %%ecx\n\t"
				"andl $3, %%ecx\n\t"
				"rep ; movsb\n\t"
				: "=&c"(i), "=&D"(j), "=&S"(k)
				: "0"(count/4), "g"(count), "1"(dest), "2"(src) : "memory");

	return dest;
}

    //--------------------------------------------------------------------------------------
    // FUNCTION: memcpy_get
    //--------------------------------------------------------------------------------------
    // optimized memcpy for copying data from MPB to private memory
    //--------------------------------------------------------------------------------------

inline static void *MPB_memcpy_get(void *dest, const void *src, size_t count)
{
	int h, i, j, k, l, m;

	asm volatile("cld;\n\t"
                "1: cmpl $0, %%eax ; je 2f\n\t"
                "movl (%%edi), %%edx\n\t"
                "movl 0(%%esi), %%ecx\n\t"
                "movl 4(%%esi), %%edx\n\t"
                "movl %%ecx, 0(%%edi)\n\t"
                "movl %%edx, 4(%%edi)\n\t"
                "movl 8(%%esi), %%ecx\n\t"
                "movl 12(%%esi), %%edx\n\t"
                "movl %%ecx, 8(%%edi)\n\t"
                "movl %%edx, 12(%%edi)\n\t"
                "movl 16(%%esi), %%ecx\n\t"
                "movl 20(%%esi), %%edx\n\t"
                "movl %%ecx, 16(%%edi)\n\t"
                "movl %%edx, 20(%%edi)\n\t"
                "movl 24(%%esi), %%ecx\n\t"
                "movl 28(%%esi), %%edx\n\t"
                "movl %%ecx, 24(%%edi)\n\t"
                "movl %%edx, 28(%%edi)\n\t"
                "addl $32, %%esi\n\t"
                "addl $32, %%edi\n\t"
                "dec %%eax ; jmp 1b\n\t"
                "2: movl %%ebx, %%ecx\n\t"
                "movl (%%edi), %%edx\n\t"
                "andl $31, %%ecx\n\t"
                "rep ; movsb\n\t"
                : "=&a"(h), "=&D"(i), "=&S"(j), "=&b"(k), "=&c"(l), "=&d"(m)
                : "0"(count/32), "1"(dest), "2"(src), "3"(count)  : "memory");

	return dest;
}


// MPBalloc allocates MPBSIZE bytes of MessagePassing buffer Memory at MPB_ADDR(x,y,core).
//
// Parameter: MPB                   - Pointer to MPB area (return value, virtal address)
//            x,y,core              - Position of tile (x,y) and core...
//
void MPB_malloc(t_vcharp *MPB, int x, int y, int core, int isOwnMPB)
{
	  t_vcharp MappedAddr;
	  // enable local MPB bypass (if trusted) by uncommenting next two lines and commenting
	  // out the two after that
	  //  unsigned int alignedAddr = (isOwnMPB?(MPB_OWN+(MPBSIZE*core)):MPB_ADDR(x,y,core)) & (~(PAGE_SIZE-1));
	  //  unsigned int pageOffset = (isOwnMPB?(MPB_OWN+(MPBSIZE*core)):MPB_ADDR(x,y,core)) -alignedAddr;
	  unsigned int alignedAddr = (MPB_ADDR(x,y,core)) & (~(PAGE_SIZE-1));
	  unsigned int pageOffset = (MPB_ADDR(x,y,core)) - alignedAddr;
	  if ((x>=NUM_COLS) || (y>=NUM_ROWS) || (core>=NUM_CORES)) {
		  PRT_DBG("MPBalloc: Invalid coordinates (x=%0d, y=%0d, core=%0d)\n", x,y,core);
		  *MPB = NULL;
		  return;
	  }

	 // because i initialize it into the Mailbox_init()
	/*  if ((MPBDeviceFD=open("/dev/rckmpb", O_RDWR))<0) {
			PRT_DBG("Error opening /dev/rckmpb!!!");
			exit(-1);
		}*/


	  MappedAddr = (t_vcharp) mmap(NULL, MPBSIZE, PROT_WRITE|PROT_READ, MAP_SHARED, MPBDeviceFD, alignedAddr);
	  if (MappedAddr == MAP_FAILED)
	  {
		   perror("mmap");
		   exit(-1);
	  }

	  *MPB = MappedAddr+pageOffset;
}

void *SNetMemAlloc( size_t s) {
  
  void *ptr;

  if( s == 0) {
    ptr = NULL;
  }
  else {
    ptr = malloc( s);
    if( ptr == NULL) {
      printf("\n\n** Fatal Error ** : Unable to Allocate Memory.\n\n");
      exit(1);
    }
  }

  return( ptr);
}

/******************************************************************************/
/* Public functions                                                           */
/******************************************************************************/

void SCCInit(unsigned char size)
{
  local_pages = size;
  remote_pages = remap ? MAX_PAGES - size : 1;

  /* Open driver device "/dev/rckdyn011" to map memory in write-through mode */
  mem = open("/dev/rckdyn011", O_RDWR|O_SYNC);
  if (mem < 0) printf("Opening /dev/rckdyn011 failed!");
  cache = open("/dev/rckdcm", O_RDWR|O_SYNC);
  if (cache < 0) printf("Opening /dev/rckdcm failed!");

  local = mmap(NULL, local_pages * PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, mem, LOCAL_LUT << 24);
  if (local == NULL) printf("Couldn't map memory!");

  remote = mmap(NULL, remote_pages * PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, mem, REMOTE_LUT << 24);
  if (remote == NULL) printf("Couldn't map memory!");

  freeList = local;
  freeList->hdr.next = freeList;
  freeList->hdr.size = (size * PAGE_SIZE) / sizeof(block_t);

  if (remap) {
    lutState = SNetMemAlloc(remote_pages * sizeof(lut_state_t));
    lutState[0].free = 1;
    lutState[0].size = remote_pages;
    lutState[remote_pages - 1] = lutState[0];
  }
}



//--------------------------------------------------------------------------------------
// FUNCTION: RC_COMM_BUFFER_START
//--------------------------------------------------------------------------------------
// return (virtual) start address of MPB for UE with rank ue
//--------------------------------------------------------------------------------------
t_vcharp MPB_comm_buffer_start(int ue)
{
    // "Allocate" MPB, using memory mapping of physical addresses
    t_vcharp retval;
    //MPBalloc2(&retval, X_PID([ue]), Y_PID(SCC_COREID[ue]), Z_PID(SCC_COREID[ue]),
    //         (X_PID(SCC_COREID[ue])) && //== X_PID(SCC_COREID[RCCE_IAM])) &&
    //         (Y_PID(SCC_COREID[ue]))// == Y_PID(SCC_COREID[RCCE_IAM]))
    //        );
    MPB_malloc(&retval, X_PID(ue), Y_PID(ue), Z_PID(ue), X_PID(ue) && Y_PID(ue));

    return retval;
}




//--------------------------------------------------------------------------------------
// FUNCTION: RCCE_put
//--------------------------------------------------------------------------------------
// copy data from address "source" in the local MPB or the calling UE's private memory
// to address "target" in the remote MPB. We do not test to see if a move from the
// calling UE's private memory stays within allocated memory
//--------------------------------------------------------------------------------------


void MPB_write(
			t_vcharp target, // target buffer, MPB
			t_vcharp source, // source buffer, MPB or private memory, message to write into MPB
			int num_bytes
			)
{
	// do the actual copy
	SCC_cache_invalidate();
    	//test address
    	//target= (char *)0xb7551020;
	//*target=*source;
	MPB_memcpy_put((void *)target, (void *)source, num_bytes);
	//return();
}

//--------------------------------------------------------------------------------------
// FUNCTION: RCCE_get
//--------------------------------------------------------------------------------------
// copy data from address "source" in the remote MPB to address "target" in either the
// local MPB, or in the calling UE's private memory. We do not test to see if a move
// into the calling UE's private memory stays within allocated memory                     *
//--------------------------------------------------------------------------------------
int MPB_read(
		t_vcharp target, // target buffer, MPB or private memory
		t_vcharp source, // source buffer, MPB
		int num_bytes   // number of bytes to copy (must be multiple of cache line size
	)
{
	// do the actual copy
	SCC_cache_invalidate();
	//	for (int i; i<num_bytes;i++){
	//	target++;
	//	source++;
		*target=*source;
	//}
	//target++;
	//*target='\n';
//	MPB_memcpy_get((void *)target, (void *)source, num_bytes);
	return(1);
}

