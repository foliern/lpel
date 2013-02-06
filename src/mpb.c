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
#include "SCC_API_test.h"
#include "lpel.h"
#include "mpb.h"
#include "configuration.h"



#define PAGE_SIZE           (16*1024*1024)

int MPBDeviceFD; // File descriptor for message passing buffers.

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
	  if ((MPBDeviceFD=open("/dev/rckmpb", O_RDWR))<0) {
			PRT_DBG("Error opening /dev/rckmpb!!!");
			exit(-1);
		}


	  MappedAddr = (t_vcharp) mmap(NULL, MPBSIZE, PROT_WRITE|PROT_READ, MAP_SHARED, MPBDeviceFD, alignedAddr);
	  if (MappedAddr == MAP_FAILED)
	  {
		   perror("mmap");
		   exit(-1);
	  }

	  *MPB = MappedAddr+pageOffset;
}

/******************************************************************************/
/* Public functions                                                           */
/******************************************************************************/




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
			int num_bytes,
			)
{
	// do the actual copy
	SCC_cache_invalidate();
    //test address
    //target= (char *)0xb7551020;
	*target=*source;
	// memcpy_put2((void *)target, (void *)source, num_bytes);
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
		int num_bytes,   // number of bytes to copy (must be multiple of cache line size
	)
{
	// do the actual copy
	SCC_cache_invalidate();

	*target=*source;
	//memcpy_get2((void *)target, (void *)source, num_bytes);
	return(1);
}

