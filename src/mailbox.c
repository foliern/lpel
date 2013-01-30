#include <fcntl.h>
#include <stdlib.h>
#include <pthread.h>
#include <assert.h>
#include <stdio.h>
#include "mailbox.h"
// for communication over MPB
#include <sys/mman.h>
#include <string.h>
#include <unistd.h>
//to use uint64_t
#include <stdint.h>

#include "RCCE.h"
#include "RCCE_lib.h"
#include "SCC_API_test.h"
#include "readTileID.h"
#include "configuration.h"
//#include "/shared/foliern/rcce/src/RCCE_memcpy.c"

#define CORES               (NUM_ROWS * NUM_COLS * NUM_CORES)
#define PAGE_SIZE           (16*1024*1024)
#define LINUX_PRIV_PAGES    (20)
#define PAGES_PER_CORE      (41)
#define MAX_PAGES           (172)
#define IRQ_BIT             (0x01 << GLCFG_XINTR_BIT)

#define B_OFFSET            64
#define FOOL_WRITE_COMBINE  (mpbs[node_location][0] = 1)
#define START(i)            (*((volatile uint16_t *) (mpbs[i] + B_OFFSET)))
#define END(i)              (*((volatile uint16_t *) (mpbs[i] + B_OFFSET + 2)))
#define HANDLING(i)         (*(mpbs[i] + B_OFFSET + 4))
#define WRITING(i)          (*(mpbs[i] + B_OFFSET + 5))
#define B_START             (B_OFFSET + 32)
#define B_SIZE              (MPBSIZE - B_START)
#define true				1
#define false				0
#define LUT(loc, idx)       (*((volatile uint32_t*)(&luts[loc][idx])))
//because from 0 to < 48
#define ACTIVE_NODES		48

//......................................................................................
// GLOBAL VARIABLES USED FOR MPB
//......................................................................................
#define RCCE_MAXNP                         48
#define RCCE_SUCCESS                       0
#define LOG2_LINE_SIZE                     5
#define RCCE_LINE_SIZE                     (1<<LOG2_LINE_SIZE)
// RCCE_BUFF_SIZE_MAX is space per UE, which is half of the space per tile
#define RCCE_BUFF_SIZE_MAX                 (1<<13)

int       RC_COREID[RCCE_MAXNP]; // array of physical core IDs for all participating
                                 // cores, sorted by rank
t_vcharp RCCE_fool_write_combine_buffer;
int       RCCE_BUFF_SIZE;        // available MPB size
t_vcharp  RCCE_comm_buffer[RCCE_MAXNP]; // starts of MPB, sorted by rank
int       NODE_ID=-1;           // rank of calling core (invalid by default)

t_vcharp RCCE_buff_ptr;
  // maximum chunk size of message payload is also specified
  size_t RCCE_chunk;
  // synchronization flags will be allocated at this address
  t_vcharp  RCCE_flags_start;

  // payload part of the MPBs starts at a specific address, not malloced space
  //t_vcharp RCCE_buff_ptr;
  t_vcharp OWN_BUFF_ptr;
  // maximum chunk size of message payload is also specified
  //size_t RCCE_chunk;
  size_t CHUNK_size;

  int MPBDeviceFD; // File descriptor for message passing buffers.

//......................................................................................
// END GLOBAL VARIABLES USED FOR MPB
//......................................................................................



//extern bool remap;
int node_location;

t_vcharp mpbs[CORES];
t_vcharp locks[CORES];
//extern volatile int *irq_pins[CORES];
//extern volatile uint64_t *luts[CORES];

static inline int min(int x, int y) { return x < y ? x : y; }

/* Flush MPBT from L1. */
static inline void flush() { __asm__ volatile ( ".byte 0x0f; .byte 0x0a;\n" ); }

static inline void lock(int core) { while (!(*locks[core] & 0x01)); }

static inline void unlock(int core) { *locks[core] = 0; }


/* mailbox structures */

typedef struct mailbox_node_t {
  struct mailbox_node_t *next;
  workermsg_t msg;
} mailbox_node_t;

struct mailbox_t {
  pthread_mutex_t  lock_free;
  pthread_mutex_t  lock_inbox;
  pthread_cond_t   notempty;
  mailbox_node_t  *list_free;
  mailbox_node_t  *list_inbox;
};


/******************************************************************************/
/* Free node pool management functions                                        */
/******************************************************************************/

static mailbox_node_t *GetFree( mailbox_t *mbox)
{
  mailbox_node_t *node;

  pthread_mutex_lock( &mbox->lock_free);
  if (mbox->list_free != NULL) {
    /* pop free node off */
    node = mbox->list_free;
    mbox->list_free = node->next; /* can be NULL */
  } else {
    /* allocate new node */
    node = (mailbox_node_t *)malloc( sizeof( mailbox_node_t));
  }
  pthread_mutex_unlock( &mbox->lock_free);

  return node;
}

static void PutFree( mailbox_t *mbox, mailbox_node_t *node)
{
  pthread_mutex_lock( &mbox->lock_free);
  if ( mbox->list_free == NULL) {
    node->next = NULL;
  } else {
    node->next = mbox->list_free;
  }
  mbox->list_free = node;
  pthread_mutex_unlock( &mbox->lock_free);
}

//--------------------------------------------------------------------------------------
// FUNCTION: RCCE_malloc_init
//--------------------------------------------------------------------------------------
// initialize memory allocator
//--------------------------------------------------------------------------------------
//void RCCE_malloc_init2(
//  t_vcharp mem, // pointer to MPB space that is to be managed by allocator
//  size_t size   // size (bytes) of managed space
//) {
//
//  // in the simplified API MPB memory allocation merely uses running pointers
//  //RCCE_flags_start = mem;
//  CHUNK_size       = size;
//  OWN_BUFF_ptr    = mem;
//
//}

// MPBalloc allocates MPBSIZE bytes of MessagePassing buffer Memory at MPB_ADDR(x,y,core).
//
// Parameter: MPB                   - Pointer to MPB area (return value, virtal address)
//            x,y,core              - Position of tile (x,y) and core...
//
	void MPBalloc2(t_vcharp *MPB, int x, int y, int core, int isOwnMPB)
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


//--------------------------------------------------------------------------------------
// FUNCTION: RC_COMM_BUFFER_START
//--------------------------------------------------------------------------------------
// return (virtual) start address of MPB for UE with rank ue
//--------------------------------------------------------------------------------------
  t_vcharp RC_COMM_BUFFER_START2(int ue)
  {
    // "Allocate" MPB, using memory mapping of physical addresses
    t_vcharp retval;
    MPBalloc2(&retval, X_PID(RC_COREID[ue]), Y_PID(RC_COREID[ue]), Z_PID(RC_COREID[ue]),
             (X_PID(RC_COREID[ue])) && //== X_PID(RC_COREID[RCCE_IAM])) &&
             (Y_PID(RC_COREID[ue]))// == Y_PID(RC_COREID[RCCE_IAM]))
            );
    return retval;
  }

//--------------------------------------------------------------------------------------
// FUNCTION: RC_cache_invalidate
//--------------------------------------------------------------------------------------
// invalidate (not flush!) lines in L1 that map to MPB lines
//--------------------------------------------------------------------------------------
    void RC_cache_invalidate2()
    {
	  __asm__ volatile ( ".byte 0x0f; .byte 0x0a;\n" ); // CL1FLUSHMB
	  	 return;
    }

//--------------------------------------------------------------------------------------
// FUNCTION: memcpy_put
//--------------------------------------------------------------------------------------
// optimized memcpy for copying data from private memory to MPB
//--------------------------------------------------------------------------------------
    inline static void *memcpy_put2(void *dest, const void *src, size_t count)
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

inline static void *memcpy_get2(void *dest, const void *src, size_t count)
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

//--------------------------------------------------------------------------------------
// FUNCTION: RCCE_put
//--------------------------------------------------------------------------------------
// copy data from address "source" in the local MPB or the calling UE's private memory
// to address "target" in the remote MPB. We do not test to see if a move from the
// calling UE's private memory stays within allocated memory
//--------------------------------------------------------------------------------------


    int RCCE_put2(
                        t_vcharp target, // target buffer, MPB
                        t_vcharp source, // source buffer, MPB or private memory, message to write into MPB
                        int num_bytes,
                        int ID
                        )
    {
        // in non-GORY mode we only need to retain the MPB target shift; we
        // already know the target is in the MPB, not private memory
        target = RCCE_comm_buffer[ID]+(target-RCCE_comm_buffer[ID]);

                // do the actual copy
                RC_cache_invalidate2();
        //test address
        //target= (char *)0xb7551020;

                memcpy_put2((void *)target, (void *)source, num_bytes);
                memcpy_get2((void *)target, (void *)source, num_bytes);
                return(RCCE_SUCCESS);
    }


/******************************************************************************/
/* Public functions                                                           */
/******************************************************************************/


mailbox_t *LpelMailboxCreate(void)
{

  mailbox_t *mbox = (mailbox_t *)malloc(sizeof(mailbox_t));

  pthread_mutex_init( &mbox->lock_free,  NULL);
  pthread_mutex_init( &mbox->lock_inbox, NULL);
  pthread_cond_init(  &mbox->notempty,   NULL);
  mbox->list_free  = NULL;
  mbox->list_inbox = NULL;


  NODE_ID=readTileID();
  // initialize MPB starting addresses for all participating cores; allow one
  // dummy cache line at front of MPB for fooling write combine buffer in case
  // of single-byte MPB access
  //RCCE_fool_write_combine_buffer = RC_COMM_BUFFER_START(RCCE_IAM);
  for (int ue=0; ue < ACTIVE_NODES; ue++){
	 RC_COREID[ue]=ue;
	  RCCE_comm_buffer[ue] = RC_COMM_BUFFER_START2(ue) + RCCE_LINE_SIZE;
  }
  // gross MPB size is set equal to maximum
  RCCE_BUFF_SIZE = RCCE_BUFF_SIZE_MAX - RCCE_LINE_SIZE;

  // initialize RCCE_malloc
  //RCCE_malloc_init2(RCCE_comm_buffer[NODE_ID],RCCE_BUFF_SIZE);

  // in the simplified API MPB memory allocation merely uses running pointers
    //RCCE_flags_start = mem;
    CHUNK_size    = RCCE_BUFF_SIZE;
    OWN_BUFF_ptr    = RCCE_comm_buffer[NODE_ID];

  //if SHMADD & SHMDBG are not defined
  //RCCE_shmalloc_init(RC_SHM_BUFFER_START(),RCCE_SHM_SIZE_MAX);
  return mbox;
}



void LpelMailboxDestroy( mailbox_t *mbox)
{
  mailbox_node_t *node;

  assert( mbox->list_inbox == NULL);
  #if 0
  pthread_mutex_lock( &mbox->lock_inbox);
  while (mbox->list_inbox != NULL) {
    /* pop node off */
    node = mbox->list_inbox;
    mbox->list_inbox = node->next; /* can be NULL */
    /* free the memory for the node */
    free( node);
  }
  pthread_mutex_unlock( &mbox->lock_inbox);
  #endif

  /* free all free nodes */
  pthread_mutex_lock( &mbox->lock_free);
  while (mbox->list_free != NULL) {
    /* pop free node off */
    node = mbox->list_free;
    mbox->list_free = node->next; /* can be NULL */
    /* free the memory for the node */
    free( node);
  }
  pthread_mutex_unlock( &mbox->lock_free);

  /* destroy sync primitives */
  pthread_mutex_destroy( &mbox->lock_free);
  pthread_mutex_destroy( &mbox->lock_inbox);
  pthread_cond_destroy(  &mbox->notempty);

  free(mbox);
}

void LpelMailboxSend( mailbox_t *mbox, workermsg_t *msg)
{
  /* get a free node from recepient */
  mailbox_node_t *node = GetFree( mbox);

  /* copy the message */
  node->msg = *msg;

  /* put node into inbox */
  pthread_mutex_lock( &mbox->lock_inbox);
  if ( mbox->list_inbox == NULL) {
    /* list is empty */
    mbox->list_inbox = node;
    node->next = node; /* self-loop */

    pthread_cond_signal( &mbox->notempty);

  } else {
    /* insert stream between last node=list_inbox
       and first node=list_inbox->next */
    node->next = mbox->list_inbox->next;
    mbox->list_inbox->next = node;
    mbox->list_inbox = node;
  }
  pthread_mutex_unlock( &mbox->lock_inbox);
}

//--------------------------------------------------------------------------------------
// FUNCTION: memcpy_get
//--------------------------------------------------------------------------------------
// optimized memcpy for copying data from MPB to private memory
//--------------------------------------------------------------------------------------



//--------------------------------------------------------------------------------------
// FUNCTION: RCCE_get
//--------------------------------------------------------------------------------------
// copy data from address "source" in the remote MPB to address "target" in either the
// local MPB, or in the calling UE's private memory. We do not test to see if a move
// into the calling UE's private memory stays within allocated memory                     *
//--------------------------------------------------------------------------------------
int RCCE_get2(
  t_vcharp target, // target buffer, MPB or private memory
  t_vcharp source, // source buffer, MPB
  int num_bytes,   // number of bytes to copy (must be multiple of cache line size
  int ID           // rank of source UE
  ) {

    // in non-GORY mode we only need to retain the MPB source shift; we
    // already know the source is in the MPB, not private memory
    source = RCCE_comm_buffer[ID]+(source-RCCE_comm_buffer[ID]);


  // do the actual copy
  RC_cache_invalidate2();

  //memcpy_get2((void *)target, (void *)source, num_bytes);

  return(RCCE_SUCCESS);
}

void LpelMailboxSend_overMPB(
	  char *privbuf,    // source buffer in local private memory (send buffer)
	  //t_vcharp combuf,  // intermediate buffer in MPB
	  //RCCE_Buff_ptr = OWN_BUFF_ptr
	  //size_t chunk,     // size of MPB available for this message (bytes)
	  //RCCE_chunk
	  //RCCE_FLAG *ready, // flag indicating whether receiver is ready
	  //RCCE_FLAG *sent,  // flag indicating whether message has been sent by source
	  size_t size,      // size of message (bytes)
	  int dest          // UE that will receive the message
	  )
{

	  //char padline[RCCE_LINE_SIZE]; // copy buffer, used if message not multiple of line size
	  //size_t wsize,    // offset within send buffer when putting in "chunk" bytes
	  //      remainder, // bytes remaining to be sent
	  //      nbytes;    // number of bytes to be sent in single RCCE_put call
	  //char *bufptr;    // running pointer inside privbuf for current location

	  // send data in units of available chunk size of comm buffer
	  //for (wsize=0; wsize< (size/chunk)*chunk; wsize+=chunk) {
	    //bufptr = privbuf + wsize;
	    //nbytes = chunk;
	    // copy private data to own comm buffer


	//RCCE_put2(OWN_BUFF_ptr, (t_vcharp) privbuf, CHUNK_size, dest);
	RCCE_put2(RCCE_comm_buffer[dest], (t_vcharp) privbuf, CHUNK_size, dest);

	//RCCE_flag_write(sent, RCCE_FLAG_SET, dest);
	    // wait for the destination to be ready to receive a message
	    //RCCE_wait_until(*ready, RCCE_FLAG_SET);
	    //RCCE_flag_write(ready, RCCE_FLAG_UNSET, RCCE_IAM);
	 // }

	 // remainder = size%chunk;
	  // if nothing is left over, we are done
	  //if (!remainder) return(RCCE_SUCCESS);
//	return (1)
	  // send remainder of data--whole cache lines
//	  bufptr = privbuf + (size/chunk)*chunk;
//	  nbytes = remainder - remainder%RCCE_LINE_SIZE;
//	  if (nbytes) {
//	    // copy private data to own comm buffer
//	    RCCE_put(combuf, (t_vcharp)bufptr, nbytes, RCCE_IAM);
//	    RCCE_flag_write(sent, RCCE_FLAG_SET, dest);
//	    // wait for the destination to be ready to receive a message
//	    RCCE_wait_until(*ready, RCCE_FLAG_SET);
//	    RCCE_flag_write(ready, RCCE_FLAG_UNSET, RCCE_IAM);
//	  }
//
//	  remainder = remainder%RCCE_LINE_SIZE;
//	  if (!remainder) return(RCCE_SUCCESS);
//
//	  // remainder is less than a cache line. This must be copied into appropriately sized
//	  // intermediate space before it can be sent to the receiver
//	  bufptr = privbuf + (size/chunk)*chunk + nbytes;
//	  nbytes = RCCE_LINE_SIZE;
//	  // copy private data to own comm buffer
//	#ifdef SCC
//	  memcpy_put(padline,bufptr,remainder);
//	#else
//	  memcpy(padline,bufptr,remainder);
//	#endif
//	  RCCE_put(combuf, (t_vcharp)padline, nbytes, RCCE_IAM);
//	  RCCE_flag_write(sent, RCCE_FLAG_SET, dest);
//	  // wait for the destination to be ready to receive a message
//	  RCCE_wait_until(*ready, RCCE_FLAG_SET);
//	  RCCE_flag_write(ready, RCCE_FLAG_UNSET, RCCE_IAM);
//
//	  return(RCCE_SUCCESS);
//	}
}


void LpelMailboxRecv( mailbox_t *mbox, workermsg_t *msg)
{
  mailbox_node_t *node;

  /* get node from inbox */
  pthread_mutex_lock( &mbox->lock_inbox);
  while( mbox->list_inbox == NULL) {
      pthread_cond_wait( &mbox->notempty, &mbox->lock_inbox);
  }

  assert( mbox->list_inbox != NULL);

  /* get first node (handle points to last) */
  node = mbox->list_inbox->next;
  if ( node == mbox->list_inbox) {
    /* self-loop, just single node */
    mbox->list_inbox = NULL;
  } else {
    mbox->list_inbox->next = node->next;
  }
  pthread_mutex_unlock( &mbox->lock_inbox);

  /* copy the message */
  *msg = node->msg;

  /* put node into free pool */
  PutFree( mbox, node);
}

void LpelMailboxRecv_overMPB(
	  char *privbuf,    // destination buffer in local private memory (receive buffer)
//	  t_vcharp combuf,  // intermediate buffer in MPB
//	  size_t chunk,     // size of MPB available for this message (bytes)
	  //RCCE_FLAG *ready, // flag indicating whether receiver is ready
	  //RCCE_FLAG *sent,  // flag indicating whether message has been sent by source
	  size_t size,      // size of message (bytes)
	  int source       // UE that sent the message
	//  int *test         // if 1 upon entry, do nonblocking receive; if message available
	                    // set to 1, otherwise to 0
	  ) {

//	  char padline[RCCE_LINE_SIZE]; // copy buffer, used if message not multiple of line size
//	  size_t wsize,   // offset within receive buffer when pulling in "chunk" bytes
//	       remainder, // bytes remaining to be received
//	       nbytes;    // number of bytes to be received in single RCCE_get call
//	  int first_test; // only use first chunk to determine if message has been received yet
//	  char *bufptr;   // running pointer inside privbuf for current location
//
//	  first_test = 1;

	  // receive data in units of available chunk size of MPB
//	  for (wsize=0; wsize< (size/chunk)*chunk; wsize+=chunk) {
//	    bufptr = privbuf + wsize;
//	    nbytes = chunk;
//	    // if function is called in test mode, check if first chunk has been sent already.
//	    // If so, proceed as usual. If not, exit immediately
//	    if (*test && first_test) {
//	      first_test = 0;
//	      if (!(*test = RCCE_probe(*sent))) return(RCCE_SUCCESS);
//	    }
	    //RCCE_wait_until(*sent, RCCE_FLAG_SET);
	    //RCCE_flag_write(sent, RCCE_FLAG_UNSET, RCCE_IAM);
	    // copy data from local MPB space to private memory
	    RCCE_get2((t_vcharp)privbuf, RCCE_comm_buffer[source], CHUNK_size, source);

	    // tell the source I have moved data out of its comm buffer
	    //RCCE_flag_write(ready, RCCE_FLAG_SET, source);
//	  }

//	  remainder = size%chunk;
//	  // if nothing is left over, we are done
//	  if (!remainder) return(RCCE_SUCCESS);
//
//	  // receive remainder of data--whole cache lines
//	  bufptr = privbuf + (size/chunk)*chunk;
//	  nbytes = remainder - remainder%RCCE_LINE_SIZE;
//	  if (nbytes) {
//	    // if function is called in test mode, check if first chunk has been sent already.
//	    // If so, proceed as usual. If not, exit immediately
//	    if (*test && first_test) {
//	      first_test = 0;
//	      if (!(*test = RCCE_probe(*sent))) return(RCCE_SUCCESS);
//	    }
//	    RCCE_wait_until(*sent, RCCE_FLAG_SET);
//	    RCCE_flag_write(sent, RCCE_FLAG_UNSET, RCCE_IAM);
//	    // copy data from local MPB space to private memory
//	    RCCE_get((t_vcharp)bufptr, combuf, nbytes, source);
//
//	    // tell the source I have moved data out of its comm buffer
//	    RCCE_flag_write(ready, RCCE_FLAG_SET, source);
//	  }
//
//	  remainder = remainder%RCCE_LINE_SIZE;
//	  if (!remainder) return(RCCE_SUCCESS);
//
//	  // remainder is less than cache line. This must be copied into appropriately sized
//	  // intermediate space before exact number of bytes get copied to the final destination
//	  bufptr = privbuf + (size/chunk)*chunk + nbytes;
//	  nbytes = RCCE_LINE_SIZE;
//	    // if function is called in test mode, check if first chunk has been sent already.
//	    // If so, proceed as usual. If not, exit immediately
//	    if (*test && first_test) {
//	      first_test = 0;
//	      if (!(*test = RCCE_probe(*sent))) return(RCCE_SUCCESS);
//	    }
//	    RCCE_wait_until(*sent, RCCE_FLAG_SET);
//	    RCCE_flag_write(sent, RCCE_FLAG_UNSET, RCCE_IAM);
//	    // copy data from local MPB space to private memory
//	    RCCE_get((t_vcharp)padline, combuf, nbytes, source);
//	#ifdef SCC
//	    memcpy_get(bufptr,padline,remainder);
//	#else
//	    memcpy(bufptr,padline,remainder);
//	#endif
//
//	    // tell the source I have moved data out of its comm buffer
//	    RCCE_flag_write(ready, RCCE_FLAG_SET, source);
//
//	  return(RCCE_SUCCESS);
//	}
}
/**
 * @return 1 if there is an incoming msg, 0 otherwise
 * @note: does not need to be locked as a 'missed' msg
 *        will be eventually fetched in the next worker loop
 */
int LpelMailboxHasIncoming( mailbox_t *mbox)
{
  return ( mbox->list_inbox != NULL);
}
