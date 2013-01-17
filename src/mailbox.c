
#include <stdlib.h>
#include <pthread.h>
#include <assert.h>
#include <stdio.h>
#include "mailbox.h"
// for communication over MPB

#include <string.h>
#include <unistd.h>
//to use uint64_t
#include <stdint.h>


#include "SCC_API.h"


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

//extern bool remap;
extern int node_location;

extern t_vcharp mpbs[CORES];
extern t_vcharp locks[CORES];
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

void LpelMailboxSend_overMPB(int node, void *src, int size)
{
	  int start, end, free;

	  if (size >= B_SIZE) printf("Message to big!");

	  flush();
	  WRITING(node) = true;
	  FOOL_WRITE_COMBINE;

	  while (size) {
	    flush();
	    start = START(node);
	    end = END(node);

	    if (end < start) free = start - end - 1;
	    else free = B_SIZE - end - (start == 0 ? 1 : 0);
	    free = min(free, size);

	    if (!free) {
	      unlock(node);
	      usleep(1);
	      lock(node);
	      continue;
	    }

	    memcpy((void*) (mpbs[node] + B_START + end), src, free);

	    flush();
	    size -= free;
	    src += free;
	    END(node) = (end + free) % B_SIZE;
	    WRITING(node) = false;
	    FOOL_WRITE_COMBINE;
	  }
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

void LpelMailboxRecv_overMPB(int node, void *dst, int size)
{
  int start, end, cpy;

  flush();
  start = START(node);
  end = END(node);

  while (size) {
    if (end < start) cpy = min(size, B_SIZE - start);
    else cpy = size;

    flush();
    memcpy(dst, (void*) (mpbs[node] + B_START + start), cpy);
    start = (start + cpy) % B_SIZE;
    dst = ((char*) dst) + cpy;
    size -= cpy;
  }

  flush();
  START(node) = start;
  FOOL_WRITE_COMBINE;
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
