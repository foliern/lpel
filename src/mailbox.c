
#include <stdlib.h>
#include <pthread.h>
#include <assert.h>
#include "mailbox.h"
#include "input.h"
#include "scc_comm_func.h"


extern static uintptr_t  addr;

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

mailbox_t *mbox[NR_WORKERS];


/******************************************************************************/
/* Free node pool management functions                                        */
/******************************************************************************/

//takes a node either from the list_free list or creates a new one

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

// add node to to list_free list at the first entry

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

void LpelMailboxInit(){
	int i;

	PRT_DBG("addr inside Mailbox: %p\n",addr);
	for (i=0; i < NR_WORKERS;i++){
		mbox[i]=addr+MEMORY_OFFSET(i)+MAILBOX_OFFSET;
		PRT_DBG("MAILBOX %d address: %p\n",i,mbox[i]);
	}
}



mailbox_t *LpelMailboxCreate(void)
{
  //mailbox_t *mbox = (mailbox_t *)malloc(sizeof(mailbox_t));
  mailbox_t *mbox = (mailbox_t *)SCCMallocPtr(sizeof(mailbox_t));
  PRT_DBG("MAILBOX address: %p\n",mbox);

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
  //pthread_mutex_lock( &mbox->lock_free);
  while (mbox->list_free != NULL) {
    /* pop free node off */
    node = mbox->list_free;
    mbox->list_free = node->next; /* can be NULL */
    /* free the memory for the node */
    free( node);
  }
  //pthread_mutex_unlock( &mbox->lock_free);

  /* destroy sync primitives */
  //pthread_mutex_destroy( &mbox->lock_free);
  //pthread_mutex_destroy( &mbox->lock_inbox);
  //pthread_cond_destroy(  &mbox->notempty);

  free(mbox);
}

void LpelMailboxSend( mailbox_t *mbox, workermsg_t *msg)
{
  /* get a free node from recepient
   * either from the list_free list or a new one gets created */
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


void LpelMailboxRecv( mailbox_t *mbox, workermsg_t *msg)
{
  mailbox_node_t *node;

  /* get node from inbox */
  pthread_mutex_lock( &mbox->lock_inbox);
  while( mbox->list_inbox == NULL) {
      pthread_cond_wait( &mbox->notempty, &mbox->lock_inbox);
  }

  /*writes a message to stderror in case of expression == zero =>
   *error in case of mbox->list_inbox is empty
   */
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


/**
 * @return 1 if there is an incoming msg, 0 otherwise
 * @note: does not need to be locked as a 'missed' msg
 *        will be eventually fetched in the next worker loop
 */
int LpelMailboxHasIncoming( mailbox_t *mbox)
{
  return ( mbox->list_inbox != NULL);
}




void LpelMailboxRecv_scc(mailbox_t *mbox, int node_location){

}

void LpelMailboxSend_scc(int node_location, workermsg_t *msg){



}


