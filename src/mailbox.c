
#include <stdlib.h>
#include <pthread.h>
#include <assert.h>
#include "mailbox.h"
#include <input.h>
//#include "scc_comm_func.h"
#include <pcl.h>
#include <debugging.h>
#include <bool.h>
#include <scc_comm_func.h>
#include <sccmalloc.h>

extern uintptr_t  addr;
extern AIR atomic_inc_regs[2*CORES];

/* mailbox structures */

typedef struct mailbox_node_t {
  struct mailbox_node_t *next;
  workermsg_t msg;
} mailbox_node_t;

struct mailbox_t {
  //pthread_mutex_t  lock_free;
  //pthread_mutex_t  lock_inbox;
  //pthread_cond_t   notempty;
	pthread_mutex_t  		*lock_free;
	pthread_mutex_t 		*lock_inbox;
	pthread_cond_t 			*notempty;
	pthread_mutexattr_t 	*attr;

  int mbox_ID;
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

  //pthread_mutex_lock( &mbox->lock_free);
  int value=-1;
  //PRT_DBG("WAIT in GetFree\n");
  /*while(value != AIR_MBOX_SYNCH_VALUE){
  		  atomic_incR(&atomic_inc_regs[CORES+mbox->mbox_ID],&value);
  }*/
  while(pthread_mutex_trylock(mbox->lock_free) != 0){
    			DCMflush();
  }

  //PRT_DBG("GO-ON in GetFree\n");

  if (mbox->list_free != NULL) {
    /* pop free node off */
    node = mbox->list_free;
    mbox->list_free = node->next; /* can be NULL */
  } else {
    /* allocate new node */
    //node = (mailbox_node_t *)malloc( sizeof( mailbox_node_t));
  	node = (mailbox_node_t *)SCCMallocPtr( sizeof( mailbox_node_t));
  }
  DCMflush();
  //pthread_mutex_unlock( &mbox->lock_free);
  //atomic_writeR(&atomic_inc_regs[CORES+mbox->mbox_ID],AIR_MBOX_SYNCH_VALUE);
  pthread_mutex_unlock(mbox->lock_free);

  return node;
}



// add node to to list_free list at the first entry

static void PutFree( mailbox_t *mbox, mailbox_node_t *node)
{
  //pthread_mutex_lock( &mbox->lock_free);
	int value=-1;
	//PRT_DBG("WAIT in PutFree\n");
	  /*while(value != AIR_MBOX_SYNCH_VALUE){
	  		  atomic_incR(&atomic_inc_regs[CORES+mbox->mbox_ID],&value);
	  }*/
		while(pthread_mutex_trylock(mbox->lock_free) != 0){
	    			DCMflush();
		}

	  //PRT_DBG("GO-ON in PutFree\n");
  DCMflush();
  if ( mbox->list_free == NULL) {
    node->next = NULL;
  } else {
    node->next = mbox->list_free;
  }
  mbox->list_free = node;
  DCMflush();
  //pthread_mutex_unlock( &mbox->lock_free);
  atomic_writeR(&atomic_inc_regs[CORES+mbox->mbox_ID],AIR_MBOX_SYNCH_VALUE);
  pthread_mutex_unlock( mbox->lock_free);
}




/******************************************************************************/
/* Public functions                                                           */
/******************************************************************************/

void LpelMailboxInit(){
	int i;

	PRT_DBG("MEMORY start address: 			%p\n",(void*)addr);
	for (i=0; i < NR_WORKERS;i++){
		//mbox[i]=addr+MEMORY_OFFSET(i)+MAILBOX_OFFSET;
		mbox[i]=addr+MEMORY_OFFSET(i)+0x1190;
//		mbox[i]->list_free  = NULL;
//  		mbox[i]->list_inbox = NULL;
		PRT_DBG("MAILBOX %d address: 		%p\n",i,mbox[i]);
		PRT_DBG("mbox[%d]->list_inbox addr:  	%p\n",i,&mbox[i]->list_inbox);
		PRT_DBG("mbox[%d]->list_inbox: 		%p\n",i,mbox[i]->list_inbox);
		PRT_DBG("mbox[%d]->mbox_ID:	        %d\n",i,mbox[i]->mbox_ID);
	}
	


}



mailbox_t *LpelMailboxCreate(int ID)
{
  //mailbox_t *mbox = (mailbox_t *)malloc(sizeof(mailbox_t));
  mailbox_t *mbox = (mailbox_t *)SCCMallocPtr(sizeof(mailbox_t));
  //PRT_DBG("MAILBOX address: %p\n",mbox);

  //pthread_mutex_init( &mbox->lock_free,  NULL);
  //pthread_mutex_init( &mbox->lock_inbox, NULL);
  //pthread_cond_init(  &mbox->notempty,   NULL);

  int *count;
  count=SCCMallocPtr(sizeof(int));
  *count=1;
  mbox->lock_inbox= SCCMallocPtr(sizeof(pthread_mutex_t));
  mbox->lock_free= SCCMallocPtr(sizeof(pthread_mutex_t));
  mbox->notempty = SCCMallocPtr(sizeof(pthread_cond_t));
  mbox->attr = SCCMallocPtr(sizeof(pthread_mutexattr_t));
  pthread_mutexattr_init(mbox->attr);
  pthread_mutexattr_setpshared(mbox->attr, PTHREAD_PROCESS_SHARED);
  pthread_mutex_init(mbox->lock_inbox,mbox->attr);
  pthread_mutex_init(mbox->lock_free,mbox->attr);
  //pthread_mutex_init(lock_inbox, NULL);
  pthread_cond_init (mbox->notempty, NULL);

  mbox->mbox_ID=ID;
  mbox->list_free  = NULL;
  mbox->list_inbox = NULL;

  PRT_DBG("MAILBOX address: 		%p\n",mbox);
  PRT_DBG("mbox->mbox_ID:             %d\n",mbox->mbox_ID);
  PRT_DBG("mbox->list_inbox: 		%p\n",mbox->list_inbox);
  
  pthread_mutex_lock(mbox->notempty);

  DCMflush();
 
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
  while(pthread_mutex_trylock(mbox->lock_free) != 0){
  	    			DCMflush();
  }

  while (mbox->list_free != NULL) {
    /* pop free node off */
    node = mbox->list_free;
    mbox->list_free = node->next; /* can be NULL */
    /* free the memory for the node */
    //free( node);
	SCCFreePtr(node);
  }
  //pthread_mutex_unlock( &mbox->lock_free);
  pthread_mutex_unlock( mbox->lock_free);

  /* destroy sync primitives */
  pthread_mutex_destroy( mbox->lock_free);
  pthread_mutex_destroy( mbox->lock_inbox);
  pthread_cond_destroy(  mbox->notempty);
  int ret;
  /* destroy an attribute */
  ret = pthread_attr_destroy(mbox->attr);
  if (ret!=0)
	  printf("pthread_attr can not be destroyed!!!\n");
  //free(mbox);
	SCCFreePtr(mbox);
}

void LpelMailboxSend( mailbox_t *mbox, workermsg_t *msg)
{
  /* get a free node from recepient
   * either from the list_free list or a new one gets created */
  mailbox_node_t *node = GetFree( mbox);
  //PRT_DBG("Node address (in send):						%p\n",node);
  /* copy the message */
  node->msg = *msg;

  /* put node into inbox */
  //pthread_mutex_lock( &mbox->lock_inbox);
  int value=-1;
  PRT_DBG("WAIT in LpelMailboxSend\n");
  /*while(value != AIR_MBOX_SYNCH_VALUE){
		  atomic_incR(&atomic_inc_regs[mbox->mbox_ID],&value);
  }*/
  while(pthread_mutex_trylock(mbox->lock_inbox) != 0){
  			DCMflush();
  }


  PRT_DBG("GO-ON in LpelMailboxSend\n");
  //PRT_DBG("MAILBOX address (in send):					%p\n",mbox);
  //PRT_DBG("mbox->list_inbox (in send):					%p\n",mbox->list_inbox);
  if ( mbox->list_inbox == NULL) {
	/* list is empty */
	mbox->list_inbox = node;
	node->next = node; /* self-loop */

	//pthread_cond_signal( &mbox->notempty);
	//PRT_DBG("mbox->mbox_ID (in send):					%d\n",mbox->mbox_ID);
	//PRT_DBG("mbox->notempty, set in register (in send):			%d\n",mbox->mbox_ID+40);
	//atomic_incR(&atomic_inc_regs[mbox->mbox_ID+40],&value);
	pthread_mutex_unlock(mbox->notempty);
  } else {
	/* insert stream between last node=list_inbox
	   and first node=list_inbox->next */
	node->next = mbox->list_inbox->next;
	mbox->list_inbox->next = node;
	mbox->list_inbox = node;
  }

  DCMflush();
  //PRT_DBG("mbox->list_inbox at the end of sending:				%p\n",mbox->list_inbox);
  //pthread_mutex_unlock( &mbox->lock_inbox);
  //atomic_writeR(&atomic_inc_regs[mbox->mbox_ID],AIR_MBOX_SYNCH_VALUE);
  pthread_mutex_unlock(mbox->lock_inbox);
}


void LpelMailboxRecv( mailbox_t *mbox, workermsg_t *msg)
{
  mailbox_node_t *node;
  bool message=false;

  /* get node from inbox */
  //pthread_mutex_lock( &mbox->lock_inbox);
  int value=-1;
  //PRT_DBG("WAIT1 in LpelMailboxRecv\n");
  /*while(value != AIR_MBOX_SYNCH_VALUE){
  		  atomic_incR(&atomic_inc_regs[mbox->mbox_ID],&value);
  }*/
  while(pthread_mutex_trylock(mbox->lock_inbox) != 0){
    			DCMflush();
    }
  DCMflush();
  //PRT_DBG("GO-ON1 in LpelMailboxRecv\n");

  //PRT_DBG("MAILBOX address:						%p\n",mbox);
  //PRT_DBG("mbox->list_inbox:						%p\n",mbox->list_inbox);
 
  //PRT_DBG("mbox->mbox_ID:							%d\n",mbox->mbox_ID);
  //PRT_DBG("mbox->notempty, check in register:				%d\n",mbox->mbox_ID+40);

  if (mbox->list_inbox == NULL){
	  //atomic_writeR(&atomic_inc_regs[mbox->mbox_ID],AIR_MBOX_SYNCH_VALUE);
	  pthread_mutex_unlock(mbox->lock_inbox);
	  value=-1;
	  PRT_DBG("WAIT2 in LpelMailboxRecv\n");
	  while( value < 1) {
		  	 //pthread_cond_wait( &mbox->notempty, &mbox->lock_inbox);
	//		PRT_DBG("mbox->notempty, check in register: %d\n",mbox->mbox_ID+40);
		  //atomic_readR(&atomic_inc_regs[mbox->mbox_ID+40],&value);
		  if (pthread_mutex_trylock(mbox->notempty)=0)
			  value=1;
//			PRT_DBG("value= %d\n", value);
	  }
	//  atomic_decR(&atomic_inc_regs[mbox->mbox_ID+40],value);
	  //PRT_DBG("GO-ON2 in LpelMailboxRecv\n");
  	

	value=-1;
  	//PRT_DBG("WAIT3 in LpelMailboxRecv\n");
    	/*while(value != AIR_MBOX_SYNCH_VALUE){
                  atomic_incR(&atomic_inc_regs[mbox->mbox_ID],&value);
    	}*/
	while(pthread_mutex_trylock(mbox->lock_inbox) != 0){
	    			DCMflush();
	    }

    //	PRT_DBG("GO-ON3 in LpelMailboxRecv\n");
// 	PRT_DBG("MAILBOX address:                                             %p\n",mbox);
 //	PRT_DBG("mbox->list_inbox:                                            %p\n",mbox->list_inbox); 
	DCMflush();
 }
	//PRT_DBG("MAILBOX address:                                             %p\n",mbox);
        //PRT_DBG("mbox->list_inbox:                                            %p\n",mbox->list_inbox);

  /*writes a message to stderror in case of expression == zero =>
   *error in case of mbox->list_inbox is empty
   */
  assert( mbox->list_inbox != NULL);

  /* get first node (handle points to last) */
  node = mbox->list_inbox->next;
  if ( node == mbox->list_inbox) {
    /* self-loop, just single node */
    mbox->list_inbox = NULL;
	//PRT_DBG("Mailbox is empty reset atomic_inc_regs: %d\n",mbox->mbox_ID+40);
	//atomic_writeR(&atomic_inc_regs[mbox->mbox_ID+40],0);
    pthread_mutex_unlock(mbox->notempty);
  } else {
    mbox->list_inbox->next = node->next;
  }
  DCMflush();
  //pthread_mutex_unlock( &mbox->lock_inbox);
  //atomic_writeR(&atomic_inc_regs[mbox->mbox_ID],AIR_MBOX_SYNCH_VALUE);
  pthread_mutex_unlock(mbox->lock_inbox);
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

	LpelMailboxSend(mbox[node_location],msg);

}


