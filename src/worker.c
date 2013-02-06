/**
 * The LPEL worker containing code for workers, master and wrappers
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <assert.h>
#include <errno.h>

#include <pthread.h>
#include "arch/mctx.h"

#include "arch/atomic.h"

#include "worker.h"
#include "configuration.h"
#include "task.h"
#include "lpel_main.h"
#include "lpelcfg.h"
#include "stream.h"
#include "mailbox.h"
#include "lpel/monitor.h"
#include "readTileID.h"
#include "SCC_API_test.h"

#define WORKER_PTR(i) (workers[(i)])
#define MASTER_PTR	master
typedef volatile unsigned char t_vchar;

static void *WorkerThread( void *arg);
static void *MasterThread( void *arg);
static void *WrapperThread( void *arg);

/******************************************************************************/


static int num_workers = -1;
static workerctx_t **workers;
static masterctx_t *master;
static int *waitworkers;	// table of waiting worker

#ifdef HAVE___THREAD
static __thread workerctx_t *workerctx_cur;
static __thread masterctx_t *masterctx;
#else /* HAVE___THREAD */
static pthread_key_t workerctx_key;
static pthread_key_t masterctx_key;
#endif /* HAVE___THREAD */
static int       NODE_ID=-1;
static int _ID = readTileID();

void LpelMasterInit( int size) {

	int i;
	assert(0 <= size);
	num_workers = size - 1;
	NODE_ID = readTileID();

	/** create master */
#ifndef HAVE___THREAD
	/* init key for thread specific data */
	pthread_key_create(&masterctx_key, NULL);
#endif /* HAVE___THREAD */

	master = (masterctx_t *) malloc(sizeof(masterctx_t));
	master->mailbox = LpelMailboxCreate(NODE_ID);
	master->ready_tasks = LpelTaskqueueInit ();


	/** create workers */
#ifndef HAVE___THREAD
	/* init key for thread specific data */
	pthread_key_create(&masterctx_key, NULL);
#endif /* HAVE___THREAD */

	/* allocate worker context table */
	workers = (workerctx_t **) malloc( num_workers * sizeof(workerctx_t*) );
	/* allocate waiting table */
	waitworkers = (int *) malloc(num_workers * sizeof(int));
	/* allocate worker contexts */
	for (i=0; i<num_workers; i++) {
		workers[i] = (workerctx_t *) malloc( sizeof(workerctx_t) );
		waitworkers[i] = 0;
	}

	/* prepare data structures */
	for( i=0; i<num_workers; i++) {
		workerctx_t *wc = WORKER_PTR(i);
		wc->wid = i;

#ifdef USE_LOGGING
		if (MON_CB(worker_create)) {
			wc->mon = MON_CB(worker_create)(wc->wid);
		} else {
			wc->mon = NULL;
		}
#else
	wc->mon = NULL;
#endif

	/* mailbox */
	wc->mailbox = LpelMailboxCreate();
	}
}

// clean up for both worker and master
void LpelMasterCleanup( void) {
	int i;
	workerctx_t *wc;

	for( i=0; i<num_workers; i++) {
		wc = WORKER_PTR(i);
		/* wait for the worker to finish */
		(void) pthread_join( wc->thread, NULL);
	}
	/* wait for the master to finish */
	(void) pthread_join( master->thread, NULL);

	/* cleanup the data structures */
	for( i=0; i<num_workers; i++) {
		wc = WORKER_PTR(i);
		LpelMailboxDestroy(wc->mailbox);
		free(wc);
	}
	/* free workers tables */
	free( workers);
	free( waitworkers);

	/* clean up master's mailbox */
	workermsg_t msg;
	while (LpelMailboxHasIncoming(MASTER_PTR->mailbox)) {
		LpelMailboxRecv(MASTER_PTR->mailbox, &msg);
		assert(msg.type == MSG_TASKREQ);
	}

	LpelMailboxDestroy(master->mailbox);
	LpelTaskqueueDestroy(master->ready_tasks);
	free(master);

#ifndef HAVE___THREAD
	pthread_key_delete(workerctx_key);
	pthread_key_delete(workerctx_key);
#endif /* HAVE___THREAD */
}


void LpelMasterSpawn( void) {
	int i;
	int temp_mpb;
	int processor_ID;
	char *cond_message= "C";
	t_vchar worker_message;
	/* master */

	processor_ID = readTileID();
	temp_mpb=4;

	if (processor_ID == 0) {
		PRT_DBG("I am processor %i, the CONDUCTOR! \n", processor_ID);
		(void) pthread_create( &master->thread, NULL, MasterThread, MASTER_PTR); 	/* spawn joinable thread */



		LpelMailboxSend_overMPB(cond_message, strlen(cond_message),temp_mpb);
		PRT_DBG("Nachricht: %s in MPB von Node %d geschrieben, size: %u\n", cond_message, temp_mpb, strlen(cond_message));
		LpelMailboxRecv_overMPB(&worker_message, strlen(cond_message),temp_mpb);
		PRT_DBG("Nachricht aus MPB von Node %d: %c \n",temp_mpb, worker_message);
	} else {
		// worker_message= "init Messag2";
//		 *wc = WORKER_PTR(0);
		PRT_DBG("I am processor %i, the WORKER \n",processor_ID);
                (void) pthread_create( &master->thread, NULL, MasterThread, MASTER_PTR);        /* spawn joinable thread */
//                LpelMailboxSend_overMPB(1, &cond_message, sizeof(cond_message));
//		(void) pthread_create( &wc->thread, NULL, WorkerThread, wc);
		LpelMailboxRecv_overMPB(&worker_message, strlen(cond_message),temp_mpb);
		PRT_DBG("Nachricht aus MPB von Node %d: %c \n",temp_mpb, worker_message);
	}

//	PRT_DBG("I am the MASTER! \n");
//	(void) pthread_create( &master->thread, NULL, MasterThread, MASTER_PTR); 	/* spawn joinable thread */
//
//	/* workers */
//	for( i=0; i<num_workers; i++) {
//		workerctx_t *wc = WORKER_PTR(i);
//		PRT_DBG("I am the SLAVE NR.:%i \n",i);
//		(void) pthread_create( &wc->thread, NULL, WorkerThread, wc);

}


void LpelMasterTerminate(void) {
	workermsg_t msg;
	msg.type = MSG_TERMINATE;
	LpelMailboxSend(MASTER_PTR->mailbox, &msg);
	LpelWorkerBroadcast(&msg);
}

void LpelStartTask( lpel_task_t *t) {
	 workermsg_t msg;
   msg.type = MSG_TASKASSIGN;
	 msg.body.task = t;

	if (t->worker_context != NULL) {	// wrapper
		LpelMailboxSend(t->worker_context->mailbox, &msg);
	}
	else {
		LpelMailboxSend(MASTER_PTR->mailbox, &msg);
	}
}

/*******************************************************************************
*******************************************************************************/
static void returnTask( lpel_task_t *t) {
	workermsg_t msg;
	msg.type = MSG_TASKRET;
	msg.body.task = t;
	LpelMailboxSend(MASTER_PTR->mailbox, &msg);
}


static void requestTask( workerctx_t *wc) {
	workermsg_t msg;
	msg.type = MSG_TASKREQ;
	msg.body.from_worker = wc->wid;
	LpelMailboxSend(MASTER_PTR->mailbox, &msg);
#ifdef USE_LOGGING
	if (wc->mon && MON_CB(worker_tskreq)) {
		MON_CB(worker_tskreq)(wc->mon);
	}
#endif
}


static void sendTask( int wid, lpel_task_t *t) {
	assert(t->state == TASK_READY);
	workermsg_t msg;
	msg.type = MSG_TASKASSIGN;
	msg.body.task = t;
	LpelMailboxSend(WORKER_PTR(wid)->mailbox, &msg);
}

static void sendWakeup( mailbox_t *mb, lpel_task_t *t)
{
	workermsg_t msg;
	msg.type = MSG_TASKWAKEUP;
	msg.body.task = t;
	LpelMailboxSend(mb, &msg);
}

/*******************************************************************************
 * MASTER FUNCTION
 ******************************************************************************/
static int servePendingReq( lpel_task_t *t) {
	int i;
	for (i = 0; i < num_workers; i++){
		if (waitworkers[i] == 1) {
			waitworkers[i] = 0;
			PRT_DBG("master: send task %d to worker %d\n", t->uid, i);
			sendTask(i, t);
			return i;
		}
	}
	return -1;
}

static void updatePriorList(taskqueue_t *tq, stream_elem_t *list, char mode) {
	double np;
	lpel_task_t *t;
	lpel_stream_t *s;
	while (list != NULL) {
		s = list->stream_desc->stream;
		if (mode == 'r')
			t = LpelStreamProducer(s);
		else if (mode == 'w')
			t = LpelStreamConsumer(s);
		if (t && t->state == TASK_INQUEUE) {
			np = LpelTaskCalPrior(t);
			LpelTaskqueueUpdatePrior(tq, t, np);
		}
		list = list->next;
	}
}

/* update prior for neighbors of t
 * @cond: called only by master to avoid concurrent access
 */
static void updatePriorNeighbour( taskqueue_t *tq, lpel_task_t *t) {
	updatePriorList(tq, t->sched_info.in_streams, 'r');
	updatePriorList(tq, t->sched_info.out_streams, 'w');
}


static void MasterLoop( void)
{
	PRT_DBG("start master\n");
	do {
		workermsg_t msg;

		LpelMailboxRecv(MASTER_PTR->mailbox, &msg);
		lpel_task_t *t;
		int wid;
		switch( msg.type) {
		case MSG_TASKASSIGN:
			/* master receive a new task */
			t = msg.body.task;
			assert (t->state == TASK_CREATED);
			t->state = TASK_READY;
			PRT_DBG("master: get task %d\n", t->uid);
			if (servePendingReq(t) < 0) {		// no pending request
#ifdef _USE_DYNAMIC_PRIORITY_
				t->sched_info.prior = LpelTaskCalPrior(t);	//update new prior before add to the queue
#endif
				t->state = TASK_INQUEUE;
				LpelTaskqueuePush(MASTER_PTR->ready_tasks, t);
			}
			break;

		case MSG_TASKRET:
			t = msg.body.task;
			PRT_DBG("master: get returned task %d\n", t->uid);
			switch(t->state) {
			case TASK_BLOCKED:
				t->state = TASK_RETURNED;
#ifdef _USE_DYNAMIC_PRIORITY_
				updatePriorNeighbour(MASTER_PTR->ready_tasks, t);
#endif
				break;

			case TASK_READY:	// task yields
				if (servePendingReq(t) < 0) {		// no pending request
#ifdef _USE_DYNAMIC_PRIORITY_
					updatePriorNeighbour(MASTER_PTR->ready_tasks, t);
					t->sched_info.prior = LpelTaskCalPrior(t);	//update new prior before add to the queue
#endif
					t->state = TASK_INQUEUE;
					LpelTaskqueuePush(MASTER_PTR->ready_tasks, t);
				}
				break;

			default: // TASK_ZOMBIE
				assert(0);
				break;
			}
			break;

		case MSG_TASKWAKEUP:
			t = msg.body.task;
			if (t->state != TASK_RETURNED) {		// task has not been returned yet
				PRT_DBG("master: put message back\n");
				LpelMailboxSend(MASTER_PTR->mailbox, &msg);		//task is not blocked yet (the other worker is a bit slow, put back to the mailbox for processing later
				break;
			}
			PRT_DBG("master: unblock task %d\n", t->uid);
			assert (t->state == TASK_RETURNED);
			t->state = TASK_READY;
			if (servePendingReq(t) < 0) {		// no pending request
#ifdef _USE_DYNAMIC_PRIORITY_
					t->sched_info.prior = LpelTaskCalPrior(t);	//update new prior before add to the queue
#endif
					t->state = TASK_INQUEUE;
					LpelTaskqueuePush(MASTER_PTR->ready_tasks, t);
			}
			break;

		case MSG_TASKREQ:
			wid = msg.body.from_worker;
			PRT_DBG("master: request task from worker %d\n", wid);
			t = LpelTaskqueuePeek(MASTER_PTR->ready_tasks);
			if (t == NULL) {
				waitworkers[wid] = 1;
			} else {
				t->state = TASK_READY;
				sendTask(wid, t);
			}
			t = LpelTaskqueuePop(MASTER_PTR->ready_tasks);
			break;

//		case MSG_UPDATEPRIOR:
//			t = msg.body.task;
//			updatePriorNeighbour(MASTER_PTR->ready_tasks, t);
//			break;

		case MSG_TERMINATE:
			assert((LpelTaskqueueSize(MASTER_PTR->ready_tasks) == 0));
			MASTER_PTR->terminate = 1;
			break;
		default:
			assert(0);
		}
	} while ( !( MASTER_PTR->terminate) );
}



static void *MasterThread( void *arg)
{
  masterctx_t *ms = (masterctx_t *)arg;

#ifdef HAVE___THREAD
  masterctx = ms;
#else /* HAVE___THREAD */
  /* set pointer to worker context as TSD */
  pthread_setspecific(masterctx_key, ms);
#endif /* HAVE___THREAD */


//FIXME
#ifdef USE_MCTX_PCL
  assert( 0 == co_thread_init());
  ms->mctx = co_current();
#endif


  /* assign to cores */
  ms->terminate = 0;
  LpelThreadAssign( LPEL_MAP_MASTER);

  // master loop, no monitor for master
  MasterLoop();


#ifdef USE_MCTX_PCL
  co_thread_cleanup();
#endif

  return NULL;
}


/*******************************************************************************
 * WRAPPER FUNCTION
 ******************************************************************************/

static void WrapperLoop( workerctx_t *wp)
{
	lpel_task_t *t = NULL;
	workermsg_t msg;

	do {
		t = wp->wraptask;
		if (t != NULL) {
			/* execute task */
			wp->current_task = t;
			mctx_switch(&wp->mctx, &t->mctx);
		} else {
			/* no ready tasks */
			LpelMailboxRecv(wp->mailbox, &msg);
			switch(msg.type) {
			case MSG_TASKASSIGN:
				t = msg.body.task;
				PRT_DBG("wrapper: get task %d\n", t->uid);
				assert(t->state == TASK_CREATED);
				t->state = TASK_READY;
				wp->wraptask = t;
#ifdef USE_LOGGING
				if (t->mon) {
					if (MON_CB(worker_create_wrapper)) {
						wp->mon = MON_CB(worker_create_wrapper)(t->mon);
					} else {
						wp->mon = NULL;
					}
				}
				if (t->mon && MON_CB(task_assign)) {
					MON_CB(task_assign)(t->mon, wp->mon);
				}
#endif
				break;

			case MSG_TASKWAKEUP:
				t = msg.body.task;
				PRT_DBG("wrapper: unblock task %d\n", t->uid);
				assert (t->state == TASK_BLOCKED);
				t->state = TASK_READY;
				wp->wraptask = t;
#ifdef USE_LOGGING
				if (t->mon && MON_CB(task_assign)) {
					MON_CB(task_assign)(t->mon, wp->mon);
				}
#endif
				break;
			default:
				assert(0);
				break;
			}
		}
	} while ( !wp->terminate);
	LpelTaskDestroy(wp->wraptask);
	/* cleanup task context marked for deletion */
}


static void *WrapperThread( void *arg)
{

	workerctx_t *wp = (workerctx_t *)arg;

#ifdef HAVE___THREAD
	workerctx_cur = wp;
#else /* HAVE___THREAD */
	/* set pointer to worker context as TSD */
	pthread_setspecific(workerctx_key, wp);
#endif /* HAVE___THREAD */

#ifdef USE_MCTX_PCL
	assert( 0 == co_thread_init());
	wp->mctx = co_current();
#endif

	LpelThreadAssign( wp->wid);
	WrapperLoop( wp);

	LpelMailboxDestroy(wp->mailbox);
	free( wp);

#ifdef USE_MCTX_PCL
	co_thread_cleanup();
#endif
	return NULL;
}

workerctx_t *LpelCreateWrapperContext(int wid) {
	workerctx_t *wp = (workerctx_t *) malloc( sizeof( workerctx_t));
	wp->wid = wid;
	wp->terminate = 0;
	/* Wrapper is excluded from scheduling module */
	wp->wraptask = NULL;
	wp->current_task = NULL;
	wp->mon = NULL;
	//wp->marked_del = NULL;

	/* mailbox */
	wp->mailbox = LpelMailboxCreate();
	/* taskqueue of free tasks */
	(void) pthread_create( &wp->thread, NULL, WrapperThread, wp);
	(void) pthread_detach( wp->thread);
	return wp;
}






/*******************************************************************************
 * WORKER FUNCTION
 ******************************************************************************/
void LpelWorkerBroadcast(workermsg_t *msg)
{
	int i;
	workerctx_t *wc;

	for( i=0; i<num_workers; i++) {
		wc = WORKER_PTR(i);
		/* send */
		LpelMailboxSend(wc->mailbox, msg);
	}
}



static void WorkerLoop( workerctx_t *wc)
{
	PRT_DBG("start worker %d\n", wc->wid);

  lpel_task_t *t = NULL;
  requestTask( wc);		// ask for the first time

  workermsg_t msg;
  do {
  	  LpelMailboxRecv(wc->mailbox, &msg);

  	  switch( msg.type) {
  	  case MSG_TASKASSIGN:
  	  	t = msg.body.task;
  	  	PRT_DBG("worker %d: get task %d\n", wc->wid, t->uid);
  	  	assert(t->state == TASK_READY);
  	  	t->worker_context = wc;
  	  	wc->current_task = t;

#ifdef USE_LOGGING
  	  	if (wc->mon && MON_CB(worker_tskass)) {
  	  		MON_CB(worker_tskass)(wc->mon);
  	  	}
  	  	if (t->mon && MON_CB(task_assign)) {
  	  		MON_CB(task_assign)(t->mon, wc->mon);
  	  	}
#endif
  	  	mctx_switch(&wc->mctx, &t->mctx);
  	  	//task return here
  	  	assert(t->state != TASK_RUNNING);
  	  	if (t->state != TASK_ZOMBIE) {
  	  		t->worker_context = NULL;
  	  		returnTask(t);
  	  	} else
  	  		LpelTaskDestroy(t);		// if task finish, destroy it and not return to master
  	  	break;
  	  case MSG_TERMINATE:
  	  	wc->terminate = 1;
  	  	break;
  	  default:
  	  	assert(0);
  	  	break;
  	  }
  	  // reach here --> message request for task has been sent
  } while ( !(wc->terminate) );
}


static void *WorkerThread( void *arg)
{
  workerctx_t *wc = (workerctx_t *)arg;

#ifdef HAVE___THREAD
  workerctx_cur = wc;
#else /* HAVE___THREAD */
  /* set pointer to worker context as TSD */
  pthread_setspecific(workerctx_key, wc);
#endif /* HAVE___THREAD */


//FIXME
#ifdef USE_MCTX_PCL
  assert( 0 == co_thread_init());
  wc->mctx = co_current();
#endif

  wc->terminate = 0;
  wc->current_task = NULL;
  //wc->marked_del = NULL;
	LpelThreadAssign( wc->wid + 1);		// 0 is for the master
	WorkerLoop( wc);

#ifdef USE_LOGGING
  /* cleanup monitoring */
  if (wc->mon && MON_CB(worker_destroy)) {
    MON_CB(worker_destroy)(wc->mon);
  }
#endif

#ifdef USE_MCTX_PCL
  co_thread_cleanup();
#endif
  return NULL;
}




workerctx_t *LpelWorkerSelf(void){
#ifdef HAVE___THREAD
  return workerctx_cur;
#else /* HAVE___THREAD */
  return (workerctx_t *) pthread_getspecific(workerctx_key);
#endif /* HAVE___THREAD */
}


lpel_task_t *LpelWorkerCurrentTask(void)
{
  return LpelWorkerSelf()->current_task;
}



/******************************************
 * TASK RELATED FUNCTIONS
 ******************************************/

void LpelWorkerTaskExit(lpel_task_t *t) {
	workerctx_t *wc = t->worker_context;
	PRT_DBG("worker %d: task %d exit\n", wc->wid, t->uid);
	if (wc->wid >= 0)
		requestTask(wc);	// FIXME: should have requested before
	else
		wc->terminate = 1;		// wrapper: terminate

	wc->current_task = NULL;
	mctx_switch( &t->mctx, &wc->mctx);		// switch back to the worker
}


void LpelWorkerTaskBlock(lpel_task_t *t){
	workerctx_t *wc = t->worker_context;
	if (wc->wid < 0) {	//wrapper
			wc->wraptask = NULL;
#ifdef USE_LOGGING
			if (wc->mon && MON_CB(worker_tskreq)) {
				MON_CB(worker_tskreq)(wc->mon);
			}
#endif
	} else {
		PRT_DBG("worker %d: block task %d\n", wc->wid, t->uid);
		//sendUpdatePrior(t);		//update prior for neighbor
		requestTask(wc);
	}
	wc->current_task = NULL;
	mctx_switch( &t->mctx, &wc->mctx);		// switch back to the worker/wrapper
}

void LpelWorkerTaskYield(lpel_task_t *t){
	workerctx_t *wc = t->worker_context;
	if (wc->wid < 0) {	//wrapper
			wc->wraptask = t;
			PRT_DBG("wrapper: task %d yields\n");
	}
	else {
		//sendUpdatePrior(t);		//update prior for neighbor
		requestTask(wc);
		PRT_DBG("worker %d: return task %d\n", wc->wid, t->uid);
	}
	wc->current_task = NULL;
	mctx_switch( &t->mctx, &wc->mctx);		// switch back to the worker/wrapper
}

void LpelWorkerTaskWakeup( lpel_task_t *t) {
	workerctx_t *wc = t->worker_context;
	PRT_DBG("worker %d: send wake up task %d\n", LpelWorkerSelf()->wid, t->uid);
	if (wc == NULL)
		sendWakeup(MASTER_PTR->mailbox, t);
	else {
		if (wc->wid < 0)
			sendWakeup(wc->mailbox, t);
		else
			sendWakeup(MASTER_PTR->mailbox, t);
	}
}
