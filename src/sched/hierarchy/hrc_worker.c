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

#include "bool.h"
#include "hrc_worker.h"
//#include "hrc_task.h"
#include "lpel_main.h"
#include "lpelcfg.h"
#include "stream.h"
//#include "mailbox.h"
#include "lpel/monitor.h"
#include <input.h>
#include <sccmalloc.h>

//to initialize MPB and LUT
#include <scc_comm_func.h>
#include <pcl.h>



//global variable to safe node ID
int node_ID;
//equal number of cores gets decremented each time a wrapper mbox is created, more or less for synchronisation in mailbox.c, just a temporary solution!!!
int wrapper_mbox_ID=CORES-1;
static void *local;
//#define WORKER_PTR(i) (workers[(i)])
#define WORKER_PTR worker

#define MASTER_PTR	master

#define RoodNode(node_ID) (node_ID==MASTER ? true : false)

static void *WorkerThread( void *arg);
static void *MasterThread( void *arg);
static void *WrapperThread( void *arg);

/******************************************************************************/


static int num_workers = -1;


static mailbox_t *mbox_workers[NR_WORKERS];
static workerctx_t *worker;

static masterctx_t *master;
static int *waitworkers;	// table of waiting worker


//#ifdef HAVE___THREAD
static __thread workerctx_t *workerctx_cur;
static __thread masterctx_t *masterctx;
//#else /* HAVE___THREAD */
static pthread_key_t workerctx_key;
//#endif /* HAVE___THREAD */





/**
 * Initialise worker globally
 *
 *
 * @param size    size of the worker set, i.e., the total number of workers including master
 */
void LpelWorkersInit( int size) {



	int i;
	assert(0 <= size);

	//init LUT and MPB directory's
//	scc_init();
	LpelMailboxInit();

	num_workers = NR_WORKERS;

	node_ID=SccGetNodeID();
#ifndef HAVE___THREAD
                /* init key for thread specific data */
                pthread_key_create(&workerctx_key, NULL);
#endif /* HAVE___THREAD */

	if (node_ID==MASTER){

		/** create MASTER */

		mailbox_t *mbox = LpelMailboxCreate(node_ID);

		master = (masterctx_t *) SCCMallocPtr(sizeof(masterctx_t));
		//PRT_DBG("masterctx address: %p\n", master);
		master->mailbox = mbox;
		PRT_DBG("MASTER mailbox address: %p\n", master->mailbox);
		master->ready_tasks = LpelTaskqueueInit ();

		/** create waitworkers array */


		/* a structure is created to give the MASTER later knowledge about the participating workers
		/* allocate worker context table */
		//workers = (workerctx_t **) malloc( num_workers * sizeof(workerctx_t*) );
		/* allocate waiting table */

		//waitworkers = (int *) malloc(num_workers * sizeof(int));
		waitworkers = (int *) SCCMallocPtr(num_workers * sizeof(int));

		//local=SccGetlocal();
		/* allocate worker contexts */
		for (i=0; i<num_workers; i++) {
			//workers[i] = (workerctx_t *) malloc( sizeof(workerctx_t) );
			waitworkers[i] = 0;

			//mbox_workers[i+1]=local+MEMORY_OFFSET(i+1);
			//PRT_DBG("address: %p\n", mbox_workers[i+1]);
		}

	}else{

		/** create SINGLE WORKER */

		mailbox_t *mbox = LpelMailboxCreate(node_ID);

		/* allocate worker context table, but only for one worker */
		worker=(workerctx_t *) SCCMallocPtr(sizeof(workerctx_t));
		worker->wid=node_ID;
		// NOT SURE IF NEEDED OR NOT
#ifdef USE_LOGGING
		if (MON_CB(worker_create)) {
			worker->mon = MON_CB(worker_create)(worker->wid);
		} else {
			worker->mon = NULL;
		}
#else
			worker->mon = NULL;
#endif

		/*mailbox*/
		worker->mailbox = mbox;
		PRT_DBG("WORKER mailbox address: %p\n", worker->mailbox);
	}//end else


}

/*
 * clean up for both worker and master
 */
void LpelWorkersCleanup( void) {
	int i;
	workerctx_t *wc;

	/* not need because only one worker
		for( i=0; i<num_workers; i++) {
			wc = WORKER_PTR(i);
			// wait for the worker to finish //
			(void) pthread_join( wc->thread, NULL);
		}
	*/
	if (node_ID!=MASTER){

	/* wait for the worker to finish */
	 	(void) pthread_join( worker->thread, NULL);

		PRT_DBG("CLEAN; worker finished");

	/* cleanup the data structures */

		/*for( i=0; i<num_workers; i++) {
			wc = WORKER_PTR(i);
			LpelMailboxDestroy(wc->mailbox);
			free(wc);
		}
		*/
		LpelMailboxDestroy(worker->mailbox);
		/* free workers tables */
		//free( workers);
		//free(worker);
		SCCFreePtr(worker);

#ifndef HAVE___THREAD
		pthread_key_delete(workerctx_key);
		pthread_key_delete(workerctx_key);
#endif /* HAVE___THREAD */
	}else{
	    /* wait for the master to finish */
		(void) pthread_join( master->thread, NULL);

		PRT_DBG("CLEAN; master finished");
		/* free waitworkers tables */
		//free( waitworkers);
		SCCFreePtr(waitworkers);
		/* clean up master's mailbox */

		workermsg_t msg;
		while (LpelMailboxHasIncoming(MASTER_PTR->mailbox)) {
			LpelMailboxRecv(MASTER_PTR->mailbox, &msg);
			assert(msg.type == WORKER_MSG_REQUEST);
		}

		LpelMailboxDestroy(master->mailbox);
		LpelTaskqueueDestroy(master->ready_tasks);
		//free(master);
		SCCFreePtr(master);
	}
}


/*
 * Spawn master and workers
 */
void LpelWorkersSpawn( void) {
	int i;

	if (node_ID==MASTER){
		/* master */
		(void) pthread_create( &master->thread, NULL, MasterThread, MASTER_PTR); 	/* spawn joinable thread */
	}else{
		/* workers */
		/*for( i=0; i<num_workers; i++) {
			workerctx_t *wc = WORKER_PTR(i);
			(void) pthread_create( &wc->thread, NULL, WorkerThread, wc);
		}*/
		(void) pthread_create( &worker->thread, NULL, WorkerThread, WORKER_PTR);
	}
}



/*
 * THIS FUNCTION IS STILL TO CHANGE, BUT I THNK THIS FUNCTION IS USELESS IN OUR APPROACH
 */
/*
 * Terminate master and workers
 */
void LpelWorkersTerminate(void) {
	workermsg_t msg;
	msg.type = WORKER_MSG_TERMINATE;
	LpelMailboxSend(MASTER_PTR->mailbox, &msg);

	LpelWorkerBroadcast(&msg);
}

/**
 * Assign a task to the worker by sending an assign message to that worker
 * FUCNTION HAS TO BE CHANGED AND LOOKED PROPERLY
 */
void LpelWorkerRunTask( lpel_task_t *t) {
	 workermsg_t msg;
     msg.type = WORKER_MSG_ASSIGN;
	 msg.body.task = t;

//just some debugging print outs
	PRT_DBG("NEW TASK INFORMATION:\n");
	PRT_DBG("task state: %c\n",t->state);
	PRT_DBG("size: %d\n",t->size);
	PRT_DBG("LUT node: %d\n",t->addr->node);
	PRT_DBG("LUT Nr.:  %d\n",t->addr->lut);
	PRT_DBG("LUT offset: %u\n",t->addr->offset);

	/*should in our case always be sent to the master..
	 * but think about the possibility that a worker creates a task!!!
	 */
	if (t->worker_context != NULL) {	// wrapper
		//NOT CLEAR WHAT HAPPENS HERE
		LpelMailboxSend(t->worker_context->mailbox, &msg);
	}
	else {
		/* This function is exectued only by LpelTaskStart which will be executed only by the Master, which creates the task therefore
		 * the task can directly be pushed into the Mailbox instead of sending it over the MPB to the Master
		 */
		LpelMailboxSend(MASTER_PTR->mailbox, &msg);
		//MPBmsgSend(MASTER,&msg);
	}
}


/************************ Private functions ***********************************/
static void returnTask( lpel_task_t *t) {
	workermsg_t msg;
	msg.type = WORKER_MSG_RETURN;
	msg.body.task = t;
	LpelMailboxSend_scc(MASTER, &msg);
}


static void requestTask( workerctx_t *wc) {
	PRT_DBG("worker %d: request task\n", wc->wid);
	workermsg_t msg;
	msg.type = WORKER_MSG_REQUEST;
	msg.body.from_worker = wc->wid;
	//send over MPB because this function is called by workers
	//LpelMailboxSend(MASTER_PTR->mailbox, &msg);
	LpelMailboxSend_scc(MASTER, &msg);

#ifdef USE_LOGGING
	if (wc->mon && MON_CB(worker_waitstart)) {
		MON_CB(worker_waitstart)(wc->mon);
	}
#endif
}


static void sendTask( int wid, lpel_task_t *t) {
	assert(t->state == TASK_READY);
	workermsg_t msg;
	msg.type = WORKER_MSG_ASSIGN;
	msg.body.task = t;
	//send Task to the worker
	//LpelMailboxSend(WORKER_PTR(wid)->mailbox, &msg);
	LpelMailboxSend_scc(wid,&msg);
}



static void sendWakeup( mailbox_t *mb, lpel_task_t *t)
{
	workermsg_t msg;
	msg.type = WORKER_MSG_WAKEUP;
	msg.body.task = t;
	if (mb == NULL)
		LpelMailboxSend_scc(MASTER, &msg);
	else
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

static void updatePriorityList(taskqueue_t *tq, stream_elem_t *list, char mode) {
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
			np = LpelTaskCalPriority(t);
			LpelTaskqueueUpdatePriority(tq, t, np);
		}
		list = list->next;
	}
}


/* update prior for neighbors of t
 * @cond: called only by master to avoid concurrent access
 */
static void updatePriorityNeigh( taskqueue_t *tq, lpel_task_t *t) {
	updatePriorityList(tq, t->sched_info->in_streams, 'r');
	updatePriorityList(tq, t->sched_info->out_streams, 'w');
}


static void MasterLoop( void)
{
	PRT_DBG("start master\n");
	do {
		workermsg_t msg;

		//LpelMailboxRecv_scc(MASTER_PTR->mailbox,MASTER);
		LpelMailboxRecv(MASTER_PTR->mailbox, &msg);
		PRT_DBG("MSG received, handle it!\n");
		lpel_task_t *t;
		int wid;
		switch( msg.type) {
		case WORKER_MSG_ASSIGN:
			/* master receive a new task */
			t = msg.body.task;
			assert (t->state == TASK_CREATED);
			t->state = TASK_READY;
			PRT_DBG("master: get task %d\n", t->uid);
			if (servePendingReq(t) < 0) {		// no pending request
				t->sched_info->prior = LpelTaskCalPriority(t);	//update new prior before add to the queue
				t->state = TASK_INQUEUE;
				LpelTaskqueuePush(MASTER_PTR->ready_tasks, t);
			}
			break;

		case WORKER_MSG_RETURN:
			t = msg.body.task;
			PRT_DBG("master: get returned task %d\n", t->uid);
			switch(t->state) {
			case TASK_BLOCKED:
				t->state = TASK_RETURNED;
				updatePriorityNeigh(MASTER_PTR->ready_tasks, t);
				break;

			case TASK_READY:	// task yields
				if (servePendingReq(t) < 0) {		// no pending request
					updatePriorityNeigh(MASTER_PTR->ready_tasks, t);
					t->sched_info->prior = LpelTaskCalPriority(t);	//update new prior before add to the queue
					t->state = TASK_INQUEUE;
					LpelTaskqueuePush(MASTER_PTR->ready_tasks, t);
				}
				break;

			default: // TASK_ZOMBIE
				assert(0);
				break;
			}
			break;

		case WORKER_MSG_WAKEUP:
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
					t->sched_info->prior = LpelTaskCalPriority(t);	//update new prior before add to the queue
					t->state = TASK_INQUEUE;
					LpelTaskqueuePush(MASTER_PTR->ready_tasks, t);
			}
			break;

		case WORKER_MSG_REQUEST:
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

//		case WORKER_MSG_PRIORITY:
//			t = msg.body.task;
//			updatePriorityNeigh(MASTER_PTR->ready_tasks, t);
//			break;

		case WORKER_MSG_TERMINATE:
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
  masterctx= ms;
#else /* HAVE___THREAD */
  /* set pointer to worker context as TSD */
  pthread_setspecific(workerctx_key, ms);
#endif /* HAVE___THREAD */


//FIXME
#ifdef USE_MCTX_PCL
  assert( 0 == co_thread_init());
  ms->mctx = co_current();
#endif
PRT_DBG("ms->mctx: %p\n",ms->mctx);

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
		t = wp->current_task;
		if (t != NULL) {
			/* execute task */
			wp->current_task = t;
			PRT_DBG("wrapper: switch to task %d\n", t->uid);
			PRT_DBG("wp->mctx:		%p\n",wp->mctx);
			PRT_DBG("wp->mctx:              %p\n",&wp->mctx);
			PRT_DBG("t->mctx:              %p\n",t->mctx);
			DCMflush();
			//mctx_switch(&wp->mctx, &t->mctx);
			(void) co_call(t->mctx);
			 DCMflush();
		} else {
			/* no ready tasks */
			LpelMailboxRecv(wp->mailbox, &msg);
			switch(msg.type) {
			case WORKER_MSG_ASSIGN:
				t = msg.body.task;
				PRT_DBG("wrapper: get task %d\n", t->uid);
				assert(t->state == TASK_CREATED);
				t->state = TASK_READY;
				wp->current_task = t;
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

			case WORKER_MSG_WAKEUP:
				t = msg.body.task;
				PRT_DBG("wrapper: unblock task %d\n", t->uid);
				assert (t->state == TASK_BLOCKED);
				t->state = TASK_READY;
				wp->current_task = t;
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

	if(t)
		LpelTaskDestroy(t);
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
	PRT_DBG("wp->mctx: %p\n",wp->mctx);

	LpelThreadAssign( wp->wid);
	WrapperLoop( wp);

	LpelMailboxDestroy(wp->mailbox);
	//free( wp);
	SCCFreePtr(wp);
#ifdef USE_MCTX_PCL
	co_thread_cleanup();
#endif
	return NULL;
}

workerctx_t *LpelCreateWrapperContext(int wid) {
	workerctx_t *wp = (workerctx_t *) SCCMallocPtr( sizeof( workerctx_t));
	wp->wid = wid;
	wp->terminate = 0;
	/* Wrapper is excluded from scheduling module */
	wp->current_task = NULL;
	wp->current_task = NULL;
	wp->mon = NULL;
	//wp->marked_del = NULL;

	/* mailbox */
	PRT_DBG("Create Wrapper Mailbox\n");
	wp->mailbox = LpelMailboxCreate(wrapper_mbox_ID);
	wrapper_mbox_ID--;
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
		//wc = WORKER_PTR(i);
		/* send */
		LpelMailboxSend_scc(i, msg);
	}
}



static void WorkerLoop( workerctx_t *wc)
{
	PRT_DBG("start worker %d\n", wc->wid);

  lpel_task_t *t = NULL;
  requestTask( wc);		// ask for the first time

  workermsg_t msg;
  do {
  		//LpelMailboxRecv_scc(WORKER_PTR->mailbox,wc->wid);
  		LpelMailboxRecv(wc->mailbox, &msg);
  		lpel_task_t *t;

  	  switch( msg.type) {
  	  case WORKER_MSG_ASSIGN:
  	  	t = msg.body.task;
  	  	PRT_DBG("worker %d: get task %d\n", wc->wid, t->uid);
  	  	assert(t->state == TASK_READY);
  	  	t->worker_context = wc;
  	  	wc->current_task = t;

#ifdef USE_LOGGING
  	  	if (wc->mon && MON_CB(worker_waitstop)) {
  	  		MON_CB(worker_waitstop)(wc->mon);
  	  	}
  	  	if (t->mon && MON_CB(task_assign)) {
  	  		MON_CB(task_assign)(t->mon, wc->mon);
  	  	}
#endif
  		PRT_DBG("t->mctx:              %p\n",t->mctx);
		DCMflush();  
		//mctx_switch(&wc->mctx, &t->mctx);
  	  	(void) co_call(t->mctx);
		 DCMflush();
		//task return here
  	  	assert(t->state != TASK_RUNNING);
  	  	if (t->state != TASK_ZOMBIE) {
  	  		t->worker_context = NULL;
  	  		PRT_DBG("worker %d: return task %d, state %c\n", wc->wid, t->uid, t->state);
  	  		returnTask(t);
  	  	} else {
  	  		LpelTaskDestroy(t);
  	  	}
  	  	break;
  	  case WORKER_MSG_TERMINATE:
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

    //LpelThreadAssign( wc->wid + 1);		// 0 is for the master
    LpelThreadAssign(LPEL_MAP_WORKER);		// 0 is for the master
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

void LpelWorkerSelfTaskExit(lpel_task_t *t) {
	workerctx_t *wc = t->worker_context;
	PRT_DBG("worker %d: task %d exit\n", wc->wid, t->uid);
	if (wc->wid >= 0)
		requestTask(wc);	// FIXME: should have requested before
	else
		wc->terminate = 1;		// wrapper: terminate
}


void LpelWorkerSelfTaskYield(lpel_task_t *t){
	workerctx_t *wc = t->worker_context;
	if (wc->wid < 0) {	//wrapper
			wc->current_task = t;
			PRT_DBG("wrapper: task %d yields\n", t->uid);
	}
	else {
		//sendUpdatePrior(t);		//update prior for neighbor
		requestTask(wc);
	}
}

void LpelWorkerTaskWakeup( lpel_task_t *t) {
	workerctx_t *wc = t->worker_context;
	PRT_DBG("worker %d: send wake up task %d\n", LpelWorkerSelf()->wid, t->uid);
	
	if (wc == NULL)
		sendWakeup(NULL, t);
	else {
		if (wc->wid < 0)
			sendWakeup(wc->mailbox, t);
		else
			sendWakeup(NULL, t);
	}
	// a task will be always send to the Master
	//sendWakeup(MASTER_PTR->mailbox, t);
}

void LpelWorkerTaskBlock(lpel_task_t *t) {
	workerctx_t *wc = t->worker_context;
	if (wc->wid >= 0) // worker
		requestTask(wc);
}

void LpelWorkerDispatcher( lpel_task_t *t) {
	workerctx_t *wc = t->worker_context;
	wc->current_task = NULL;
	//mctx_switch( &t->mctx, &wc->mctx);
	 DCMflush();
	(void)co_call(wc->mctx);
	 DCMflush();
}
