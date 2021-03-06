#include <stdlib.h>
#include <assert.h>
#include <stdio.h>
#include "arch/atomic.h"

#include "hrc_task.h"
#include "stream.h"
#include "lpelcfg.h"
#include "hrc_worker.h"
#include "lpel/monitor.h"
#include "taskpriority.h"
#include <readTileID.h>
#include <debugging.h>
#include <scc_comm_func.h>

// includes for the LUT mapping
#include <scc_config.h>
#include <stdarg.h>
#include <input.h>
#include <pcl.h>


//JUST FOR DEBUGGING PURPOSE

#include <ucontext.h>
typedef ucontext_t co_core_ctx_t;

//#include <setjmp.h>
//typedef jmp_buf co_core_ctx_t;

typedef struct s_co_ctx {
	co_core_ctx_t cc;
} co_ctx_t;


typedef struct s_coroutine {
	co_ctx_t ctx;
	int alloc;
	struct s_coroutine *caller;
	struct s_coroutine *restarget;
	void (*func)(void *);
	void *data;
} coroutine_temp;
//END DEBUGGING DEF'S

static atomic_t taskseq = ATOMIC_INIT(0);

static double (*prior_cal) (int in, int out) = priorfunc1;


lpel_task_t *LpelMasterTaskCreate( int map, lpel_taskfunc_t func,
                void *inarg, int size);

int countRec(stream_elem_t *list);

lpel_task_t *LpelTaskCreate( int map, lpel_taskfunc_t func,
		void *inarg, int size)
{

	lpel_task_t *t=NULL;

		if (SccGetNodeID()==MASTER){
			// node is MASTER NODE
			PRT_DBG("Node is MASTER --> create task!!!\n");
			t = LpelMasterTaskCreate(map, func, inarg, size);
		} else {
			PRT_DBG("Node is WORKER --> don't create task!!!\n");
		}
	return t;

}



/**
 * Create a task.
 *
 * @param worker  id of the worker where to create the task (0 = master, -1 = others, -2 = sosi)
 * @param func    task function
 * @param arg     arguments
 * @param size    size of the task, including execution stack
 * @param sc			scheduling condition, decide when a task should yield
 * @pre           size is a power of two, >= 4096
 *
 * @return the task handle of the created task (pointer to TCB)
 *
 */
lpel_task_t *LpelMasterTaskCreate( int map, lpel_taskfunc_t func,
		void *inarg, int size)
{
	lpel_task_t *t;
	char *stackaddr;
	int offset;

	if (size <= 0) {
		size = LPEL_TASK_SIZE_DEFAULT;
	}
	assert( size >= TASK_MINSIZE );

	/* aligned to page boundary */

// I use specific malloc to get the LUT entry position
//	t = valloc( size );

	// specific malloc
	//t=SCCMallocPtr(sizeof(lpel_task_t));
	t=SCCMallocPtr(size);
	// specific malloc for addr pointer 
	t->addr=SCCMallocPtr(sizeof(lut_addr_t));
	// assign LUT entry to the task
	*(t->addr)=SCCPtr2Addr(t);
	
/*		PRT_DBG("NEW TASK INFORMATION INSIDE CREATE:\n");
		PRT_DBG("LUT node: %d\n",t->addr->node);
		PRT_DBG("LUT Nr.:  %d\n",t->addr->lut);
		PRT_DBG("LUT offset: %u\n",t->addr->offset);
*/
	/* calc stackaddr */
	offset = (sizeof(lpel_task_t) + TASK_STACK_ALIGN-1) & ~(TASK_STACK_ALIGN-1);
	stackaddr = (char *) t + offset;
	t->size = size;
	PRT_DBG("t: 		%p\n",t);
	PRT_DBG("stackaddr: 	%p\n",stackaddr);
	PRT_DBG("t->addr: 	%p\n",t->addr);

	if (map == LPEL_MAP_OTHERS ){	/** others wrapper or source/sink */
		t->worker_context = LpelCreateWrapperContext(map);
		printf("--------------------WRAPPER-TASK-----------------");
	}	
	else
		t->worker_context = NULL;

	t->uid = fetch_and_inc( &taskseq);  /* obtain a unique task id */
	t->func = func;
	t->inarg = inarg;

	/* initialize poll token to 0 */
//	atomic_init( &t->poll_token, 0);

	t->state = TASK_CREATED;

	t->prev = t->next = NULL;

	t->mon = NULL;

	printf("\n\n!!!!!!!!!CO_CREATE!!!!!!!!!!\n");
	/* function, argument (data), stack base address, stacksize */
	//mctx_create( &t->mctx, TaskStartup, (void*)t, stackaddr, t->size - offset);
	// t->mctx=co_create(TaskStartup, (void*)t, stackaddr, t->size - offset);

	t->mctx=co_create(TaskStartup, (void*)t, NULL,8192);
	
	printf("t->mctx: %p\n",t->mctx);
	coroutine_temp *co = (coroutine_temp *) t->mctx;
	printf("co: %p\n",co);
	printf("co->ctx: %p\n",co->ctx);
#ifdef USE_MCTX_PCL
	assert(t->mctx != NULL);
#endif

	// default scheduling info
	t->sched_info = (sched_task_t *) SCCMallocPtr(sizeof(sched_task_t));
	t->sched_info->prior = 0;
	t->sched_info->rec_cnt = 0;
	t->sched_info->rec_limit = 1;
	t->sched_info->in_streams = NULL;
	t->sched_info->out_streams = NULL;

	return t;
}




/**
 * Destroy a task
 * - completely free the memory for that task
 */
void LpelTaskDestroy( lpel_task_t *t)
{
	assert( t->state == TASK_ZOMBIE);

#ifdef USE_TASK_EVENT_LOGGING
	/* if task had a monitoring object, destroy it */
	if (t->mon && MON_CB(task_destroy)) {
		MON_CB(task_destroy)(t->mon);
	}
#endif

	atomic_destroy( &t->poll_token);

	//FIXME
#ifdef USE_MCTX_PCL
	co_delete(t->mctx);
#endif


	assert(t->sched_info->in_streams == NULL);
	assert(t->sched_info->out_streams == NULL);
	
	//free(t->sched_info);
	SCCFreePtr(t->sched_info);
	/* free the TCB itself*/
	//free(t);
	SCCFreePtr(t);
}



/**
 * Unblock a task. Called from StreamRead/StreamWrite procedures
 */
void LpelTaskUnblock( lpel_task_t *by, lpel_task_t *t)
{
	assert(t != NULL);
	LpelWorkerTaskWakeup( t);
}




/*
 * this will be called after producing each record
 * increase the rec count by 1, if it reaches the limit then yield
 */
void LpelTaskCheckYield(lpel_task_t *t) {

	assert( t->state == TASK_RUNNING );

	if (t->sched_info->rec_limit < 0) {		//limit < 0 --> no yield
		return;
	}

	t->sched_info->rec_cnt ++;
	if (t->sched_info->rec_cnt >= t->sched_info->rec_limit) {
		t->state = TASK_READY;
		TaskStop( t);
		LpelWorkerSelfTaskYield(t);
		TaskStart( t);
	}
}

/*
 * set limit of produced records
 */
void LpelTaskSetRecLimit(lpel_task_t *t, int lim) {
	t->sched_info->rec_limit = lim;
}

void LpelTaskSetPriority(lpel_task_t *t, double p) {
	t->sched_info->prior = p;
}

/*
 * Add a tracked stream, used for calculate task priority
 * @param t			task
 * @param des		stream description
 * @param mode	read/write mode
 */
void LpelTaskAddStream( lpel_task_t *t, lpel_stream_desc_t *des, char mode) {
	stream_elem_t **list;
	stream_elem_t *head;
	switch (mode) {
	case 'r':
		list = &t->sched_info->in_streams;
		break;
	case 'w':
		list = &t->sched_info->out_streams;
		break;
	}
	head = *list;
	//stream_elem_t *new = (stream_elem_t *) malloc(sizeof(stream_elem_t));
	stream_elem_t *new = (stream_elem_t *) SCCMallocPtr(sizeof(stream_elem_t));
	new->stream_desc = des;
	if (head)
		new->next = head;
  else
		new->next = NULL;
	*list = new;
}

/*
 * Remove a tracked stream
 * @param t			task
 * @param des		stream description
 * @param mode	read/write mode
 */
void LpelTaskRemoveStream( lpel_task_t *t, lpel_stream_desc_t *des, char mode) {
	stream_elem_t **list;
	stream_elem_t *head;
	switch (mode) {
	case 'r':
		list = &t->sched_info->in_streams;
		break;
	case 'w':
		list = &t->sched_info->out_streams;
		break;
	}
	head = *list;

	stream_elem_t *prev = NULL;

	while (head != NULL) {
		if (head->stream_desc == des)
			break;
		prev = head;
		head = head->next;
	}

	assert(head != NULL);		//item must be in the list
	if (prev == NULL)
		*list = head->next;
	else
		prev->next = head->next;

	//free(head);
	SCCFreePtr(head);
}



/*
 * Calculate dynamic priority for task
 */
double LpelTaskCalPriority(lpel_task_t *t) {
	int in, out;
	in = countRec(t->sched_info->in_streams);
	out = countRec(t->sched_info->out_streams);
	return prior_cal(in, out);
//	return (in + 1.0)/((out + 1.0)*(in + out + 1.0));

}


/*
 * Set function used to calculate task priority
 */
void LpelTaskSetPriorityFunc(int func){
	switch (func){
	case 1: prior_cal = priorfunc1;
					break;
	case 2: prior_cal = priorfunc2;
					break;
	case 3: prior_cal = priorfunc3;
					break;
	case 4: prior_cal = priorfunc4;
					break;
	case 5: prior_cal = priorfunc5;
					break;
	case 6: prior_cal = priorfunc6;
					break;
	case 7: prior_cal = priorfunc7;
					break;
	case 8: prior_cal = priorfunc8;
					break;
	case 9: prior_cal = priorfunc9;
					break;
	case 10: prior_cal = priorfunc10;
					break;
	case 11: prior_cal = priorfunc11;
					break;
	case 12: prior_cal = priorfunc12;
					break;
	default: prior_cal = priorfunc1;
	}
}

/*
 * get WorkerId where task is currently running
 * 		used for debugging
 */
int LpelTaskGetWorkerId(lpel_task_t *t) {
	if (t->worker_context)
		return t->worker_context->wid;
	else
		return -1;
}


/******************************************************************************/
/* PRIVATE FUNCTIONS                                                          */
/******************************************************************************/
void TaskStart( lpel_task_t *t)
{
	assert( t->state == TASK_READY );

	/* MONITORING CALLBACK */
#ifdef USE_TASK_EVENT_LOGGING
//	if (t->mon && MON_CB(task_start)) {
//		MON_CB(task_start)(t->mon);
//	}
#endif
	t->sched_info->rec_cnt = 0;	// reset rec_cnt
	t->state = TASK_RUNNING;
}

/*
 * count records in the list of tracked stream
 */
int countRec(stream_elem_t *list) {
	if (list == NULL)
		return -1;
	int cnt = 0;
	while (list != NULL) {
		cnt += LpelStreamFillLevel(list->stream_desc->stream);
		list = list->next;
	}
	return cnt;
}


