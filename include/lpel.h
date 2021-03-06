/******************************************************************************
 * LPEL LIBRARY INTERFACE
 *
 * Author: Daniel Prokesch <dlp@snet-home.org>
 * Modified: Nga
 *
 * Common interface for DECEN and HRC
 *****************************************************************************/
#ifndef _LPEL_H_
#define _LPEL_H_

/******************************************************************************/
/* LPEL MAPPING LOCATION 													  */
/******************************************************************************/
#define LPEL_MAP_OTHERS		-1
#define LPEL_MAP_MASTER		0
#define LPEL_MAP_WORKER		0

/******************************************************************************/
/*  RETURN VALUES OF LPEL FUNCTIONS                                           */
/******************************************************************************/

#define LPEL_ERR_SUCCESS     0
#define LPEL_ERR_FAIL       -1

#define LPEL_ERR_INVAL       1 /* Invalid argument */
#define LPEL_ERR_ASSIGN      2 /* Cannot assign thread to processors */
#define LPEL_ERR_EXCL        3 /* Cannot assign core exclusively */


/******************************************************************************/
/*  LPEL TASK STATE									                                          */
/******************************************************************************/

typedef char lpel_taskstate_t;

#define  TASK_CREATED             'C'
#define  TASK_RUNNING             'U'
#define  TASK_READY               'R'
#define  TASK_BLOCKED             'B'
#define  TASK_MUTEX               'X'
#define  TASK_ZOMBIE              'Z'

/******************************************************************************/
/*  LPEL FLAGS											                                          */
/******************************************************************************/
#define LPEL_FLAG_NONE           (0)
#define LPEL_FLAG_PINNED      (1<<0)
#define LPEL_FLAG_EXCLUSIVE   (1<<1)

/******************************************************************************/
/*  GENERAL CONFIGURATION AND SETUP                                           */
/******************************************************************************/

typedef struct mon_worker_t mon_worker_t;
typedef struct mon_task_t   mon_task_t;
typedef struct mon_stream_t mon_stream_t;

typedef struct lpel_monitoring_cb_t {
  /* worker callbacks*/
  mon_worker_t *(*worker_create)(int);
  mon_worker_t *(*worker_create_wrapper)(mon_task_t *);
  void (*worker_destroy)(mon_worker_t*);
  void (*worker_waitstart)(mon_worker_t*);
  void (*worker_waitstop)(mon_worker_t*);
  //void (*worker_debug)(mon_worker_t*, const char *fmt, ...);
  /* task callbacks */
  /* note: no callback for task creation
   * - manual attachment required
   */
  void (*task_destroy)(mon_task_t*);
  void (*task_assign)(mon_task_t*, mon_worker_t*);
  void (*task_start)(mon_task_t*);
  void (*task_stop)(mon_task_t*, lpel_taskstate_t);
  /* stream callbacks */
  mon_stream_t *(*stream_open)(mon_task_t*, unsigned int, char);
  void (*stream_close)(mon_stream_t*);
  void (*stream_replace)(mon_stream_t*, unsigned int);
  void (*stream_readprepare)(mon_stream_t*);
  void (*stream_readfinish)(mon_stream_t*, void*);
  void (*stream_writeprepare)(mon_stream_t*, void*);
  void (*stream_writefinish)(mon_stream_t*);
  void (*stream_blockon)(mon_stream_t*);
  void (*stream_wakeup)(mon_stream_t*);
} lpel_monitoring_cb_t;


typedef enum {
	DECEN_LPEL,
	HRC_LPEL
} lpel_backend_type;

/**
 * Specification for configuration:
 *
 * num_workers defines the number of worker threads (PThreads) spawned.
 * proc_workers is the number of processors used for workers.
 * proc_others is the number of processors assigned to other than
 *   worker threads.
 * flags:
 *   AUTO - use default setting for num_workers, proc_workers, proc_others
 *   REALTIME - set realtime priority for workers, will succeed only if
 *              there is a 1:1 mapping of workers to procs,
 *              proc_others > 0 and the process has needed privileges.
 */
typedef struct {
  int num_workers;
  int proc_workers;
  int proc_others;
  int flags;
  lpel_monitoring_cb_t mon;
  lpel_backend_type type;
} lpel_config_t;



int LpelInit( lpel_config_t *cfg);
void LpelCleanup( void);

void LpelStart(void);
void LpelStop(void);

int LpelGetNumCores( int *result);
int LpelCanSetExclusive( int *result);



/******************************************************************************/
/*  DATATYPES                                                                 */
/******************************************************************************/


/* a task */
typedef struct lpel_task_t lpel_task_t;

/* task function signature */
typedef void *(*lpel_taskfunc_t)(void *inarg);


/* stream type */
typedef struct lpel_stream_t         lpel_stream_t;

/**
 * A stream descriptor
 *
 * A producer/consumer must open a stream before using it, and by opening
 * a stream, a stream descriptor is created and returned.
 */
typedef struct lpel_stream_desc_t    lpel_stream_desc_t;

/** stream set */
typedef lpel_stream_desc_t          *lpel_streamset_t;

/** iterator for streamset */
typedef struct lpel_stream_iter_t    lpel_stream_iter_t;



/******************************************************************************/
/*  TASK FUNCTIONS                                                            */
/******************************************************************************/

lpel_task_t *LpelTaskCreate( int worker, lpel_taskfunc_t func,
    void *inarg, int stacksize );

/** monitor a task */
void LpelTaskMonitor(lpel_task_t *t, mon_task_t *mt);

unsigned int LpelTaskGetId( lpel_task_t *t );
mon_task_t *LpelTaskGetMon( lpel_task_t *t );

/** let the previously created task run */
void LpelTaskStart( lpel_task_t *t );


/** to be called from within a task: */
lpel_task_t *LpelTaskSelf(void);
void LpelTaskExit(void *outarg);
void LpelTaskYield(void);


/******************************************************************************/
/*  STREAM FUNCTIONS                                                          */
/******************************************************************************/

lpel_stream_t *LpelStreamCreate( int);
void LpelStreamDestroy( lpel_stream_t *s);

void LpelStreamSetUsrData( lpel_stream_t *s, void *usr_data);
void *LpelStreamGetUsrData( lpel_stream_t *s);

lpel_stream_desc_t *
LpelStreamOpen( lpel_stream_t *s, char mode);

void  LpelStreamClose(    lpel_stream_desc_t *sd, int destroy_s);
void  LpelStreamReplace(  lpel_stream_desc_t *sd, lpel_stream_t *snew);
void *LpelStreamPeek(     lpel_stream_desc_t *sd);
void *LpelStreamRead(     lpel_stream_desc_t *sd);
void  LpelStreamWrite(    lpel_stream_desc_t *sd, void *item);
int   LpelStreamTryWrite( lpel_stream_desc_t *sd, void *item);

lpel_stream_t *LpelStreamGet(lpel_stream_desc_t *sd);
int LpelStreamGetId(lpel_stream_desc_t *sd);


/** stream set functions*/

lpel_stream_desc_t *LpelStreamPoll(    lpel_streamset_t *set);



void LpelStreamsetPut(  lpel_streamset_t *set, lpel_stream_desc_t *node);
int  LpelStreamsetRemove( lpel_streamset_t *set, lpel_stream_desc_t *node);
int  LpelStreamsetIsEmpty( lpel_streamset_t *set);


/** stream iterator functions */


lpel_stream_iter_t *LpelStreamIterCreate( lpel_streamset_t *set);
void LpelStreamIterDestroy( lpel_stream_iter_t *iter);

void LpelStreamIterReset(  lpel_stream_iter_t *iter, lpel_streamset_t *set);
int  LpelStreamIterHasNext( lpel_stream_iter_t *iter);
lpel_stream_desc_t *LpelStreamIterNext( lpel_stream_iter_t *iter);
void LpelStreamIterAppend(  lpel_stream_iter_t *iter, lpel_stream_desc_t *node);
void LpelStreamIterRemove(  lpel_stream_iter_t *iter);


#endif /* _LPEL_H_ */
