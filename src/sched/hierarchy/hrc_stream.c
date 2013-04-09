#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <pthread.h>

#include <lpel.h>

#include "arch/atomic.h"
#include "lpelcfg.h"
#include "buffer.h"
#include "task.h"
#include "hrc_task.h"

#include "stream.h"
#include "lpel/monitor.h"

// includes for the LUT mapping
#include "include_lut/config.h"
#include "include_lut/RCCE_memcpy.c"
#include "include_lut/distribution.h"
#include "include_lut/scc.h"
#include "include_lut/sccmalloc.h"
#include <stdarg.h>



/**
 * Create a stream
 *
 * Allocate and initialize memory for a stream.
 *
 * @return pointer to the created stream
 */
lpel_stream_t *LpelStreamCreate(int size)
{
  assert( size >= 0);
  if (0==size) size = STREAM_BUFFER_SIZE;

  /* allocate memory for both the stream struct and the buffer area */
  // I use specific malloc to get the LUT entry position
  //lpel_stream_t *s = (lpel_stream_t *) malloc( sizeof(lpel_stream_t) );

  // specific malloc
   lpel_stream_t *s=(lpel_stream_t *) SCCMallocPtr(sizeof(lpel_stream_t));
  // assign LUT entry to the task
  	s->addr = SCCPtr2Addr(s);



  /* reset buffer (including buffer area) */
  s->buffer = LpelBufferInit( size);

  s->uid = fetch_and_inc( &stream_seq);
  PRODLOCK_INIT( &s->prod_lock );
  atomic_init( &s->n_sem, 0);
  atomic_init( &s->e_sem, size);
  s->is_poll = 0;
  s->prod_sd = NULL;
  s->cons_sd = NULL;
  s->usr_data = NULL;
  return s;
}


/**
 * Blocking write to a stream
 *
 * If the stream is full, the task is suspended until the consumer
 * reads items from the stream, freeing space for more items.
 *
 * @param sd    stream descriptor
 * @param item  data item (a pointer) to write
 * @pre         current task is single writer
 * @pre         item != NULL
 */
void LpelStreamWrite( lpel_stream_desc_t *sd, void *item)
{
  lpel_task_t *self = sd->task;
  int poll_wakeup = 0;

  /* check if opened for writing */
  assert( sd->mode == 'w' );
  assert( item != NULL );

  /* MONITORING CALLBACK */
#ifdef USE_TASK_EVENT_LOGGING
  if (sd->mon && MON_CB(stream_writeprepare)) {
    MON_CB(stream_writeprepare)(sd->mon, item);
  }
#endif


  /* writing to the buffer and checking if consumer polls must be atomic */
  PRODLOCK_LOCK( &sd->stream->prod_lock);
  {
    /* there must be space now in buffer */
    assert( LpelBufferIsSpace( sd->stream->buffer) );
    /* put item into buffer */
    LpelBufferPut( sd->stream->buffer, item);

    if ( sd->stream->is_poll) {
      /* get consumer's poll token */
      poll_wakeup = atomic_swap( &sd->stream->cons_sd->task->poll_token, 0);
      sd->stream->is_poll = 0;
    }
  }
  PRODLOCK_UNLOCK( &sd->stream->prod_lock);



  /* quasi V(n_sem) */
  if ( fetch_and_inc( &sd->stream->n_sem) < 0) {
    /* n_sem was -1 */
    lpel_task_t *cons = sd->stream->cons_sd->task;
    /* wakeup consumer: make ready */
    LpelTaskUnblock( self, cons);

    /* MONITORING CALLBACK */
#ifdef USE_TASK_EVENT_LOGGING
    if (sd->mon && MON_CB(stream_wakeup)) {
      MON_CB(stream_wakeup)(sd->mon);
    }
#endif
  } else {
    /* we are the sole producer task waking the polling consumer up */
    if (poll_wakeup) {
      lpel_task_t *cons = sd->stream->cons_sd->task;
      cons->wakeup_sd = sd->stream->cons_sd;

      LpelTaskUnblock( self, cons);

      /* MONITORING CALLBACK */
#ifdef USE_TASK_EVENT_LOGGING
      if (sd->mon && MON_CB(stream_wakeup)) {
        MON_CB(stream_wakeup)(sd->mon);
      }
#endif
    }
  }

  /* MONITORING CALLBACK */
#ifdef USE_TASK_EVENT_LOGGING
  if (sd->mon && MON_CB(stream_writefinish)) {
    MON_CB(stream_writefinish)(sd->mon);
  }
#endif

  /* check if a task should be yield after writing this output record */
  LpelTaskCheckYield(self);

}



/**
 * Blocking, consuming read from a stream
 *
 * If the stream is empty, the task is suspended until
 * a producer writes an item to the stream.
 *
 * @param sd  stream descriptor
 * @return    the next item of the stream
 * @pre       current task is single reader
 */
void *LpelStreamRead( lpel_stream_desc_t *sd)
{
  void *item;
  lpel_task_t *self = sd->task;

  assert( sd->mode == 'r');

  /* MONITORING CALLBACK */
#ifdef USE_TASK_EVENT_LOGGING
  if (sd->mon && MON_CB(stream_readprepare)) {
    MON_CB(stream_readprepare)(sd->mon);
  }
#endif

  /* quasi P(n_sem) */
  if ( fetch_and_dec( &sd->stream->n_sem) == 0) {

#ifdef USE_TASK_EVENT_LOGGING
    /* MONITORING CALLBACK */
    if (sd->mon && MON_CB(stream_blockon)) {
      MON_CB(stream_blockon)(sd->mon);
    }
#endif

    /* wait on stream: */
    LpelTaskBlockStream( self);
  }


  /* read the top element */
  item = LpelBufferTop( sd->stream->buffer);
  assert( item != NULL);
  /* pop off the top element */
  LpelBufferPop( sd->stream->buffer);


  /* MONITORING CALLBACK */
#ifdef USE_TASK_EVENT_LOGGING
  if (sd->mon && MON_CB(stream_readfinish)) {
    MON_CB(stream_readfinish)(sd->mon, item);
  }
#endif
  return item;
}
