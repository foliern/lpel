/**
 * File: stream.c
 * Auth: Daniel Prokesch <daniel.prokesch@gmail.com>
 * Modified: Nga
 *
 * Desc: Common implementations for DECEN and HRC
 *
 * Core stream handling functions, including stream descriptors.
 *
 * A stream is the communication and synchronization primitive between two
 * tasks. If a task wants to use a stream, it must open it in order to
 * retrieve a stream descriptor. A task can open it for reading ('r',
 * a consumer) or writing ('w', a producer) but not both, meaning that
 * streams are uni-directional. A single stream can be opened by at most one
 * producer task and by at most one consumer task simultaneously.
 *
 * After opening, a consumer can read from a stream and a producer can write
 * to a stream, using the retrieved stream descriptor. Note that only the
 * streams are shared, not the stream descriptors.
 *
 * Within a stream a buffer struct is holding the actual data, which is
 * implemented as circular single-producer single-consumer in a concurrent,
 * lock-free manner.
 *
 * For synchronization between tasks, three blocking functions are provided:
 * LpelStreamRead() suspends the consumer trying to read from an empty stream,
 * LpelStreamWrite() suspends the producer trying to write to a full stream,
 * and a consumer can use LpelStreamPoll() to wait for the arrival of data
 * on any of the streams specified in a set.
 *
 *
 * @see http://www.cs.colorado.edu/department/publications/reports/docs/CU-CS-1023-07.pdf
 *      accessed Aug 26, 2010
 *      for more details on the FastForward queue.
 *
 *
 */

#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <pthread.h>

#include <lpel.h>

#include "arch/atomic.h"
#include "lpelcfg.h"
#include "buffer.h"
#include "task.h"

#include "stream.h"
#include "lpel/monitor.h"
#include <sccmalloc.h>

static atomic_t stream_seq = ATOMIC_INIT(0);


/**
 * Destroy a stream
 *
 * Free the memory allocated for a stream.
 *
 * @param s   stream to be freed
 * @pre       stream must not be opened by any task!
 */
void LpelStreamDestroy( lpel_stream_t *s)
{
  PRODLOCK_DESTROY( &s->prod_lock);
  atomic_destroy( &s->n_sem);
  atomic_destroy( &s->e_sem);
  LpelBufferCleanup( s->buffer);
  
  //free( s);
	SCCFreePtr(s);
}


/**
 * Store arbitrary user data in stream
 * CAUTION use at own risk
 */
void LpelStreamSetUsrData(lpel_stream_t *s, void *usr_data)
{
  s->usr_data = usr_data;
}

/**
 * Load user data from stream
 * CAUTION use at own risk
 */
void *LpelStreamGetUsrData(lpel_stream_t *s)
{
  return s->usr_data;
}


/**
  * Open a stream for reading/writing
 *
 * @param s     pointer to stream
 * @param mode  either 'r' for reading or 'w' for writing
 * @return      a stream descriptor
 * @pre         only one task may open it for reading resp. writing
 *              at any given point in time
 */
lpel_stream_desc_t *LpelStreamOpen( lpel_stream_t *s, char mode)
{
  lpel_stream_desc_t *sd;
  lpel_task_t *ct = LpelTaskSelf();

  assert( mode == 'r' || mode == 'w' );
//  sd = (lpel_stream_desc_t *) malloc( sizeof( lpel_stream_desc_t));
	sd = (lpel_stream_desc_t *)SCCMallocPtr( sizeof( lpel_stream_desc_t));
  sd->task = ct;
  sd->stream = s;
  sd->mode = mode;
  sd->next  = NULL;

#ifdef USE_TASK_EVENT_LOGGING
  /* create monitoring object, or NULL if stream
   * is not going to be monitored (depends on ct->mon)
   */
  if (ct->mon && MON_CB(stream_open)) {
    sd->mon = MON_CB(stream_open)( ct->mon, s->uid, mode);
  } else {
    sd->mon = NULL;
  }
#else
  sd->mon = NULL;
#endif

  switch(mode) {
    case 'r': s->cons_sd = sd; break;
    case 'w': s->prod_sd = sd; break;
  }

  /* add stream desc to the task, used for calculate dynamic task priority in HRC
   * This function has no effect in DECEN
   * It is implemented this way to avoid 2 implementations: one for HRC and one for DECEN
   */
  LpelTaskAddStream(ct, sd, mode);
  return sd;
}

/**
 * Close a stream previously opened for reading/writing
 *
 * @param sd          stream descriptor
 * @param destroy_s   if != 0, destroy the stream as well
 */
void LpelStreamClose( lpel_stream_desc_t *sd, int destroy_s)
{
  /* MONITORING CALLBACK */
#ifdef USE_TASK_EVENT_LOGGING
  if (sd->mon && MON_CB(stream_close)) {
    MON_CB(stream_close)(sd->mon);
  }
#endif

  if (sd->mode == 'r')
  	sd->stream->cons_sd = NULL;
  else if (sd->mode == 'w')
  	sd->stream->prod_sd = NULL;

  /* add stream desc to the task, used for calculate dynamic task priority in HRC
   * This function has no effect in DECEN
   * It is implemented this way to avoid 2 implementations: one for HRC and one for DECEN
   */
  LpelTaskRemoveStream(sd->task, sd, sd->mode);

  if (destroy_s) {
    LpelStreamDestroy( sd->stream);
  }
  //free(sd);
	SCCFreePtr(sd);
}


/**
 * Replace a stream opened for reading by another stream
 * Destroys old stream.
 *
 * @param sd    stream descriptor for which the stream must be replaced
 * @param snew  the new stream
 * @pre         snew must not be opened by same or other task
 */
void LpelStreamReplace( lpel_stream_desc_t *sd, lpel_stream_t *snew)
{
  assert( sd->mode == 'r');
  /* destroy old stream */
  LpelStreamDestroy( sd->stream);
  /* assign new stream */
  sd->stream = snew;
  /* new consumer sd of stream */
  sd->stream->cons_sd = sd;


  /* MONITORING CALLBACK */
#ifdef USE_TASK_EVENT_LOGGING
  if (sd->mon && MON_CB(stream_replace)) {
    MON_CB(stream_replace)(sd->mon, snew->uid);
  }
#endif

}


/**
 * Get the stream opened by a stream descriptor
 *
 * @param sd  the stream descriptor
 * @return    the stream opened by the stream descriptor
 */
lpel_stream_t *LpelStreamGet(lpel_stream_desc_t *sd)
{
  return sd->stream;
}



/**
 * Non-blocking, non-consuming read from a stream
 *
 * @param sd  stream descriptor
 * @return    the top item of the stream, or NULL if stream is empty
 */
void *LpelStreamPeek( lpel_stream_desc_t *sd)
{
  assert( sd->mode == 'r');
  return LpelBufferTop( sd->stream->buffer);
}


/**
 * Non-blocking write to a stream
 *
 * @param sd    stream descriptor
 * @param item  data item (a pointer) to write
 * @pre         current task is single writer
 * @pre         item != NULL
 * @return 0 if the item could be written, -1 if the stream was full
 */
int LpelStreamTryWrite( lpel_stream_desc_t *sd, void *item)
{
  if (!LpelBufferIsSpace(sd->stream->buffer)) {
    return -1;
  }
  LpelStreamWrite( sd, item );
  return 0;
}

/**
 * Poll a set of streams
 *
 * This is a blocking function called by a consumer which wants to wait
 * for arrival of data on any of a specified set of streams.
 * The consumer task is suspended while there is no new data on all streams.
 *
 * @param set     a stream descriptor set the task wants to poll
 * @pre           set must not be empty (*set != NULL)
 *
 * @post          The first element when iterating through the set after
 *                LpelStreamPoll() will be the one after the one which
 *                caused the task to wakeup,
 *                i.e., the first stream where data arrived.
 */
lpel_stream_desc_t *LpelStreamPoll( lpel_streamset_t *set)
{
  lpel_task_t *self;
  lpel_stream_iter_t *iter;
  int do_ctx_switch = 1;
  int cnt = 0;

  assert( *set != NULL);

  /* get 'self', i.e. the task calling LpelStreamPoll() */
  self = (*set)->task;

  iter = LpelStreamIterCreate( set);

  /* fast path*/
  while( LpelStreamIterHasNext( iter)) {
    lpel_stream_desc_t *sd = LpelStreamIterNext( iter);
    lpel_stream_t *s = sd->stream;
    if ( LpelBufferTop( s->buffer) != NULL) {
      LpelStreamIterDestroy(iter);
      *set = sd;
      return sd;
    }
  }


  /* place a poll token */
  atomic_set( &self->poll_token, 1);

  /* for each stream in the set */
  LpelStreamIterReset(iter, set);
  while( LpelStreamIterHasNext( iter)) {
    lpel_stream_desc_t *sd = LpelStreamIterNext( iter);
    lpel_stream_t *s = sd->stream;
    /* lock stream (prod-side) */
    PRODLOCK_LOCK( &s->prod_lock);
    { /* CS BEGIN */
      /* check if there is something in the buffer */
      if ( LpelBufferTop( s->buffer) != NULL) {
        /* yes, we can stop iterating through streams.
         * determine, if we have been woken up by another producer:
         */
        int tok = atomic_swap( &self->poll_token, 0);
        if (tok) {
          /* we have not been woken yet, no need for ctx switch */
          do_ctx_switch = 0;
          self->wakeup_sd = sd;
        }
        /* unlock stream */
        PRODLOCK_UNLOCK( &s->prod_lock);
        /* exit loop */
        break;

      } else {
        /* nothing in the buffer, register stream as activator */
        s->is_poll = 1;
        cnt++;
        //sd->event_flags |= STDESC_WAITON;
        /* TODO marking all streams does potentially flood the log-files
           - is it desired to have anyway?
        MarkDirty( sd);
        */
      }
    } /* CS END */
    /* unlock stream */
    PRODLOCK_UNLOCK( &s->prod_lock);
  } /* end for each stream */

  /* context switch */
  if (do_ctx_switch) {
    /* set task as blocked */
    LpelTaskBlockStream( self);
  }
  assert( atomic_read( &self->poll_token) == 0);

  /* unregister activators
   * - would only be necessary, if the consumer task closes the stream
   *   while the producer is in an is_poll state,
   *   as this could result in a SEGFAULT when the producer
   *   is trying to dereference sd->stream->cons_sd
   * - a consumer closes the stream if it reads
   *   a terminate record or a sync record, and between reading the record
   *   and closing the stream the consumer issues no LpelStreamPoll()
   *   and no entity writes a record on the stream after these records.
   * UPDATE: with static/dynamc collectors in S-Net, this is possible!
   */
  LpelStreamIterReset(iter, set);
  while( LpelStreamIterHasNext( iter)) {
    lpel_stream_t *s = (LpelStreamIterNext(iter))->stream;
    PRODLOCK_LOCK( &s->prod_lock);
    s->is_poll = 0;
    PRODLOCK_UNLOCK( &s->prod_lock);
    if (--cnt == 0) break;
  }

  LpelStreamIterDestroy(iter);

  /* 'rotate' set to stream descriptor for non-empty buffer */
  *set = self->wakeup_sd;

  return self->wakeup_sd;
}

int LpelStreamGetId(lpel_stream_desc_t *sd) {
	if (sd)
		if (sd->stream)
			return sd->stream->uid;
	return -1;
}

int LpelStreamFillLevel(lpel_stream_t *s) {
	int n;
	PRODLOCK_LOCK( &s->prod_lock);
	n = LpelBufferCount(s->buffer);
	PRODLOCK_UNLOCK( &s->prod_lock);
	return n;
}

lpel_task_t *LpelStreamConsumer(lpel_stream_t *s) {
	if (s->cons_sd != NULL)
		return s->cons_sd->task;
	else
		return NULL;
}

lpel_task_t *LpelStreamProducer(lpel_stream_t *s) {
	lpel_task_t *t;
	PRODLOCK_LOCK( &s->prod_lock);
	if (s->prod_sd != NULL)
		t = s->prod_sd->task;
	else
		t = NULL;
	PRODLOCK_UNLOCK( &s->prod_lock);
	return t;
}

