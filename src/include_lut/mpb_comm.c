
#include <stdlib.h>
#include <pthread.h>
#include <assert.h>
#include "mailbox.h"
#include "mpb_comm.h"

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




/******************************************************************************/
/* Public functions                                                           */
/******************************************************************************/


void MPBCommunicationCreate(void)
{



}



void MPBCommunicationDestroy(void)
{

}



