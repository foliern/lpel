#ifndef _MAILBOX_H_
#define _MAILBOX_H_

#include "SCC_API_test.h"
#include "workermsg.h"

typedef struct mailbox_t mailbox_t;

/* mailbox structures */


//char *bsp[i] ist ein Array von i Elementen des Typs Zeiger auf char

typedef struct  {
	t_vcharp writing_flag[DLPEL_ACTIVE_NODES];
	t_vcharp reading_flag[DLPEL_ACTIVE_NODES];
	t_vcharp msg_type[DLPEL_ACTIVE_NODES];
	t_vcharp start_pointer[DLPEL_ACTIVE_NODES];
	t_vcharp end_pointer[DLPEL_ACTIVE_NODES];

} master_mailbox_t;

typedef struct {
	t_vcharp writing_flag;
	t_vcharp reading_flag;
	t_vcharp msg_type;
	t_vcharp start_pointer;
	t_vcharp end_pointer;

} worker_mailbox_t;

void LpelMailboxCreate(int Node_ID);
//void LpelMailboxDestroy(mailbox_t *mbox);
//void LpelMailboxSend(mailbox_t *mbox, workermsg_t *msg);
//void LpelMailboxRecv(mailbox_t *mbox, workermsg_t *msg);
//int  LpelMailboxHasIncoming(mailbox_t *mbox);

void LpelMailboxSend_overMPB(char *privbuf, size_t size, int dest);
void LpelMailboxRecv_overMPB(t_vcharp privbuf, size_t size, int source);  



#endif /* _MAILBOX_H_ */
