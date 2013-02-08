#include <fcntl.h>
#include <stdlib.h>
#include <pthread.h>
#include <assert.h>
#include <stdio.h>

// for communication over MPB
#include <sys/mman.h>
#include <string.h>
#include <unistd.h>
//to use uint64_t
#include <stdint.h>
#include "SCC_API_test.h"
#include "readTileID.h"
#include "configuration.h"
#include "lpel.h"
#include "mailbox.h"
#include "mpb.h"


#define CORES               (NUM_ROWS * NUM_COLS * NUM_CORES)
#define PAGE_SIZE           (16*1024*1024)
#define LINUX_PRIV_PAGES    (20)
#define PAGES_PER_CORE      (41)
#define MAX_PAGES           (172)
#define IRQ_BIT             (0x01 << GLCFG_XINTR_BIT)

#define B_OFFSET            64
#define FOOL_WRITE_COMBINE  (mpbs[node_location][0] = 1)
#define START(i)            (*((volatile uint16_t *) (mpbs[i] + B_OFFSET)))
#define END(i)              (*((volatile uint16_t *) (mpbs[i] + B_OFFSET + 2)))
#define HANDLING(i)         (*(mpbs[i] + B_OFFSET + 4))
#define WRITING(i)          (*(mpbs[i] + B_OFFSET + 5))
#define B_START             (B_OFFSET + 32)
#define B_SIZE              (MPBSIZE - B_START)
#define true				1
#define false				0
#define LUT(loc, idx)       (*((volatile uint32_t*)(&luts[loc][idx])))
//because from 0 to < 48


//......................................................................................
// GLOBAL VARIABLES USED FOR MPB
//......................................................................................


//int       SCC_COREID[RCCE_MAXNP]; // array of physical core IDs for all participating cores, sorted

t_vcharp  SCC_MESSAGE_PASSING_BUFFER[SCC_NR_NODES]; // starts of MPB, sorted by rank
static int       node_ID=-1;           // rank of calling core (invalid by default)
static int	MASTER=FALSE;
// Variables
int NCMDeviceFD; // File descriptor for non-cachable memory (e.g. config regs).
int MPBDeviceFD; // File descriptor for message passing buffers.


// payload part of the MPBs starts at a specific address, not malloced space
//t_vcharp RCCE_buff_ptr;
t_vcharp my_mpb_ptr;
// maximum chunk size of message payload is also specified

//......................................................................................
// END GLOBAL VARIABLES USED FOR MPB
//......................................................................................



//extern bool remap;
int node_location;
t_vcharp mpbs[CORES];
t_vcharp locks[CORES];
//extern volatile int *irq_pins[CORES];
//extern volatile uint64_t *luts[CORES];

static inline int min(int x, int y) { return x < y ? x : y; }

/* Flush MPBT from L1. */
static inline void flush() { __asm__ volatile ( ".byte 0x0f; .byte 0x0a;\n" ); }

static inline void lock(int core) { while (!(*locks[core] & 0x01)); }

static inline void unlock(int core) { *locks[core] = 0; }

master_mailbox_t master_mbox;
worker_mailbox_t worker_mbox;





/******************************************************************************/
/* Free node pool management functions                                        */
/******************************************************************************/

void setReadFlag(int dest){
	if (MASTER)
			MPB_write(master_mbox.writing_flag[dest], (t_vcharp) "1", FLAG_SIZE);
		else
			MPB_write(worker_mbox.writing_flag, (t_vcharp) "1", FLAG_SIZE);

}

void resetReadFlag(int dest){
	if (MASTER)
			MPB_write(master_mbox.writing_flag[dest], (t_vcharp) "0", FLAG_SIZE);
		else
			MPB_write(worker_mbox.writing_flag, (t_vcharp) "0", FLAG_SIZE);

}

void setWriteFlag(int dest){
	if (MASTER)
				MPB_write(master_mbox.reading_flag[dest], (t_vcharp) "1", FLAG_SIZE);
			else
				MPB_write(worker_mbox.reading_flag, (t_vcharp) "1", FLAG_SIZE);
}


void resetWriteFlag(int dest){
	if (MASTER)
				MPB_write(master_mbox.reading_flag[dest], (t_vcharp) "0", FLAG_SIZE);
			else
				MPB_write(worker_mbox.reading_flag, (t_vcharp) "0", FLAG_SIZE);
}




/******************************************************************************/
/* Public functions                                                           */
/******************************************************************************/


void LpelMailboxCreate(void)
{

	int offset;


	// Open driver device "/dev/rckncm" for memory mapped register access
	// or access to other non cachable memory locations...
	if ((NCMDeviceFD=open("/dev/rckncm", O_RDWR|O_SYNC))<0) {
		perror("open");
		exit(-1);
	}

	// Open driver device "/dev/rckmpb" for message passing buffer access...
	if ((MPBDeviceFD=open("/dev/rckmpb", O_RDWR))<0) {
		perror("open");
	    exit(-1);
	}

    // Success message
	PRT_DBG("Successfully opened RCKMEM driver devices!\n");

	node_ID=readTileID();

	if (node_ID == SCC_MASTER_NODE)		//create MASTER Mailbox
	{
		MASTER=TRUE;


		// initialize MPB starting addresses for all participating cores; allow one
		// dummy cache line at front of MPB for fooling write combine buffer in case
		// of single-byte MPB access
		//RCCE_fool_write_combine_buffer = RC_COMM_BUFFER_START(RCCE_IAM);
		for (int ue=0; ue < DLPEL_ACTIVE_NODES; ue++){
		//SCC_COREID[ue]=ue;
			if ((ue % 2) == 1){
//				PRT_DBG("Note %d is ungerade \n", ue);
                	        SCC_MESSAGE_PASSING_BUFFER[ue] = MPB_comm_buffer_start(ue) + MPB_BUFF_SIZE;
		        }else{
                        	SCC_MESSAGE_PASSING_BUFFER[ue] = MPB_comm_buffer_start(ue);
			}
			master_mbox.start_pointer[ue]= SCC_MESSAGE_PASSING_BUFFER[ue]	+MPB_BUFFER_OFFSET;
//			PRT_DBG("ADRESSE Node %d: %x \n",ue, master_mbox.start_pointer[ue]);
			master_mbox.end_pointer[ue]= SCC_MESSAGE_PASSING_BUFFER[ue]		+MPB_BUFFER_OFFSET;
			master_mbox.writing_flag[ue]= SCC_MESSAGE_PASSING_BUFFER[ue]	+WRITING_FLAG_OFFSET;
			master_mbox.reading_flag[ue]= SCC_MESSAGE_PASSING_BUFFER[ue]	+READING_FLAG_OFFSET;
			master_mbox.msg_type[ue]= SCC_MESSAGE_PASSING_BUFFER[ue]		+MSG_TYPE_OFFSET;

		}

	} else								//create WORKER Mailbox
		if ((NODE_ID%2) == 1)
			SCC_MESSAGE_PASSING_BUFFER[NODE_ID] = MPB_comm_buffer_start(NODE_ID) + MPB_BUFF_SIZE;
		else
			SCC_MESSAGE_PASSING_BUFFER[NODE_ID] = MPB_comm_buffer_start(NODE_ID);

		worker_mbox.start_pointer= SCC_MESSAGE_PASSING_BUFFER[NODE_ID]	+MPB_BUFFER_OFFSET;
//		PRT_DBG("ADRESSE Node %d: %x \n",NODE_ID, worker_mbox.start_pointer);
		worker_mbox.end_pointer= SCC_MESSAGE_PASSING_BUFFER[NODE_ID]	+MPB_BUFFER_OFFSET;
		worker_mbox.writing_flag= SCC_MESSAGE_PASSING_BUFFER[NODE_ID]	+WRITING_FLAG_OFFSET;
		worker_mbox.reading_flag= SCC_MESSAGE_PASSING_BUFFER[NODE_ID]	+READING_FLAG_OFFSET;
		worker_mbox.msg_type= SCC_MESSAGE_PASSING_BUFFER[NODE_ID]		+MSG_TYPE_OFFSET;

}


void LpelMailboxSend_overMPB(
		char *privbuf,    // source buffer in local private memory (send buffer)
		size_t size,      // size of message (bytes)
		int dest          // UE that will receive the message
	)
{
	setReadFlag(dest);
	setWriteFlag(dest);

	if (MASTER)
	//	for (int i; i<size;i++)
	//		MPB_write(master_mbox.start_pointer[dest]+i, (t_vcharp) privbuf+i, size);
		MPB_write(master_mbox.start_pointer[dest], (t_vcharp) privbuf, size);
	else
		MPB_write(worker_mbox.start_pointer, (t_vcharp) privbuf, size);
			
}



inline void cpy_mpb_to_mem(int node, void *dst, int size)
{


    flush();
    memcpy(dst, (void*) (master_mbox.start_pointer[node]), size);
    flush();
}

void LpelMailboxRecv_overMPB(
	  char *privbuf,
		//t_vcharp privbuf,    // destination buffer in local private memory (receive buffer)
	  size_t size,      // size of message (bytes)
	  int source       // UE that sent the message
	                    // set to 1, otherwise to 0
	  )
{
	if (MASTER)
		//for (int i=0; i<size;i++)
		//	MPB_read((t_vcharp)privbuf+i,master_mbox.start_pointer[source]+(1<<3)+i,size);
		cpy_mpb_to_mem(source, privbuf, size);		
	else{
		// copy data from local MPB space to private memory
	//	for (int i; i<size;i++)
		//MPB_read((t_vcharp)privbuf ,worker_mbox.start_pointer , size);
	flush();
	memcpy(privbuf, (void*) (worker_mbox.start_pointer), size);
}
}





