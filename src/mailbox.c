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
#include "SCC_API.h"
#include "readTileID.h"
#include "configuration.h"
#include "mailbox.h"

#include "distribution.h"
//for mpb, locks, LUT, irq_pins
#include "scc.h"





//......................................................................................
// GLOBAL VARIABLES USED FOR MPB
//......................................................................................


//......................................................................................
// END GLOBAL VARIABLES USED FOR MPB
//......................................................................................



/******************************************************************************/
/* Free node pool management functions                                        */
/******************************************************************************/







/******************************************************************************/
/* Public functions                                                           */
/******************************************************************************/


void LpelMailboxCreate(void)
{
	SNetDistribImplementationInit();
	
}


void LpelMailboxSend_overMPB(
		char *privbuf,    // source buffer in local private memory (send buffer)
		size_t size,      // size of message (bytes)
		int dest          // UE that will receive the message
	)
{

	cpy_mem_to_mpb(dest, privbuf,size);
	/*setReadFlag(dest);
	setWriteFlag(dest);

	if (MASTER)
	//	for (int i; i<size;i++)
	//		MPB_write(master_mbox.start_pointer[dest]+i, (t_vcharp) privbuf+i, size);
		MPB_write(mpbs[dest], (t_vcharp) privbuf, size);
	else
		MPB_write(mpbs[dest], (t_vcharp) privbuf, size);
	*/


}


void LpelMailboxRecv_overMPB(
	  char *privbuf,
		//t_vcharp privbuf,    // destination buffer in local private memory (receive buffer)
	  size_t size,      // size of message (bytes)
	  int source       // UE that sent the message
	                    // set to 1, otherwise to 0
	  )
{

	cpy_mpb_to_mem(source, privbuf, size);
	/*if (MASTER)
		//for (int i=0; i<size;i++)
		//	MPB_read((t_vcharp)privbuf+i,master_mbox.start_pointer[source]+(1<<3)+i,size);
		cpy_mpb_to_mem(source, privbuf, size);		
	else{
		// copy data from local MPB space to private memory
	//	for (int i; i<size;i++)
		//MPB_read((t_vcharp)privbuf ,worker_mbox.start_pointer , size);
	flush();
	memcpy(privbuf, (void*) (mpbs[source]), size);
	}*/
}




