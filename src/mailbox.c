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



// MPBalloc allocates MPBSIZE bytes of MessagePassing buffer Memory at MPB_ADDR(x,y,core).
// 
// Parameter: MPB                   - Pointer to MPB area (return value, virtal address)
//            x,y,core              - Position of tile (x,y) and core...
// 
void MPBalloc(t_vcharp *MPB, int x, int y, int core, int isOwnMPB) {
 /* t_vcharp MappedAddr;
  // enable local MPB bypass (if trusted) by uncommenting next two lines and commenting
  // out the two after that
  //  unsigned int alignedAddr = (isOwnMPB?(MPB_OWN+(MPBSIZE*core)):MPB_ADDR(x,y,core)) & (~(PAGE_SIZE-1));
  //  unsigned int pageOffset = (isOwnMPB?(MPB_OWN+(MPBSIZE*core)):MPB_ADDR(x,y,core)) -alignedAddr;
  unsigned int alignedAddr = (MPB_ADDR(x,y,core)) & (~(PAGE_SIZE_temp-1));
  unsigned int pageOffset = (MPB_ADDR(x,y,core)) - alignedAddr;
  if ((x>=NUM_COLS) || (y>=NUM_ROWS) || (core>=NUM_CORES)) {
    printf("MPBalloc: Invalid coordinates (x=%0d, y=%0d, core=%0d)\n", x,y,core);
    *MPB = NULL;
    return;
  }
  MappedAddr = (t_vcharp) mmap(NULL, MPBSIZE, PROT_WRITE|PROT_READ, MAP_SHARED, MPBDeviceFD, alignedAddr);
  if (MappedAddr == MAP_FAILED)
  {
          perror("mmap");
          exit(-1);
  }

  *MPB = MappedAddr+pageOffset;
}

// MallocConfigReg performs a memory map operation on ConfigAddr (physical address) and
// returns a virtual address that can be used in the application. Use this function to
// allocate memory locations that you access frequently!
// 
// Parameter: ConfigAddr                - Physical address of configuration register.
// 
// Return value: ConfigRegVirtualAddr   - Virtual address of configuration register.
// 
int* MallocConfigReg(unsigned int ConfigAddr) {
  t_vcharp MappedAddr;
  unsigned int alignedAddr = ConfigAddr & (~(PAGE_SIZE_temp-1));
  unsigned int pageOffset = ConfigAddr - alignedAddr;

  MappedAddr = (t_vcharp) mmap(NULL, PAGE_SIZE_temp, PROT_WRITE|PROT_READ, MAP_SHARED, 
                               NCMDeviceFD, alignedAddr);
  if (MappedAddr == MAP_FAILED) {
          perror("mmap");
          exit(-1);
  }
}

// ReadConfigReg reads a value from a specified config register using a single read. Only use
// function to access memory locations that are not (!) performance critical (e.g. Tile-ID).
// Use MallocConfigReg() function for performance critical memory locations!
// 
// Parameter: ConfigAddr                - Address of configuration register...
// 
// Return value: Content of the specified config register
// 
int ReadConfigReg(unsigned int ConfigAddr) {
  int result;
  t_vcharp MappedAddr;
  unsigned int alignedAddr = ConfigAddr & (~(PAGE_SIZE_temp-1));
  unsigned int pageOffset = ConfigAddr - alignedAddr;

  MappedAddr = (t_vcharp) mmap(NULL, PAGE_SIZE_temp, PROT_WRITE|PROT_READ, MAP_SHARED, NCMDeviceFD, alignedAddr);
  if (MappedAddr == MAP_FAILED) {
          perror("mmap");
          exit(-1);
  }

  result = *(int*)(MappedAddr+pageOffset);
  munmap((void*)MappedAddr, PAGE_SIZE_temp);
  return result;
*/
}



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




