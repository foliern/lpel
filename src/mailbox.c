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
#include "scc.h"
#include "mailbox.h"

//#include <signal.h>




//......................................................................................
// GLOBAL VARIABLES USED FOR MPB
//......................................................................................


//int       SCC_COREID[RCCE_MAXNP]; // array of physical core IDs for all participating cores, sorted

t_vcharp  SCC_MESSAGE_PASSING_BUFFER[CORES]; // starts of MPB, sorted by rank
static int       node_ID=-1;           // rank of calling core (invalid by default)
static int	MASTER=FALSE;
// Variables
int NCMDeviceFD; // File descriptor for non-cachable memory (e.g. config regs).
int MPBDeviceFD; // File descriptor for message passing buffers

// payload part of the MPBs starts at a specific address, not malloced space
//t_vcharp RCCE_buff_ptr;
t_vcharp my_mpb_ptr;
// maximum chunk size of message payload is also specified

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
  //(void) info; /* NOT USED */
 /* int x, y, z, address;
  sigset_t signal_mask;
  unsigned char num_pages;


	
	num_nodes=DLPEL_ACTIVE_NODES;

*/
 /* for (int i = 0; i < argc; i++) {
    if (strcmp(argv[i], "-np") == 0 && ++i < argc) {
      num_nodes = atoi(argv[i]);
    } else if (strcmp(argv[i], "-sccremap") == 0) {
      remap = true;
    }
  }

  if (num_nodes == 0) {
    SNetUtilDebugFatal("Number of nodes not specified using -np flag!\n");
  }*/

/*  sigemptyset(&signal_mask);
  sigaddset(&signal_mask, SIGUSR1);
  sigaddset(&signal_mask, SIGUSR2);
  pthread_sigmask(SIG_BLOCK, &signal_mask, NULL);

  InitAPI(0);
  z = ReadConfigReg(CRB_OWN+MYTILEID);
  x = (z >> 3) & 0x0f; // bits 06:03
  y = (z >> 7) & 0x0f; // bits 10:07
  z = z & 7; // bits 02:00
  node_location = PID(x, y, z);

  for (unsigned char cpu = 0; cpu < CORES; cpu++) {
    x = X_PID(cpu);
    y = Y_PID(cpu);
    z = Z_PID(cpu);

    if (cpu == node_location) address = CRB_OWN;
    else address = CRB_ADDR(x, y);

    irq_pins[cpu] = MallocConfigReg(address + (z ? GLCFG1 : GLCFG0));
    luts[cpu] = (uint64_t*) MallocConfigReg(address + (z ? LUT1 : LUT0));
    locks[cpu] = (t_vcharp) MallocConfigReg(address + (z ? LOCK1 : LOCK0));
    MPBalloc(&mpbs[cpu], x, y, z, cpu == node_location);
  }

  num_pages = PAGES_PER_CORE - LINUX_PRIV_PAGES;
  int max_pages = remap ? MAX_PAGES/2 : MAX_PAGES - 1;

  for (int i = 1; i < CORES / num_nodes && num_pages < max_pages; i++) {
    for (int lut = 0; lut < PAGES_PER_CORE && num_pages < max_pages; lut++) {
      LUT(node_location, LINUX_PRIV_PAGES + num_pages++) = LUT(node_location + i * num_nodes, lut);
    }
  }

  int extra = ((CORES % num_nodes) * PAGES_PER_CORE) / num_nodes;
  int node = num_nodes + (node_location * extra) / PAGES_PER_CORE;
  int lut = (node_location * extra) % PAGES_PER_CORE;

  for (int i = 0; i < extra && num_pages < max_pages; i++ ) {
    LUT(node_location, LINUX_PRIV_PAGES + num_pages++) = LUT(node, lut + i);

    if (lut + i + 1 == PAGES_PER_CORE) {
      lut = 0;
      node++;
    }
  }

  flush();
  START(node_location) = 0;
  END(node_location) = 0;
  // Start with an initial handling run to avoid a cross-core race.
  HANDLING(node_location) = 1;
  WRITING(node_location) = false;

  SCCInit(num_pages);

  FOOL_WRITE_COMBINE;
  unlock(node_location);
  */
}
/*{

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
		if ((node_ID%2) == 1)
			SCC_MESSAGE_PASSING_BUFFER[node_ID] = MPB_comm_buffer_start(node_ID) + MPB_BUFF_SIZE;
		else
			SCC_MESSAGE_PASSING_BUFFER[node_ID] = MPB_comm_buffer_start(node_ID);

		worker_mbox.start_pointer= SCC_MESSAGE_PASSING_BUFFER[node_ID]	+MPB_BUFFER_OFFSET;
//		PRT_DBG("ADRESSE Node %d: %x \n",NODE_ID, worker_mbox.start_pointer);
		worker_mbox.end_pointer= SCC_MESSAGE_PASSING_BUFFER[node_ID]	+MPB_BUFFER_OFFSET;
		worker_mbox.writing_flag= SCC_MESSAGE_PASSING_BUFFER[node_ID]	+WRITING_FLAG_OFFSET;
		worker_mbox.reading_flag= SCC_MESSAGE_PASSING_BUFFER[node_ID]	+READING_FLAG_OFFSET;
		worker_mbox.msg_type= SCC_MESSAGE_PASSING_BUFFER[node_ID]		+MSG_TYPE_OFFSET;

}*/


void LpelMailboxSend_overMPB(
		char *privbuf,    // source buffer in local private memory (send buffer)
		size_t size,      // size of message (bytes)
		int dest          // UE that will receive the message
	)
{
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




