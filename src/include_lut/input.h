 /*
 * Inputs for D-LPEL on the SCC
 */
#include scc.h

#define MASTER 0
#define NR_WORKERS 2 
#define POWER_DOMAIN 1
#define POWER_DOMAIN_WORKERS 8
#define MEMORY_DOMAIN 0
#define MEMORY_DOMAIN_WORKERS 10
#define LUT_MEMORY_DOMAIN_OFFSET 6
#define AIR_LUT_SYNCH_VALUE 1
//Global Memory Start Address
#define GMS_ADDRESS 2572472320

//Global Memory Start Address
#define GMS_ADDRESS 0x9954D000 

//0x10000 is just for testing is not the real value at the end
#define LOCAL_SHMSIZE  SHM_MEMORY_SIZE/NR_WORKERS
#define MEMORY_OFFSET(id) (id *(SHM_MEMORY_SIZE/NR_WORKERS))
#define MAILBOX_OFFSET 8
