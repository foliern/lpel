#include <fcntl.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/mman.h>

#include "memfun.h"
#include "scc.h"
#include "sccmalloc.h"
#include "bool.h"
#include "input.h"
#include "scc_comm_func.h"
#include "debugging.h"


typedef union block {
  struct {
    union block *next;
    size_t size;
  } hdr;
  uint32_t align;   // Forces proper allignment
} block_t;

typedef struct {
  unsigned char free;
  unsigned char size;
} lut_state_t;

//void *remote;
unsigned char local_pages=MAX_PAGES;
int node_ID;

static void *local;
static int mem, cache;
static block_t *freeList;
static lut_state_t *lutState;
//static unsigned char remote_pages;
uintptr_t shmem_start_address;

lut_addr_t SCCPtr2Addr(void *p)
{
  uint32_t offset;
  unsigned char lut;
  //if (local <= p && p <= local + local_pages * PAGE_SIZE) {
  if (local <= p && p <= local + SHM_MEMORY_SIZE) {
    offset = (p - local) % PAGE_SIZE;
    lut = LOCAL_LUT + (p - local) / PAGE_SIZE;
  /*} else if (remote <= p && p <= remote + remote_pages * PAGE_SIZE) {
    offset = (p - remote) % PAGE_SIZE;
    lut = REMOTE_LUT + (p - remote) / PAGE_SIZE;
  */
  } else {
    printf("Invalid pointer\n");
  }

  lut_addr_t result = {node_location, lut, offset};
  return result;
}

void *SCCAddr2Ptr(lut_addr_t addr)
{
  if (LOCAL_LUT <= addr.lut && addr.lut < LOCAL_LUT + local_pages) {
    return (void*) ((addr.lut - LOCAL_LUT) * PAGE_SIZE + addr.offset + local);
  /*} else if (REMOTE_LUT <= addr.lut && addr.lut < REMOTE_LUT + remote_pages) {
    return (void*) ((addr.lut - REMOTE_LUT) * PAGE_SIZE + addr.offset + remote);
  */
  } else {
    printf("Invalid SCC LUT address\n");
  }

  return NULL;
}


void SCCInit(uintptr_t *addr)
{
  node_ID= SccGetNodeID();
  // Open driver device "/dev/rckdyn011" to map memory in write-through mode 
  mem = open("/dev/rckdcm", O_RDWR|O_SYNC);
  PRT_DBG("mem: %i\n", mem);
  if (mem < 0) {
	printf("Opening /dev/rckdyn011 failed!\n");
  }	


 if (*addr==0x0){ 
	 PRT_DBG("MASTER MMAP\n\n");
 	 local = mmap(NULL, 		SHM_MEMORY_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, mem, LOCAL_LUT << 24);
 	 if (local == NULL) printf("Couldn't map memory!\n");
	 else	munmap(local, SHM_MEMORY_SIZE);
 	 local = mmap((void*)local, 	SHM_MEMORY_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_FIXED, mem, LOCAL_LUT << 24);
  	if (local == NULL) printf("Couldn't map memory!\n");

	*addr=local;
  }else{
	PRT_DBG("WORKER MMAP\n\n");
	local=*addr;
	local = mmap((void*)local,     	SHM_MEMORY_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_FIXED, mem, LOCAL_LUT << 24);

	if (local == NULL) printf("Couldn't map memory!");

  }  

  PRT_DBG("addr:                                %p\n",*addr);
  PRT_DBG("local:				%p\n",local);

  PRT_DBG("MEMORY_OFFSET(node_ID): 	%u\n",MEMORY_OFFSET(node_ID)); 
  freeList = local+MEMORY_OFFSET(node_ID);
  PRT_DBG("freelist address: 		%p\n",freeList);
  	lut_addr_t *addr_t=(lut_addr_t*)malloc(sizeof(lut_addr_t));
  	*addr_t= SCCPtr2Addr(freeList);
  PRT_DBG("LUT-entry of freelist:		%d\n",addr_t->lut);
  PRT_DBG("freelist's LUT offset: 		%u\n",addr_t->offset);

  freeList->hdr.next = freeList;

 // freeList->hdr.size = (size * PAGE_SIZE) / sizeof(block_t);
  freeList->hdr.size = SHM_MEMORY_SIZE / sizeof(block_t);
  /*if(msync(local,SHM_MEMORY_SIZE,MS_SYNC | MS_INVALIDATE))
                printf("Couldn't sync memory");
  */
}



void SCCStop(void)
{
  //munmap(remote, remote_pages * PAGE_SIZE);
  munmap(local, local_pages * PAGE_SIZE);

  close(mem);
  //close(cache);
}

void *SCCMallocPtr(size_t size)
{
  size_t nunits;
  block_t *curr, *prev, *new;


  if (freeList == NULL) printf("Couldn't allocate memory!");
  
//  PRT_DBG("freeList:                            %p\n",freeList);
//  PRT_DBG("freeList->hdr.next:                  %p\n",freeList->hdr.next);
//  PRT_DBG("freeList->hdr.size:                  %zu\n",freeList->hdr.size);
  prev = freeList;
//  PRT_DBG("prev:				%p\n",prev);
//  PRT_DBG("prev->hdr.next:                      %p\n",prev->hdr.next);
//  PRT_DBG("prev->hdr.size:                      %zu\n",prev->hdr.size);
  curr = prev->hdr.next;
//  PRT_DBG("curr:				%p\n",curr);
//  PRT_DBG("curr->hdr.next:                      %p\n",curr->hdr.next);
//  PRT_DBG("curr->hdr.size:                      %zu\n",curr->hdr.size);
  nunits = (size + sizeof(block_t) - 1) / sizeof(block_t) + 1;
//  PRT_DBG("size:				%zu\n",size);
//  PRT_DBG("nunits:				%zu\n",nunits);
  do {
//    PRT_DBG("curr->hdr.size:			%zu\n\n",curr->hdr.size);
    if (curr->hdr.size >= nunits) {
      if (curr->hdr.size == nunits) {
      
	  if (prev == curr){
		PRT_DBG("SET prev TO NULL");
		 prev = NULL;
          }else{
		 prev->hdr.next = curr->hdr.next;
	  }
      } else if (curr->hdr.size > nunits) {

//	PRT_DBG("curr:                                %p\n",curr);
//  PRT_DBG("curr->hdr.next:                      %p\n",curr->hdr.next);
//  PRT_DBG("curr->hdr.size:                      %zu\n",curr->hdr.size);


	new = curr + nunits;
	
//	        PRT_DBG("new:                                   %p\n",new);
//        PRT_DBG("new->hdr.next:                         %p\n",new->hdr.next);
//        PRT_DBG("new->hdr.size:                         %zu\n",new->hdr.size);
	*new = *curr;
//	new->hdr.next=curr->hdr.next;
//	PRT_DBG("new->hdr.size:                         %zu\n\n\n",new->hdr.size);
//	PRT_DBG("curr->hdr.size:                        %zu\n\n\n",curr->hdr.size);

//	new->hdr.size=curr->hdr.size;

//	PRT_DBG("new->hdr.size:                         %zu\n\n\n",new->hdr.size);
//        PRT_DBG("curr->hdr.size:                        %zu\n\n\n",curr->hdr.size);


//	        PRT_DBG("new:                                   %p\n",new);
//        PRT_DBG("new->hdr.next:                         %p\n",new->hdr.next);
//        PRT_DBG("new->hdr.size:                         %zu\n",new->hdr.size);

        new->hdr.size -= nunits;

//	PRT_DBG("new->hdr.size:                    %zu\n\n",new->hdr.size);
  	curr->hdr.size = nunits;
//	PRT_DBG("curr->hdr.size:                    %zu\n\n",curr->hdr.size);
	

//	PRT_DBG("curr:                                	%p\n",curr);
// 	PRT_DBG("curr->hdr.next:                      	%p\n",curr->hdr.next);
// 	PRT_DBG("curr->hdr.size:                     	%zu\n",curr->hdr.size);

//	PRT_DBG("new:					%p\n",new);
//	PRT_DBG("new->hdr.next:				%p\n",new->hdr.next);
// 	PRT_DBG("new->hdr.size:				%zu\n",new->hdr.size);


//	PRT_DBG("prev:                                %p\n",prev);
//  	PRT_DBG("prev->hdr.next:                      %p\n",prev->hdr.next);
//  	PRT_DBG("prev->hdr.size:                      %zu\n",prev->hdr.size);
        if (prev == curr) prev = new;
	prev->hdr.next = new;
//	PRT_DBG("prev:                                %p\n",prev);
//  	PRT_DBG("prev->hdr.next:                      %p\n",prev->hdr.next);
//  	PRT_DBG("prev->hdr.size:                      %zu\n",prev->hdr.size);

      }
      freeList = prev;
//      PRT_DBG("freeList:                            %p\n",freeList);
//      PRT_DBG("freeList->hdr.next:                  %p\n",freeList->hdr.next);
//      PRT_DBG("freeList->hdr.size:                  %zu\n\n",freeList->hdr.size);
//      PRT_DBG("RETURN: 	                       	    %p\n\n",curr+1);
        	
//	if (msync(local,SHM_MEMORY_SIZE,MS_SYNC | MS_INVALIDATE))
//		printf("Couldn't sync memory");
	return (void*) (curr + 1);
     }
//	PRT_DBG("						WHILE LOOP!!!");
  } while (curr != freeList && (prev = curr, curr = curr->hdr.next));


  printf("Couldn't allocate memory!");
  return NULL;
}

void SCCFreePtr(void *p)
{
  block_t *block = (block_t*) p - 1,
          *curr = freeList;

  if (freeList == NULL) {
    freeList = block;
    freeList->hdr.next = freeList;
    return;
  }

  while (!(block > curr && block < curr->hdr.next)) {
    if (curr >= curr->hdr.next && (block > curr || block < curr->hdr.next)) break;
    curr = curr->hdr.next;
  }


  if (block + block->hdr.size == curr->hdr.next) {
    block->hdr.size += curr->hdr.next->hdr.size;
    if (curr == curr->hdr.next) block->hdr.next = block;
    else block->hdr.next = curr->hdr.next->hdr.next;
  } else {
    block->hdr.next = curr->hdr.next;
  }

  if (curr + curr->hdr.size == block) {
    curr->hdr.size += block->hdr.size;
    curr->hdr.next = block->hdr.next;
  } else {
    curr->hdr.next = block;
  }

  freeList = curr;
}

unsigned char SCCMallocLut(size_t size)
{
/*  lut_state_t *curr = lutState;

  do {
    if (curr->free && curr->size >= size) {
      if (curr->size == size) {
        curr->free = 0;
        curr[size - 1].free = 0;
      } else {
        lut_state_t *next = curr + size;

        next->free = 1;
        next->size = curr->size - size;
        next[next->size - 1] = next[0];

        curr->free = 0;
        curr->size = size;
        curr[curr->size - 1] = curr[0];
      }

      return REMOTE_LUT + curr - lutState;
    }

    curr += curr->size;
  } while (curr < lutState + remote_pages);

  printf("Not enough available LUT entries!\n");
 */ 
 return 0;
}


void SCCFreeLut(void *p)
{
/*  lut_state_t *lut = lutState + (p - remote) / PAGE_SIZE;

  if (lut + lut->size < lutState + remote_pages && lut[lut->size].free) {
    lut->size += lut[lut->size].size;
  }

  if (lutState < lut && lut[-1].free) {
    lut -= lut[-1].size;
    lut->size += lut[lut->size].size;
  }

  lut->free = 1;
  lut[lut->size - 1] = lut[0];
*/
}

void SCCFree(void *p)
{
  if (local <= p && p <= local + local_pages * PAGE_SIZE) {
    SCCFreePtr(p);
  }/* else if (remote <= p && p <= remote + remote_pages * PAGE_SIZE) {
    SCCFreeLut(p);
  }*/
}
