#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "config.h"
#include "RCCE_memcpy.c"

// includes for the LUT mapping
#include "includes/distribution.h"
//#include "includes/SCC_API.h"
#include "includes/scc.h"
#include "includes/sccmalloc.h"
#include <stdarg.h>
t_vcharp mpbs[48];

int node_location;

// global variables for the LUT, PINS and LOCKS
t_vcharp locks[CORES];
volatile int *irq_pins[CORES];
volatile uint64_t *luts[CORES];
bool remap =false;
static int num_nodes = 0;

// test task
typedef struct task_t {
  int task_dest;
  char *task_info[1000];
  int task_origin;
} task_t;


void readFromMpb(int core, int size){
  char *result = (char*) malloc (size);
  memcpy_get((void*)result, (void*)mpbs[core],size);
  result[size-1]='\0';
  printf("%s\n",result);
  free(result);
}

void writeToMpb(int core, int size,char *str){
  char *buf = (char*) malloc (size);
  int len = strlen(str);
  printf("%d %s\n\n",len,str);
  if(len == 1){
    memset(buf,(char)*str, size);
  }else{
    int i;
    for(i=0; i+len <=size; i+=len){
      memcpy(buf+i,str,len);
    } 
  }

  buf[size-1] = '\0';


  printf("%s\n\nCopying to mpb now\n",buf);

  memcpy_put((void*)mpbs[core],(void*)buf,size);

  free(buf);
}


int main(int argc, char **argv){

//variables for the MPB init
   int core,size;
   int x, y, z, address;
   unsigned char cpu;

//variables for the LUT init
   sigset_t signal_mask;
   unsigned char num_pages;

//INIT START!!!

   remap=true;
   num_nodes = DLPEL_ACTIVE_NODES;

   sigemptyset(&signal_mask);
   sigaddset(&signal_mask, SIGUSR1);
   sigaddset(&signal_mask, SIGUSR2);
   pthread_sigmask(SIG_BLOCK, &signal_mask, NULL);

  
//**********************************

   InitAPI(0);

//**********************************


   z = ReadConfigReg(CRB_OWN+MYTILEID);
   x = (z >> 3) & 0x0f; // bits 06:03
   y = (z >> 7) & 0x0f; // bits 10:07
   z = z & 7; // bits 02:00
   node_location = PID(x, y, z);
 
  for (cpu = 0; cpu < 48; cpu++) {
    x = X_PID(cpu);
    y = Y_PID(cpu);
    z = Z_PID(cpu);

    if (cpu == node_location) address = CRB_OWN;
    else address = CRB_ADDR(x, y);
    
    //LUT, PINS, LOCK allocation
    irq_pins[cpu] = MallocConfigReg(address + (z ? GLCFG1 : GLCFG0));
    luts[cpu] = (uint64_t*) MallocConfigReg(address + (z ? LUT1 : LUT0));
    locks[cpu] = (t_vcharp) MallocConfigReg(address + (z ? LOCK1 : LOCK0));

    //MPB allocation
    MPBalloc(&mpbs[cpu], x, y, z, cpu == node_location);
  }

//***********************************************
//LUT remapping

  num_pages = PAGES_PER_CORE - LINUX_PRIV_PAGES;
    int max_pages = remap ? MAX_PAGES/2 : MAX_PAGES - 1;

    printf("First for loops\n");
    
   int i, lut;
 
   for (i = 1; i < CORES / num_nodes && num_pages < max_pages; i++) {
      for (lut = 0; lut < PAGES_PER_CORE && num_pages < max_pages; lut++) {
        printf("Copy to %i  node's LUT entry Nr.: %i / %x from (node_location+i*num_nodes)= %i node's LUT entry Nr.: %i / %x.  Condition: num_pages: %i < max_pages: %i\n",
                  node_location, LINUX_PRIV_PAGES + num_pages,LINUX_PRIV_PAGES+num_pages, node_location + i * num_nodes,  lut, lut, num_pages, max_pages);
  	LUT(node_location, LINUX_PRIV_PAGES + num_pages++) = LUT(node_location + i * num_nodes, lut);
    }
    }

    int extra = ((CORES % num_nodes) * PAGES_PER_CORE) / num_nodes;
    int node = num_nodes + (node_location * extra) / PAGES_PER_CORE;
    int lut2 = (node_location * extra) % PAGES_PER_CORE;

    printf("Second for loop\n");
    i=0;
    for (i = 0; i < extra && num_pages < max_pages; i++ ) {
  	printf("Copy to %i  node's LUT entry Nr.: %i from (node_location+i*num_nodes)= %i node's LUT entry Nr.: %i\n",
                  node_location, LINUX_PRIV_PAGES + num_pages, node_location + i * num_nodes,  lut2);
      LUT(node_location, LINUX_PRIV_PAGES + num_pages++) = LUT(node, lut2 + i);

      if (lut + i + 1 == PAGES_PER_CORE) {
        lut2 = 0;
        node++;
      }
    }

//***********************************************

//LUT settings
    flush();
    START(node_location) = 0;
    END(node_location) = 0;
    /* Start with an initial handling run to avoid a cross-core race. */
    HANDLING(node_location) = 1;
    WRITING(node_location) = false;

//***********************************************

    SCCInit(num_pages);

//***********************************************

  FOOL_WRITE_COMBINE;
  unlock(node_location);


// general Input handling

    if (argc ==  3 && !strcmp("test", argv[1])) {
      core = atoi(argv[2]);
      //testMPB(core);       
    } else if (argc == 4 && !strcmp("read", argv[1])){
      core = atoi(argv[2]);
      size = atoi(argv[3]);
      readFromMpb(core, size);
    }else if (argc == 5 && !strcmp("write", argv[1])){//string
      core = atoi(argv[2]);
      size = atoi(argv[3]);
      printf("core=%d size=%d char=%s\n\n",core,size,argv[4]);
      writeToMpb(core,size,argv[4]);
    }else if (argc == 4 && !strcmp("lutsend",argv[1])){ //lut send over mpb
	//create a task in the local prvate memory
	printf("allocate ptr\n");
	task_t *test_task= SCCMallocPtr(sizeof(task_t));
	printf("ptrallocated\n");
	test_task->task_dest=atoi(argv[2]);
	memcpy(test_task->task_info,argv[3],100);
	test_task->task_origin=node_location;
	//create lut address struct, declaration of lut_addr_t in malloc.h 
	printf("debugging check\n");


	lut_addr_t *addr=(lut_addr_t*)malloc(sizeof(lut_addr_t));
	*addr= SCCPtr2Addr(test_task);

	printf("addr->node: %d addr->lut: %d addr->offset %u\n",addr->node, addr->lut, addr->offset);
//	*addr= SCCPtr2Addr(SCCMallocPtr(sizeof(task_t)));


	printf("send LUT entry: \n dest=%d info=%s origin=%d\n\n",test_task->task_dest,test_task->task_info,test_task->task_origin);
	// send lut entry
	size=sizeof(task_t);
	//SNetDistribPack(test_task,buffer, sizeof(test_task), true);
      cpy_mem_to_mpb(atoi(argv[2]), addr, sizeof(lut_addr_t));
      cpy_mem_to_mpb(atoi(argv[2]), &size, sizeof(size_t)); 
   }else{
      fprintf(stderr, "Usage:\n"
          "%s test <destination core> \n"
          "read <source core ID> <data size> \n"
          "write <destination core ID> <data size> (<char to write> || <string to write>) \n"
    	  "lutsend <destination core ID> (<char to write> || <string to write>) \n", argv[0]);
       exit(1);
    }





  return 0;
}


