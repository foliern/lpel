#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "scc_comm_func.h"

// includes for the LUT mapping
#include "bool.h"
#include "config.h"
#include "RCCE_memcpy.c"
#include "distribution.h"
#include "scc.h"
#include "sccmalloc.h"
#include <stdarg.h>
#include "input.h"
#include "debugging.h"




// global variables for the LUT, PINS and LOCKS
t_vcharp mpbs[48];
t_vcharp locks[CORES];
volatile int *irq_pins[CORES];
volatile uint64_t *luts[CORES];
AIR atomic_inc_regs[2*CORES];

//AIR function first declaration forLUT remapping synchronisation
void atomic_inc(AIR *reg, int *value);
void atomic_dec(AIR *reg, int value);
void atomic_read(AIR *reg, int *value);
void atomic_write(AIR *reg, int value);


bool remap =false;

int node_location;
static int num_nodes = 0;

void scc_init(){

//variables for the MPB init
   int core,size;
   int x, y, z, address;
   unsigned char cpu;

//variables for the LUT init
   sigset_t signal_mask;
   unsigned char num_pages;

//variables for the AIR init
   int *air_baseE, *air_baseF;

//INIT START!!!

   remap=true;
   num_nodes = NR_WORKERS;

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

   air_baseE = (int *) MallocConfigReg(FPGA_BASE + 0xE000);
   air_baseF = (int *) MallocConfigReg(FPGA_BASE + 0xF000);

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

    //FIRST SET OF AIR
	atomic_inc_regs[cpu].counter 	= air_baseE + 2*cpu;
	atomic_inc_regs[cpu].init 		= air_baseE + 2*cpu + 1;

	//SECOND SET OF AIR
	atomic_inc_regs[CORES+cpu].counter 	= air_baseF + 2*cpu;
	atomic_inc_regs[CORES+cpu].init 	= air_baseF + 2*cpu + 1;
	if(node_location == MASTER){
		// only one core needs to call this
		*atomic_inc_regs[cpu].init = 0;
		*atomic_inc_regs[CORES+cpu].init = 0;
	}
  } //end for


//***********************************************
//LUT remapping

    num_pages = PAGES_PER_CORE - LINUX_PRIV_PAGES;
  
    int max_pages = MAX_PAGES-1;

    printf("First for loops\n");

   int i, lut, origin;

   if(node_location != MASTER){
	   int value=-1;
	   while(value < AIR_LUT_SYNCH_VALUE){
		   atomic_read(&atomic_inc_regs[MASTER],&value);
	   }
	   
           if (value == (num_nodes-1))
		   atomic_write(&atomic_inc_regs[MASTER],0);
	   else
		   atomic_inc(&atomic_inc_regs[MASTER],&value);
	//debugging lines
		atomic_read(&atomic_inc_regs[MASTER],&value);
		PRT_DBG("value= %d\n",value);
	//debugging lines end		


	   PRT_DBG("AIR==1 -> MASTER has finished LUT mapping.\n");
   }

   if(node_location == MASTER){
       for (i = 1; i < CORES && num_pages < max_pages; i++) {
	   for (lut = 20; lut < PAGES_PER_CORE && num_pages < max_pages; lut++) {

				if ((node_location+i) > 5)
					origin=node_location+i+LUT_MEMORY_DOMAIN_OFFSET;
				else
					origin=node_location+i;

				LUT(node_location, LINUX_PRIV_PAGES + num_pages++) = LUT(origin, lut);
				
				PRT_DBG("Copy to %i  node's LUT entry Nr.: %i / %x from %i node's LUT entry Nr.: %i / %x. Num_pages: %i, Max_pages: %i\n",
                                   node_location, LINUX_PRIV_PAGES + (num_pages-1),LINUX_PRIV_PAGES+(num_pages-1), origin,  lut, lut, num_pages-1, max_pages);

	   }
        }
	atomic_write(&atomic_inc_regs[MASTER],AIR_LUT_SYNCH_VALUE);
   }
   else{
	for (lut = 20; lut < max_pages && num_pages < max_pages; lut++) {
		
		LUT(node_location, LINUX_PRIV_PAGES + num_pages++) = LUT(MASTER, lut);
                                
                PRT_DBG("Copy to %i  node's LUT entry Nr.: %i / %x from %i node's LUT entry Nr.: %i / %x. Num_pages: %i, Max_pages: %i\n",
                                   node_location, LINUX_PRIV_PAGES + (num_pages-1),LINUX_PRIV_PAGES+(num_pages-1), origin,  lut, lut, num_pages-1, max_pages);
	}
   }












   for (i = 1; i < CORES / num_nodes && num_pages < max_pages; i++) {
      for (lut = 0; lut < PAGES_PER_CORE && num_pages < max_pages; lut++) {
/*        printf("Copy to %i  node's LUT entry Nr.: %i / %x from (node_location+i*num_nodes)= %i node's LUT entry Nr.: %i / %x.  Condition: num_pages: %i < max_pages: %i\n",
                  node_location, LINUX_PRIV_PAGES + num_pages,LINUX_PRIV_PAGES+num_pages, node_location + i * num_nodes,  lut, lut, num_pages, max_pages);
*/ 

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

}


//--------------------------------------------------------------------------------------
// FUNCTION: atomic_inc
//--------------------------------------------------------------------------------------
// Increments an AIR register and returns its privious content
//--------------------------------------------------------------------------------------
void atomic_inc(AIR *reg, int *value)
{
  (*value) = (*reg->counter);
}

//--------------------------------------------------------------------------------------
// FUNCTION: atomic_dec
//--------------------------------------------------------------------------------------
// Decrements an AIR register and returns its privious content
//--------------------------------------------------------------------------------------
void atomic_dec(AIR *reg, int value)
{
  (*reg->counter) = value;
}

//--------------------------------------------------------------------------------------
// FUNCTION: atomic_read
//--------------------------------------------------------------------------------------
// Returns the current value of an AIR register with-out modification to AIR
//--------------------------------------------------------------------------------------
void atomic_read(AIR *reg, int *value)
{
  (*value) = (*reg->init);
}

//--------------------------------------------------------------------------------------
// FUNCTION: atomic_write
//--------------------------------------------------------------------------------------
// Initializes an AIR register by writing a start value
//--------------------------------------------------------------------------------------
void atomic_write(AIR *reg, int value)
{
  (*reg->init) = value;
}






