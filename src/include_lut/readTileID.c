/*
 * readTileID.c
 *
 *  Created on: Jan 15, 2013
 *      Author: Simon
 */

 #include <stdio.h>
 #include <unistd.h>
 #include <sys/mman.h>
 #include <sys/types.h>
 #include <sys/stat.h>
 #include <fcntl.h>
 #include <stdlib.h>
 //#define CRB_OWN   0xf8000000
 //#define MYTILEID  0x100
// includes for the LUT mapping


#include "config.h"
#include "RCCE_memcpy.c"
#include "distribution.h"
#include "scc.h"
#include "sccmalloc.h"
 #include "readTileID.h"

#include "debugging.h"



int readTileID( void){

	typedef volatile unsigned char* t_vcharp;
	int PAGE_SIZE_var, NCMDeviceFD;
	// NCMDeviceFD is the file descriptor for non-cacheable memory (e.g. config regs).
	unsigned int result, tileID, coreID, x_val, y_val, coreID_mask=0x00000007, x_mask=0x00000078, y_mask=0x00000780;
	t_vcharp MappedAddr;
	unsigned int alignedAddr, pageOffset, ConfigAddr;
	ConfigAddr = CRB_OWN+MYTILEID; PAGE_SIZE_var = getpagesize();
	if ((NCMDeviceFD=open("/dev/rckncm", O_RDWR|O_SYNC))<0) { perror("open"); exit(-1);
	}
	alignedAddr = ConfigAddr & (~(PAGE_SIZE_var-1)); pageOffset = ConfigAddr - alignedAddr;
	MappedAddr = (t_vcharp) mmap(NULL, PAGE_SIZE_var, PROT_WRITE|PROT_READ, MAP_SHARED, NCMDeviceFD, alignedAddr);
				if (MappedAddr == MAP_FAILED) {
				   perror("mmap");exit(-1);
	}
	result = *(unsigned int*)(MappedAddr+pageOffset); munmap((void*)MappedAddr, PAGE_SIZE_var);
	PRT_DBG("result = %x %d \n",result, result);
				coreID = result & coreID_mask;
				x_val  = (result & x_mask) >> 3;
				y_val  = (result & y_mask) >> 7;
				tileID = y_val*16 + x_val;
	PRT_DBG("My (x,y) = (%d,%d)\n", x_val, y_val);
	//PRT_DBG("My tileID = 0x%2x\n",tileID);
	//PRT_DBG("My coreID = %1d\n",coreID);
	PRT_DBG("My processorID = %2d\n",(x_val +(6*y_val))*2 + coreID);
	return (x_val +(6*y_val))*2 + coreID;
}
