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
 #include "SCC_API_test.h"
 #include "readTileID.h"


int readTileID( void){

	typedef volatile unsigned char* t_vcharp;
	int PAGE_SIZE, NCMDeviceFD;
	// NCMDeviceFD is the file descriptor for non-cacheable memory (e.g. config regs).
	unsigned int result, tileID, coreID, x_val, y_val, coreID_mask=0x00000007, x_mask=0x00000078, y_mask=0x00000780;
	t_vcharp MappedAddr;
	unsigned int alignedAddr, pageOffset, ConfigAddr;
	ConfigAddr = CRB_OWN+MYTILEID; PAGE_SIZE = getpagesize();
	if ((NCMDeviceFD=open("/dev/rckncm", O_RDWR|O_SYNC))<0) { perror("open"); exit(-1);
	}
	alignedAddr = ConfigAddr & (~(PAGE_SIZE-1)); pageOffset = ConfigAddr - alignedAddr;
	MappedAddr = (t_vcharp) mmap(NULL, PAGE_SIZE, PROT_WRITE|PROT_READ, MAP_SHARED, NCMDeviceFD, alignedAddr);
				if (MappedAddr == MAP_FAILED) {
				   perror("mmap");exit(-1);
	}
	result = *(unsigned int*)(MappedAddr+pageOffset); munmap((void*)MappedAddr, PAGE_SIZE);
	printf("result = %x %d \n",result, result);
				coreID = result & coreID_mask;
				x_val  = (result & x_mask) >> 3;
				y_val  = (result & y_mask) >> 7;
				tileID = y_val*16 + x_val;
	printf("My (x,y) = (%d,%d)\n", x_val, y_val);
	printf("My tileID = 0x%2x\n",tileID);
	printf("My coreID = %1d\n",coreID);
	printf("My processorID = %2d\n",(x_val +(6*y_val))*2 + coreID);
	return (x_val +(6*y_val))*2 + coreID;
}
