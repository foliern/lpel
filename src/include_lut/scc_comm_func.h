/*  scc_comm_func.h
 *
 *  Created on: April 09, 2013
 *      Author: Simon
 */

#include <stdint.h>

typedef volatile struct _AIR {
        int *counter;
        int *init;
} AIR;

uintptr_t  addr;

void scc_init();
int  SccGetNodeID();
void atomic_incR(AIR *reg, int *value);
void atomic_decR(AIR *reg, int value);
void atomic_readR(AIR *reg, int *value);
void atomic_writeR(AIR *reg, int value);

