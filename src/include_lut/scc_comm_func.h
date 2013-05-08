/*
 * scc_comm_func.h
 *
 *  Created on: April 09, 2013
 *      Author: Simon
 */

typedef volatile struct _AIR {
        int *counter;
        int *init;
} AIR;

void scc_init();

int SccGetNodeId(void);
void atomic_inc(AIR *reg, int *value);
void atomic_dec(AIR *reg, int value);
void atomic_read(AIR *reg, int *value);
void atomic_write(AIR *reg, int value);
