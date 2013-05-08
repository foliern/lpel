/*
 * scc_comm_func.h
 *
 *  Created on: April 09, 2013
 *      Author: Simon
 */

void scc_init();

void atomic_inc(AIR *reg, int *value);
void atomic_dec(AIR *reg, int value);
void atomic_read(AIR *reg, int *value);
void atomic_write(AIR *reg, int value);
