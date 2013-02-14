/*
 * mpb.h
 *
 *  Created on: Feb 6, 2013
 *      Author: Simon
 */

#ifndef MPB_H_
#define MPB_H_



t_vcharp MPB_comm_buffer_start(int ue);
void MPB_write(t_vcharp target, t_vcharp source, int num_bytes);
int MPB_read(t_vcharp target, t_vcharp source, int num_bytes);
void SCCInit(unsigned char size);
#endif /* MPB_H_ */
