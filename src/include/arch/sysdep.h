#ifndef _SYSDEP_H_
#define _SYSDEP_H_

/*
 * The following has been taken from fastflow (v 1.0.0rc2) file sysdep.h. 
 */

/*
 * Copyright (c) 2000 Massachusetts Institute of Technology
 * Copyright (c) 2000 Matteo Frigo
 *
 *  This library is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation; either version 2.1 of the License, or (at
 *  your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307,
 *  USA.
 *
 */

/* Don't just include config.h, since that is not installed. */
/* Instead, we must actually #define the useful things here. */
/* #include "../config.h" */
/* Compiler-specific dependencies here, followed by the runtime system dependencies.
 * The compiler-specific dependencies were originally written by Eitan Ben Amos.
 * Modified by Bradley.
 */


/***********************************************************\
 * Various types of memory barriers and atomic operations
\***********************************************************/

/*------------------------
       POWERPC 
 ------------------------*/
#if defined(__powerpc__) || defined(__ppc__)
/* This version contributed by Matteo Frigo Wed Jul 13 2005.   He wrote:
 *   lwsync is faster than eieio and has the desired store-barrier
 *   behavior.  The isync in the lock is necessary because the processor is
 *   allowed to speculate on loads following the branch, which makes the
 *   program without isync incorrect (in theory at least---I have never
 *   observed such a speculation).
 */

#define WMB()  __asm__ __volatile__ ("lwsync" : : : "memory")

/* atomic swap operation */
static __inline__ int xchg(volatile int *ptr, int x)
{
    int result;
    __asm__ __volatile__ (
			  "0: lwarx %0,0,%1\n stwcx. %2,0,%1\n bne- 0b\n isync\n" :
			  "=&r"(result) : 
			  "r"(ptr), "r"(x) :
			  "cr0");
    
    return result;
}
#endif

/*------------------------
       IA64
 ------------------------*/
#ifdef __ia64__

#define WMB()  __asm__ __volatile__ ("mf" : : : "memory")

/* atomic swap operation */
static inline int xchg(volatile int *ptr, int x)
{
    int result;
    __asm__ __volatile ("xchg4 %0=%1,%2" : "=r" (result)
			: "m" (*(int *) ptr), "r" (x) : "memory");
    return result;
}
#endif

/*------------------------
         I386 
 ------------------------*/
#ifdef __i386__ 

#define WMB() __asm__ __volatile__ ("": : :"memory")

/* atomic swap operation */
static inline int xchg(volatile int *ptr, int x)
{
    __asm__("xchgl %0,%1" :"=r" (x) :"m" (*(ptr)), "0" (x) :"memory");
    return x;
}
#endif /* __i386__ */

/*------------------------
         amd_64
 ------------------------*/
#ifdef __x86_64

#define WMB() __asm__ __volatile__ ("": : :"memory")

/* atomic swap operation */
static inline int xchg(volatile int *ptr, int x)
{
    __asm__("xchgl %0,%1" :"=r" (x) :"m" (*(ptr)), "0" (x) :"memory");
    return x;
}
#endif /* __x86_64 */

#endif /* _SYSDEP_H_ */
