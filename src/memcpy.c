/*
 * mpb.c
 *
 *  Created on: Feb 6, 2013
 *      Author: Simon
 */
#include <stdlib.h>
#include <string.h>
#include "memcpy.h"



//--------------------------------------------------------------------------------------
// FUNCTION: memcpy_put
//--------------------------------------------------------------------------------------
// optimized memcpy for copying data from private memory to MPB
//--------------------------------------------------------------------------------------
inline static void *MPB_memcpy_put(void *dest, const void *src, size_t count)
{
	int i, j, k;

	asm volatile (
				"cld; rep ; movsl\n\t"
				"movl %4, %%ecx\n\t"
				"andl $3, %%ecx\n\t"
				"rep ; movsb\n\t"
				: "=&c"(i), "=&D"(j), "=&S"(k)
				: "0"(count/4), "g"(count), "1"(dest), "2"(src) : "memory");

	return dest;
}

	//--------------------------------------------------------------------------------------
    // FUNCTION: memcpy_get
    //--------------------------------------------------------------------------------------
    // optimized memcpy for copying data from MPB to private memory
    //--------------------------------------------------------------------------------------

inline static void *MPB_memcpy_get(void *dest, const void *src, size_t count)
{
	int h, i, j, k, l, m;

	asm volatile("cld;\n\t"
                "1: cmpl $0, %%eax ; je 2f\n\t"
                "movl (%%edi), %%edx\n\t"
                "movl 0(%%esi), %%ecx\n\t"
                "movl 4(%%esi), %%edx\n\t"
                "movl %%ecx, 0(%%edi)\n\t"
                "movl %%edx, 4(%%edi)\n\t"
                "movl 8(%%esi), %%ecx\n\t"
                "movl 12(%%esi), %%edx\n\t"
                "movl %%ecx, 8(%%edi)\n\t"
                "movl %%edx, 12(%%edi)\n\t"
                "movl 16(%%esi), %%ecx\n\t"
                "movl 20(%%esi), %%edx\n\t"
                "movl %%ecx, 16(%%edi)\n\t"
                "movl %%edx, 20(%%edi)\n\t"
                "movl 24(%%esi), %%ecx\n\t"
                "movl 28(%%esi), %%edx\n\t"
                "movl %%ecx, 24(%%edi)\n\t"
                "movl %%edx, 28(%%edi)\n\t"
                "addl $32, %%esi\n\t"
                "addl $32, %%edi\n\t"
                "dec %%eax ; jmp 1b\n\t"
                "2: movl %%ebx, %%ecx\n\t"
                "movl (%%edi), %%edx\n\t"
                "andl $31, %%ecx\n\t"
                "rep ; movsb\n\t"
                : "=&a"(h), "=&D"(i), "=&S"(j), "=&b"(k), "=&c"(l), "=&d"(m)
                : "0"(count/32), "1"(dest), "2"(src), "3"(count)  : "memory");

	return dest;
}

