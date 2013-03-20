#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

//#include "SCC_API.h"
#include "distribution.h"
#include "scc.h"
#include "sccmalloc.h"
#include "../config.h"
#include <stdint.h>
#include "bool.h"
#include <stdarg.h>

extern  node_location;

extern bool remap;


void SNetDistribPack(void *src, ...)
{
  bool isData;
  va_list args;
  lut_addr_t *addr;
  unsigned char node;
  size_t size, cpySize;

  va_start(args, src);
  addr = va_arg(args, void*);
  size = va_arg(args, size_t);
  isData = va_arg(args, bool);
  va_end(args);

printf("addr->node:%s\n",addr->node);
printf("addr->lut:%s\n",addr->lut);
printf("addr->offset:%u\n",addr->offset);
printf("size:%i\n",size);
printf("remap:%d\n",remap);
printf("isData:%d\n",isData);

  flush();
  if (isData) {
    if (remap) {
      node = addr->node;
      *addr = SCCPtr2Addr(src);
      
      cpy_mem_to_mpb(node, addr, sizeof(lut_addr_t));
      cpy_mem_to_mpb(node, &size, sizeof(size_t));
    } else {
      while (size > 0) {
        LUT(node_location, REMOTE_LUT) = LUT(addr->node, addr->lut++);

        cpySize = min(size, PAGE_SIZE - addr->offset);
        memcpy(((char*) remote + addr->offset), src, cpySize);

        size -= cpySize;
        src += cpySize;

        if (addr->offset) addr->offset = 0;
      }

      FOOL_WRITE_COMBINE;
    }
  } else {
    cpy_mem_to_mpb(addr->node, src, size);
  }
}

void SNetDistribUnpack(void *dst, ...)
{
  bool isData;
  size_t size;
  va_list args;
  lut_addr_t *addr;
  unsigned char i;

  va_start(args, dst);
  addr = va_arg(args, void*);
  isData = va_arg(args, bool);
  if (!isData) size = va_arg(args, size_t);
  va_end(args);

  if (isData) {
    if (remap) {
      unsigned char node, lut, count;
      cpy_mpb_to_mem(node_location, addr, sizeof(lut_addr_t));
      cpy_mpb_to_mem(node_location, &size, sizeof(size_t));

      node = addr->node;
      count = (size + addr->offset + PAGE_SIZE - 1) / PAGE_SIZE;
      lut = SCCMallocLut(count);

      for (i = 0; i < count; i++) {
        LUT(node_location, lut + i) = LUT(node, addr->lut + i);
      }

      addr->lut = lut;
    }

    *(void**) dst = SCCAddr2Ptr(*addr);
  } else {
    cpy_mpb_to_mem(node_location, dst, size);
  }
}
