#ifndef SCCMALLOC_H
#define SCCMALLOC_H


#define LOCAL_LUT   0x14
#define REMOTE_LUT  (LOCAL_LUT + local_pages)

//following definitions are added by Simon to run SCCInit normaly they ar located in sys/mman.h
// Path: /shared/foliern/sccLinux/buildroot-2011.11/output/host/usr/i586-unknown-linux-gnu/sysroot/usr/include/bits/mman.h
/*
 * Protections are chosen from these bits, or-ed together
 */
//#define	PROT_NONE	0x00	/* [MC2] no permissions */
//#define	PROT_READ	0x01	/* [MC2] pages can be read */
//#define	PROT_WRITE	0x02	/* [MC2] pages can be written */
//#define	PROT_EXEC	0x04	/* [MC2] pages can be executed */

/*
 * Flags contain sharing type and options.
 * Sharing types; choose one.
 */
//#define	MAP_SHARED	0x0001		/* [MF|SHM] share changes */
//#define	MAP_PRIVATE	0x0002		/* [MF|SHM] changes are private */
//#if !defined(_POSIX_C_SOURCE) || defined(_DARWIN_C_SOURCE)
//#define	MAP_COPY	MAP_PRIVATE	/* Obsolete */
//#endif	/* (!_POSIX_C_SOURCE || _DARWIN_C_SOURCE) */



extern void *remote;
extern unsigned char local_pages;

typedef struct {
  unsigned char node, lut;
  uint32_t offset;
} lut_addr_t;

lut_addr_t SCCPtr2Addr(void *p);
void *SCCAddr2Ptr(lut_addr_t addr);

void SCCInit(unsigned char size);
void SCCStop(void);

void *SCCMallocPtr(size_t size);
unsigned char SCCMallocLut(size_t size);
void SCCFree(void *p);
#endif
