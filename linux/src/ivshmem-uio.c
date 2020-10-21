/*
 * IVSHMEM extensions for use in Linux userspace through UIO
 *
 * Builds on Jailhouse's ivshmem demo code.
 *
 * Copyright (c) Universität Rostock, 2020
 *
 * Authors:
 *  Thorsten Schulz <thorsten.schulz@uni-rostock.de>
 *
 * SPDX-License-Identifier: GPL-2
 */

#define _GNU_SOURCE

#include <errno.h>
#include <error.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>

/* for dirname, basename */
#include <libgen.h>

#include "ivshmem.h"

static size_t _msize;

void ivshmem_reserve(size_t msize) {
	_msize = msize;
}

/* provide memory barriers against compiler optimizations */
/* copied from Jailhouse sources */
static inline void cpu_relax(void)
{
	asm volatile("rep; nop" : : : "memory");
}

static inline void memory_barrier(void)
{
	asm volatile("mfence" : : : "memory");
}

static uint64_t rdtsc(void) {
        uint32_t lo, hi;
        asm volatile("rdtsc" : "=a" (lo), "=d" (hi));
        return (uint64_t)lo | (((uint64_t)hi) << 32);
}

static inline uint32_t mmio_read32(void *address)
{
	return *(volatile uint32_t *)address;
}

static inline void mmio_write32(void *address, uint32_t value)
{
	*(volatile uint32_t *)address = value;
}

static bool readable(int no, const char *p3) {
	char cat_path[64];
	snprintf(cat_path, sizeof(cat_path), SYSCU, no, p3);

	return access(cat_path, R_OK) == 0;
}

static char * read3_s(int no, const char *p3, char *s, size_t length) {
	char cat_path[64];
	FILE *f;
	snprintf(cat_path, sizeof(cat_path), SYSCU, no, p3);

	if ( access(cat_path, R_OK) != 0 ) {
#ifdef DEBUG
		printf("IVSHMEM_setup: %s is missing.\n", cat_path);
#endif
		return NULL; /* it is possible, that an attribt does not exist */
	}

	f = fopen(cat_path, "r");
	if ( !fgets(s, length, f) ) {
		error(1, errno, "read(%s)", cat_path); /* but if it exists, it must be readable */
	} else {
		char *eol = strchr(s, '\n');
		if (eol) *eol = '\0'; /* discard trailing line break, looks ugly when printed. */
#ifdef DEBUG
		printf("IVSHMEM_setup: %s \"%s\"\n", cat_path, s);
#endif
	}
	fclose(f);
	return s;
}

static uint64_t read3_ull(int no, const char *p3) {
	uint64_t ret = 0;
	char s[32];
	char *endptr = s;
	s[0] = 0;

	if ( read3_s(no, p3, s, sizeof(s)) ) {
		ret = strtoull(s, &endptr, 0);
#ifdef DEBUG
		if (ret || *endptr)	printf("    ---> %ld\n", ret); else printf("    -!-> what is '%c' @ %d ?\n", *endptr, (int)(endptr-s));
#endif
	}

	return ret;
}

static void ivshmem_init( struct ivshmem_dev *dev, int bdf);

/* ivshmem requires the device to exist - achievable through modprobe */
/* though, it may not yet be connected to the peer yet.*/

void ivshmem_open( struct ivshmem_dev *dev, int uio_no, struct ivshmem_cb *cb ) {
	ivshmem_init( dev, uio_no);
	if ( !dev || !dev->regs ) return;

	if (cb) {
		cb->fds.fd = dev->device;
		cb->fds.events = POLLIN;
	}
	ivshmem_sti ( dev );
}

static void ivshmem_init( struct ivshmem_dev *dev, int bdf) {
	char mname[16];
	size_t length;
	int mapn = 2;
	char dev_path[16];
	if (!dev) error(1, ENOMEM, "assertion not met: dev!=NULL");
	memset(dev, 0, sizeof(struct ivshmem_dev));

	snprintf( dev_path, sizeof(dev_path), DEVUIOPATH, bdf);
	int fd = open(dev_path, O_RDWR);
	if (fd < 0) { /* the only reason we may gracefully fail is when the uio dev is missing. */
		error(0, errno, "open(%s)", dev_path);
		return;
	}

	dev->device = fd;
	dev->has_msix = readable( bdf, "device/msi_irqs");

/* dear reader, to understand the mmaps, you could read the docs of jailhouse,
   or just take it as is. The @offset (last param) indices the area to map. It 
   is not a physical address offset. The @size always needs a page granularity.
*/
/*  /sys/class/uio/uio0/maps/map{0,1,2,3,4}/{addr,name,offset,size}
	offset is always "0x0", name are hardcoded secion names, addr is from configuration, size is of the whole mapping 
	as 64bit hex val, e.g., 0x0000000000001000 */
	
	dev->regs = mmap(NULL, CONST_PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, MAP(0));
	if (dev->regs == MAP_FAILED)
		error(1, errno, "mmap(uio%d:regs)", bdf);

	/* provides a unique ID for each peer of a shmem */
	dev->id = mmio_read32(&dev->regs->id);
	dev->peers = mmio_read32(&dev->regs->max_peers);
	//printf("ID = %d\n", id);

	/* state table, size is typically max_peers * sizeof(u32) rounded up to pagesize */
	length = read3_ull( bdf, "maps/map1/size");
	dev->state = (u32 *)mmap(NULL, length, PROT_READ, MAP_SHARED, fd, MAP(1));
	if (dev->state == MAP_FAILED)
		error(1, errno, "mmap(uio%d:state[%ld])", bdf, length);
	dev->state_size = length;
	dev->lstate = malloc(length);
	memcpy(dev->lstate, (void *)dev->state, length); /* import initial state, esp. if we are not the first peer. */

	length = read3_ull( bdf, "maps/map2/size");
	if (length) {/* don't other, if map2 does not even exist */
		read3_s( bdf, "maps/map2/name", mname, sizeof(mname));
	}
	
	if (length && mname[0] == 'r' && mname[1] == 'w') {
		dev->rw_size = length;
		
		dev->rw = (u8 *)mmap(NULL, dev->rw_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, MAP(mapn));
		if (dev->rw == MAP_FAILED)
			error(1, errno, "mmap(uio%d:rw)", bdf);
	}

	if (dev->rw_size) {
		length = read3_ull( bdf, "maps/map3/size");
		mapn++;
	}
	
	if (length) {
		dev->out_size = read3_ull( bdf, dev->rw_size ? "maps/map4/size" : "maps/map3/size" );
		if ( dev->out_size*dev->peers != length ) /* make this issue fatal */
			error(1, errno, "mmap(uio%d) section-size:in != size:out/n", bdf);

		/* read-section, size is max_peers * output_pages * pagesize */
		dev->in = (u8 *)mmap(NULL, length, PROT_READ, MAP_SHARED, fd, MAP(mapn));
		if (dev->in == MAP_FAILED)
			error(1, errno, "mmap(uio%d:in)", bdf);

		/* write-section, size is output_pages * pagesize */
		dev->out = (u8 *)mmap(NULL, dev->out_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, MAP(mapn+1));
		if (dev->out == MAP_FAILED)
			error(1, errno, "mmap(uio%d:out)", bdf);
		memset(dev->out, 0, dev->out_size); /* wipe output for safety. */
	}
	ivshmem_tell( dev );
}

/* Whether this needs to be called after each INT (one-shot-mode) is configured in the vendor PCI capability on device initialization. Currently, in the Linux-UIO driver, this is always the case. */
void ivshmem_sti( struct ivshmem_dev *dev ) {
	mmio_write32(&dev->regs->int_control, 1);
}

/* This function may block endless if the peer was not checked for proper state before */
uint64_t ivshmem_set_blocking( struct ivshmem_dev *dev, u32 state, volatile int *passing_flag ) {
	*passing_flag = 1;
	memory_barrier();
	mmio_write32(&dev->regs->state, state);
	while (*passing_flag) memory_barrier();
	return rdtsc();
}

toSt ivshmem_set_state( struct ivshmem_dev *dev, u32 state ) {
	mmio_write32(&dev->regs->state, state);
	return STATE_CHANGED_OK;
}

/* send out an INT to target.
   However, the Linux UIO driver cannot distinguish between INT_no when notifying. */

void ivshmem_signal( struct ivshmem_dev *dev, u32 intno, u32 target ) {
	u32 bell_no = dev->has_msix ? (intno & 0xFFFF) : 0;
	bell_no |= target << 16;
	mmio_write32(&dev->regs->doorbell, bell_no);
}

int ivshmem_poll( struct ivshmem_dev *dev, u64 timeout ) {
	int fds_changed;
	struct timespec tv = { .tv_nsec = timeout, };
	if (timeout >= NS_PER_SEC) {
		tv.tv_nsec = timeout % NS_PER_SEC;
		tv.tv_sec  = timeout / NS_PER_SEC;
	}
	struct pollfd fds[1];
	fds[0].fd = dev->device;
	fds[0].events = POLLIN;

	fds_changed = ppoll(fds, 1, timeout == IVSHMEM_BLOCK ? NULL : &tv, NULL);
	if (fds_changed < 0)
		error(IVSHMEM_ERROR_EXIT_VALUE, errno, "poll()->%d", fds_changed);

	dev->int_count[0] += fds_changed;
	return fds_changed;
}

u32 ivshmem_read_device( struct ivshmem_dev *dev ) {
	u32 count = 1;
	size_t r;
	if ( sizeof(count) != (r = read(dev->device, &count, sizeof(count))) )
		error(IVSHMEM_ERROR_EXIT_VALUE, errno, "read(%d:uio)->%ld", dev->device, r);
	ivshmem_sti ( dev );
	return count;
}

void ivshmem_finalize( struct ivshmem_dev *dev ) {
	if ( !dev || !dev->regs ) return;
	ivshmem_set_state( dev, 0 );
	if (dev->in ) munmap((void *)dev->in,  dev->out_size*dev->peers);
	if (dev->out) munmap((void *)dev->out, dev->out_size);
	if (dev->rw)  munmap((void *)dev->rw, dev->rw_size);
	munmap((void *)dev->state, dev->state_size);
	munmap((void *)dev->regs, CONST_PAGE_SIZE);
	free(dev->lstate);
	close(dev->device);
}

unsigned long tsc_read_ns() {
	struct timespec tv;
	clock_gettime(CLOCK_MONOTONIC, &tv);
	return tv.tv_sec*NS_PER_SEC + tv.tv_nsec;
}

unsigned long process_clock_ns() {
	struct timespec tv;
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &tv);
	return tv.tv_sec*NS_PER_SEC + tv.tv_nsec;
}

unsigned long rtc_read_ns(void) {
	struct timespec tv;
	clock_gettime(CLOCK_REALTIME, &tv);
	return tv.tv_sec*NS_PER_SEC + tv.tv_nsec;
}

