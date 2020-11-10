/*
 * IVSHMEM extensions common functions
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

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "ivshmem.h"

#ifdef PIKEOS_NATIVE
/* PikeOS Personality Extensions API */
#include <p4ext/p4ext_vmem.h>
#include <p4ext/p4ext_assert.h>

#elif defined(POSIX)||defined(LINUX)
#include <errno.h>

#endif


const char *snprint_bdf(char *bdfb, int l, int bdf) {
	if (l < 8) return NULL;
	bdfb[0] = (bdf >> 12)&0xf; bdfb[0] += (bdfb[0] >= 10) ? 'a'-0xa : '0';
	bdfb[1] = (bdf >>  8)&0xf; bdfb[1] += (bdfb[1] >= 10) ? 'a'-0xa : '0';
	bdfb[2] = ':';
	bdfb[3] = (bdf >>  7)&1;   bdfb[3] += '0';
	bdfb[4] = (bdf >>  3)&0xf; bdfb[4] += (bdfb[4] >= 10) ? 'a'-0xa : '0';
	bdfb[5] = '.';
	bdfb[6] = (bdf      )&3;   bdfb[6] += '0';
	bdfb[7] = '\0';
	return bdfb;
}

/* neg numbers not supported */
int ivshmem_parse_ids(const char *arg, int * const num, int len, const char sep) {
	int step = 10;
	int a=0, input=0;
	if (!arg) return 0;
	for (int i=0; i<len;) {
		if (*arg >= '0' && *arg <= '9') {
			a = step*a + *arg - '0';
			input=1;
		} else if (step > 10 && *arg >= 'A' && *arg <= 'F') {
			a = step*a + *arg - 'A'+10;
			input=1;
		} else if (step > 10 && *arg >= 'a' && *arg <= 'f') {
			a = step*a + *arg - 'a'+10;
			input=1;
		} else if (input && !a && (*arg == 'x' || *arg == 'X')) {
			step = 0x10;
			input=0;
		} else if (*arg == sep) {
			if (input) num[i] = a;
			a = 0;
			step = 10;
			i++;
		} else if ( !*arg || *arg==' ' || *arg=='\n' || *arg=='\r') {
			if (input) num[i] = a;
			return 0;
		} else {
			return -1;
		}
		arg++;
	}
	return 0;
}

void ivshmem_tell( const struct ivshmem_dev *dev ) {
	if ( !dev || !dev->regs || !dev->state || !dev->lstate || 
#ifndef PIKEOS_NATIVE
		dev->device < 0 || 
#endif
		dev->peers < 2 ) {
		print("IVSHMEM: invalid data / not initialized.\n");
	} else {
#ifdef DEBUG
		if (dev->rw || dev->rw_size)
			print("IVSHMEM: R/W [0x%08x] @ 0x%08lx \n", (unsigned)dev->rw_size, (size_t)dev->rw);
		if (dev->in || dev->out || dev->out_size) {
			print("IVSHMEM: R/O [0x%08x] @ 0x%08lx \n", (unsigned)dev->out_size*dev->peers, (size_t)dev->in);
			print("IVSHMEM: OUT [0x%08x] @ 0x%08lx \n", (unsigned)dev->out_size, (size_t)dev->out);
		}
		for (u32 i=0; i<dev->peers && i<16; i++)
			if (i==dev->id) {
				print("IVSHMEM[%2u/%2u]: %s0x%08x %s=%u \n",
						i, dev->peers, dev->state[i]?"own-state=":"starting..",  dev->state[i], dev->has_msix?"#INTx":"#MSIX", dev->int_count[0]);
			} else {
				print("IVSHMEM[%2u/%2u]: %s0x%08x\n",
						i, dev->peers, dev->state[i]?"state=":"not available..", dev->state[i]);
			}
#endif
	}
}

u64 ivshmem_check( struct ivshmem_dev *dev, u64 timeout ) {
	if ( ivshmem_poll( dev, timeout ) ) {
		u64 bit;
		u32 count, i;
		count = ivshmem_read_device( dev );
		if (count) for (i=0,bit=1; i<dev->peers; i++,bit<<=1) {
			if (dev->lstate[i] != dev->state[i]) {
				dev->lstate[i] =  dev->state[i];
				if (i != dev->id) dev->state_change_mask |= bit;
			}
		}
	}
	return dev->state_change_mask;
}

toSt ivshmem_wait_for_state( struct ivshmem_dev *dev, u64 before_ns, u32 target, u32 *state ) {
	if (!dev || target >= dev->peers) return STATE_INVALID;
	u64 target_mask = 1 << target;
	
	u64 tsc = tsc_read_ns();
	u64 dl = before_ns > tsc ? before_ns - tsc : 0;
	do {
		if ( ivshmem_check( dev, dl ) & target_mask ) {
			*state = dev->lstate[target];
			dev->state_change_mask &= ~target_mask;
			return *state ? STATE_CHANGED_OK : STATE_CHANGED_GONE;
		}
		tsc = tsc_read_ns();
		dl = before_ns > tsc ? before_ns - tsc : 0;
	} while ( dl );

	return STATE_TIMEOUT;
}

toSt ivshmem_expect_state( struct ivshmem_dev *dev, u64 before_ns, u32 target, u32 expectation, u32 *state ) {
	toSt status;

	switch ( (status = ivshmem_wait_for_state( dev, before_ns, target, state )) ) {
	case STATE_CHANGED_OK:
		if ( *state != expectation) {
			error(IVSHMEM_EXPECT_EXIT_VALUE, 0, "Oops, looks like peer %d is out of sync (got 0x%0x instead of 0x%0x).\n", target, *state, expectation);
			status = STATE_CHANGED_BAD;
		}
		break;
	case STATE_CHANGED_GONE:
		error(IVSHMEM_EXPECT_EXIT_VALUE, 0, "Oops, looks like peer %d disappeared while expecting 0x%0x.\n", target, expectation);
		break;
	default:
		break;
	}
	
	return status;
}
