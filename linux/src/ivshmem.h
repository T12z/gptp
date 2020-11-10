/*
 * Jailhouse, a Linux-based partitioning hypervisor
 *
 * Wrapper for ivshmem calls, derived from ivshmem demos
 *
 * Copyright (c) Siemens AG, 2014-2020
 * Copyright (c) Universität Rostock, 2020
 *
 * Authors:
 *  Henning Schild <henning.schild@siemens.com>
 *  Jan Kiszka <jan.kiszka@siemens.com>
 *  Thorsten Schulz <thorsten.schulz@uni-rostock.de>
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the COPYING file in the top-level directory.
 * SPDX-License-Identifier: GPL-2
 */

#ifndef IVSHMEM_H
#define IVSHMEM_H

#include <stddef.h>

#ifndef __unused
#define __unused __attribute__ ((__unused__))
#endif

#ifdef JAILHOUSE /* bare inmate, in Linux handled by uio-driver */
#include <inmate.h>

#define VENDORID			0x110a
#define DEVICEID			0x4106

#define BAR_BASE			0xff000000

#define IVSHMEM_CFG_STATE_TAB_SZ	0x04
#define IVSHMEM_CFG_RW_SECTION_SZ	0x08
#define IVSHMEM_CFG_OUT_SECTION_SZ	0x10
#define IVSHMEM_CFG_ADDRESS		0x18

#define JAILHOUSE_SHMEM_PROTO_UNDEFINED	0x0000

#define IVSHM_PRIV_CNTL_ONESHOT_INT	 (1 << 0)
#define IVSHM_CFG_PRIV_CNTL		0x03

#if defined(__x86_64__)
#define DEFAULT_IRQ_BASE	32
#elif defined(__arm__) || defined(__aarch64__)
#define DEFAULT_IRQ_BASE	(comm_region->vpci_irq_base + 32)
#else
#error Not implemented!
#endif

struct ivshmem_cb {
	int irq_base;
};

#define exit(n) stop()

/* add a helper that is normally present in POSIX, but not in inmate-lib. */
char *strncpy(char *dest, const char *src, unsigned n);

#define CONST_PAGE_SIZE PAGE_SIZE

#else  /* ^JH */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>

#define NS_PER_MSEC 1000000ULL
#define NS_PER_USEC 1000ULL
#define US_PER_MSEC 1000ULL
#define NS_PER_SEC  (NS_PER_MSEC*1000ULL)

/* would be provided by inmate_common.h */
typedef uint8_t  u8;
typedef uint32_t u32;
typedef uint64_t u64;
//typedef enum { true = 1, false = 0 } bool;

#define CONST_PAGE_SIZE 0x1000

#define SVCW_FILENAME "from0pipe" /* pipe is opened for RD */
#define SVCR_FILENAME "to0pipe"   /* pipe is opened for WR */
#define SHM_FILENAME  "ivshmem"  /* mmapped shm / file */

#if defined( ELINOS_PREFIX )

#include <linux-p4-4.1/asm-p4/vmdrv_ioctl.h>

//#undef SVCW_FILENAME
//#undef SVCR_FILENAME
//#define SVCW_FILENAME "vmport_char0" /* pipe is opened for RD */
//#define SVCR_FILENAME "vmport_char1" /* pipe is opened for WR */

#define EVM_SHM_CONFIG "/sys/bus/platform/drivers/vmfile_shm/config"
#define EVM_SHM_FILENAME  "/dev/vmfile_shm"  /* file reference used in ELinOS */
// to create the local reference, echo "0 shm:$(SHM_FILENAME)" > $(EVM_SHM_CONFIG)

#define EVM_FILE_CONFIG "/sys/bus/platform/drivers/vmfile_char/config"
#define EVM_FILE_FILENAME "/dev/vmfile"
// to create the local reference, echo "0 rfs:$(CONFNAME)      > $(EVM_FILE_CONFIG)
// 0 is the index to be appended to the NAME
#endif

#ifdef PIKEOS_POSIX

#define CLOCK_MONOTONIC CLOCK_REALTIME
#define CLOCK_PROCESS_CPUTIME_ID CLOCK_REALTIME

#include <sys/select.h>
struct ivshmem_cb {
	// n/a: struct pollfd fds;
	int reserved;
};

#elif defined PIKEOS_NATIVE

#include <vm.h>
struct ivshmem_cb {
	// n/a: struct pollfd fds;
	u8 *vmem; /* allowed memory range for SHM */
};

#else

#include <signal.h>
#include <poll.h>

struct ivshmem_cb {
	struct pollfd fds;
};
#endif
#endif /* JH */


/* use common, but fatal error reporting, please */
#ifdef PIKEOS_NATIVE
	#define print vm_cprintf

#elif defined(JAILHOUSE)
	#define print printk
#else
	#include <stdio.h>
	#define print printf
#endif

#if defined(POSIX) || defined(LINUX)
#include <stdlib.h>
#endif

#if (defined(LINUX)||defined(POSIX) ) && !defined(PIKEOS_POSIX)
#include <error.h>
#else
#ifdef PIKEOS_POSIX
#define error(E,errno, fstr, ...) { \
			print( "\n" __FILE__ ": " fstr, ##__VA_ARGS__);\
			if (errno) print(":%s",strerror(errno));\
			print("\n");\
			if (E) exit(E); \
		}
#elif defined ( PIKEOS_NATIVE )
#define error(E,errno, fstr, ...) { \
			print( "\n" __FILE__ ": " fstr, ##__VA_ARGS__);\
			if (errno) print(":%s",p4_strerror(errno));\
			print("\n");\
			if (E) vm_shutdown(VM_RESPART_MYSELF); \
		}
#else
/* the errno does not make much sense in the non-posix envs */
#define error(E,errno, fstr, ...) { \
			print( "\n" __FILE__ ": " fstr, ##__VA_ARGS__);\
			print("\n");\
			if (E) exit(E); \
		}
#endif
#endif

/* note on MSI-X IRQs: (beyond current documentation)
 *   current "user-space" drivers do not really differentiate between IRQs in terms of triggered behaviour. It only
 *   makes sense to have a non-state-change IRQ (!= 0). Or, if the targeted guest uses does not subscribe to all IRQs.
 *   The latter seems only relevant to higher-level drivers implementing a HW-driver protocol.
 *   Thus, it will be limited to 2 for now.
 */

#define IVSHMEM_MAX_VECTORS 2

#if (IVSHMEM_MAX_VECTORS >= 64)
#error IVSHMEM_MAX_VECTORS is badly defined.
#endif

/* These could be part of Inmates-Lib */
#define IVSHMEM_NSEC(ns) (ns)
#define IVSHMEM_MSEC(ms) ((ms)*NS_PER_MSEC)
#define IVSHMEM_USEC(us) ((us)*NS_PER_USEC)
#define IVSHMEM_SEC(  s) (( s)*NS_PER_SEC )
#define IVSHMEM_NONBLOCKING  0ULL
#define IVSHMEM_BLOCK      (~(IVSHMEM_NONBLOCKING))

/* derived from TRDP consts, could be anything */
#define IVSHMEM_ERROR_EXIT_VALUE (-8)
/* if this is non-zero, ivshmem_expect_state() will terminate on STATE_CHANGED_BAD/GONE */
#define IVSHMEM_EXPECT_EXIT_VALUE 0

/* some shortcuts used in uio version */
#define MAP(n) CONST_PAGE_SIZE*(n)
#define SYSCU "/sys/class/uio/uio%d/%s"
#define DEVUIOPATH "/dev/uio%d"

typedef enum {
	STATE_CHANGED_OK = 0,
	STATE_TIMEOUT = -1,
	STATE_CHANGED_GONE = -2,
	STATE_CHANGED_BAD = -3,
	STATE_INVALID = -4,
} toSt;

struct ivshm_regs {
	u32 id;
	u32 max_peers;
	u32 int_control;
	u32 doorbell;
	u32 state;
};

struct ivshmem_dev {
	struct ivshm_regs *regs;
	volatile u32 *state;
	u32 *lstate;
	u32 state_size;
	volatile u8 *rw;
	u64 rw_size;
	const u8 *in;
	u8 *out;
	u64 out_size;
	u32 *msix;
	u32 id;
	u32 peers;
	bool has_msix;
	u32 int_count[IVSHMEM_MAX_VECTORS];
	u32 irq_base;
	volatile u64 irq_mask;
	u64 state_change_mask;

#ifdef PIKEOS_NATIVE
	vm_port_desc_t
#else
	int
#endif
		device,  /* the bdf in Jailhouse/BM, the dev-fd in Linux/UIO, the RD-fifo-fd in Linux/POSIX */
		send;    /* the write pipe's fd in Linux/POSIX */
};

void ivshmem_sti           ( struct ivshmem_dev *dev );
toSt ivshmem_set_state     ( struct ivshmem_dev *dev, u32 state );
/* Signal state change and block for acknowledge. The nack field must be in the RW-page. The location is application
   specific and must be known to both sides. Take precautions not to deadlock! */
 u64 ivshmem_set_blocking  ( struct ivshmem_dev *dev, u32 state, volatile int *passing_flag );
void ivshmem_signal        ( struct ivshmem_dev *dev, u32 intno, u32 target );
toSt ivshmem_wait_for_state( struct ivshmem_dev *dev, u64 before_ns, u32 target, u32 *state );
toSt ivshmem_expect_state  ( struct ivshmem_dev *dev, u64 before_ns, u32 target, u32 expectation, u32 *state );
void ivshmem_tell    ( const struct ivshmem_dev *dev );

/* private helper */
 u32 ivshmem_read_device   ( struct ivshmem_dev *dev );
 int ivshmem_poll          ( struct ivshmem_dev *dev, u64 timeout );
 u64 ivshmem_check         ( struct ivshmem_dev *dev, u64 timeout );

/* this needs a better approach */
#ifdef JAILHOUSE
bool ivshmem_irq_handler( struct ivshmem_dev *dev, unsigned int irq );
#endif

/* When the client is in Linux, the SHM is allocated dynamically and we need to announce that to the driver.
 * It sets an internal variable, _before_ calling ivshmem_open(). Do not call from the service side. */
void ivshmem_reserve(size_t msize);

/* when using @fds set timeout in later calls to INMATE_NONBLOCK (=0) */
void ivshmem_open          ( struct ivshmem_dev *dev, int uio_no, struct ivshmem_cb *fds );
void ivshmem_finalize      ( struct ivshmem_dev *dev );

/* helper, search arg for number separated by sep */
int ivshmem_parse_ids(const char *arg, int * const num, int len, const char sep);

/* seems to come in helpful, though requires different "local" implementations. */
#ifndef JAILHOUSE
unsigned long tsc_read_ns( void );
#endif

/* only reasonable in multi-process OS, otherwise maps to tsc_read_ns */
unsigned long process_clock_ns( void );
unsigned long rtc_read_ns( void );

const char *snprint_bdf(char *bdfb, int l, int bdf);

/* This is a bit of a system-specific function to open a PikeOS-resource via special drivers. Bad place here, however
   needed by ivshmem-open.posix()  and trdpSvc. */
int open_p4_resource(const char *p4_res, int oflags, const char *configfn, const char *cdevfn, int idx);

#endif /* _H */
