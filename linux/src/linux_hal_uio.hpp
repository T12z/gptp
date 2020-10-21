/******************************************************************************

  Copyright (c) 2012, Intel Corporation
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

   3. Neither the name of the Intel Corporation nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.

******************************************************************************/

/* This is mostly cut&paste from linux_hal_common.hpp */

#ifndef LINUX_HAL_UIO_HPP
#define LINUX_HAL_UIO_HPP

/**@file*/

#include "avbts_osipc.hpp"
#include "ieee1588.hpp"
#include <linux_ipc.hpp>

extern "C" {
#include "ivshmem.h"
}

#define DEFAULT_UIO_NO 0          /*!< Default index for the UIO SHM device */
#define DEFAULT_UIO_NO_TARGET -1  /*!< If the peer should not be blocked for */

/**
 * @brief Extends IPC ARG generic interface to linux
 */
class UIOIPCArg : public OS_IPC_ARG {
private:
	int dev_no;
	int target;
public:
	/**
	 * @brief  Initializes IPCArg object
	 * @param dev_no    [in] UIO device index, ie., 0 for /dev/uio0
	 * @param target_id [in] index of the other SHM-peer. Leave to -1 if the IPC
	 *                       update should not block for the other peer to 
	 *                       ackowledge and measure the delay.
	 */
	UIOIPCArg( const char *dev_no, const int target_id = DEFAULT_UIO_NO_TARGET ) {
		int ret[2] = { 0, target_id};
		ivshmem_parse_ids(dev_no, ret, 2, '.');
		this->dev_no = ret[0];
		this->target = ret[1];
	}
	/**
	 * @brief Destroys IPCArg internal variables
	 */
	virtual ~UIOIPCArg() { }
	friend class UIOSharedMemoryIPC;
};


/**
 * @brief Linux shared memory interface
 */
class UIOSharedMemoryIPC:public OS_IPC {
private:
	struct ivshmem_dev dev;
	pthread_mutex_t *lock;
	gPtpTimeData *ptimedata;
	int32_t lstate;
	int32_t target;

public:
	/**
	 * @brief Initializes the internal flags
	 */
	UIOSharedMemoryIPC() {
		lock = NULL;
		ptimedata = NULL;
		lstate = 0x0;
		target = DEFAULT_UIO_NO_TARGET;
	};

	/**
	 * @brief Destroys and unlinks shared memory
	 */
	~UIOSharedMemoryIPC();

	/**
	 * @brief  Initializes shared memory with DEFAULT_UIO_NO case arg is null
	 * @param  barg number of the UIO device
	 * @return TRUE if no error, FALSE otherwise
	 */
	virtual bool init( OS_IPC_ARG *barg = NULL );

	/**
	 * @brief Updates IPC values
	 *
	 * @param ml_phoffset Master to local phase offset
	 * @param ls_phoffset Local to slave phase offset
	 * @param ml_freqoffset Master to local frequency offset
	 * @param ls_freqoffset Local to slave frequency offset
	 * @param local_time Local time
	 * @param sync_count Count of syncs
	 * @param pdelay_count Count of pdelays
	 * @param port_state Port's state
	 * @param asCapable asCapable flag
	 *
	 * @return TRUE
	 */
	virtual bool update(
		int64_t ml_phoffset,
		int64_t ls_phoffset,
		FrequencyRatio ml_freqoffset,
		FrequencyRatio ls_freqoffset,
		uint64_t local_time,
		uint32_t sync_count,
		uint32_t pdelay_count,
		PortState port_state,
		bool asCapable,
		uint64_t tsc);

	/**
	 * @brief Updates grandmaster IPC values
	 *
	 * @param gptp_grandmaster_id Current grandmaster id (all 0's if no grandmaster selected)
	 * @param gptp_domain_number gPTP domain number
	 *
	 * @return TRUE
	 */
	virtual bool update_grandmaster(
		uint8_t gptp_grandmaster_id[],
		uint8_t gptp_domain_number );

	/**
	 * @brief Updates network interface IPC values
	 *
	 * @param  clock_identity  The clock identity of the interface
	 * @param  priority1  The priority1 field of the grandmaster functionality of the interface, or 0xFF if not supported
	 * @param  clock_class  The clockClass field of the grandmaster functionality of the interface, or 0xFF if not supported
	 * @param  offset_scaled_log_variance  The offsetScaledLogVariance field of the grandmaster functionality of the interface, or 0x0000 if not supported
	 * @param  clock_accuracy  The clockAccuracy field of the grandmaster functionality of the interface, or 0xFF if not supported
	 * @param  priority2  The priority2 field of the grandmaster functionality of the interface, or 0xFF if not supported
	 * @param  domain_number  The domainNumber field of the grandmaster functionality of the interface, or 0 if not supported
	 * @param  log_sync_interval  The currentLogSyncInterval field of the grandmaster functionality of the interface, or 0 if not supported
	 * @param  log_announce_interval  The currentLogAnnounceInterval field of the grandmaster functionality of the interface, or 0 if not supported
	 * @param  log_pdelay_interval  The currentLogPDelayReqInterval field of the grandmaster functionality of the interface, or 0 if not supported
	 * @param  port_number  The portNumber field of the interface, or 0x0000 if not supported
	 *
	 * @return TRUE
	 */
	virtual bool update_network_interface(
		uint8_t  clock_identity[],
		uint8_t  priority1,
		uint8_t  clock_class,
		int16_t  offset_scaled_log_variance,
		uint8_t  clock_accuracy,
		uint8_t  priority2,
		uint8_t  domain_number,
		int8_t   log_sync_interval,
		int8_t   log_announce_interval,
		int8_t   log_pdelay_interval,
		uint16_t port_number );

	/**
	 * @brief unmaps and unlink shared memory
	 * @return void
	 */
	void stop();
};


#endif/*LINUX_HAL_UIO_HPP*/
