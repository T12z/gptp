
#include <fcntl.h>
#include <unistd.h>
#include <cstddef>
#include <sys/types.h>
#include <sys/mman.h>
#include <pthread.h>

#include <linux_hal_uio.hpp>

UIOSharedMemoryIPC::~UIOSharedMemoryIPC() {
	ivshmem_finalize(&dev);
}

bool UIOSharedMemoryIPC::init( OS_IPC_ARG *barg ) {
	int dev_no = DEFAULT_UIO_NO;
	pthread_mutexattr_t shared;

	if( barg ) {
		UIOIPCArg *arg = dynamic_cast<UIOIPCArg *> (barg);
		if( arg == NULL ) {
			GPTP_LOG_ERROR( "Wrong IPC init arg type (implementation fault)." );
			return false;
		} else {
			dev_no = arg->dev_no;
		}
	}

	/*create mutex attr */
	if (0 != pthread_mutexattr_init(&shared))
		error(1, errno, "mutex attr initialization failed.");
	pthread_mutexattr_setpshared(&shared, 1);

	/* any failure, other than the UIO dev missing, will halt the whole program */
	ivshmem_open(&dev, dev_no, NULL);
	if ( !dev.rw ) return false;

	lock = (pthread_mutex_t *)dev.rw;

	/* create a mutex */
	if (0 != pthread_mutex_init(lock, &shared))
		error(1, errno, "sharedmem - Mutex initialization failed.");

	ptimedata = (gPtpTimeData *)dev.out;
	return true;
}

/* could be inherited from LinuxSHMIPC, however, the mutex and the data should go to different pages */
bool UIOSharedMemoryIPC::update(
	int64_t ml_phoffset,
	int64_t ls_phoffset,
	FrequencyRatio ml_freqoffset,
	FrequencyRatio ls_freqoffset,
	uint64_t local_time,
	uint32_t sync_count,
	uint32_t pdelay_count,
	PortState port_state,
	bool asCapable,
	uint64_t tsc)
{
	pid_t process_id = getpid();
	uint64_t iv_reception_tsc;
	if( ptimedata != NULL ) {
		pthread_mutex_lock( lock );
			ptimedata->ml_phoffset  = ml_phoffset;
			ptimedata->ls_phoffset  = ls_phoffset;
			ptimedata->ml_freqoffset= ml_freqoffset;
			ptimedata->ls_freqoffset= ls_freqoffset;
			ptimedata->local_time   = local_time;
			ptimedata->sync_count   = sync_count;
			ptimedata->pdelay_count = pdelay_count;
			ptimedata->asCapable    = asCapable;
			ptimedata->port_state   = port_state;
			ptimedata->process_id   = process_id;
			ptimedata->x_tsc        = tsc;
		pthread_mutex_unlock( lock );
		state += 0x100;
		iv_reception_tsc = ivshmem_set_blocking( &dev, state, &lock->__data.__lock );
		pthread_mutex_lock( lock );
			/* I could update the delta throughout blocking, however, being non-atomic operations, would lead to data races. */
			ptimedata->xiv_tsc_offset = iv_reception_tsc - ptimedata->x_tsc;
		pthread_mutex_unlock( lock );
	}
	return true;
}

bool UIOSharedMemoryIPC::update_grandmaster(
	uint8_t gptp_grandmaster_id[],
	uint8_t gptp_domain_number )
{
	if( ptimedata ) {
		pthread_mutex_lock( lock );
			memcpy(ptimedata->gptp_grandmaster_id, gptp_grandmaster_id, PTP_CLOCK_IDENTITY_LENGTH);
			ptimedata->gptp_domain_number = gptp_domain_number;
		pthread_mutex_unlock( lock );
	}
	return true;
}

bool UIOSharedMemoryIPC::update_network_interface(
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
	uint16_t port_number )
{
	if( ptimedata ) {
		pthread_mutex_lock( lock );
			memcpy(ptimedata->clock_identity, clock_identity, PTP_CLOCK_IDENTITY_LENGTH);
			ptimedata->priority1 = priority1;
			ptimedata->clock_class = clock_class;
			ptimedata->offset_scaled_log_variance = offset_scaled_log_variance;
			ptimedata->clock_accuracy = clock_accuracy;
			ptimedata->priority2 = priority2;
			ptimedata->domain_number = domain_number;
			ptimedata->log_sync_interval = log_sync_interval;
			ptimedata->log_announce_interval = log_announce_interval;
			ptimedata->log_pdelay_interval = log_pdelay_interval;
			ptimedata->port_number   = port_number;
		pthread_mutex_unlock( lock );
	}
	return true;
}

void UIOSharedMemoryIPC::stop() {
	if( ptimedata ) {
		ivshmem_finalize( &dev );
		ptimedata = NULL;
	}
}
