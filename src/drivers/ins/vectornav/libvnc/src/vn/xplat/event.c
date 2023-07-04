/* Enable IEEE Std 1003.1b-1993 functionality required for clock_gettime. */
#if defined(__linux__) || defined(__NUTTX__)
	/* Works for Ubuntu 15.10 */
	#define _POSIX_C_SOURCE 199309L
#elif defined __CYGWIN__
	/* Works for Cygwin 2.4.0 64-bit */
	#define _POSIX_TIMERS 	1
#endif

#include "vn/xplat/event.h"

#if defined __linux__ || defined __APPLE__ || defined __CYGWIN__ || defined __QNXNTO__ || defined __NUTTX__
	#include <time.h>
	#include <errno.h>
#endif

#ifdef __APPLE__
	#include <sys/time.h>
	#include <pthread.h>
	#include <mach/clock.h>
	#include <mach/mach.h>
#endif

VnError VnEvent_initialize(VnEvent *e)
{
	#ifdef _WIN32

	e->handle = CreateEvent(
		NULL,
		FALSE,
		FALSE,
		NULL);

	if (e->handle == NULL)
		return E_UNKNOWN;

	#elif defined __linux__ || defined __APPLE__ || defined __CYGWIN__ || defined __QNXNTO__ || defined __NUTTX__

	if (pthread_mutex_init(&e->mutex, NULL))
		return E_UNKNOWN;

	if (pthread_cond_init(&e->condition, NULL))
		return E_UNKNOWN;

	e->isTriggered = false;

	#else
		#error "Unknown System"
	#endif

	return E_NONE;
}

VnError VnEvent_wait(VnEvent *e)
{
	#ifdef _WIN32

	DWORD result;

	result = WaitForSingleObject(e->handle, 0xffffffff);

	if (result == WAIT_OBJECT_0)
		return E_NONE;

	return E_UNKNOWN;

	#elif defined __linux__ || defined __APPLE__ || defined __CYGWIN__ || defined __QNXNTO__ || defined __NUTTX__

	if (pthread_mutex_lock(&e->mutex))
		return E_UNKNOWN;

	if (pthread_cond_wait(
		&e->condition,
		&e->mutex))
		return E_UNKNOWN;

	if (pthread_mutex_unlock(&e->mutex))
		return E_UNKNOWN;

	return E_NONE;

	#else
		#error "Unknown System"
	#endif
}

VnError VnEvent_waitMs(VnEvent *e, uint32_t timeoutMs)
{
	#ifdef _WIN32

	DWORD result;

	result = WaitForSingleObject(e->handle, timeoutMs);

	if (result == WAIT_OBJECT_0)
		return E_SIGNALED;

	if (result == WAIT_TIMEOUT)
		return E_TIMEOUT;

	return E_UNKNOWN;

	#elif defined __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__ || __NUTTX__

	return VnEvent_waitUs(e, timeoutMs * 1000);

	#else
	#error "Unknown System"
	#endif
}

VnError VnEvent_waitUs(VnEvent *e, uint32_t timeoutUs)
{
	#ifdef _WIN32

	DWORD result;

	result = WaitForSingleObject(
		e->handle,
		timeoutUs / 1000);

	if (result == WAIT_OBJECT_0)
		return E_SIGNALED;

	if (result == WAIT_TIMEOUT)
		return E_TIMEOUT;

#elif (defined __linux__ || defined __CYGWIN__ || defined __QNXNTO__ || defined __NUTTX__)

	struct timespec now;
	int errorCode;
	uint32_t numOfSecs, numOfNanoseconds;

	if (pthread_mutex_lock(&e->mutex))
		return E_UNKNOWN;

	if (clock_gettime(CLOCK_REALTIME, &now))
		return E_UNKNOWN;

	numOfSecs = timeoutUs / 1000000;
	numOfNanoseconds = (timeoutUs % 1000000) * 1000;

	now.tv_sec += numOfSecs;
	now.tv_nsec += numOfNanoseconds;

	if (now.tv_nsec > 1000000000)
	{
		now.tv_nsec %= 1000000000;
		now.tv_sec++;
	}

	errorCode = pthread_cond_timedwait(
		&e->condition,
		&e->mutex,
		&now);

	if (pthread_mutex_unlock(&e->mutex))
		return E_UNKNOWN;

	if (!errorCode)
		return E_SIGNALED;

	if (errorCode == ETIMEDOUT)
		return E_TIMEOUT;

	#elif defined __APPLE__

	pthread_mutex_lock(&e->mutex);

	clock_serv_t cclock;
	mach_struct timespec_t mts;
	host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cclock);
	clock_get_time(cclock, &mts);
	mach_port_deallocate(mach_task_self(), cclock);

	struct timespec now;
	now.tv_sec = mts.tv_sec;
	now.tv_nsec = mts.tv_nsec;

	uint32_t numOfSecs = timeoutUs / 1000000;
	uint32_t numOfNanoseconds = (timeoutUs % 1000000) * 1000;

	now.tv_sec += numOfSecs;
	now.tv_nsec += numOfNanoseconds;

	if (now.tv_nsec > 1000000000)
	{
		now.tv_nsec %= 1000000000;
		now.tv_sec++;
	}

	int errorCode = pthread_cond_timedwait(
		&e->condition,
		&e->mutex,
		&now);

	pthread_mutex_unlock(&e->mutex);

	if (errorCode == 0)
		return E_SIGNALED;

	if (errorCode == ETIMEDOUT)
		return E_TIMEOUT;

	#else

	#error "Unknown System"

	#endif

	return E_UNKNOWN;
}

VnError VnEvent_signal(VnEvent *e)
{
	#ifdef _WIN32

	if (!SetEvent(e->handle))
		return E_UNKNOWN;

	#elif defined __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__ || __NUTTX__

	if (pthread_mutex_lock(&e->mutex))
		return E_UNKNOWN;

	e->isTriggered = true;

	if (pthread_cond_signal(&e->condition))
		return E_UNKNOWN;

	if (pthread_mutex_unlock(&e->mutex))
		return E_UNKNOWN;

	#else
	#error "Unknown System"
	#endif

	return E_NONE;
}
