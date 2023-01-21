/* Enable IEEE Std 1003.1b-1993 functionality required for clock_gettime. */
#ifdef __linux__
	/* Works for Ubuntu 15.10 */
	#define _POSIX_C_SOURCE 199309L
#elif defined __CYGWIN__
	/* Works for Cygwin 2.4.0 64-bit */
	#define _POSIX_TIMERS 				1
	#define _POSIX_MONOTONIC_CLOCK		200112L
#endif

#include "vn/xplat/time.h"

#if (defined __linux__ || defined __CYGWIN__ || defined __QNXNTO__ || defined __NUTTX__)
	#include <time.h>
#elif defined __APPLE__
	#include <mach/clock.h>
	#include <mach/mach.h>
#endif

VnError VnStopwatch_initializeAndStart(VnStopwatch *sw)
{
	#if _WIN32

	sw->pcFrequency = 0;
	sw->counterStart = -1;

	#elif (defined __linux__ || defined __APPLE__ || defined __CYGWIN__ || defined __QNXNTO__ || defined __NUTTX__)

	sw->clockStart = -1;

	#else
	#error "Unknown System"
	#endif

	/* Start the stopwatch. */
	return VnStopwatch_reset(sw);
}

VnError VnStopwatch_reset(VnStopwatch *sw)
{
	#if _WIN32

	LARGE_INTEGER li;
	if(!QueryPerformanceFrequency(&li))
		/* The hardware must not support a high-resolution performance counter. */
		return E_NOT_SUPPORTED;

	sw->pcFrequency = li.QuadPart / 1000.0;

	QueryPerformanceCounter(&li);

	sw->counterStart = li.QuadPart;

	#elif (defined __linux__ || defined __CYGWIN__ || defined __QNXNTO__ || defined __NUTTX__)

	struct timespec time;
	int error;

	error = clock_gettime(CLOCK_MONOTONIC, &time);

	if (error)
		return E_UNKNOWN;

	sw->clockStart = (time.tv_sec * 1000.0) + (time.tv_nsec / 1000000.0);

	#elif defined __APPLE__

	clock_serv_t cclock;
	mach_struct timespec_t mts;

	host_get_clock_service(mach_host_self(), SYSTEM_CLOCK, &cclock);
	clock_get_time(cclock, &mts);
	mach_port_deallocate(mach_task_self(), cclock);

	sw->clockStart = (mts.tv_sec * 1000.0) + (mts.tv_nsec / 1000000.0);

	#else
	#error "Unknown System"
	#endif

	return E_NONE;
}

VnError VnStopwatch_elapsedMs(VnStopwatch *sw, float *elapsedMs)
{
	#if _WIN32

	LARGE_INTEGER li;

	if (sw->counterStart == -1)
		return E_UNKNOWN;

	QueryPerformanceCounter(&li);

	*elapsedMs = (float) ((li.QuadPart - sw->counterStart) / sw->pcFrequency);

	#elif (defined __linux__ || defined __CYGWIN__ || defined __QNXNTO__ || defined __NUTTX__)

	struct timespec time;
	int error;

	if (sw->clockStart < 0)
		/* Clock not started. */
		return E_INVALID_OPERATION;

	error = clock_gettime(CLOCK_MONOTONIC, &time);

	if (error)
		return E_UNKNOWN;

	*elapsedMs = (time.tv_sec * 1000.0) + (time.tv_nsec / 1000000.0) - sw->clockStart;

	#elif defined __APPLE__

	clock_serv_t cclock;
	mach_struct timespec_t mts;

	host_get_clock_service(mach_host_self(), SYSTEM_CLOCK, &cclock);
	clock_get_time(cclock, &mts);
	mach_port_deallocate(mach_task_self(), cclock);

	return (mts.tv_sec * 1000.0) + (mts.tv_nsec / 1000000.0) - sw->clockStart;

	#else
	#error "Unknown System"
	#endif

	return E_NONE;
}
