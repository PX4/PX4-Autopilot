#define _BSD_SOURCE

#include "vn/xplat/thread.h"

#if (defined __linux__ || defined __APPLE__ || defined __CYGWIN__ || defined __QNXNTO__ || defined __NUTTX__)
	#include <stdlib.h>
	#include <unistd.h>
	#include <stddef.h>
	#include <pthread.h>
	#include <sched.h>
#endif

#undef __cplusplus

typedef struct
{
	VnThread_StartRoutine startRoutine;
	void *routineData;
} VnThreadStarter;

#if _WIN32

DWORD WINAPI VnThread_intermediateStartRoutine(LPVOID userData)
{
	VnThreadStarter *starter = (VnThreadStarter*) userData;

	VnThread_StartRoutine routine = starter->startRoutine;
	void *routineData = starter->routineData;

	free(starter);

	routine(routineData);

	return 0;
}

#elif (defined __linux__ || defined __APPLE__ || defined __CYGWIN__ || defined __QNXNTO__ || defined __NUTTX__)

void* VnThread_intermediateStartRoutine(void* data)
{
	VnThreadStarter *starter = (VnThreadStarter*) data;

	VnThread_StartRoutine routine = starter->startRoutine;
	void *routineData = starter->routineData;

	free(starter);

	routine(routineData);

	return NULL;
}

#endif

VnError VnThread_startNew(VnThread *thread, VnThread_StartRoutine startRoutine, void* routineData)
{
	#if (defined __linux__ || defined __APPLE__ || defined __CYGWIN__ || defined __QNXNTO__ || defined __NUTTX__)
	int errorCode;
	#endif

	VnThreadStarter *starter = (VnThreadStarter*) malloc(sizeof(VnThreadStarter));

	starter->startRoutine = startRoutine;
	starter->routineData = routineData;

	#if _WIN32

	thread->handle = CreateThread(
		NULL,
		0,
		VnThread_intermediateStartRoutine,
		starter,
		0,
		NULL);

	if (thread->handle == NULL)
		return E_UNKNOWN;

	#elif (defined __linux__ || defined __APPLE__ || defined __CYGWIN__ || defined __QNXNTO__ || defined __NUTTX__)

	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setstacksize(&attr, 5120);

	// schedule policy FIFO
	pthread_attr_setschedpolicy(&attr, SCHED_FIFO);

	// priority
	struct sched_param param;
	pthread_attr_getschedparam(&attr, &param);
	param.sched_priority = sched_get_priority_max(SCHED_FIFO);
	pthread_attr_setschedparam(&attr, &param);

	errorCode = pthread_create(
		&(thread->handle),
		&attr,
		VnThread_intermediateStartRoutine,
		starter);

	if (errorCode != 0)
		return E_UNKNOWN;

	#else
	#error "Unknown System"
	#endif

	return E_NONE;
}

VnError VnThread_join(VnThread *thread)
{
	#if _WIN32

	WaitForSingleObject(
		thread->handle,
		INFINITE);

	#elif (defined __linux__ || defined __APPLE__ || defined __CYGWIN__ || defined __QNXNTO__ || defined __NUTTX__)

	int error = pthread_join(
		thread->handle,
		NULL);

	if (error != 0)
		return E_UNKNOWN;

	#else
	#error "Unknown System"
	#endif

	return E_NONE;
}

void VnThread_sleepSec(uint32_t numOfSecsToSleep)
{
	#if _WIN32
	Sleep(numOfSecsToSleep * 1000);
	#elif (defined __linux__ || defined __APPLE__ || defined __CYGWIN__ || defined __QNXNTO__ || defined __NUTTX__)
	sleep(numOfSecsToSleep);
	#else
	#error "Unknown System"
	#endif
}

void VnThread_sleepMs(uint32_t numOfMsToSleep)
{
	#if _WIN32
	Sleep(numOfMsToSleep);
	#elif (defined __linux__ || defined __APPLE__ || defined __CYGWIN__ || defined __QNXNTO__ || defined __NUTTX__)
	VnThread_sleepUs(numOfMsToSleep * 1000);
	#else
	#error "Unknown System"
	#endif
}

void VnThread_sleepUs(uint32_t numOfUsToSleep)
{
	#if _WIN32
	/* Not implemented. */
	int exitValue = numOfUsToSleep;
	exitValue = -1;
	exit(exitValue);

	#elif (defined __linux__ || defined __APPLE__ || defined __CYGWIN__ || defined __QNXNTO__ || defined __NUTTX__)
	usleep(numOfUsToSleep);

	#else
	#error "Unknown System"
	#endif
}
