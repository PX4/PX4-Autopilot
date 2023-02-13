/** \file
* {COMMON_HEADER}
*
* \section DESCRIPTION
* This header file contains structures and functions useful working with threads.
*/
#ifndef _VN_THREAD_H_
#define _VN_THREAD_H_

#include "vn/error.h"
#include "vn/int.h"

#ifdef _WIN32

	/* Disable some warnings for Visual Studio with -Wall. */
	#if defined(_MSC_VER)
		#pragma warning(push)
		#pragma warning(disable:4668)
		#pragma warning(disable:4820)
		#pragma warning(disable:4255)
	#endif

	#include <Windows.h>

	#if defined(_MSC_VER)
		#pragma warning(pop)
	#endif

#endif

#if defined __linux__ || defined __APPLE__ || defined __CYGWIN__ || defined __QNXNTO__
	#include <pthread.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** \brief Structure for working with threads. */
typedef struct
{
	#ifdef _WIN32
	HANDLE handle;
	#elif defined __linux__ || defined __APPLE__ || defined __CYGWIN__ || defined __QNXNTO__
	pthread_t handle;
	#else
		#error "Unknown System"
	#endif
} VnThread;

/** \brief Function signature for a start routine for a thread. */
typedef void (*VnThread_StartRoutine)(void*);

/** \brief Starts a new thread immediately which calls the provided start routine.
 *
 * \param[in] thread Associated VnThread structure.
 * \param[in] startRoutine The routine to be called when the new thread is started.
 * \param[in] routineData Pointer to data that will be passed to the routine on the new thread.
 * \return Any errors encountered. */
VnError VnThread_startNew(VnThread *thread, VnThread_StartRoutine startRoutine, void* routineData);

/** \brief Blocks the calling thread until the referenced thread finishes.
 *
 * \param[in] thread The associated VnThread.
 * \return Any errors encountered. */
VnError VnThread_join(VnThread *thread);

/** \brief Causes the calling thread to sleep the specified number of seconds.
 *
 * \param[in] numOfSecsToSleep The number of seconds to sleep. */
void VnThread_sleepSec(uint32_t numOfSecsToSleep);

/** \brief Causes the calling thread to sleep the specified number of milliseconds
*
* \param[in] numOfMsToSleep The number of milliseconds to sleep. */
void VnThread_sleepMs(uint32_t numOfMsToSleep);

/** \brief Causes the calling thread to sleep the specified number of microseconds
*
* \param[in] numOfUsToSleep The number of microseconds to sleep. */
void VnThread_sleepUs(uint32_t numOfUsToSleep);

#ifdef __cplusplus
}
#endif

#endif
