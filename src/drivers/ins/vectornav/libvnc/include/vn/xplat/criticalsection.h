/** \file
* {COMMON_HEADER}
*
* \section DESCRIPTION
* This header file contains structures and functions useful for critical sections.
*/
#ifndef _VN_CRITICALSECTION_H_
#define _VN_CRITICALSECTION_H_

#include "vn/error.h"

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

#if (defined __linux__ || defined __APPLE__ || defined __CYGWIN__ || defined __QNXNTO__)
	#include <pthread.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
	#if _WIN32
	CRITICAL_SECTION handle;
	#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__
	pthread_mutex_t handle;
	#else
	#error "Unknown System"
	#endif
} VnCriticalSection;

/** \breif Initializes a VnCriticalSection structure.
 *
 * \param[in] criticalSection The VnCriticalSection structure to initialize.
 * \return Any errors encountered. */
VnError VnCriticalSection_initialize(VnCriticalSection *criticalSection);

/** \brief Disposes of a VnCriticalSection structure and associated resources.
 *
 * \param[in] criticalSection The associated VnCriticalSection structure.
 * \return Any errors encountered. */
VnError VnCriticalSection_deinitialize(VnCriticalSection *criticalSection);

/** \brief Attempt to enter a critical section.
 *
 * \param[in] criticalSection The associated VnCriticalSection structure.
 * \return Any errors encountered. */
VnError VnCriticalSection_enter(VnCriticalSection *criticalSection);

/** \brief Leave a critical section.
*
* \param[in] criticalSection The associated VnCriticalSection structure.
* \return Any errors encountered. */
VnError VnCriticalSection_leave(VnCriticalSection *criticalSection);

#ifdef __cplusplus
}
#endif

#endif
