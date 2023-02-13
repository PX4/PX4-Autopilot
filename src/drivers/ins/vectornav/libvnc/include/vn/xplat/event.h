/** \file
 * {COMMON_HEADER}
 *
 * \section DESCRIPTION
 * This header file contains structures and functions useful events and signals.
 */
#ifndef _VNEVENT_H_
#define _VNEVENT_H_

#include "vn/int.h"
#include "vn/error.h"
#include "vn/bool.h"

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

/** \brief Structure representing an event. */
typedef struct
{
	#ifdef _WIN32
	HANDLE handle;
	#elif (defined __linux__ || defined __APPLE__ || defined __CYGWIN__ || defined __QNXNTO__)
	pthread_mutex_t mutex;
	pthread_cond_t condition;
	bool isTriggered;
	#else
		#error "Unknown System"
	#endif
} VnEvent;

/** \brief Initializes a VnEvent structure.
 *
 * \param[in] event The VnEvent structure to initialize.
 * \return Any errors encountered. */
VnError VnEvent_initialize(VnEvent *event);

/** \brief Causes the calling thread to wait on an event until the event is signalled.
 *
 * If the event is signalled, the value E_SIGNALED will be returned.
 *
 * \param[in] event The associated VnEvent.
 * \return Any errors encountered. */
VnError VnEvent_wait(VnEvent *event);

/** \brief Causes the calling thread to wait on an event until the event is signalled.
 *
 * If the event is signalled, the value E_SIGNALED will be returned.
 *
 * \param[in] event The associated VnEvent.
 * \param[in] timeoutUs The number of microseconds to wait before the thread stops waiting on the event. If a timeout
 *     does occur, the value E_TIMEOUT will be returned.
 * \return Any errors encountered. */
VnError VnEvent_waitUs(VnEvent *event, uint32_t timeoutUs);

/** \brief Causes the calling thread to wait on an event until the event is signalled.
 *
 * If the event is signalled, the value E_SIGNALED will be returned.
 *
 * \param[in] event The associated VnEvent.
 * \param[in] timeoutMs The number of milliseconds to wait before the thread stops waiting on the event. If a timeout
 *     does occur, the value E_TIMEOUT will be returned.
 * \return Any errors encountered. */
VnError VnEvent_waitMs(VnEvent *event, uint32_t timeoutMs);

/** \brief Signals an event.
 *
 * \param[in] event The associated event.
 * \return Any errors encountered. */
VnError VnEvent_signal(VnEvent *event);

#ifdef __cplusplus
}
#endif

#endif
