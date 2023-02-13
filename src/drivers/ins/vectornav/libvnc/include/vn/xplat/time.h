/** \file
* {COMMON_HEADER}
*
* \section DESCRIPTION
* This header file contains structures and functions useful timing.
*/
#ifndef _VNTIME_H_
#define _VNTIME_H_

#include "vn/int.h"
#include "vn/error.h"
#include "vn/enum.h"

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

#ifdef __cplusplus
extern "C" {
#endif

/** \brief Provides simple timing capabilities. */
typedef struct
{
	#if _WIN32
	double pcFrequency;
	__int64 counterStart;
	#elif __linux__ || __APPLE__ ||__CYGWIN__ || __QNXNTO__ || __NUTTX__
	double clockStart;
	#else
	#error "Unknown System"
	#endif
} VnStopwatch;

/** \brief Initializes and starts a stopwatch.
 *
 * \param[in] stopwatch The VnStopwatch to initialize and start.
 * \return Any errors encountered. */
VnError VnStopwatch_initializeAndStart(VnStopwatch *stopwatch);

/** \brief Resets the stopwatch's timing.
 *
 * \param[in] stopwatch The associated VnStopwatch.
 * \return Any errors encountered. */
VnError VnStopwatch_reset(VnStopwatch *stopwatch);

/** \brief Determines the number of milliseconds elapsed since the last reset
 *      of the stopwatch.
 *
 * \param[in] stopwatch The associated VnStopwatch.
 * \param[out] elapsedMs The elapsed time in milliseconds.
 * \return Any errors encountered. */
VnError VnStopwatch_elapsedMs(VnStopwatch *stopwatch, float *elapsedMs);

#ifdef __cplusplus
}
#endif

#endif
