/*************************************************************************//**
 * @file
 * @brief       This file is part of the AFBR-S50 API.
 * @details     This file provides utility functions for timing necessities.
 *
 * @copyright
 *
 * Copyright (c) 2023, Broadcom Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#ifndef TIME_H
#define TIME_H
#ifdef __cplusplus
extern "C" {
#endif

/*!***************************************************************************
 * @defgroup    argus_time Time Utility
 * @ingroup     argus_util
 *
 * @brief       Timer utilities for time measurement duties.
 *
 * @details     This module provides time measurement utility functions like
 *              delay or time measurement methods, or time math functions.
 *
 * @addtogroup  argus_time
 * @{
 *****************************************************************************/

#include "platform/argus_timer.h"
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>

/*!***************************************************************************
 * @brief   A data structure to represent current time.
 *
 * @details Value is obtained from the PIT time which must be configured as
 *          lifetime counter.
 *
 *          Range: [0.000000, 4294967296.999999] seconds
 *****************************************************************************/
typedef struct ltc_t {
	/*! Seconds;
	 *  Range: [0, UINT32_MAX] seconds */
	uint32_t sec;

	/*! Microseconds;
	 *  Range: [0, 999999] microseconds */
	uint32_t usec;

} ltc_t;

/*!***************************************************************************
 * @brief   Converts #ltc_t to microseconds (uint32_t).
 * @details The specified time value (type #ltc_t) is converted to microseconds.
 *          The value is truncated to UINT32_MAX value if the result would
 *          exceed UINT32_MAX microseconds.
 * @param   t Input #ltc_t structure.
 * @return  Time value in microseconds.
 *****************************************************************************/
inline uint32_t Time_ToUSec(ltc_t const *t)
{
	assert(t != 0);

	// max. value to convert correctly is 4294.967295 sec (UINT32_MAX/1000000)
	return ((t->sec < 4294U) || (t->sec == 4294U && t->usec < 967295U)) ?
	       t->usec + t->sec * 1000000U : UINT32_MAX;
}

/*!***************************************************************************
 * @brief   Converts #ltc_t to milliseconds (uint32_t).
 * @details The specified time value (type #ltc_t) is converted to milliseconds.
 *          The value is truncated to UINT32_MAX value if the result would
 *          exceed UINT32_MAX milliseconds.
 *          The returned value is correctly rounded to the nearest value.
 * @param   t Input #ltc_t structure.
 * @return  Time value in milliseconds.
 *****************************************************************************/
inline uint32_t Time_ToMSec(ltc_t const *t)
{
	assert(t != 0);

	// max. value to convert correctly is 4294967.295499 sec (UINT32_MAX/1000)
	return ((t->sec < 4294967U) || (t->sec == 4294967U && t->usec < 295500U)) ?
	       (t->usec + 500U) / 1000U + t->sec * 1000U : UINT32_MAX;
}

/*!***************************************************************************
 * @brief   Converts #ltc_t to seconds (uint32_t).
 * @details The specified time value (type #ltc_t) is converted to seconds.
 *          The returned value is correctly rounded to the nearest value.
 * @param   t Input #ltc_t structure.
 * @return  Time value in seconds.
 *****************************************************************************/
inline uint32_t Time_ToSec(ltc_t const *t)
{
	assert(t != 0);

	// max. value to convert correctly is 4294967295.499999 sec (UINT32_MAX/1000)
	return (t->sec < 4294967295U || t->usec < 500000U) ?
	       (t->usec + 500000U) / 1000000U + t->sec : UINT32_MAX;
}

/*!***************************************************************************
 * @brief   Converts microseconds (uint32_t) to #ltc_t.
 * @details The specified time value in microseconds is converted to type #ltc_t.
 * @param   t Output #ltc_t structure.
 * @param   t_usec Input time in microseconds.
 *****************************************************************************/
inline void Time_FromUSec(ltc_t *t, uint32_t t_usec)
{
	assert(t != 0);
	t->sec = t_usec / 1000000U;
	t->usec = t_usec % 1000000U;
}

/*!***************************************************************************
 * @brief   Converts milliseconds (uint32_t) to #ltc_t.
 * @details The specified time value in milliseconds is converted to type #ltc_t.
 * @param   t Output #ltc_t structure.
 * @param   t_msec Input time in milliseconds.
 *****************************************************************************/
inline void Time_FromMSec(ltc_t *t, uint32_t t_msec)
{
	assert(t != 0);
	t->sec = t_msec / 1000U;
	t->usec = (t_msec % 1000U) * 1000U;
}

/*!***************************************************************************
 * @brief   Converts seconds (uint32_t) to #ltc_t.
 * @details The specified time value in seconds is converted to type #ltc_t.
 * @param   t Output #ltc_t structure.
 * @param   t_sec Input time in seconds.
 *****************************************************************************/
inline void Time_FromSec(ltc_t *t, uint32_t t_sec)
{
	assert(t != 0);
	t->usec = 0;
	t->sec = t_sec;
}


/*!***************************************************************************
 * @brief   Checks if /p t1 is greater or equal that /p t2.
 * @details Handles overflow.
 * @param   t1 1st operand.
 * @param   t2 2nd operand.
 * @return  Returns (t1 >= t2);
 *****************************************************************************/
inline bool Time_GreaterEqual(ltc_t const *t1, ltc_t const *t2)
{
	assert(t1 != 0);
	assert(t2 != 0);
	return (t1->sec == t2->sec) ? (t1->usec >= t2->usec) : (t1->sec > t2->sec);
}


/*!***************************************************************************
 * @brief   Obtains the elapsed time since MCU startup.
 * @param   t_now returned current time
 *****************************************************************************/
inline void Time_GetNow(ltc_t *t_now)
{
	assert(t_now != 0);
	Timer_GetCounterValue(&(t_now->sec), &(t_now->usec));
	assert(t_now->usec < 1000000U);
}

/*!***************************************************************************
 * @brief   Obtains the elapsed time since MCU startup.
 * @return  Returns the current time.
 *****************************************************************************/
inline ltc_t Time_Now(void)
{
	ltc_t t_now;
	Time_GetNow(&t_now);
	return t_now;
}

/*!***************************************************************************
 * @brief   Obtains the elapsed microseconds since MCU startup.
 * @details Wrap around effect due to uint32_t result format!!
 * @return  Elapsed microseconds since MCU startup as uint32_t.
 *****************************************************************************/
inline uint32_t Time_GetNowUSec(void)
{
	ltc_t t_now = Time_Now();
	return Time_ToUSec(&t_now);
}

/*!***************************************************************************
 * @brief   Obtains the elapsed milliseconds (rounded) since MCU startup.
 * @details Wrap around effect due to uint32_t result format!!
 * @return  Elapsed milliseconds since MCU startup as uint32_t.
 *****************************************************************************/
inline uint32_t Time_GetNowMSec(void)
{
	ltc_t t_now = Time_Now();
	return Time_ToMSec(&t_now);
}

/*!***************************************************************************
 * @brief   Obtains the elapsed seconds (rounded) since MCU startup.
 * @return  Elapsed seconds since MCU startup as uint32_t.
 *****************************************************************************/
inline uint32_t Time_GetNowSec(void)
{
	ltc_t t_now = Time_Now();
	return Time_ToSec(&t_now);
}


/*!***************************************************************************
 * @brief   Obtains the time difference between two given time points.
 * @details Result is defined as t_diff = t_end - t_start.
 *          Note: since no negative time differences are supported, t_end has
 *          to be later/larger than t_start. Otherwise, the result is undefined!
 * @param   t_diff Returned time difference.
 * @param   t_start Start time point.
 * @param   t_end End time point.
 *****************************************************************************/
inline void Time_Diff(ltc_t *t_diff, ltc_t const *t_start, ltc_t const *t_end)
{
	assert(t_diff != 0);
	assert(t_start != 0);
	assert(t_end != 0);
	assert(t_diff != t_start);
	assert(t_diff != t_end);
	assert(Time_GreaterEqual(t_end, t_start));

	if (t_start->usec <= t_end->usec) { // no carry over
		t_diff->sec = t_end->sec - t_start->sec;
		t_diff->usec = t_end->usec - t_start->usec;

	} else { // with carry over
		t_diff->sec = t_end->sec - 1 - t_start->sec;
		t_diff->usec = (1000000U - t_start->usec) + t_end->usec;
	}
}

/*!***************************************************************************
 * @brief   Obtains the time difference between two given time points in
 *          microseconds.
 * @details Result is defined as t_diff = t_end - t_start.
 *          Refers to Time_Diff() and handles overflow such that to large
 *          values are limited by 0xFFFFFFFF µs.
 * @param   t_start Start time point.
 * @param   t_end End time point.
 * @return  Time difference in microseconds.
 *****************************************************************************/
inline uint32_t Time_DiffUSec(ltc_t const *t_start, ltc_t const *t_end)
{
	ltc_t t_diff;
	Time_Diff(&t_diff, t_start, t_end);
	return Time_ToUSec(&t_diff);
}

/*!***************************************************************************
 * @brief   Obtains the time difference between two given time points in
 *          milliseconds.
 * @details Result is defined as t_diff = t_end - t_start.
 *          Refers to Time_Diff() and handles overflow.
 *          Wrap around effect due to uint32_t result format!!
 * @param   t_start Start time point.
 * @param   t_end End time point.
 * @return  Time difference in milliseconds.
 *****************************************************************************/
inline uint32_t Time_DiffMSec(ltc_t const *t_start, ltc_t const *t_end)
{
	ltc_t t_diff;
	Time_Diff(&t_diff, t_start, t_end);
	return Time_ToMSec(&t_diff);
}

/*!***************************************************************************
 * @brief   Obtains the time difference between two given time points in
 *          seconds.
 * @details Result is defined as t_diff = t_end - t_start.
 *          Refers to Time_Diff() and handles overflow.
 * @param   t_start Start time point.
 * @param   t_end End time point.
 * @return  Time difference in seconds.
 *****************************************************************************/
inline uint32_t Time_DiffSec(ltc_t const *t_start, ltc_t const *t_end)
{
	ltc_t t_diff;
	Time_Diff(&t_diff, t_start, t_end);
	return Time_ToSec(&t_diff);
}


/*!***************************************************************************
 * @brief   Obtains the elapsed time since a given time point.
 * @details Calculates the currently elapsed time since a specified start time
 *          (/p t_start).
 *
 *          Note that /p t_start must be in the past! Otherwise, the behavior is
 *          undefined!
 *
 * @param   t_elapsed Returns the elapsed time since /p t_start.
 * @param   t_start Start time point.
 *****************************************************************************/
inline void Time_GetElapsed(ltc_t *t_elapsed, ltc_t const *t_start)
{
	assert(t_elapsed != 0);
	assert(t_start != 0);
	assert(t_elapsed != t_start);
	ltc_t t_now = Time_Now();
	Time_Diff(t_elapsed, t_start, &t_now);
}

/*!***************************************************************************
 * @brief   Obtains the elapsed microseconds since a given time point.
 * @details Wrap around effect due to uint32_t result format!!
 * @param   t_start Start time point.
 * @return  Elapsed microseconds since t_start as uint32_t.
 *****************************************************************************/
inline uint32_t Time_GetElapsedUSec(ltc_t const *t_start)
{
	assert(t_start != 0);
	ltc_t t_now = Time_Now();
	return Time_DiffUSec(t_start, &t_now);
}

/*!***************************************************************************
 * @brief   Obtains the elapsed milliseconds since a given time point.
 * @details Wrap around effect due to uint32_t result format!!
 * @param   t_start Start time point.
 * @return  Elapsed milliseconds since t_start as uint32_t.
 *****************************************************************************/
inline uint32_t Time_GetElapsedMSec(ltc_t const *t_start)
{
	assert(t_start != 0);
	ltc_t t_now = Time_Now();
	return Time_DiffMSec(t_start, &t_now);
}

/*!***************************************************************************
 * @brief   Obtains the elapsed seconds since a given time point.
 * @param   t_start Start time point.
 * @return  Elapsed seconds since t_start as uint32_t.
 *****************************************************************************/
inline uint32_t Time_GetElapsedSec(ltc_t const *t_start)
{
	assert(t_start != 0);
	ltc_t t_now = Time_Now();
	return Time_DiffSec(t_start, &t_now);
}


/*!***************************************************************************
 * @brief   Adds two #ltc_t values.
 * @details Result is defined as t = t1 + t2.
 *          The results are wrapped around at maximum values just like integers.
 *          The references for t, t1 and t2 may point to the same instance(s).
 *
 * @param   t Return value: t = t1 + t2.
 * @param   t1 1st operand.
 * @param   t2 2nd operand.
 *****************************************************************************/
inline void Time_Add(ltc_t *t, ltc_t const *t1, ltc_t const *t2)
{
	assert(t != 0);
	assert(t1 != 0);
	assert(t2 != 0);

	t->sec = t1->sec + t2->sec;
	t->usec = t1->usec + t2->usec;

	if (t->usec > 999999U) {
		t->sec += 1U;
		t->usec -= 1000000U;
	}
}

/*!***************************************************************************
 * @brief   Adds a given time in microseconds to an #ltc_t value.
 * @details Result is defined as t = t1 + t2.
 *          The results are wrapped around at maximum values just like integers.
 *          The references for t and t1 may point to the same instance.
 *
 * @param   t Return value: t = t1 + t2.
 * @param   t1 1st operand.
 * @param   t2_usec 2nd operand in microseconds.
 *****************************************************************************/
inline void Time_AddUSec(ltc_t *t, ltc_t const *t1, uint32_t t2_usec)
{
	assert(t != 0);
	assert(t1 != 0);
	ltc_t t2;
	Time_FromUSec(&t2, t2_usec);
	Time_Add(t, t1, &t2);
}

/*!***************************************************************************
 * @brief   Adds a given time in milliseconds to an #ltc_t value.
 * @details Result is defined as t = t1 + t2.
 *          The results are wrapped around at maximum values just like integers.
 *          The references for t and t1 may point to the same instance.
 *
 * @param   t Return value: t = t1 + t2.
 * @param   t1 1st operand.
 * @param   t2_msec 2nd operand in milliseconds.
 *****************************************************************************/
inline void Time_AddMSec(ltc_t *t, ltc_t const *t1, uint32_t t2_msec)
{
	assert(t != 0);
	assert(t1 != 0);
	ltc_t t2;
	Time_FromMSec(&t2, t2_msec);
	Time_Add(t, t1, &t2);
}

/*!***************************************************************************
 * @brief   Adds a given time in seconds to an #ltc_t value.
 * @details Result is defined as t = t1 + t2.
 *          The results are wrapped around at maximum values just like integers.
 *          The references for t and t1 may point to the same instance.
 *
 * @param   t Return value: t = t1 + t2.
 * @param   t1 1st operand.
 * @param   t2_sec 2nd operand in seconds.
 *****************************************************************************/
inline void Time_AddSec(ltc_t *t, ltc_t const *t1, uint32_t t2_sec)
{
	assert(t != 0);
	assert(t1 != 0);
	ltc_t t2;
	Time_FromSec(&t2, t2_sec);
	Time_Add(t, t1, &t2);
}


/*!***************************************************************************
 * @brief   Checks if /p t is within the time interval /p t_start and /p t_end.
 * @details The interval is from /p t_start to /p t_end.
 *          The function returns true if /p t >= /p t_start AND /p t < /p t_end.
 *          If /p t_end is before /p t_start, /p t_end is consider to be wrapped
 *          around and the condition inverts (i.e. the function returns true if
 *          /p < /p t_end OR /p t >= t_start.
 * @param   t_start The start of the time interval.
 * @param   t_end The end of the time interval.
 * @param   t The time to be checked if it is with the interval.
 * @return  True if t is within t_start and t_stop.
 *****************************************************************************/
inline bool Time_CheckWithin(ltc_t const *t_start, ltc_t const *t_end, ltc_t const *t)
{
	if (Time_GreaterEqual(t_end, t_start)) {
		return Time_GreaterEqual(t, t_start) && !Time_GreaterEqual(t, t_end);

	} else {
		return Time_GreaterEqual(t, t_start) || !Time_GreaterEqual(t, t_end);
	}
}


/*!***************************************************************************
 * @brief   Checks if timeout is reached from a given starting time.
 * @details Checks if a specified time (/p t_timeout) has elapsed since a
 *          specified start time (/p t_start).
 *          Handles overflow/wraparound of time values at the maximum value.
 * @param   t_start Start time.
 * @param   t_timeout Timeout period.
 * @return  Timeout elapsed? True/False (boolean value)
 *****************************************************************************/
inline bool Time_CheckTimeout(ltc_t const *t_start, ltc_t const *t_timeout)
{
	assert(t_start != 0);
	assert(t_timeout != 0);

	ltc_t t_end;
	ltc_t t_now = Time_Now();
	Time_Add(&t_end, t_start, t_timeout);
	return !Time_CheckWithin(t_start, &t_end, &t_now);
}

/*!***************************************************************************
 * @brief   Checks if timeout is reached from a given starting time.
 * @details Handles overflow.
 * @param   t_start Start time.
 * @param   t_timeout_usec Timeout period in microseconds.
 * @return  Timeout elapsed? True/False (boolean value)
 *****************************************************************************/
inline bool Time_CheckTimeoutUSec(ltc_t const *t_start, uint32_t const t_timeout_usec)
{
	ltc_t t_timeout;
	Time_FromUSec(&t_timeout, t_timeout_usec);
	return Time_CheckTimeout(t_start, &t_timeout);
}

/*!***************************************************************************
 * @brief   Checks if timeout is reached from a given starting time.
 * @details Handles overflow.
 * @param   t_start Start time.
 * @param   t_timeout_msec Timeout period in milliseconds.
 * @return  Timeout elapsed? True/False (boolean value)
 *****************************************************************************/
inline bool Time_CheckTimeoutMSec(ltc_t const *t_start, uint32_t const t_timeout_msec)
{
	ltc_t t_timeout;
	Time_FromMSec(&t_timeout, t_timeout_msec);
	return Time_CheckTimeout(t_start, &t_timeout);
}

/*!***************************************************************************
 * @brief   Checks if timeout is reached from a given starting time.
 * @details Handles overflow.
 * @param   t_start Start time.
 * @param   t_timeout_sec Timeout period in seconds.
 * @return  Timeout elapsed? True/False (boolean value)
 *****************************************************************************/
inline bool Time_CheckTimeoutSec(ltc_t const *t_start, uint32_t const t_timeout_sec)
{
	ltc_t t_timeout;
	Time_FromSec(&t_timeout, t_timeout_sec);
	return Time_CheckTimeout(t_start, &t_timeout);
}


/*!***************************************************************************
 * @brief   Time delay for a given time period.
 * @param   dt Delay time.
 *****************************************************************************/
inline void Time_Delay(ltc_t const *dt)
{
	assert(dt != 0);
	ltc_t t_start = Time_Now();

	while (!Time_CheckTimeout(&t_start, dt));
}

/*!***************************************************************************
 * @brief   Time delay for a given time period in microseconds.
 * @param   dt_usec Delay time in microseconds.
 *****************************************************************************/
inline void Time_DelayUSec(uint32_t dt_usec)
{
	ltc_t t_start = Time_Now();

	while (!Time_CheckTimeoutUSec(&t_start, dt_usec));
}

/*!***************************************************************************
 * @brief   Time delay for a given time period in milliseconds.
 * @param   dt_msec Delay time in milliseconds.
 *****************************************************************************/
inline void Time_DelayMSec(uint32_t dt_msec)
{
	ltc_t t_start = Time_Now();

	while (!Time_CheckTimeoutMSec(&t_start, dt_msec));
}

/*!***************************************************************************
 * @brief   Time delay for a given time period in seconds.
 * @param   dt_sec Delay time in seconds.
 *****************************************************************************/
inline void Time_DelaySec(uint32_t dt_sec)
{
	ltc_t t_start = Time_Now();

	while (!Time_CheckTimeoutSec(&t_start, dt_sec));
}


/*! @} */
#ifdef __cplusplus
} // extern "C"
#endif
#endif /* TIME_H */
