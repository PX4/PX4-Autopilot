/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 API.
 * @details		This file provides utility functions for timing necessities.
 *
 * @copyright
 *
 * Copyright (c) 2021, Broadcom Inc
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

/*!***************************************************************************
 * @defgroup	time Time Utility
 * @ingroup		argusutil
 * @brief 		Timer utilities for time measurement duties.
 * @details		This module provides time measurement utility functions like
 * 				delay or time measurement methods, or time math functions.
 * @addtogroup 	time
 * @{
 *****************************************************************************/

#include <stdint.h>
#include <stdbool.h>

/*!***************************************************************************
 * @brief	A data structure to represent current time.
 *
 * @details	Value is obtained from the PIT time which must be configured as
 * 			lifetime counter.
 *****************************************************************************/
typedef struct {
	/*! Seconds. */
	uint32_t sec;

	/*! Microseconds. */
	uint32_t usec;

} ltc_t;

/*!***************************************************************************
 * @brief	Obtains the elapsed time since MCU startup.
 * @param	t_now returned current time
 *****************************************************************************/
void Time_GetNow(ltc_t *t_now);

/*!***************************************************************************
 * @brief	Obtains the elapsed microseconds since MCU startup.
 * @details	Wrap around effect due to uint32_t result format!!
 * @param 	-
 * @return	Elapsed microseconds since MCU startup as uint32_t.
 *****************************************************************************/
uint32_t Time_GetNowUSec(void);

/*!***************************************************************************
 * @brief	Obtains the elapsed milliseconds since MCU startup.
 * @details	Wrap around effect due to uint32_t result format!!
 * @param 	-
 * @return	Elapsed milliseconds since MCU startup as uint32_t.
 *****************************************************************************/
uint32_t Time_GetNowMSec(void);

/*!***************************************************************************
 * @brief	Obtains the elapsed seconds since MCU startup.
 * @param 	-
 * @return	Elapsed seconds since MCU startup as uint32_t.
 *****************************************************************************/
uint32_t Time_GetNowSec(void);

/*!***************************************************************************
 * @brief	Obtains the elapsed time since a given time point.
 * @param	t_elapsed Returns the elapsed time since t_start.
 * @param	t_start Start time point.
 *****************************************************************************/
void Time_GetElapsed(ltc_t *t_elapsed, ltc_t *const t_start);

/*!***************************************************************************
 * @brief	Obtains the elapsed microseconds since a given time point.
 * @details	Wrap around effect due to uint32_t result format!!
 * @param	t_start Start time point.
 * @return	Elapsed microseconds since t_start as uint32_t.
 *****************************************************************************/
uint32_t Time_GetElapsedUSec(ltc_t *const t_start);

/*!***************************************************************************
 * @brief	Obtains the elapsed milliseconds since a given time point.
 * @details	Wrap around effect due to uint32_t result format!!
 * @param	t_start Start time point.
 * @return	Elapsed milliseconds since t_start as uint32_t.
 *****************************************************************************/
uint32_t Time_GetElapsedMSec(ltc_t *const t_start);

/*!***************************************************************************
 * @brief	Obtains the elapsed seconds since a given time point.
 * @param	t_start Start time point.
 * @return	Elapsed seconds since t_start as uint32_t.
 *****************************************************************************/
uint32_t Time_GetElapsedSec(ltc_t *const t_start);

/*!***************************************************************************
 * @brief	Obtains the time difference between two given time points.
 * @details	Result is defined as t_diff = t_end - t_start.
 * 			Note: since no negative time differences are supported, t_end has
 * 			to be later/larger than t_start. Otherwise, the result won't be
 * 			a negative time span but given by max value.
 * @param	t_diff Returned time difference.
 * @param	t_start Start time point.
 * @param	t_end End time point.
 *****************************************************************************/
void Time_Diff(ltc_t *t_diff, ltc_t const *t_start, ltc_t const *t_end);

/*!***************************************************************************
 * @brief	Obtains the time difference between two given time points in
 * 			microseconds.
 * @details	Result is defined as t_diff = t_end - t_start.
 * 			Refers to Time_Diff() and handles overflow such that to large
 * 			values are limited by 0xFFFFFFFF µs.
 * @param	t_start Start time point.
 * @param	t_end End time point.
 * @return	Time difference in microseconds.
 *****************************************************************************/
uint32_t Time_DiffUSec(ltc_t const *t_start, ltc_t const *t_end);

/*!***************************************************************************
 * @brief	Obtains the time difference between two given time points in
 * 			milliseconds.
 * @details	Result is defined as t_diff = t_end - t_start.
 * 			Refers to Time_Diff() and handles overflow.
 *			Wrap around effect due to uint32_t result format!!
 * @param	t_start Start time point.
 * @param	t_end End time point.
 * @return	Time difference in milliseconds.
 *****************************************************************************/
uint32_t Time_DiffMSec(ltc_t const *t_start, ltc_t const *t_end);

/*!***************************************************************************
 * @brief	Obtains the time difference between two given time points in
 * 			seconds.
 * @details	Result is defined as t_diff = t_end - t_start.
 * 			Refers to Time_Diff() and handles overflow.
 * @param	t_start Start time point.
 * @param	t_end End time point.
 * @return	Time difference in seconds.
 *****************************************************************************/
uint32_t Time_DiffSec(ltc_t const *t_start, ltc_t const *t_end);

/*!***************************************************************************
 * @brief	Time delay for a given time period.
 * @param	dt Delay time.
 *****************************************************************************/
void Time_Delay(ltc_t const *dt);

/*!***************************************************************************
 * @brief	Time delay for a given time period in microseconds.
 * @param	dt_usec Delay time in microseconds.
 *****************************************************************************/
void Time_DelayUSec(uint32_t dt_usec);

/*!***************************************************************************
 * @brief	Time delay for a given time period in milliseconds.
 * @param	dt_msec Delay time in milliseconds.
 *****************************************************************************/
void Time_DelayMSec(uint32_t dt_msec);

/*!***************************************************************************
 * @brief	Time delay for a given time period in seconds.
 * @param	dt_sec Delay time in seconds.
 *****************************************************************************/
void Time_DelaySec(uint32_t dt_sec);

/*!***************************************************************************
 * @brief	Checks if timeout is reached from a given starting time.
 * @details	Handles overflow.
 * @param	t_start Start time.
 * @param	t_timeout Timeout period.
 * @return	Timeout elapsed? True/False (boolean value)
 *****************************************************************************/
bool Time_CheckTimeout(ltc_t const *t_start, ltc_t const *t_timeout);

/*!***************************************************************************
 * @brief	Checks if timeout is reached from a given starting time.
 * @details	Handles overflow.
 * @param	t_start Start time.
 * @param	t_timeout_usec Timeout period in microseconds.
 * @return	Timeout elapsed? True/False (boolean value)
 *****************************************************************************/
bool Time_CheckTimeoutUSec(ltc_t const *t_start, uint32_t const t_timeout_usec);

/*!***************************************************************************
 * @brief	Checks if timeout is reached from a given starting time.
 * @details	Handles overflow.
 * @param	t_start Start time.
 * @param	t_timeout_msec Timeout period in milliseconds.
 * @return	Timeout elapsed? True/False (boolean value)
 *****************************************************************************/
bool Time_CheckTimeoutMSec(ltc_t const *t_start, uint32_t const t_timeout_msec);

/*!***************************************************************************
 * @brief	Checks if timeout is reached from a given starting time.
 * @details	Handles overflow.
 * @param	t_start Start time.
 * @param	t_timeout_sec Timeout period in seconds.
 * @return	Timeout elapsed? True/False (boolean value)
 *****************************************************************************/
bool Time_CheckTimeoutSec(ltc_t const *t_start, uint32_t const t_timeout_sec);

/*!***************************************************************************
 * @brief	Adds two ltc_t values.
 * @details	Result is defined as t = t1 + t2. Results are wrapped around at
 * 			maximum values just like integers.
 * @param	t Return value: t = t1 + t2.
 * @param	t1 1st operand.
 * @param	t2 2nd operand.
 *****************************************************************************/
void Time_Add(ltc_t *t, ltc_t const *t1, ltc_t const *t2);

/*!***************************************************************************
 * @brief	Adds a given time in microseconds to an ltc_t value.
 * @param	t Return value: t = t1 + t2.
 * @param	t1 1st operand.
 * @param	t2_usec 2nd operand in microseconds.
 *****************************************************************************/
void Time_AddUSec(ltc_t *t, ltc_t const *t1, uint32_t t2_usec);

/*!***************************************************************************
 * @brief	Adds a given time in milliseconds to an ltc_t value.
 * @param	t Return value: t = t1 + t2.
 * @param	t1 1st operand.
 * @param	t2_msec 2nd operand in milliseconds.
 *****************************************************************************/
void Time_AddMSec(ltc_t *t, ltc_t const *t1, uint32_t t2_msec);

/*!***************************************************************************
 * @brief	Adds a given time in seconds to an ltc_t value.
 * @param	t Return value: t = t1 + t2.
 * @param	t1 1st operand.
 * @param	t2_sec 2nd operand in seconds.
 *****************************************************************************/
void Time_AddSec(ltc_t *t, ltc_t const *t1, uint32_t t2_sec);

/*!***************************************************************************
 * @brief	Converts ltc_t to microseconds (uint32_t).
 * @param	t Input ltc_t struct.
 * @return	Time value in microseconds.
 *****************************************************************************/
uint32_t Time_ToUSec(ltc_t const *t);

/*!***************************************************************************
 * @brief	Converts ltc_t to milliseconds (uint32_t).
 * @param	t Input ltc_t struct.
 * @return	Time value in milliseconds.
 *****************************************************************************/
uint32_t Time_ToMSec(ltc_t const *t);

/*!***************************************************************************
 * @brief	Converts ltc_t to seconds (uint32_t).
 * @param	t Input ltc_t struct.
 * @return	Time value in seconds.
 *****************************************************************************/
uint32_t Time_ToSec(ltc_t const *t);

/*! @} */
#endif /* TIME_H */
