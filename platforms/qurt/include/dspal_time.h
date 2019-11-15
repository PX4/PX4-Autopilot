/****************************************************************************
 * Copyright (c) 2015 Mark Charlebois. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

/*==========================================================================
 * FILE:         time.h
 *
 * SERVICES:     POSIX Timer API interface
 *
 * DESCRIPTION:  POSIX Timer API interface based upon POSIX 1003.1-2004
 *==========================================================================*/

#include "dspal_types.h"
#include <sys/timespec.h>

typedef int              clockid_t; /* ignored */
#define _CLOCKID_T

typedef int timer_t;

#if !defined(CLOCK_REALTIME)
# define CLOCK_REALTIME 0
#endif

#if !defined(CLOCK_MONOTONIC)
# define CLOCK_MONOTONIC 1
#endif

/* have to move #include here to solve circular include problems between time.h and signal.h */
#include "dspal_signal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Timer functions */

/** \details
 * POSIX timers can be either of two types: a one-shot type or a periodic
 * type.
 *
 * A one-shot is an armed timer that is set to an expiration time relative
 * to either a current time or an absolute time. The timer expires once and
 * is disarmed.
 *
 * A periodic timer is armed with an initial expiration time and a repetition
 * interval. Every time the interval timer
 * expires, the timer is reloaded with the repetition interval. The timer
 * is then rearmed.
 */

/** \defgroup timer POSIX Timer API */

/** \ingroup timer */
/** @{ */

/** Create a POSIX timer.
 * Please refer to POSIX standard for details.
 * @param clockid [in] ignored in this implementation
 * @param evp     [in] if non-NULL, points to a sigevent structure. This
 * structure, allocated by the application, defines the asynchronous
 * notification to occur when the timer expires. If the evp argument is
 * NULL, the effect is as if the evp argument pointed to a sigevent
 * structure with the sigev_notify member having the value SIGEV_SIGNAL,
 * the sigev_signo having a default signal number (SIGALRM), and the
 * sigev_value member having the value of the timer ID.
 */
int timer_create(clockid_t clockid, struct sigevent *restrict evp,
		 timer_t *restrict timerid);

/** Delete a POSIX timer.
 * Please refer to POSIX standard for details.
 */
int timer_delete(timer_t timerid);

/** Get the time remaining on a POSIX timer.
 * Please refer to POSIX standard for details.
 */
int timer_gettime(timer_t timerid, struct itimerspec *value);


/** Set the time remaining on a POSIX timer.
 * Please refer to POSIX standard for details.
 * @param flags [in] ignored in this implementation
 */
int timer_settime(timer_t timerid, int flags,
		  const struct itimerspec *restrict value,
		  struct itimerspec *restrict ovalue);

/** Get the resolution of the specified clock
 * Please refer to POSIX standard for details
 */
int clock_getres(clockid_t clk_id, struct timespec *res);

/**
 * Get the time of the specified clock
 * Please refer to POSIX standard for details
 */
int clock_gettime(clockid_t clk_id, struct timespec *tp);

/**
 * Set the time of the specified clock
 * Please refer to POSIX standard for details
 */
int clock_settime(clockid_t clk_id, const struct timespec *tp);

/** Returns the value of time in seconds since the epoch.
 * This may return the value in seconds since last power up.
 * Please refer to POSIX standard for details.
 */
/*
 * Use the definition of the time provided by time.h to avoid conflicts.  The implementation
 * of time.h in the static image will still be called.
 */
time_t time(time_t *t);

/** @} */

#ifdef __cplusplus
}
#endif
