/********************************************************************************
 * timer_getoverrun.c
 *
 *   Copyright (C) 2007, 2008, 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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
 ********************************************************************************/

/********************************************************************************
 * Included Files
 ********************************************************************************/

#include <nuttx/config.h>

#include <time.h>
#include <errno.h>

#include "timer_internal.h"

#ifndef CONFIG_DISABLE_POSIX_TIMERS

/********************************************************************************
 * Definitions
 ********************************************************************************/

/********************************************************************************
 * Private Data
 ********************************************************************************/

/********************************************************************************
 * Public Data
 ********************************************************************************/

/********************************************************************************
 * Private Functions
 ********************************************************************************/

/********************************************************************************
 * Public Functions
 ********************************************************************************/

/********************************************************************************
 * Name: timer_getoverrun
 *
 * Description:
 *   Only a single signal will be queued to the process for a given timer at any
 *   point in time.  When a timer for which a signal is still pending expires, no
 *   signal will be queued, and a timer overrun will occur. When a timer
 *   expiration signal is delivered to or accepted by a process, if the
 *   implementation  supports  the  Realtime Signals Extension, the
 *   timer_getoverrun() function will return the timer expiration overrun count for
 *   the specified timer. The overrun count returned contains the number of extra
 *   timer expirations that occurred between the time the signal was generated
 *   (queued) and when it was delivered or accepted, up to but not including an 
 *   implementation-defined  maximum of DELAYTIMER_MAX. If the number of such
 *   extra expirations is greater than or equal to DELAYTIMER_MAX, then the
 *   overrun count will be set to DELAYTIMER_MAX. The value returned by
 *   timer_getoverrun() will apply to the most recent expiration signal delivery
 *   or acceptance for the timer.  If no expiration signal has been delivered
 *   for the timer, or if the Realtime Signals Extension is not supported, the
 *   return value of timer_getoverrun() is unspecified.
 *
 * Parameters:
 *   timerid - The pre-thread timer, previously created by the call to
 *   timer_create(), whose overrun count will be returned..
 *
 * Return Value:
 *   If the timer_getoverrun() function succeeds, it will return the timer
 *   expiration overrun count as explained above. timer_getoverrun() will fail if:
 *
 *   EINVAL - The timerid argument does not correspond to an ID returned by
 *     timer_create() but not yet deleted by timer_delete().
 *
 * Assumptions:
 *
 ********************************************************************************/

int timer_getoverrun(timer_t timerid)
{
  errno = ENOSYS;
  return ERROR;
}

#endif /* CONFIG_DISABLE_POSIX_TIMERS */
