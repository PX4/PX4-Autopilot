/********************************************************************************
 * timer_internal.h
 *
 *   Copyright (C) 2007-2009 Gregory Nutt. All rights reserved.
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

#ifndef __SCHED_TIMER_INTERNAL_H
#define __SCHED_TIMER_INTERNAL_H

/********************************************************************************
 * Included Files
 ********************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <wdog.h>

#include <nuttx/compiler.h>

/********************************************************************************
 * Definitions
 ********************************************************************************/

#define PT_FLAGS_PREALLOCATED 0x01 /* Timer comes from a pool of preallocated timers */

/********************************************************************************
 * Public Types
 ********************************************************************************/

/* This structure represents one POSIX timer */

struct posix_timer_s
{
  FAR struct posix_timer_s *flink;

  uint8_t         pt_flags;        /* See PT_FLAGS_* definitions */
  uint8_t         pt_crefs;        /* Reference count */
  uint8_t         pt_signo;        /* Notification signal */
  pid_t           pt_owner;        /* Creator of timer */
  int             pt_delay;        /* If non-zero, used to reset repetitive timers */
  int             pt_last;         /* Last value used to set watchdog */
  WDOG_ID         pt_wdog;         /* The watchdog that provides the timing */
  union sigval    pt_value;        /* Data passed with notification */
};

/********************************************************************************
 * Public Data
 ********************************************************************************/

/* This is a list of free, preallocated timer structures */

#if CONFIG_PREALLOC_TIMERS > 0
extern volatile sq_queue_t g_freetimers;
#endif

/* This is a list of instantiated timer structures -- active and inactive.  The
 * timers are place on this list by timer_create() and removed from the list by
 * timer_delete() or when the owning thread exits.
 */

extern volatile sq_queue_t g_alloctimers;

/********************************************************************************
 * Public Function Prototypes
 ********************************************************************************/

void weak_function timer_initialize(void);
void weak_function timer_deleteall(pid_t pid);
int  timer_release(FAR struct posix_timer_s *timer);

#endif /* __SCHED_TIMER_INTERNAL_H */
