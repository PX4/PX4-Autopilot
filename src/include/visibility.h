/****************************************************************************
 *
 *   Copyright (C) 2012-2019 PX4 Development Team. All rights reserved.
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

/**
 * @file visibility.h
 *
 * Definitions controlling symbol naming and visibility.
 *
 * This file is normally included automatically by the build system.
 */

#pragma once

#ifdef __EXPORT
#  undef __EXPORT
#endif
#define __EXPORT __attribute__ ((visibility ("default")))

#ifdef __PRIVATE
#  undef __PRIVATE
#endif
#define __PRIVATE __attribute__ ((visibility ("hidden")))

#ifdef __cplusplus
#  define __BEGIN_DECLS		extern "C" {
#  define __END_DECLS		}
#else
#  define __BEGIN_DECLS
#  define __END_DECLS
#endif


#define system_exit exit
#define system_clock_gettime clock_gettime
#define system_clock_settime clock_settime
#define system_pthread_cond_timedwait pthread_cond_timedwait
#define system_usleep usleep
#define system_sleep sleep

#ifndef PX4_DISABLE_GCC_POISON

/* exit() is used on NuttX to exit a task. However on Posix, it will exit the
 * whole application, so we prevent its use there. There are cases where it
 * still needs to be used, thus we remap system_exit to exit.
 */
#if !defined(__PX4_NUTTX)
#include <stdlib.h>
#ifdef __cplusplus
#include <cstdlib>
#endif
/* We should include cstdlib or stdlib.h but this doesn't
 * compile because many C++ files include stdlib.h and would
 * need to get changed. */
//#pragma GCC poison exit
#endif // !defined(__PX4_NUTTX)


/* For SITL lockstep we fake the clock, sleeping, and timedwaits
 * Therefore, we prefix these syscalls with system_. */
#include <time.h>
/* We can't poison clock_settime/clock_gettime because they are
 * used in DriverFramework. */

#if !defined(__PX4_NUTTX) && !defined(__PX4_QURT)
#include <pthread.h>
// We can't include this for NuttX otherwise we get conflicts for read/write
// symbols in cannode.
// We can't include this for Qurt because it uses it's own thread primitives
#endif // !defined(__PX4_NUTTX) && !defined(__PX4_QURT)
#define system_pthread_cond_timedwait pthread_cond_timedwait
/* We can't poison pthread_cond_timedwait because it seems to be used in the
 * <string> include. */


/* We don't poison usleep and sleep because it is used in dependencies
 * like uavcan. */
#if !defined(__PX4_NUTTX)
#include <unistd.h>
// We can't include this for NuttX otherwise we get conflicts for read/write
// symbols in cannode.
#endif // !defined(__PX4_NUTTX)


/* On NuttX we call clearenv() so we cannot use getenv() and others (see
 * px4_task_spawn_cmd() in px4_nuttx_tasks.c).
 * We need to include the headers declaring getenv() before the pragma,
 * otherwise it will trigger a poison error.  */
#if defined(__PX4_NUTTX)
#include <stdlib.h>
#ifdef __cplusplus
#include <cstdlib>
#endif
/* We should include cstdlib or stdlib.h but this doesn't
 * compile because many C++ files include stdlib.h and would
 * need to get changed. */
#pragma GCC poison getenv setenv putenv
#endif // defined(__PX4_NUTTX)

#endif // PX4_DISABLE_GCC_POISON
