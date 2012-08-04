/********************************************************************************
 * include/limits.h
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

#ifndef __INCLUDE_LIMITS_H
#define __INCLUDE_LIMITS_H

/********************************************************************************
 * Included Files
 ********************************************************************************/

#include <nuttx/config.h>

/* Architecture specific limits */

#include <arch/limits.h>

/********************************************************************************
 * Pre-processor Definitions
 ********************************************************************************/
/* Default values for user configurable limits **********************************/
/* Maximum number of bytes in a filename (not including terminating null). */

#ifndef CONFIG_NAME_MAX
#  define CONFIG_NAME_MAX 32
#endif

/* Maximum number of bytes in a pathname, including the terminating null
 * character.
 */

#ifndef CONFIG_PATH_MAX
#  if CONFIG_NAME_MAX < 64
#    define CONFIG_PATH_MAX (4*CONFIG_NAME_MAX + 1)
#  else
#    define CONFIG_PATH_MAX 256
#  endif
#endif

/* Configurable limits required by POSIX ****************************************
 *
 * Required for all implementations:
 *
 *   _POSIX_ARG_MAX        Total length of string arguments
 *   _POSIX_CHILD_MAX      Number of child tasks active
 *   _POSIX_LINK_MAX       The number of links a file can have
 *   _POSIX_MAX_CANON      Number bytes in TTY canonical input queue
 *   _POSIX_MAX_INPUT      Number bytes in TTY canonical input queue
 *   _POSIX_NAME_MAX       Number of bytes in a file or pathname component
 *   _POSIX_NGROUPS_MAX    Number supplementary group IDs
 *   _POSIX_OPEN_MAX       Number of files a task can have open at once
 *   _POSIX_PATH_MAX       Number of bytes in a full pathname (including NULL)
 *   _POSIX_PIPE_BUF       Number of bytes for atomic write into pipe
 *   _POSIX_SSIZE_MAX      Largest filesystem write; also max value of ssize_t
 *   _POSIX_STREAM_MAX     Number of std I/O streams open at once
 *   _POSIX_TZNAME_MAX     Max number of bytes of a timezone name
 *
 * Required for sigqueue
 *
 *   _POSIX_RTSIG_MAX      Difference between SIGRTMIN and SIGRTMAX
 *   _POSIX_SIGQUEUE_MAX   Max number signals a task can queue
 *
 * Required for POSIX timers
 *
 *   _POSIX_DELAYTIMER_MAX Max number timer overruns
 *   _POSIX_TIMER_MAX      Max number of timers per task
 *   _POSIX_CLOCKRES_MIN   Clock resolution in nanoseconds
 *
 * Required for asynchronous I/O
 *
 *   _POSIX_AIO_LISTIO_MAX Max number of AIOs in single listio call
 *   _POSIX_AIO_MAX        Max number of simultaneous AIO operations
 *
 * Required for POSIX message passing
 *
 *   _POSIX_MQ_OPEN_MAX    Max number message queues task may open (mq_open)
 *   _POSIX_MQ_PRIO_MAX    Max message priority (mq_send)
 *
 * Required for POSIX semaphores
 *
 *   _POSIX_SEM_NSEMS_MAX  Max number of open semaphores per task
 *   _POSIX_SEM_VALUE_MAX  Max value a semaphore may have
 */

#define _POSIX_ARG_MAX        4096
#define _POSIX_CHILD_MAX      6
#define _POSIX_LINK_MAX       8
#define _POSIX_MAX_CANON      255
#define _POSIX_MAX_INPUT      255
#define _POSIX_NAME_MAX       CONFIG_NAME_MAX
#define _POSIX_NGROUPS_MAX    0
#define _POSIX_OPEN_MAX       CONFIG_NFILE_DESCRIPTORS
#define _POSIX_PATH_MAX       CONFIG_PATH_MAX
#define _POSIX_PIPE_BUF       512
#define _POSIX_SSIZE_MAX      INT_MAX
#define _POSIX_STREAM_MAX     CONFIG_NFILE_STREAMS
#define _POSIX_TZNAME_MAX     3

/* Requred for sigqueue */

#define _POSIX_RTSIG_MAX      31
#define _POSIX_SIGQUEUE_MAX   32

/* Required for POSIX timers.
 *
 * _POSIX_DELAYTIMER_MAX is the number of timer expiration overruns.
 *
 * _POSIX_TIMER_MAX is the per-process number of timers.
 *
 * _POSIX_CLOCKRES_MIN is the resolution of the CLOCK_REALTIME clock in nanoseconds.
 * CLOCK_REALTIME is controlled by the NuttX system time.  The default value is the
 * system timer which has a resolution of 10 milliseconds.  This default setting can
 * be overridden by defining the clock interval in milliseconds as CONFIG_MSEC_PER_TICK
 * in the board configuration file.
 */

#define _POSIX_DELAYTIMER_MAX 32
#define _POSIX_TIMER_MAX      32

#ifdef CONFIG_MSEC_PER_TICK
# define _POSIX_CLOCKRES_MIN  ((CONFIG_MSEC_PER_TICK)*1000000)
#else
# define _POSIX_CLOCKRES_MIN  (10*1000000)
#endif

/* Required for asynchronous I/O */

#define _POSIX_AIO_LISTIO_MAX 2
#define _POSIX_AIO_MAX        1

/* Required for POSIX message passing */

#define _POSIX_MQ_OPEN_MAX    8
#define _POSIX_MQ_PRIO_MAX    UCHAR_MAX

/* Required for POSIX semaphores */

#define _POSIX_SEM_NSEMS_MAX  INT_MAX
#define _POSIX_SEM_VALUE_MAX  0x7fff

/* Actual limits.  These values may be increased from the POSIX minimum
 * values above or made indeterminate
 */

#define ARG_MAX        _POSIX_ARG_MAX
#define CHILD_MAX      _POSIX_CHILD_MAX
#define LINK_MAX       _POSIX_LINK_MAX
#define MAX_CANON      _POSIX_MAX_CANON
#define MAX_INPUT      _POSIX_MAX_INPUT
#define NAME_MAX       _POSIX_NAME_MAX
#define NGROUPS_MAX    _POSIX_NGROUPS_MAX
#define OPEN_MAX       _POSIX_OPEN_MAX
#define PATH_MAX       _POSIX_PATH_MAX
#define PIPE_BUF       _POSIX_PIPE_BUF
#define SSIZE_MAX      _POSIX_SSIZE_MAX
#define STREAM_MAX     _POSIX_STREAM_MAX
#define TZNAME_MAX     _POSIX_TZNAME_MAX

#define RTSIG_MAX      _POSIX_RTSIG_MAX
#define SIGQUEUE_MAX   _POSIX_SIGQUEUE_MAX

#define DELAYTIMER_MAX _POSIX_DELAYTIMER_MAX
#define TIMER_MAX      _POSIX_TIMER_MAX
#define CLOCKRES_MIN   _POSIX_CLOCKRES_MIN

/* Required for asynchronous I/O */

#define AIO_LISTIO_MAX _POSIX_AIO_LISTIO_MAX
#define AIO_MAX        _POSIX_AIO_MAX

/* Required for POSIX message passing */

#define MQ_OPEN_MAX    _POSIX_MQ_OPEN_MAX
#define MQ_PRIO_MAX    _POSIX_MQ_PRIO_MAX

/* Required for POSIX semaphores */

#define SEM_NSEMS_MAX  _POSIX_SEM_NSEMS_MAX
#define SEM_VALUE_MAX  _POSIX_SEM_VALUE_MAX

#endif /* __INCLUDE_LIMITS_H */
