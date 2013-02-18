/****************************************************************************
 * examples/poll/poll_internal.h
 *
 *   Copyright (C) 2008, 2009 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

#ifndef __EXAMPLES_PIPE_PIPE_H
#define __EXAMPLES_PIPE_PIPE_H

/****************************************************************************
 * Compilation Switches
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <pthread.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

#ifdef CONFIG_DISABLE_POLL
#  error "The polling API is disabled"
#endif

/* Here are all of the configuration settings that must be met to have TCP/IP
 * poll/select support.  This kind of looks like overkill.
 *
 * CONFIG_NET                        - Network support must be enabled
 * CONFIG_NSOCKET_DESCRIPTORS        - Socket descriptors must be allocated
 * CONFIG_NET_TCP                    - Only support on TCP (because read-ahead
 *                                     ibuffering s not yet support for UDP)
 * CONFIG_NET_NTCP_READAHEAD_BUFFERS - TCP/IP read-ahead buffering must be enabled
 */

#if defined(CONFIG_NET) && CONFIG_NSOCKET_DESCRIPTORS > 0 && \
    defined(CONFIG_NET_TCP) && CONFIG_NET_NTCP_READAHEAD_BUFFERS > 0
#  define HAVE_NETPOLL 1
#else
#  undef HAVE_NETPOLL
#endif

/* If debug is enabled, then use syslog so that OS debug output and
 * the test output are synchronized.
 *
 * These macros will differ depending upon if the toolchain supports
 * macros with a variable number of arguments or not.
 */

#ifdef CONFIG_CPP_HAVE_VARARGS
# ifdef CONFIG_DEBUG
#   define message(...) syslog(__VA_ARGS__)
#   define msgflush()
# else
#   define message(...) printf(__VA_ARGS__)
#   define msgflush()   fflush(stdout)
# endif
#else
# ifdef CONFIG_DEBUG
#   define message      syslog
#   define msgflush()
# else
#   define message      printf
#   define msgflush()   fflush(stdout)
# endif
#endif

#define FIFO_PATH1 "/dev/fifo0"
#define FIFO_PATH2 "/dev/fifo1"

#define POLL_LISTENER_DELAY   2000   /* 2 seconds */
#define SELECT_LISTENER_DELAY 4      /* 4 seconds */
#define NET_LISTENER_DELAY    3      /* 3 seconds */
#define WRITER_DELAY          6      /* 6 seconds */

#define LISTENER_PORT         5471

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

extern void *poll_listener(pthread_addr_t pvarg);
extern void *select_listener(pthread_addr_t pvarg);

#ifdef HAVE_NETPOLL
extern void *net_listener(pthread_addr_t pvarg);
extern void *net_reader(pthread_addr_t pvarg);
#endif
#endif /* __EXAMPLES_PIPE_PIPE_H */
