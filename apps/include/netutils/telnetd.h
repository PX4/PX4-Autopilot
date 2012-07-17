/****************************************************************************
 *  apps/include/netutils/telnetd.h
 *
 *   Copyright (C) 2007, 2011-2012 Gregory Nutt. All rights reserved.
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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

#ifndef __APPS_INCLUDE_NETUTILS_TELNETD_H
#define __APPS_INCLUDE_NETUTILS_TELNETD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* CONFIG_TELNETD_CONSOLE - Use the first telnet session as the default
 *   console.
 * CONFIG_TELNETD_RXBUFFER_SIZE - The size of the telnet receive buffer.
 *   Default: 256 bytes.
 * CONFIG_TELNETD_TXBUFFER_SIZE - The size of the telnet transmit buffer.
 *   Default: 256 bytes.
 * CONFIG_TELNETD_DUMPBUFFER - dumping of all input/output buffers.
 */

#ifndef CONFIG_TELNETD_RXBUFFER_SIZE
# define CONFIG_TELNETD_RXBUFFER_SIZE 256
#endif

#ifndef CONFIG_TELNETD_TXBUFFER_SIZE
# define CONFIG_TELNETD_TXBUFFER_SIZE 256
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

 /* An instance of the struct telnetd_config_s structure must be passed to
 * telnetd_start in order to configure the new telnet daemon.
 */

struct telnetd_config_s
{
  /* These fields describe the telnet daemon */

  int    d_port;      /* The port to listen on (in network byte order) */
  int    d_priority;  /* The execution priority of the telnet daemon task */
  int    d_stacksize; /* The stack size needed by the telnet daemon task */

  /* These fields describe the priority of each thread created by the telnet
   * daemon.
   */

  int    t_priority;  /* The execution priority of the spawned task, */
  int    t_stacksize; /* The stack size needed by the spawned task */
  main_t t_entry;     /* The entrypoint of the task to spawn when a new
                       * connection is accepted. */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: telnetd_start
 *
 * Description:
 *   Start the telnet daemon.
 *
 * Parameters:
 *   config    A pointer to a configuration structure that characterizes the
 *             telnet daemon.  This configuration structure may be defined
 *             on the caller's stack because it is not retained by the
 *             daemon.
 *
 * Return:
 *   The process ID (pid) of the new telnet daemon is returned on
 *   success; A negated errno is returned if the daemon was not successfully
 *   started.
 *
 ****************************************************************************/

EXTERN int telnetd_start(FAR struct telnetd_config_s *config);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __APPS_INCLUDE_NETUTILS_TELNETD_H */
