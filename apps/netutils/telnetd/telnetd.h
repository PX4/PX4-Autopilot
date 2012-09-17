/****************************************************************************
 * apps/netutils/telnetd/telnetd.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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

#ifndef __APPS_NETUTILS_TELNETD_TELNETD_H
#define __APPS_NETUTILS_TELNETD_TELNETD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure represents the overall state of one telnet daemon instance
 * (Yes, multiple telnet daemons are supported).
 */

struct telnetd_s
{
  int                   port;      /* The port to listen on (in network byte order) */
  int                   priority;  /* The execution priority of the spawned task, */
  int                   stacksize; /* The stack size needed by the spawned task */
  main_t                entry;     /* The entrypoint of the task to spawn when a new
                                    * connection is accepted. */
};

/* This structure is used to passed information to telnet daemon when it 
 * started.  It contains global information visable to all telnet daemons.
 */

struct telnetd_common_s
{
  uint8_t               ndaemons;  /* The total number of daemons running */
  sem_t                 startsem;  /* Enforces one-at-a-time startup */
  sem_t                 exclsem;   /* Enforces exclusive access to 'minor' */
  FAR struct telnetd_s *daemon;    /* Describes the new daemon */
  int                   minor;     /* The next minor number to use */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* This structure is used to passed information to telnet daemon when it 
 * started.  It contains global information visable to all telnet daemons.
 */

extern struct telnetd_common_s g_telnetdcommon;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: telnetd_driver
 *
 * Description:
 *   Create a character driver to "wrap" the telnet session.  This function
 *   will select and return a unique path for the new telnet device.
 *
 * Parameters:
 *   sd - The socket descriptor that represents the new telnet connection.
 *   daemon - A pointer to the structure representing the overall state of
 *     this instance of the telnet daemon.
 *
 * Return:
 *   An allocated string represent the full path to the created driver.  The
 *   receiver of the string must de-allocate this memory when it is no longer
 *   needed.  NULL is returned on a failure. 
 *
 ****************************************************************************/

FAR char *telnetd_driver(int sd, FAR struct telnetd_s *daemon);

#endif /* __APPS_NETUTILS_TELNETD_TELNETD_H */

