/****************************************************************************
 * include/apps/nsh.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#ifndef __INCLUDE_APPS_NSHLIB_H
#define __INCLUDE_APPS_NSHLIB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

#if CONFIG_RR_INTERVAL > 0
# define SCHED_NSH SCHED_RR
#else
# define SCHED_NSH SCHED_FIFO
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Interfaces needed to initialize and execute the NuttShell (NSH).
 *
 * nsh_initialize() - This function function should be called one during
 *   application start-up prior to executing nsh_consolemain() or
 *   nsh_telnetmain().
 */

EXTERN void nsh_initialize(void);

/* The following interfaces maybe to called or started with task_start to
 * start an NSH instance.
 *
 * nsh_consolemain() starts NSH on the console (/dev/console).
 * nsh_telnetmain() starts a telnet daemon that will allow multiple
 *   connections via telnet.
 *
 * These functions do not return.
 */

EXTERN int nsh_consolemain(int argc, char *argv[]);
EXTERN int nsh_telnetmain(int argc, char *argv[]);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_APPS_NSHLIB_H */
