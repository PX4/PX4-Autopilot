/****************************************************************************
 * include/nuttx/ramlog.h
 * The RAM logging driver
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
/* The RAM logging driver is a driver that was intended to support debugging
 * output (syslogging) when the normal serial output is not available.  For
 * example, if you are using a telnet or USB serial console, the debug
 * output will get lost.
 * 
 * The RAM logging  driver is similar to a pipe in that it saves the
 * debugging output in a FIFO in RAM.  It differs from a pipe in numerous
 * details as needed to support logging.
 *
 * This driver is built when CONFIG_RAMLOG is defined in the Nuttx
 * configuration.
 */

#ifndef __INCLUDE_NUTTX_RAMLOG_H
#define __INCLUDE_NUTTX_RAMLOG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_RAMLOG

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* CONFIG_RAMLOG - Enables the RAM logging feature
 * CONFIG_RAMLOG_CONSOLE - Use the RAM logging device as a system console.
 * CONFIG_RAMLOG_SYSLOG - Use the RAM logging device for the syslogging
 *   interface.  This should have:
 *   CONFIG_SYSLOG=ramlog
 * CONFIG_RAMLOG_NPOLLWAITERS - The number of threads than can be waiting
 *   for this driver on poll().  Default: 4
 *
 * If CONFIG_RAMLOG_CONSOLE is selected, then the following may also be
 * provided:
 *
 * CONFIG_RAMLOG_CONSOLE_BUFSIZE - Size of the console RAM log.  Default: 1024
 */

#ifndef CONFIG_RAMLOG_NPOLLWAITERS
#  define CONFIG_RAMLOG_NPOLLWAITERS 4
#endif

#ifndef CONFIG_SYSLOG
#  undef CONFIG_RAMLOG_SYSLOG
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
/****************************************************************************
 * Name: ramlog_register
 *
 * Description:
 *   Create the RAM logging device and register it at the specified path.
 *   Mostly likely this path will be /dev/console
 *
 ****************************************************************************/

EXTERN int ramlog_register(FAR const char *devpath, FAR char *buffer,
                           size_t buflen);

/****************************************************************************
 * Name: ramlog_consoleinit
 *
 * Description:
 *   Create the RAM logging device and register it at the specified path.
 *   Mostly likely this path will be /dev/console
 *
 ****************************************************************************/

#ifdef CONFIG_RAMLOG_CONSOLE
EXTERN int ramlog_consoleinit(void)
#endif

/****************************************************************************
 * Name: ramlog
 *
 * Description:
 *   This is the low-level system logging interface.  The debugging/syslogging
 *   interfaces are lib_rawprintf() and lib_lowprinf().  The difference is
 *   the lib_rawprintf() writes to fd=1 (stdout) and lib_lowprintf() uses
 *   a lower level interface that works from interrupt handlers.  This
 *   function is a a low-level interface used to implement lib_lowprintf()
 *   when CONFIG_RAMLOG_SYSLOG=y and CONFIG_SYSLOG=ramlog
 *
 ****************************************************************************/

#ifdef CONFIG_RAMLOG_SYSLOG
#  warning "Missing logic"
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_RAMLOG */
#endif /* __INCLUDE_NUTTX_RAMLOG_H */
