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
#include <nuttx/syslog.h>

#ifdef CONFIG_RAMLOG

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* CONFIG_RAMLOG - Enables the RAM logging feature
 * CONFIG_RAMLOG_CONSOLE - Use the RAM logging device as a system console.
 *   If this feature is enabled (along with CONFIG_DEV_CONSOLE), then all
 *   console output will be re-directed to a circular buffer in RAM.  This
 *   is useful, for example, if the only console is a Telnet console.  Then
 *   in that case, console output from non-Telnet threads will go to the
 *   circular buffer and can be viewed using the NSH 'dmesg' command.
 * CONFIG_RAMLOG_SYSLOG - Use the RAM logging device for the syslogging
 *   interface.  If this feature is enabled (along with CONFIG_SYSLOG),
 *   then all debug output (only) will be re-directed to the circular
 *   buffer in RAM.  This RAM log can be view from NSH using the 'dmesg'
 *   command.  NOTE:  Unlike the limited, generic character driver SYSLOG
 *   device, the RAMLOG *can* be used to generate debug output from interrupt
 *   level handlers.
 * CONFIG_RAMLOG_NPOLLWAITERS - The number of threads than can be waiting
 *   for this driver on poll().  Default: 4
 *
 * If CONFIG_RAMLOG_CONSOLE or CONFIG_RAMLOG_SYSLOG is selected, then the
 * following may also be provided:
 *
 * CONFIG_RAMLOG_CONSOLE_BUFSIZE - Size of the console RAM log.  Default: 1024
 */

#ifndef CONFIG_DEV_CONSOLE
#  undef CONFIG_RAMLOG_CONSOLE
#endif

#ifndef CONFIG_SYSLOG
#  undef CONFIG_RAMLOG_SYSLOG
#endif

#if defined(CONFIG_RAMLOG_SYSLOG) && !defined(CONFIG_SYSLOG_DEVPATH)
#  define CONFIG_SYSLOG_DEVPATH "/dev/ramlog"
#endif

#ifndef CONFIG_RAMLOG_NPOLLWAITERS
#  define CONFIG_RAMLOG_NPOLLWAITERS 4
#endif

#ifndef CONFIG_SYSLOG
#  undef CONFIG_RAMLOG_SYSLOG
#endif

#ifndef CONFIG_RAMLOG_CONSOLE_BUFSIZE
#  define CONFIG_RAMLOG_CONSOLE_BUFSIZE 1024
#endif

/* The normal behavior of the RAM log when used as a SYSLOG is to return
 * end-of-file if there is no data in the RAM log (rather than blocking until
 * data is available).  That allows you to 'cat' the SYSLOG with no ill
 * consequences.
 */

#ifdef CONFIG_SYSLOG
#  undef CONFIG_RAMLOG_NONBLOCKING
#  define CONFIG_RAMLOG_NONBLOCKING 1
#endif

/* When used as a console or syslogging device, the RAM log will pre-pend
 * line-feeds with carriage returns.
 */

#if defined(CONFIG_RAMLOG_CONSOLE) || defined(CONFIG_RAMLOG_SYSLOG)
#  undef CONFIG_RAMLOG_CRLF
#  define CONFIG_RAMLOG_CRLF 1
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
 *   Mostly likely this path will be /dev/console.
 *
 *   This interface is not normally used but can be made available is
 *   someone just wants to tinker with the RAM log as a generic character
 *   device.  Normally both CONFIG_RAMLOG_CONSOLE and CONFIG_RAMLOG_SYSLOG
 *   would be set (to capture all output in the log) -OR- just
 *   CONFIG_RAMLOG_SYSLOG would be set to capture debug output only
 *   in the log.
 *
 ****************************************************************************/

#if !defined(CONFIG_RAMLOG_CONSOLE) && !defined(CONFIG_RAMLOG_SYSLOG)
EXTERN int ramlog_register(FAR const char *devpath, FAR char *buffer,
                           size_t buflen);
#endif

/****************************************************************************
 * Name: ramlog_consoleinit
 *
 * Description:
 *   Create the RAM logging device and register it at the specified path.
 *   Mostly likely this path will be /dev/console.
 *
 *   If CONFIG_RAMLOG_SYSLOG is also defined, then the same RAM logging
 *   device is also registered at CONFIG_SYSLOG_DEVPATH
 *
 ****************************************************************************/

#ifdef CONFIG_RAMLOG_CONSOLE
EXTERN int ramlog_consoleinit(void);
#endif

/****************************************************************************
 * Name: ramlog_sysloginit
 *
 * Description:
 *   Create the RAM logging device and register it at the specified path.
 *   Mostly likely this path will be CONFIG_SYSLOG_DEVPATH
 *
 *   If CONFIG_RAMLOG_CONSOLE is also defined, then this functionality is
 *   performed when ramlog_consoleinit() is called.
 *
 ****************************************************************************/

#ifdef CONFIG_RAMLOG_SYSLOG
EXTERN int ramlog_sysloginit(void);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_RAMLOG */
#endif /* __INCLUDE_NUTTX_RAMLOG_H */
