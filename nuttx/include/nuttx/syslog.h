/****************************************************************************
 * include/nuttx/syslog.h
 * The NuttX SYSLOGing interface
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

#ifndef __INCLUDE_NUTTX_SYSLOG_H
#define __INCLUDE_NUTTX_SYSLOG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* CONFIG_SYSLOG - Enables generic system logging features.
 * CONFIG_SYSLOG_DEVPATH - The full path to the system logging device
 *
 * In addition, some SYSLOG device must also be enabled that will provide
 * the syslog_putc() function.  As of this writing, there are two SYSLOG
 * devices avaiable:
 *
 *   1. A RAM SYSLOGing device that will log data into a circular buffer
 *      that can be dumped using the NSH dmesg command.  This device is
 *      described in the include/nuttx/ramlog.h header file.
 *
 *   2. And a generic character device that may be used as the SYSLOG.  The
 *      generic device interfaces are described in this file.  A disadvantage
 *      of using the generic character device for the SYSLOG is that it
 *      cannot handle debug output generated from interrupt level handlers.
 *
 * CONFIG_SYSLOG_CHAR - Enable the generic character device for the SYSLOG.
 *   The full path to the SYSLOG device is provided by CONFIG_SYSLOG_DEVPATH.
 *   A valid character device must exist at this path.  It will by opened
 *   by syslog_initialize.
 *
 *   NOTE:  No more than one SYSLOG device should be configured.
 */

#ifndef CONFIG_SYSLOG
#  undef CONFIG_SYSLOG_CHAR
#endif

#if defined(CONFIG_SYSLOG_CHAR) && !defined(CONFIG_SYSLOG_DEVPATH)
#  define CONFIG_SYSLOG_DEVPATH "/dev/ttyS1"
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
 * Name: syslog_initialize
 *
 * Description:
 *   Initialize to use the character device (or file) at
 *   CONFIG_SYSLOG_DEVPATH as the SYSLOG sink.
 *
 *   NOTE that this implementation excludes using a network connection as
 *   SYSLOG device.  That would be a good extension.
 *
 ****************************************************************************/

#ifdef CONFIG_SYSLOG_CHAR
EXTERN int syslog_initialize(void);
#endif

/****************************************************************************
 * Name: syslog_putc
 *
 * Description:
 *   This is the low-level system logging interface.  The debugging/syslogging
 *   interfaces are syslog() and lowsyslog().  The difference is that
 *   the syslog() internface writes to fd=1 (stdout) whereas lowsyslog() uses
 *   a lower level interface that works from interrupt handlers.  This
 *   function is a a low-level interface used to implement lowsyslog().
 *
 ****************************************************************************/

#ifdef CONFIG_SYSLOG
EXTERN int syslog_putc(int ch);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __INCLUDE_NUTTX_SYSLOG_H */
