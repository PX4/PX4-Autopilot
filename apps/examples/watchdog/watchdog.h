/****************************************************************************
 * examples/examples/watchdog/watchdog.h
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

#ifndef __APPS_EXAMPLES_WATCHDOG_WATCHDOG_H
#define __APPS_EXAMPLES_WATCHDOG_WATCHDOG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* CONFIG_NSH_BUILTIN_APPS - Build the WATCHDOG test as an NSH built-in
 *   function. Default: Not built!  The example can only be used as an NSH
 *   built-in application
 * CONFIG_EXAMPLES_WATCHDOG_DEVPATH - The path to the Watchdog device.
 *   Default: /dev/watchdog0
 * CONFIG_EXAMPLES_WATCHDOG_PINGTIME - Time in milliseconds that the example
 *   will ping the watchdog before letting the watchdog expire. Default: 5000
 *   milliseconds
 * CONFIG_EXAMPLES_WATCHDOG_PINGDELAY - Time delay between pings in
 *   milliseconds. Default: 500 milliseconds.
 * CONFIG_EXAMPLES_WATCHDOG_TIMEOUT - The watchdog timeout value in
 *   milliseconds before the watchdog timer expires.  Default:  2000
 *   milliseconds.
 */

#ifndef CONFIG_WATCHDOG
#  error "WATCHDOG device support is not enabled (CONFIG_WATCHDOG)"
#endif

#ifndef CONFIG_NSH_BUILTIN_APPS
#  warning "The WATCHDOG example only works as an NSH built-in application (CONFIG_NSH_BUILTIN_APPS)"
#endif

#ifndef CONFIG_EXAMPLES_WATCHDOG_DEVPATH
#  define CONFIG_EXAMPLES_WATCHDOG_DEVPATH "/dev/watchdog0"
#endif

#ifndef CONFIG_EXAMPLES_WATCHDOG_PINGTIME
#  define CONFIG_EXAMPLES_WATCHDOG_PINGTIME 5000
#endif

#ifndef CONFIG_EXAMPLES_WATCHDOG_PINGDELAY
#  define CONFIG_EXAMPLES_WATCHDOG_PINGDELAY 500
#endif

#ifndef CONFIG_EXAMPLES_WATCHDOG_TIMEOUT
#  define CONFIG_EXAMPLES_WATCHDOG_TIMEOUT 2000
#endif

/* Debug ********************************************************************/

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_DEBUG
#    define message(...) syslog(__VA_ARGS__)
#    define msgflush()
#  else
#    define message(...) printf(__VA_ARGS__)
#    define msgflush() fflush(stdout)
#  endif
#else
#  ifdef CONFIG_DEBUG
#    define message syslog
#    define msgflush()
#  else
#    define message printf
#    define msgflush() fflush(stdout)
#  endif
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __APPS_EXAMPLES_WATCHDOG_WATCHDOG_H */
