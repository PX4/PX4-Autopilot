/****************************************************************************
 * examples/examples/can/can.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

#ifndef __APPS_EXAMPLES_CAN_CAN_H
#define __APPS_EXAMPLES_CAN_CAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* This test depends on these specific CAN configurations settings (your
 * specific CAN settings might require additional settings).
 *
 * CONFIG_CAN - Enables CAN support.
 * CONFIG_CAN_LOOPBACK - A CAN driver may or may not support a loopback
 *   mode for testing. The STM32 CAN driver does support loopback mode.
 *
 * Specific configuration options for this example include:
 *
 * CONFIG_NSH_BUILTIN_APPS - Build the CAN test as an NSH built-in function.
 *   Default: Built as a standalone problem
 * CONFIG_CAN_LOOPBACK
 * CONFIG_EXAMPLES_CAN_DEVPATH - The path to the CAN device. Default: /dev/can0
 * CONFIG_EXAMPLES_CAN_NMSGS - If CONFIG_NSH_BUILTIN_APPS
 *   is defined, then the number of loops is provided on the command line
 *   and this value is ignored.  Otherwise, this number of CAN message is
 *   collected and the program terminates.  Default:  If built as an NSH
 *   built-in, the default is 32.  Otherwise messages are sent and received
 *   indefinitely.
 * CONFIG_EXAMPLES_CAN_READONLY - Only receive messages
 * CONFIG_EXAMPLES_CAN_WRITEONLY - Only send messages
 */

#ifndef CONFIG_CAN
#  error "CAN device support is not enabled (CONFIG_CAN)"
#endif

#ifndef CONFIG_CAN_LOOPBACK
#  warning "CAN loopback is not enabled (CONFIG_CAN_LOOPBACK)"
#endif

#ifndef CONFIG_EXAMPLES_CAN_DEVPATH
#  define CONFIG_EXAMPLES_CAN_DEVPATH "/dev/can0"
#endif

#if defined(CONFIG_NSH_BUILTIN_APPS) && !defined(CONFIG_EXAMPLES_CAN_NMSGS)
#  define CONFIG_EXAMPLES_CAN_NMSGS 32
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

/****************************************************************************
 * Name: can_devinit()
 *
 * Description:
 *   Perform architecuture-specific initialization of the CAN hardware.  This
 *   interface must be provided by all configurations using apps/examples/can
 *
 ****************************************************************************/

int can_devinit(void);

#endif /* __APPS_EXAMPLES_CAN_CAN_H */
