/****************************************************************************
 * examples/cdcacm/cdcacm.h
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

#ifndef __EXAMPLES_CDCACM_CDCACM_H
#define __EXAMPLES_CDCACM_CDCACM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdlib.h>

#include <nuttx/usb/usbdev_trace.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* Prerequisites */

#ifndef CONFIG_USBDEV
#  error "CONFIG_USBDEV is not defined"
#endif

#ifndef CONFIG_CDCACM
#  error "CONFIG_CDCACM is not defined"
#endif

#ifndef CONFIG_NSH_BUILTIN_APPS
#  error "This example can only be built as an NSH built-in application"
#endif

/* Default configuration values */

#ifndef CONFIG_EXAMPLES_CDCACM_DEVMINOR
#  define CONFIG_EXAMPLES_CDCACM_DEVMINOR 0
#endif

/* Trace Configuration ******************************************************/

#ifdef CONFIG_EXAMPLES_CDCACM_TRACEINIT
#  define TRACE_INIT_BITS       (TRACE_INIT_BIT)
#else
#  define TRACE_INIT_BITS       (0)
#endif

#define TRACE_ERROR_BITS        (TRACE_DEVERROR_BIT|TRACE_CLSERROR_BIT)

#ifdef CONFIG_EXAMPLES_CDCACM_TRACECLASS
#  define TRACE_CLASS_BITS      (TRACE_CLASS_BIT|TRACE_CLASSAPI_BIT|TRACE_CLASSSTATE_BIT)
#else
#  define TRACE_CLASS_BITS      (0)
#endif

#ifdef CONFIG_EXAMPLES_CDCACM_TRACETRANSFERS
#  define TRACE_TRANSFER_BITS   (TRACE_OUTREQQUEUED_BIT|TRACE_INREQQUEUED_BIT|TRACE_READ_BIT|\
                                 TRACE_WRITE_BIT|TRACE_COMPLETE_BIT)
#else
#  define TRACE_TRANSFER_BITS   (0)
#endif

#ifdef CONFIG_EXAMPLES_CDCACM_TRACECONTROLLER
#  define TRACE_CONTROLLER_BITS (TRACE_EP_BIT|TRACE_DEV_BIT)
#else
#  define TRACE_CONTROLLER_BITS (0)
#endif

#ifdef CONFIG_EXAMPLES_CDCACM_TRACEINTERRUPTS
#  define TRACE_INTERRUPT_BITS  (TRACE_INTENTRY_BIT|TRACE_INTDECODE_BIT|TRACE_INTEXIT_BIT)
#else
#  define TRACE_INTERRUPT_BITS  (0)
#endif

#define TRACE_BITSET            (TRACE_INIT_BITS|TRACE_ERROR_BITS|TRACE_CLASS_BITS|\
                                 TRACE_TRANSFER_BITS|TRACE_CONTROLLER_BITS|TRACE_INTERRUPT_BITS)

/* Debug ********************************************************************/

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_DEBUG
#    define message(...) lowsyslog(__VA_ARGS__)
#    define msgflush()
#  else
#    define message(...) printf(__VA_ARGS__)
#    define msgflush() fflush(stdout)
#  endif
#else
#  ifdef CONFIG_DEBUG
#    define message lowsyslog
#    define msgflush()
#  else
#    define message printf
#    define msgflush() fflush(stdout)
#  endif
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* All global variables used by this example are packed into a structure in
 * order to avoid name collisions.
 */

struct cdcacm_state_s
{
  /* This is the handle that references to this particular USB storage driver
   * instance.  It is only needed if the USB mass storage device example is
   * built using CONFIG_NSH_BUILTIN_APPS.  In this case, the value
   * of the driver handle must be remembered between the 'sercon' and 'msdis'
   * commands.
   */

  FAR void *handle;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* All global variables used by this example are packed into a structure in
 * order to avoid name collisions.
 */

extern struct cdcacm_state_s g_cdcacm;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#endif /* __EXAMPLES_CDCACM_CDCACM_H */
