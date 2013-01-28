/****************************************************************************
 * examples/usbterm/usbterm.h
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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

#ifndef __APPS_EXAMPLES_USBTERM_USBTERM_H
#define __APPS_EXAMPLES_USBTERM_USBTERM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifndef CONFIG_EXAMPLES_USBTERM_BUFLEN
#  define CONFIG_EXAMPLES_USBTERM_BUFLEN 256
#endif

#ifdef CONFIG_EXAMPLES_USBTERM_TRACEINIT
#  define TRACE_INIT_BITS       (TRACE_INIT_BIT)
#else
#  define TRACE_INIT_BITS       (0)
#endif

#define TRACE_ERROR_BITS        (TRACE_DEVERROR_BIT|TRACE_CLSERROR_BIT)

#ifdef CONFIG_EXAMPLES_USBTERM_TRACECLASS
#  define TRACE_CLASS_BITS      (TRACE_CLASS_BIT|TRACE_CLASSAPI_BIT|TRACE_CLASSSTATE_BIT)
#else
#  define TRACE_CLASS_BITS      (0)
#endif

#ifdef CONFIG_EXAMPLES_USBTERM_TRACETRANSFERS
#  define TRACE_TRANSFER_BITS   (TRACE_OUTREQQUEUED_BIT|TRACE_INREQQUEUED_BIT|TRACE_READ_BIT|\
                                 TRACE_WRITE_BIT|TRACE_COMPLETE_BIT)
#else
#  define TRACE_TRANSFER_BITS   (0)
#endif

#ifdef CONFIG_EXAMPLES_USBTERM_TRACECONTROLLER
#  define TRACE_CONTROLLER_BITS (TRACE_EP_BIT|TRACE_DEV_BIT)
#else
#  define TRACE_CONTROLLER_BITS (0)
#endif

#ifdef CONFIG_EXAMPLES_USBTERM_TRACEINTERRUPTS
#  define TRACE_INTERRUPT_BITS  (TRACE_INTENTRY_BIT|TRACE_INTDECODE_BIT|TRACE_INTEXIT_BIT)
#else
#  define TRACE_INTERRUPT_BITS  (0)
#endif

#define TRACE_BITSET            (TRACE_INIT_BITS|TRACE_ERROR_BITS|TRACE_CLASS_BITS|\
                                 TRACE_TRANSFER_BITS|TRACE_CONTROLLER_BITS|TRACE_INTERRUPT_BITS)

#ifdef CONFIG_CDCACM
#  define USBTERM_DEVNAME "/dev/ttyACM0"
#else
#  define USBTERM_DEVNAME "/dev/ttyUSB0"
#endif

/* Debug ********************************************************************/

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_DEBUG
#    define message(...) syslog(__VA_ARGS__)
#    define trmessage    syslog
#  else
#    define message(...) printf(__VA_ARGS__)
#    define trmessage    printf
#  endif
#else
#  ifdef CONFIG_DEBUG
#    define message      lowsyslog
#    define trmessage    lowsyslog
#  else
#    define message      printf
#    define trmessage    printf
#  endif
#endif

#define IOBUFFER_SIZE 256

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* All USB terminal state data is packaged in a single structure to minimize
 * name conflicts with other global symbols -- a poor man's name space.
 */

struct usbterm_globals_s
{
  FILE     *instream;  /* Stream for incoming USB data */
  FILE     *outstream; /* Stream for outgoing USB data */
  pthread_t listener;  /* USB terminal listener thread */
  bool      peer;      /* True: A peer is connected to the serial port on
                        * the remote host */

  /* Buffers for incoming and outgoing data */

  char     inbuffer[CONFIG_EXAMPLES_USBTERM_BUFLEN];
  char     outbuffer[CONFIG_EXAMPLES_USBTERM_BUFLEN];
};

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/* USB terminal state data */

extern struct usbterm_globals_s g_usbterm;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
/****************************************************************************
 * Name:
 *
 * Description:
 *   If CONFIG_EXAMPLES_USBTERM_DEVINIT is defined, then the example will
 *   call this user provided function as part of its initialization.
 *
 ****************************************************************************/

#ifdef CONFIG_EXAMPLES_USBTERM_DEVINIT
int usbterm_devinit(void);
#endif

/****************************************************************************
 * Name:
 *
 * Description:
 *   If CONFIG_EXAMPLES_USBTERM_DEVINIT is defined, then the example will
 *   call this user provided function as part of its termination sequeunce.
 *
 ****************************************************************************/

#ifdef CONFIG_EXAMPLES_USBTERM_DEVINIT
void usbterm_devuninit(void);
#endif

#endif /* __APPS_EXAMPLES_USBTERM_USBTERM_H */
