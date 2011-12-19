/****************************************************************************
 * drivers/usbdev/usbdev_trprintf.c
 *
 *   Copyright (C) 2008-2010 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <debug.h>

#include <nuttx/usb/usbdev_trace.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/*******************************************************************************
 * Name: usbtrace_trprintf
 *
 * Description:
 *   Print the trace record using the supplied printing function
 *
 *******************************************************************************/

void usbtrace_trprintf(trprintf_t trprintf, uint16_t event, uint16_t value)
{
  switch (event)
    {
    case TRACE_DEVINIT:
      trprintf("USB controller initialization: %04x\n", value);
      break;

    case TRACE_DEVUNINIT:
      trprintf("USB controller un-initialization: %04x\n", value);
      break;

    case TRACE_DEVREGISTER:
      trprintf("usbdev_register(): %04x\n", value);
      break;

    case TRACE_DEVUNREGISTER:
      trprintf("usbdev_unregister(): %04x\n", value);
      break;

    case TRACE_EPCONFIGURE:
      trprintf("Endpoint configure(): %04x\n", value);
      break;

    case TRACE_EPDISABLE:
      trprintf("Endpoint disable(): %04x\n", value);
      break;

    case TRACE_EPALLOCREQ:
      trprintf("Endpoint allocreq(): %04x\n", value);
      break;

    case TRACE_EPFREEREQ:
      trprintf("Endpoint freereq(): %04x\n", value);
      break;

    case TRACE_EPALLOCBUFFER:
      trprintf("Endpoint allocbuffer(): %04x\n", value);
      break;

    case TRACE_EPFREEBUFFER:
      trprintf("Endpoint freebuffer(): %04x\n", value);
      break;

    case TRACE_EPSUBMIT:
      trprintf("Endpoint submit(): %04x\n", value);
      break;

    case TRACE_EPCANCEL:
      trprintf("Endpoint cancel(): %04x\n", value);
      break;

    case TRACE_EPSTALL:
      trprintf("Endpoint stall(true): %04x\n", value);
      break;

    case TRACE_EPRESUME:
      trprintf("Endpoint stall(false): %04x\n", value);
      break;

    case TRACE_DEVALLOCEP:
      trprintf("Device allocep(): %04x\n", value);
      break;

    case TRACE_DEVFREEEP:
      trprintf("Device freeep(): %04x\n", value);
      break;

    case TRACE_DEVGETFRAME:
      trprintf("Device getframe(): %04x\n", value);
      break;

    case TRACE_DEVWAKEUP:
      trprintf("Device wakeup(): %04x\n", value);
      break;

    case TRACE_DEVSELFPOWERED:
      trprintf("Device selfpowered(): %04x\n", value);
      break;

    case TRACE_DEVPULLUP:
      trprintf("Device pullup(): %04x\n", value);
      break;

    case TRACE_CLASSBIND:
      trprintf("Class bind(): %04x\n", value);
      break;

    case TRACE_CLASSUNBIND:
      trprintf("Class unbind(): %04x\n", value);
      break;

    case TRACE_CLASSDISCONNECT:
      trprintf("Class disconnect(): %04x\n", value);
      break;

    case TRACE_CLASSSETUP:
      trprintf("Class setup(): %04x\n", value);
      break;

    case TRACE_CLASSSUSPEND:
      trprintf("Class suspend(): %04x\n", value);
      break;

    case TRACE_CLASSRESUME:
      trprintf("Class resume(): %04x\n", value);
      break;

    case TRACE_CLASSRDCOMPLETE:
      trprintf("Class RD request complete: %04x\n", value);
      break;

    case TRACE_CLASSWRCOMPLETE:
      trprintf("Class WR request complete: %04x\n", value);
      break;

    default:
      switch (TRACE_ID(event))
        {
        case TRACE_CLASSAPI_ID:        /* Other class driver system API calls */
          trprintf("Class API call %d: %04x\n", TRACE_DATA(event), value);
          break;

        case TRACE_CLASSSTATE_ID:      /* Track class driver state changes */
          trprintf("Class state %d: %04x\n", TRACE_DATA(event), value);
          break;

        case TRACE_INTENTRY_ID:        /* Interrupt handler entry */
          trprintf("Interrupt %d entry: %04x\n", TRACE_DATA(event), value);
          break;

        case TRACE_INTDECODE_ID:       /* Decoded interrupt event */
          trprintf("Interrupt decode %d: %04x\n", TRACE_DATA(event), value);
          break;

        case TRACE_INTEXIT_ID:         /* Interrupt handler exit */
          trprintf("Interrupt %d exit: %04x\n", TRACE_DATA(event), value);
          break;

        case TRACE_OUTREQQUEUED_ID:    /* Request queued for OUT endpoint */
          trprintf("EP%d OUT request queued: %04x\n", TRACE_DATA(event), value);
          break;

        case TRACE_INREQQUEUED_ID:     /* Request queued for IN endpoint */
          trprintf("EP%d IN request queued: %04x\n", TRACE_DATA(event), value);
          break;

        case TRACE_READ_ID:            /* Read (OUT) action */
          trprintf("EP%d OUT read: %04x\n", TRACE_DATA(event), value);
          break;

        case TRACE_WRITE_ID:           /* Write (IN) action */
          trprintf("EP%d IN write: %04x\n", TRACE_DATA(event), value);
          break;

        case TRACE_COMPLETE_ID:        /* Request completed */
          trprintf("EP%d request complete: %04x\n", TRACE_DATA(event), value);
          break;

        case TRACE_DEVERROR_ID:        /* USB controller driver error event */
          trprintf("Controller error: %02x:%04x\n", TRACE_DATA(event), value);
          break;

        case TRACE_CLSERROR_ID:        /* USB class driver error event */
          trprintf("Class error: %02x:%04x\n", TRACE_DATA(event), value);
          break;

        default:
          trprintf("Unrecognized event: %02x:%02x:%04x\n",
                TRACE_ID(event) >> 8, TRACE_DATA(event), value);
          break;
        }
    }
}
