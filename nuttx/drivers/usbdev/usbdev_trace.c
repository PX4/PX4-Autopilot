/****************************************************************************
 * drivers/usbdev/usbdev_trace.c
 *
 *   Copyright (C) 2008-2010, 2012 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <errno.h>
#include <debug.h>

#include <arch/irq.h>
#include <nuttx/usb/usbdev_trace.h>
#undef usbtrace

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_USBDEV_TRACE_NRECORDS
#  define CONFIG_USBDEV_TRACE_NRECORDS 128
#endif

#ifndef CONFIG_USBDEV_TRACE_INITIALIDSET
#  define CONFIG_USBDEV_TRACE_INITIALIDSET 0
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_USBDEV_TRACE
static struct usbtrace_s g_trace[CONFIG_USBDEV_TRACE_NRECORDS];
static uint16_t g_head = 0;
static uint16_t g_tail = 0;
#endif

#if defined(CONFIG_USBDEV_TRACE) || (defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_USB))
static usbtrace_idset_t g_maskedidset = CONFIG_USBDEV_TRACE_INITIALIDSET;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/*******************************************************************************
 * Name: usbtrace_enable
 *
 * Description:
 *  Enable/disable tracing per trace ID.  The initial state is all IDs enabled.
 *
 * Input Parameters:
 *  idset - The bitset of IDs to be masked.  TRACE_ALLIDS enables all IDS; zero
 *  masks all IDs.
 *
 * Returned Value:
 *  The previous idset value.
 *
 * Assumptions:
 * - May be called from an interrupt handler
 *
 *******************************************************************************/

#if defined(CONFIG_USBDEV_TRACE) || (defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_USB))
usbtrace_idset_t usbtrace_enable(usbtrace_idset_t idset)
{
  irqstate_t flags;
  usbtrace_idset_t ret;

  /* The following read and write must be atomic */

  flags         = irqsave();
  ret           = g_maskedidset;
  g_maskedidset = idset;
  irqrestore(flags);
  return ret;
}
#endif /* CONFIG_USBDEV_TRACE || CONFIG_DEBUG && CONFIG_DEBUG_USB */

/*******************************************************************************
 * Name: usbtrace
 *
 * Description:
 *  Record a USB event (tracing must be enabled)
 *
 * Assumptions:
 *   May be called from an interrupt handler
 *
 *******************************************************************************/

#if defined(CONFIG_USBDEV_TRACE) || (defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_USB))
void usbtrace(uint16_t event, uint16_t value)
{
  irqstate_t flags;

  /* Check if tracing is enabled for this ID */

  flags = irqsave();
  if ((g_maskedidset & TRACE_ID2BIT(event)) != 0)
    {
#ifdef CONFIG_USBDEV_TRACE
      /* Yes... save the new trace data at the head */

      g_trace[g_head].event = event;
      g_trace[g_head].value = value;

      /* Increment the head and (probably) the tail index */

      if (++g_head >= CONFIG_USBDEV_TRACE_NRECORDS)
        {
          g_head = 0;
        }

      if (g_head == g_tail)
        {
          if (++g_tail >= CONFIG_USBDEV_TRACE_NRECORDS)
            {
              g_tail = 0;
            }
        }
#else
      /* Just print the data using lowsyslog */

      usbtrace_trprintf((trprintf_t)lowsyslog, event, value);
#endif
    }
  irqrestore(flags);
}
#endif /* CONFIG_USBDEV_TRACE || CONFIG_DEBUG && CONFIG_DEBUG_USB */

/*******************************************************************************
 * Name: usbtrace_enumerate
 *
 * Description:
 *   Enumerate all buffer trace data (will temporarily disable tracing)
 *
 * Assumptions:
 *   NEVER called from an interrupt handler
 *
 *******************************************************************************/

#ifdef CONFIG_USBDEV_TRACE
int usbtrace_enumerate(trace_callback_t callback, void *arg)
{
  uint16_t ndx;
  uint32_t idset;
  int ret = OK;

  /* Temporarily disable tracing */

  idset = usbtrace_enable(0);

  /* Visit every entry, starting with the tail */

  for (ndx = g_tail; ndx != g_head; )
    {
      /* Call the user provided callback */

      ret = callback(&g_trace[ndx], arg);
      if (ret != OK)
        {
          /* Abort the enumeration */

          break;
        }

      /* Increment the index */

      if (++ndx >= CONFIG_USBDEV_TRACE_NRECORDS)
        {
          ndx = 0;
        }
    }

  /* Discard the trace data after it has been reported */

  g_tail = g_head;

  /* Restore tracing state */

  (void)usbtrace_enable(idset);
  return ret;
}
#endif /* CONFIG_USBDEV_TRACE */
