/****************************************************************************
 * drivers/usbhost/usbhost_registerclass.c
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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
#include <errno.h>
#include <debug.h>

#include <arch/irq.h>
#include <nuttx/usb/usbhost.h>

#include "usbhost_registry.h"

/****************************************************************************
 * Pre-processor Definitions
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

/****************************************************************************
 * Name: usbhost_registerclass
 *
 * Description:
 *   Register a USB host class implementation.  The caller provides an
 *   instance of struct usbhost_registry_s that contains all of the
 *   information that will be needed later to (1) associate the USB host
 *   class implementation with a connected USB device, and (2) to obtain and
 *   bind a struct usbhost_class_s instance for the device.
 *
 * Input Parameters:
 *   class - An write-able instance of struct usbhost_registry_s that will be
 *     maintained in a registry.
 *
 * Returned Values:
 *   On success, this function will return zero (OK).  Otherwise, a negated
 *   errno value is returned.
 *
 ****************************************************************************/

int usbhost_registerclass(struct usbhost_registry_s *class)
{
  irqstate_t flags;

  uvdbg("Registering class:%p nids:%d\n", class, class->nids);

  /* g_classregistry is a singly-linkedlist of class ID information added by
   * calls to usbhost_registerclass().  Since this list is accessed from USB
   * host controller interrupt handling logic, accesses to this list must be
   * protected by disabling interrupts.
   */

  flags = irqsave();

  /* Add the new class ID info to the head of the list */

  class->flink    = g_classregistry;
  g_classregistry = class;

  irqrestore(flags);
  return OK;
}

