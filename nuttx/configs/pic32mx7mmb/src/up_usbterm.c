/************************************************************************************
 * configs/pic32mx7mmb/src/up_usbterm.c
 * arch/arm/src/board/up_usbterm.c
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <debug.h>

#include <nuttx/usb/usbdev.h>

#include "pic32mx-internal.h"
#include "pic32mx7mmb_internal.h"

#if defined(CONFIG_PIC32MX_USBDEV) && defined(CONFIG_EXAMPLES_USBTERM_DEVINIT)

/************************************************************************************
 * Definitions
 ************************************************************************************/

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/****************************************************************************
 * Name:
 *
 * Description:
 *   If CONFIG_EXAMPLES_USBTERM_DEVINIT is defined, then the example will
 *   call this user provided function as part of its initialization.
 *
 ****************************************************************************/

int usbterm_devinit(void)
{
  /* The Mikroelektronika PIC32MX7 MMB has no way to know when the USB is
   * connected.  So we will fake it and tell the USB driver that the USB is
   * connected now.
   *
   * If examples/usbterm is built as an NSH built-in application, then
   * pic32mx_usbattach() will be called in nsh_archinitialize().
   */

#ifndef CONFIG_EXAMPLES_USBTERM_BUILTIN
  pic32mx_usbattach();
#endif
  return OK;
}

/****************************************************************************
 * Name:
 *
 * Description:
 *   If CONFIG_EXAMPLES_USBTERM_DEVINIT is defined, then the example will
 *   call this user provided function as part of its termination sequence.
 *
 ****************************************************************************/

void usbterm_devuninit(void)
{
  /* Tell the USB driver that the USB is no longer connected */

  pic32mx_usbdetach();
}

#endif /* CONFIG_PIC32MX_USBDEV && CONFIG_EXAMPLES_USBTERM_DEVINIT */
