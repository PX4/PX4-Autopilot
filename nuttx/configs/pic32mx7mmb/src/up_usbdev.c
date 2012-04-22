/************************************************************************************
 * configs/pic32mx7mmb/src/up_usbdev.c
 * arch/arm/src/board/up_usbdev.c
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

#if defined(CONFIG_PIC32MX_USBDEV)

/************************************************************************************
 * Definitions
 ************************************************************************************/
/*
 * PIN  NAME                            SIGNAL         NOTES
 * ---- ------------------------------- -------------- ------------------------------
 *  20  VBUSON/C1IN+/AN5/CN7/RB5        VBUSON         To USB VBUS circuitry
 *  43  C2IN+/AN3/CN5/RB3               USB_PGOOD      Power good (low if bad)
 */

#define GPIO_USB_VBUSON (GPIO_INPUT|GPIO_PORTB|GPIO_PIN5)
#define GPIO_USB_PGOOD  (GPIO_INPUT|GPIO_PORTB|GPIO_PIN3)

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: pic32mx_usbdevinitialize
 *
 * Description:
 *   Called to configure the mini-A/B J5 on the Mikroelektronika PIC32MX7 MMB for the
 *   USB device
 *
 ************************************************************************************/

void weak_function pic32mx_usbdevinitialize(void)
{
  /* Connect the PHY to the USB mini-B port.  Order and timing matter! */

  pic32mx_configgpio(GPIO_USB_PGOOD);

  /* Notes from the Sure Electronics sample code:
   *
   * "The USB specifications require that USB peripheral devices must never source
   *  current onto the Vbus pin.  Additionally, USB peripherals should not source
   *  current on D+ or D- when the host/hub is not actively powering the Vbus line.
   *  When designing a self powered (as opposed to bus powered) USB peripheral
   *  device, the firmware should make sure not to turn on the USB module and D+
   *  or D- pull up resistor unless Vbus is actively powered.  Therefore, the
   *  firmware needs some means to detect when Vbus is being powered by the host.
   *  A 5V tolerant I/O pin can be connected to Vbus (through a resistor), and
   *  can be used to detect when Vbus is high (host actively powering), or low
   *  (host is shut down or otherwise not supplying power).  The USB firmware
   *  can then periodically poll this I/O pin to know when it is okay to turn on
   *  the USB module/D+/D- pull up resistor.  When designing a purely bus powered
   *  peripheral device, it is not possible to source current on D+ or D- when the
   *  host is not actively providing power on Vbus. Therefore, implementing this
   *  bus sense feature is optional. ..."
   */

#ifdef CONFIG_USBHOST
 // pic32mx_configgpio(GPIO_USB_VBUSON);
#endif
    
  /* "If the host PC sends a GetStatus (device) request, the firmware must respond
   *  and let the host know if the USB peripheral device is currently bus powered
   *  or self powered.  See chapter 9 in the official USB specifications for details
   *  regarding this request.  If the peripheral device is capable of being both
   *  self and bus powered, it should not return a hard coded value for this request.
   *  Instead, firmware should check if it is currently self or bus powered, and
   *  respond accordingly.  If the hardware has been configured like demonstrated
   *  on the PICDEM FS USB Demo Board, an I/O pin can be polled to determine the
   *  currently selected power source. ..."
   */

#ifdef CONFIG_USB_PWRSENSE
 // pic32mx_configgpio(GPIO_USB_PWRSENSE);
#endif
}

/************************************************************************************
 * Name: pic32mx_usbpullup
 *
 * Description:
 *   If USB is supported and the board supports a pullup via GPIO (for USB
 *   software connect and disconnect), then the board software must provide
 *   stm32_pullup. See include/nuttx/usb/usbdev.h for additional description
 *   of this method.  Alternatively, if no pull-up GPIO the following EXTERN
 *   can be redefined to be NULL.
 *
 ************************************************************************************/

int pic32mx_usbpullup(FAR struct usbdev_s *dev,  bool enable)
{
  /* The Mikroelektronika PIC32MX7 MMB does not have a USB pull-up */

  return OK;
}

/************************************************************************************
 * Name: pic32mx_usbsuspend
 *
 * Description:
 *   Board logic must provide the stm32_usbsuspend logic if the USBDEV driver
 *   is used.  This function is called whenever the USB enters or leaves
 *   suspend mode. This is an opportunity for the board logic to shutdown
 *   clocks, power, etc. while the USB is suspended.
 *
 ************************************************************************************/

void pic32mx_usbsuspend(FAR struct usbdev_s *dev, bool resume)
{
  /* Do nothing */
}

#endif /* CONFIG_PIC32MX_USBDEV */
