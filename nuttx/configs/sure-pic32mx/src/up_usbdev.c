/************************************************************************************
 * configs/sure-pic32mx/src/up_usbdev.c
 * arch/arm/src/board/up_usbdev.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *   - Sample code and schematics provided with the Sure Electronics PIC32 board.
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

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/spi.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "chip.h"
#include "pic32mx-internal.h"
#include "sure-internal.h"

#if defined(CONFIG_PIC32MX_USBDEV)

/************************************************************************************
 * Definitions
 ************************************************************************************/
/*
 * PIN  NAME                            SIGNAL         NOTES
 * ---- ------------------------------- -------------- ------------------------------
 *  11  AN5/C1IN+/Vbuson/CN7/RB5        Vbuson/AN5/RB5 To USB VBUS circuitry
 *  43  U1CTS/SDA1/IC2/INT2/RD9         USB_OPTEN      USB PHY
 *  44  SCL1/IC3/PMCS2/PMA15/INT3/RD10  USB_OPT        USB PHY
 */

#define GPIO_USB_VBUSON (GPIO_INPUT|GPIO_PORTB|GPIO_PIN5)
#define GPIO_USB_OPTEN  (GPIO_OUTPUT|GPIO_VALUE_ZERO|GPIO_PORTD|GPIO_PIN9)
#define GPIO_USB_OPT    (GPIO_OUTPUT|GPIO_VALUE_ONE|GPIO_PORTD|GPIO_PIN10)

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
 *   Called to configure the mini-B PHY on the Sure PIC32MX board for the USB device
 *
 ************************************************************************************/

void weak_function pic32mx_usbdevinitialize(void)
{
  /* Connect the PHY to the USB mini-B port.  Order and timing matter! */

  pic32mx_configgpio(GPIO_USB_OPTEN);
  pic32mx_configgpio(GPIO_USB_OPT);

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
#endif /* CONFIG_PIC32MX_USBDEV */
