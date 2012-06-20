/************************************************************************************
 * configs/mirtoo/src/up_boot.c
 * arch/mips/src/board/up_boot.c
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

#include <debug.h>

#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"

#include "pic32mx-internal.h"
#include "pic32mx-pps.h"
#include "mirtoo-internal.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

#define GPIO_U2TX  (GPIO_OUTPUT|GPIO_PORTB|GPIO_PIN10)
#define GPIO_U2RX  (GPIO_INPUT|GPIO_PORTB|GPIO_PIN11)

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: pic32mx_uartinitialize
 *
 * Description:
 *   When mounted on the DTX1-4000L EV-kit1 board, serial output is avaiable through
 *   an FT230X device via the FUNC0 and FUNC1 module outputs
 * 
 *   ---------- ------ ----- ------ -------------------------
 *      BOARD   OUTPUT  PIN  SIGNAL NOTES
 *   ---------- ------ ----- ------ -------------------------
 *   FT230X RXD  FUNC0 RPB11  U2RX  UART2 RX (Also PGEC2)
 *   FT230X TXD  FUNC1 RPB10  U2TX  UART2 TX (Also PGED2)
 *
 ************************************************************************************/

static inline void pic32mx_uartinitialize(void)
{
#ifdef CONFIG_PIC32MX_UART2
  /* Make sure that TRIS pins are set correctly.  Configure the UART pins as digital
   * inputs and outputs first.
   */

  pic32mx_configgpio(GPIO_U2TX);
  pic32mx_configgpio(GPIO_U2RX);

  /* Configure UART TX and RX pins to RPB10 and 11, respectively */

  putreg32(PPS_INSEL_RPB11,  PIC32MX_PPS_U2RXR);
  putreg32(PPS_OUTSEL_U2TX, PIC32MX_PPS_RPB10R);
#endif
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: pic32mx_boardinitialize
 *
 * Description:
 *   All PIC32MX architectures must provide the following entry point.  This entry
 *   point is called early in the intitialization -- after all memory has been
 *   configured and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

void pic32mx_boardinitialize(void)
{
  /* Configure the console UART */

  pic32mx_uartinitialize();

  /* Configure SPI chip selects if 1) at least one SPI is enabled, and 2) the weak
   * function pic32mx_spi2initialize() has been brought into the link.
   */

#ifdef CONFIG_PIC32MX_SPI2
  if (pic32mx_spi2initialize)
    {
      pic32mx_spi2initialize();
    }
#endif

  /* Configure on-board LEDs if LED support has been selected. */

#ifdef CONFIG_ARCH_LEDS
  pic32mx_ledinit();
#endif
}
