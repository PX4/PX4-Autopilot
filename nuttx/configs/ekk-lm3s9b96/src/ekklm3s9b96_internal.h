/************************************************************************************
 * configs/ekk-lm3s9b96/src/ekklm3s9b96_internal.h
 * arch/arm/src/board/lm3s6965ek_internal.h
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Jose Pablo Rojas V. <jrojas@nx-engineering.com>
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

#ifndef __CONFIGS_EKK_LM3S9B96_SRC_EKKLM3S9B96_INTERNAL_H
#define __CONFIGS_EKK_LM3S9B96_SRC_EKKLM3S9B96_INTERNAL_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include "chip.h"
#include "lm_gpio.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* How many SSI modules does this chip support? The LM3S9B96 supports 2 SSI
 * modules (others may support more than 2 -- in such case, the following must be
 * expanded).
 */

#if LM_NSSI == 0
#  undef CONFIG_SSI0_DISABLE
#  define CONFIG_SSI0_DISABLE 1
#  undef CONFIG_SSI1_DISABLE
#  define CONFIG_SSI1_DISABLE 1
#elif LM_NSSI == 1
#  undef CONFIG_SSI1_DISABLE
#  define CONFIG_SSI1_DISABLE 1
#endif

/* EKK-LM3S9B96 Eval Kit ************************************************************/

/* GPIO Usage
 *
 * PIN SIGNAL      EVB Function
 * --- ----------- ---------------------------------------
 *  26 PA0/U0RX      Virtual COM port receive
 *  27 PA1/U0TX      Virtual COM port transmit
 *  66 PB0/USB0ID    USBID signal from the USB-On-the-Go
 *  67 PB1/USB0VBUS  USB VBUS input signal from USB-OTG
 *  92 PB4/GPIO      User pushbutton SW2.
 *  80 PC0/TCK/SWCLK JTAG or SWD clock input
 *  79 PC1/TMS/SWDIO JTAG TMS input or SWD bidirectional signal SWDIO
 *  78 PC2/TDI       JTAG TDI signal input
 *  77 PC3/TDO/SWO   JTAG TDO output or SWD trace signal SWO output.
 *  10 PD0/GPIO      User LED
 *  60 PF2/LED1      Ethernet LED1 (yellow)
 *  59 PF3/LED0      Ethernet LED0 (green)
 *  83 PH3/USB0EPEN  USB-OTG power switch
 *  76 PH4/USB0PFLT  Overcurrent input status from USB-OTG power switch
 */

/* GPIO for LED's:
 * - PD0: User LED
 */

#define LED_GPIO          (GPIO_FUNC_OUTPUT | GPIO_VALUE_ONE | GPIO_PORTD | 0)

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Name: lm_ssiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the LM3S6965 Eval Kit.
 *
 ************************************************************************************/

extern void weak_function lm_ssiinitialize(void);

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_EKK_LM3S9B96_SRC_EKKLM3S9B96_INTERNAL_H */

