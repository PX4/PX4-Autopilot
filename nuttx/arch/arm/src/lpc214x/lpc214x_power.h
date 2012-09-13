/************************************************************************************
 * arch/arm/src/lpc214x/lpc214x_power.h
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
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

#ifndef _ARCH_ARM_SRC_LPC214X_POWER_H
#define _ARCH_ARM_SRC_LPC214X_POWER_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Register address definitions *****************************************************/

#define LPC214X_PCON_PCON     (0xe01fc0c0)  /* Power control register */
#define LPC214X_PCON_PCONP    (0xe01fc0c4)  /* Power controls for peripherals register */

/* Register bit definitions *********************************************************/

/* Power control register */

#define LPC214X_PCON_IDL      (0x01)        /* Bit 0=1: Idle mode ON */
#define LPC214X_PCON_PD       (0x02)        /* Bit 1=1: Power down mode ON */
#define LPC214X_PCON_BODPDM   (0x04)        /* Bit 2=1: Brown out power down mode ON */
#define LPC214X_PCON_BOGD     (0x08)        /* Bit 3=1: Brown out global disable */
#define LPC214X_PCON_BORD     (0x10)        /* Bit 4=1: Brown out reset disable */

/* Peripheral power control register */

#define LPC214X_PCONP_PCTIM0  (0x00000002)  /* Bit 1=1: Timer/counter0 control */
#define LPC214X_PCONP_PCTIM1  (0x00000004)  /* Bit 2=1: Timer/counter1 control */
#define LPC214X_PCONP_PCUART0 (0x00000008)  /* Bit 3=1: UART0 control */
#define LPC214X_PCONP_PCUART1 (0x00000010)  /* Bit 4=1: UART1 control */
#define LPC214X_PCONP_PCWM0   (0x00000020)  /* Bit 5=1: PWM0 control */
#define LPC214X_PCONP_PCI2C0  (0x00000080)  /* Bit 7=1: I2C0 control */
#define LPC214X_PCONP_PCSPI0  (0x00000100)  /* Bit 8=1: SPI0 control */
#define LPC214X_PCONP_PCRTC   (0x00000200)  /* Bit 9=1: RTCcontrol */
#define LPC214X_PCONP_PCSPI1  (0x00000400)  /* Bit 10=1: SPI1 control */
#define LPC214X_PCONP_PCAD0   (0x00001000)  /* Bit 12=1: A/C converter 0 control */
#define LPC214X_PCONP_PCI2C1  (0x00080000)  /* Bit 19=1: I2C1 control */
#define LPC214X_PCONP_PCAD1   (0x00100000)  /* Bit 20=1: A/C converter 1 control */
#define LPC214X_PCONP_PCUSB   (0x80000000)  /* Bit 31=1: USB power/clock control */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#endif  /* _ARCH_ARM_SRC_LPC214X_POWER_H */
