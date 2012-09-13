/************************************************************************************
 * arch/arm/src/lpc2378/lpc23xx_pinsel.h
 *
 *   Copyright (C) 2010 Rommel Marcelo. All rights reserved.
 *   Author: Rommel Marcelo
 *
 * This file is part of the NuttX RTOS and based on the lpc2148 port:
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
 ************************************************************************************/

#ifndef _ARCH_ARM_SRC_LPC23XX_PINSEL_H
#define _ARCH_ARM_SRC_LPC23XX_PINSEL_H

/************************************************************************************
 * Included Files
 ************************************************************************************/
#include "chip.h"
/************************************************************************************
 * Definitions
 ************************************************************************************/

#define PINSEL_BASE			0xE002C000

#define pinsel_putreg8(v,o)  putreg8( (v), LPC23XX_PINSEL_BASE+(o) )
#define pinsel_putreg(v,r)   putreg32( (v),LPC23XX_PINSEL_BASE+ (r) )
#define pinsel_getreg(o)     getreg32( PINSEL_BASE+(o) )

/* Register address definitions *****************************************************/
#define LPC23XX_PINSEL0		(PINSEL_BASE + PINSEL0_OFFSET)
#define LPC23XX_PINSEL1		(PINSEL_BASE + PINSEL1_OFFSET)
#define LPC23XX_PINSEL2		(PINSEL_BASE + PINSEL2_OFFSET)
#define LPC23XX_PINSEL3		(PINSEL_BASE + PINSEL3_OFFSET)
#define LPC23XX_PINSEL4		(PINSEL_BASE + PINSEL4_OFFSET)
#define LPC23XX_PINSEL5		(PINSEL_BASE + PINSEL5_OFFSET)
#define LPC23XX_PINSEL6		(PINSEL_BASE + PINSEL6_OFFSET)
#define LPC23XX_PINSEL7		(PINSEL_BASE + PINSEL7_OFFSET)
#define LPC23XX_PINSEL8		(PINSEL_BASE + PINSEL8_OFFSET)
#define LPC23XX_PINSEL9		(PINSEL_BASE + PINSEL9_OFFSET)
#define LPC23XX_PINSEL10	(PINSEL_BASE + PINSEL10_OFFSET)

/* PINSEL 0 */
#define PSEL0_P00_GPIO      (0x00000000)
#define PSEL0_P00_RD1       (0x00000001)
#define PSEL0_P00_TXD3      (0x00000002)
#define PSEL0_P00_SDA1      (0x00000003)
#define PSEL0_P00_MASK      (0x00000003)

#define PSEL0_P0_1_GPIO      (0x00000000)
#define PSEL0_P0_1_TD1       (0x00000004)
#define PSEL0_P0_1_RXD3      (0x00000008)
#define PSEL0_P0_1_SCL1      (0x0000000c)
#define PSEL0_P0_1_MASK      (0x0000000c)

#define PSEL0_P0_2_GPIO      (0x00000000)
#define PSEL0_P0_2_TXD0      (0x00000010)
#define PSEL0_P0_2_RSVD2     (0x00000020)
#define PSEL0_P0_2_RSVD3     (0x00000030)
#define PSEL0_P0_2_MASK      (0x00000030)

#define PSEL0_P0_3_GPIO      (0x00000000)
#define PSEL0_P0_3_RXD0      (0x00000040)
#define PSEL0_P0_3_RSVD2     (0x00000080)
#define PSEL0_P0_3_RSVD3     (0x000000c0)
#define PSEL0_P0_3_MASK      (0x000000c0)

#define PSEL0_P0_4_GPIO      (0x00000000)
#define PSEL0_P0_4_I2SRX_CLK (0x00000100)
#define PSEL0_P0_4_RD2       (0x00000200)
#define PSEL0_P0_4_CAP20     (0x00000300)
#define PSEL0_P0_4_MASK      (0x00000400)

#define PSEL0_P0_5_GPIO      (0x00000000)
#define PSEL0_P0_5_I2SRX_WS  (0x00000400)
#define PSEL0_P0_5_TD2       (0x00000800)
#define PSEL0_P0_5_CAP21     (0x00000c00)
#define PSEL0_P0_5_MASK      (0x00000c00)

#define PSEL0_P0_6_GPIO      (0x00000000)
#define PSEL0_P0_6_I2SRX_SDA (0x00001000)
#define PSEL0_P0_6_SSEL1     (0x00002000)
#define PSEL0_P0_6_MAT20     (0x00003000)
#define PSEL0_P0_6_MASK      (0x00003000)

#define PSEL0_P0_7_GPIO      (0x00000000)
#define PSEL0_P0_7_I2STX_CLK (0x00004000)
#define PSEL0_P0_7_SCK1      (0x00008000)
#define PSEL0_P0_7_MAT21     (0x0000c000)
#define PSEL0_P0_7_MASK      (0x0000c000)

#define PSEL0_P0_8_GPIO      (0x00000000)
#define PSEL0_P0_8_I2STX_WS  (0x00010000)
#define PSEL0_P0_8_MISO1     (0x00020000)
#define PSEL0_P0_8_MAT22     (0x00030000)
#define PSEL0_P0_8_MASK      (0x00030000)

#define PSEL0_P0_9_GPIO      (0x00000000)
#define PSEL0_P0_9_I2STX_SDA (0x00040000)
#define PSEL0_P0_9_MOSI1     (0x00080000)
#define PSEL0_P0_9_MAT23     (0x000c0000)
#define PSEL0_P0_9_MASK      (0x000c0000)

#define PSEL0_P0_10_GPIO     (0x00000000)
#define PSEL0_P0_10_TXD2     (0x00100000)
#define PSEL0_P0_10_SDA2     (0x00200000)
#define PSEL0_P0_10_MAT30    (0x00300000)
#define PSEL0_P0_10_MASK     (0x00300000)

#define PSEL0_P0_11_GPIO     (0x00000000)
#define PSEL0_P0_11_RXD2     (0x00400000)
#define PSEL0_P0_11_SCL2     (0x00800000)
#define PSEL0_P0_11_MAT31    (0x00c00000)
#define PSEL0_P0_11_MASK     (0x00c00000)

#define PSEL0_P0_12_GPIO     (0x00000000)
#define PSEL0_P0_12_RSVD1    (0x01000000)
#define PSEL0_P0_12_MISO1    (0x02000000)
#define PSEL0_P0_12_AD06     (0x03000000)
#define PSEL0_P0_12_MASK     (0x03000000)

#define PSEL0_P0_13_GPIO     (0x00000000)
#define PSEL0_P0_13_USB_UPLED2     (0x04000000)
#define PSEL0_P0_13_MOSI1    (0x08000000)
#define PSEL0_P0_13_AD07     (0x0c000000)
#define PSEL0_P0_13_MASK     (0x0c000000)

#define PSEL0_P0_14_GPIO     (0x00000000)
#define PSEL0_P0_14_RSVD1    (0x10000000)
#define PSEL0_P0_14_USB_CONNECT2    (0x20000000)
#define PSEL0_P0_14_SSEL1    (0x30000000)
#define PSEL0_P0_14_MASK     (0x30000000)

#define PSEL0_P0_15_GPIO     (0x00000000)
#define PSEL0_P0_15_TXD1     (0x40000000)
#define PSEL0_P0_15_SCK0     (0x80000000)
#define PSEL0_P0_15_SCK      (0xc0000000)
#define PSEL0_P0_15_MASK     (0xc0000000)

/* PINSEL 1 */
#define PSEL1_P0_16_GPIO     (0x00000000)
#define PSEL1_P0_16_RXD1     (0x00000001)
#define PSEL1_P0_16_SSEL0    (0x00000002)
#define PSEL1_P0_16_SSEL     (0x00000003)
#define PSEL1_P0_16_MASK     (0x00000003)

#define PSEL1_P0_17_GPIO     (0x00000000)
#define PSEL1_P0_17_CTS1     (0x00000004)
#define PSEL1_P0_17_MISO0    (0x00000008)
#define PSEL1_P0_17_MISO     (0x0000000c)
#define PSEL1_P0_17_MASK     (0x0000000c)

#define PSEL1_P0_18_GPIO     (0x00000000)
#define PSEL1_P0_18_DCD1     (0x00000010)
#define PSEL1_P0_18_MOSI0    (0x00000020)
#define PSEL1_P0_18_MOSI     (0x00000030)
#define PSEL1_P0_18_MASK     (0x00000030)

#define PSEL1_P0_19_GPIO     (0x00000000)
#define PSEL1_P0_19_DSR1     (0x00000040)
#define PSEL1_P0_19_MCICLK   (0x00000080)
#define PSEL1_P0_19_SDA1     (0x000000c0)
#define PSEL1_P0_19_MASK     (0x000000c0)

#define PSEL1_P0_20_GPIO     (0x00000000)
#define PSEL1_P0_20_DTR1     (0x00000100)
#define PSEL1_P0_20_MCICMD   (0x00000200)
#define PSEL1_P0_20_SCL1     (0x00000300)
#define PSEL1_P0_20_MASK     (0x00000300)

#define PSEL1_P0_21_GPIO     (0x00000000)
#define PSEL1_P0_21_RI1      (0x00000400)
#define PSEL1_P0_21_MCIPWR   (0x00000800)
#define PSEL1_P0_21_RD1      (0x00000c00)
#define PSEL1_P0_21_MASK     (0x00000c00)

#define PSEL1_P0_22_GPIO     (0x00000000)
#define PSEL1_P0_22_RTS1     (0x00001000)
#define PSEL1_P0_22_MCIDA0   (0x00002000)
#define PSEL1_P0_22_TD1      (0x00003000)
#define PSEL1_P0_22_MASK     (0x00003000)

#define PSEL1_P0_23_GPIO     (0x00000000)
#define PSEL1_P0_23_AD00     (0x00004000)
#define PSEL1_P0_23_I2SRX_CLK (0x00008000)
#define PSEL1_P0_23_CAP30    (0x0000c000)
#define PSEL1_P0_23_MASK     (0x0000c000)

#define PSEL1_P0_24_GPIO     (0x00000000)
#define PSEL1_P0_24_AD01     (0x00010000)
#define PSEL1_P0_24_I2SRX_WS (0x00020000)
#define PSEL1_P0_24_CAP31    (0x00030000)
#define PSEL1_P0_24_MASK     (0x00030000)

#define PSEL1_P0_25_GPIO     (0x00000000)
#define PSEL1_P0_25_AD02     (0x00040000)
#define PSEL1_P0_25_I2SRX_SDA (0x00080000)
#define PSEL1_P0_25_TXD3     (0x000c0000)
#define PSEL1_P0_25_MASK     (0x000c0000)

#define PSEL1_P0_26_GPI0     (0x00000000)
#define PSEL1_P0_26_AD031    (0x00100000)
#define PSEL1_P0_26_AOUT     (0x00200000)
#define PSEL1_P0_26_RXD3     (0x00300000)
#define PSEL1_P0_26_MASK     (0x00300000)

#define PSEL1_P0_27_GPI0     (0x00000000)
#define PSEL1_P0_27_SDA0     (0x00400000)
#define PSEL1_P0_27_RSVD2    (0x00800000)
#define PSEL1_P0_27_RSVD3    (0x00c00000)
#define PSEL1_P0_27_MASK     (0x00c00000)

#define PSEL1_P0_28_GPIO     (0x00000000)
#define PSEL1_P0_28_SCL0     (0x01000000)
#define PSEL1_P0_28_RSVD2    (0x02000000)
#define PSEL1_P0_28_RSVD3    (0x03000000)
#define PSEL1_P0_28_MASK     (0x03000000)

#define PSEL1_P0_29_GPIO     (0x00000000)
#define PSEL1_P0_29_USB_DPOS1 (0x04000000)
#define PSEL1_P0_29_RSVD2    (0x08000000)
#define PSEL1_P0_29_RSVD3    (0x0c000000)
#define PSEL1_P0_29_MASK     (0x0c000000)

#define PSEL1_P0_30_GPIO     (0x00000000)
#define PSEL1_P0_30_USB_DNEG1 (0x10000000)
#define PSEL1_P0_30_RSVD2    (0x20000000)
#define PSEL1_P0_30_RSVD3    (0x30000000)
#define PSEL1_P0_30_MASK     (0x30000000)

#define PSEL1_P0_31_GPIO     (0x00000000)
#define PSEL1_P0_31_USB_DPOS2 (0x40000000)
#define PSEL1_P0_31_RSVD2    (0x80000000)
#define PSEL1_P0_31_RSVD3    (0xc0000000)
#define PSEL1_P0_31_MASK     (0xc0000000)


/* PINSEL 2 */
#define PSEL2_P1_0_GPIO     (0x00000000)
#define PSEL2_P1_0_ENET_TXD0 (0x00000001)
#define PSEL2_P1_0_RSVD2    (0x00000002)
#define PSEL2_P1_0_RSVD3    (0x00000003)
#define PSEL2_P1_0_MASK     (0x00000003)

#define PSEL2_P1_1_GPIO     (0x00000000)
#define PSEL2_P1_1_ENET_TXD1 (0x00000004)
#define PSEL2_P1_1_RSVD2    (0x00000008)
#define PSEL2_P1_1_RSVD3    (0x0000000c)
#define PSEL2_P1_1_MASK     (0x0000000c)

#define PSEL2_P1_2_RSVD0   (0x00000000)
#define PSEL2_P1_2_RSVD1   (0x00000010)
#define PSEL2_P1_2_RSVD2   (0x00000020)
#define PSEL2_P1_2_RSVD3   (0x00000030)
#define PSEL2_P1_2_MASK    (0x00000030)

#define PSEL2_P1_3_RSVD0    (0x00000000)
#define PSEL2_P1_3_RSVD1    (0x00000040)
#define PSEL2_P1_3_RSVD2    (0x00000080)
#define PSEL2_P1_3_RSVD3    (0x000000c0)
#define PSEL2_P1_3_MASK     (0x000000c0)

#define PSEL2_P1_4_GPIO     (0x00000000)
#define PSEL2_P1_4_ENET_TX_EN (0x00000100)
#define PSEL2_P1_4_RSVD2    (0x00000200)
#define PSEL2_P1_4_RSVD3    (0x00000300)
#define PSEL2_P1_4_MASK     (0x00000300)

#define PSEL2_P1_5_RSVDO    (0x00000000)
#define PSEL2_P1_5_RSVD1    (0x00000400)
#define PSEL2_P1_5_RSVD2    (0x00000800)
#define PSEL2_P1_5_RSVD3    (0x00000c00)
#define PSEL2_P1_5_MASK     (0x00000c00)

#define PSEL2_P1_6_RSVD0    (0x00000000)
#define PSEL2_P1_6_RSVD1    (0x00001000)
#define PSEL2_P1_6_RSVD2    (0x00002000)
#define PSEL2_P1_6_RSVD3    (0x00003000)
#define PSEL2_P1_6_MASK     (0x00003000)

#define PSEL2_P1_7_RSVD0    (0x00000000)
#define PSEL2_P1_7_RSVD1    (0x00004000)
#define PSEL2_P1_7_RSVD2    (0x00008000)
#define PSEL2_P1_7_RSVD3    (0x0000c000)
#define PSEL2_P1_7_MASK     (0x0000c000)

#define PSEL2_P1_8_GPIO     (0x00000000)
#define PSEL2_P1_8_ENET_CRS (0x00010000)
#define PSEL2_P1_8_RSVD2    (0x00020000)
#define PSEL2_P1_8_RSVD3    (0x00030000)
#define PSEL2_P1_8_MASK     (0x00030000)

#define PSEL2_P1_9_GPIO     (0x00000000)
#define PSEL2_P1_9_ENET_RXD0 (0x00040000)
#define PSEL2_P1_9_RSVD2 	 (0x00080000)
#define PSEL2_P1_9_RSVD3    (0x000c0000)
#define PSEL2_P1_9_MASK     (0x000c0000)

#define PSEL2_P1_10_GPI0    (0x00000000)
#define PSEL2_P1_10_ENET_RXD1 (0x00100000)
#define PSEL2_P1_10_RSVD2   (0x00200000)
#define PSEL2_P1_10_RSVD3   (0x00300000)
#define PSEL2_P1_10_MASK    (0x00300000)

#define PSEL2_P1_11_RSVD0    (0x00000000)
#define PSEL2_P1_11_RSVD1    (0x00400000)
#define PSEL2_P1_11_RSVD2    (0x00800000)
#define PSEL2_P1_11_RSVD3    (0x00c00000)
#define PSEL2_P1_11_MASK     (0x00c00000)

#define PSEL2_P1_12_RSVD0    (0x00000000)
#define PSEL2_P1_12_RSVD1    (0x01000000)
#define PSEL2_P1_12_RSVD2    (0x02000000)
#define PSEL2_P1_12_RSVD3    (0x03000000)
#define PSEL2_P1_12_MASK     (0x03000000)

#define PSEL2_P1_13_RSVD0    (0x00000000)
#define PSEL2_P1_13_RSVD1	  (0x04000000)
#define PSEL2_P1_13_RSVD2    (0x08000000)
#define PSEL2_P1_13_RSVD3    (0x0c000000)
#define PSEL2_P1_13_MASK     (0x0c000000)

#define PSEL2_P1_14_GPIO     (0x00000000)
#define PSEL2_P1_14_ENET_RX_ER (0x10000000)
#define PSEL2_P1_14_RSVD2    (0x20000000)
#define PSEL2_P1_14_RSVD3    (0x30000000)
#define PSEL2_P1_14_MASK     (0x30000000)

#define PSEL2_P1_15_GPIO     (0x00000000)
#define PSEL2_P1_15_ENET_REF_CLK (0x40000000)
#define PSEL2_P1_15_RSVD2    (0x80000000)
#define PSEL2_P1_15_RSVD3    (0xc0000000)
#define PSEL2_P1_15_MASK     (0xc0000000)


/* PINSEL 3 */
#define PSEL3_P1_16_GPIO     (0x00000000)
#define PSEL3_P1_16_ENET_MDC (0x00000001)
#define PSEL3_P1_16_RSVD2    (0x00000002)
#define PSEL3_P1_16_RSVD3    (0x00000003)
#define PSEL3_P1_16_MASK     (0x00000003)

#define PSEL3_P1_17_GPIO     (0x00000000)
#define PSEL3_P1_17_ENET_MDIO (0x00000004)
#define PSEL3_P1_17_RSVD2    (0x00000008)
#define PSEL3_P1_17_RSVD3    (0x0000000c)
#define PSEL3_P1_17_MASK     (0x0000000c)

#define PSEL3_P1_18_GPIO    (0x00000000)
#define PSEL3_P1_18_USB_UP_LED1 (0x00000010)
#define PSEL3_P1_18_PWM1_1  (0x00000020)
#define PSEL3_P1_18_CAP1_0  (0x00000030)
#define PSEL3_P1_18_MASK    (0x00000030)

#define PSEL3_P1_19_GPIO      (0x00000000)
#define PSEL3_P1_19_USB_TX_E1 (0x00000040) /* 2388 only Reserved on 2377/78 */
#define PSEL3_P1_19_USB_PPWR1 (0x00000080) /* 2388 only Reserved on 2377/78 */
#define PSEL3_P1_19_CAP1_1    (0x000000c0)
#define PSEL3_P1_19_MASK      (0x000000c0)

#define PSEL3_P1_20_GPIO     (0x00000000)
#define PSEL3_P1_20_USB_TX_DP1 (0x00000100)/* 2388 only Reserved on 2377/78 */
#define PSEL3_P1_20_PWM1_2   (0x00000200)
#define PSEL3_P1_20_SCK0     (0x00000300)
#define PSEL3_P1_20_MASK     (0x00000300)

#define PSEL3_P1_21_GPIO     (0x00000000)
#define PSEL3_P1_21_USB_TX_DM1 (0x00000400)/* 2388 only Reserved on 2377/78 */
#define PSEL3_P1_21_PWM1_3   (0x00000800)
#define PSEL3_P1_21_SSEL0    (0x00000c00)
#define PSEL3_P1_21_MASK     (0x00000c00)

#define PSEL3_P1_22_GPIO     (0x00000000)
#define PSEL3_P1_22_USB_RCV1 (0x00001000) /* 2388 only Reserved on 2377/78 */
#define PSEL3_P1_22_USB_PWRD1 (0x00002000)/* 2388 only Reserved on 2377/78 */
#define PSEL3_P1_22_MAT1_0   (0x00003000)
#define PSEL3_P1_22_MASK     (0x00003000)

#define PSEL3_P1_23_GPIO     (0x00000000)
#define PSEL3_P1_23_USB_RX_DP1 (0x00004000)/* 2388 only Reserved on 2377/78 */
#define PSEL3_P1_23_PWM1_4    (0x00008000)
#define PSEL3_P1_23_MISO0    (0x0000c000)
#define PSEL3_P1_23_MASK     (0x0000c000)

#define PSEL3_P1_24_GPIO     (0x00000000)
#define PSEL3_P1_24_USB_RX_DM1 (0x00010000)/* 2388 only Reserved on 2377/78 */
#define PSEL3_P1_24_PWM1_5   (0x00020000)
#define PSEL3_P1_24_MOSI0    (0x00030000)
#define PSEL3_P1_24_MASK     (0x00030000)

#define PSEL3_P1_25_GPIO     (0x00000000)
#define PSEL3_P1_25_USB_LS1 (0x00040000)/* 2388 only Reserved on 2377/78 */
#define PSEL3_P1_25_USB_HSTEN1 (0x00080000)/* 2388 only Reserved on 2377/78 */
#define PSEL3_P1_25_MAT1_1   (0x000c0000)
#define PSEL3_P1_25_MASK     (0x000c0000)

#define PSEL3_P1_26_GPI0    (0x00000000)
#define PSEL3_P1_26_USB_SSPND1 (0x00100000)/* 2388 only Reserved on 2377/78 */
#define PSEL3_P1_26_PWM1_6   (0x00200000)
#define PSEL3_P1_26_CAP0_0   (0x00300000)
#define PSEL3_P1_26_MASK     (0x00300000)

#define PSEL3_P1_27_GPIO     (0x00000000)
#define PSEL3_P1_27_USB_INT1 (0x00400000)/* 2388 only Reserved on 2377/78 */
#define PSEL3_P1_27_USB_OVRCR1 (0x00800000)/* 2388 only Reserved on 2377/78 */
#define PSEL3_P1_27_CAP0_1   (0x00c00000)
#define PSEL3_P1_27_MASK     (0x00c00000)

#define PSEL3_P1_28_GPIO     (0x00000000)
#define PSEL3_P1_28_USB_SCL1 (0x01000000)/* 2388 only Reserved on 2377/78 */
#define PSEL3_P1_28_PCAP1_0  (0x02000000)
#define PSEL3_P1_28_MAT0_0   (0x03000000)
#define PSEL3_P1_28_MASK     (0x03000000)

#define PSEL3_P1_29_GPIO     (0x00000000)
#define PSEL3_P1_29_USB_SDA1 (0x04000000)/* 2388 only Reserved on 2377/78 */
#define PSEL3_P1_29_PCAP1_1  (0x08000000)
#define PSEL3_P1_29_MAT0_1   (0x0c000000)
#define PSEL3_P1_29_MASK     (0x0c000000)

#define PSEL3_P1_30_GPIO     (0x00000000)
#define PSEL3_P1_30_USB_PWRD2 (0x10000000)
#define PSEL3_P1_30_VBUS     (0x20000000)
#define PSEL3_P1_30_AD0_4    (0x30000000)
#define PSEL3_P1_30_MASK     (0x30000000)

#define PSEL3_P1_31_GPIO     (0x00000000)
#define PSEL3_P1_31_USB_OVRCR2 (0x40000000)/* 2388 only Reserved on 2377/78 */
#define PSEL3_P1_31_SCK1     (0x80000000)
#define PSEL3_P1_31_AD0_5    (0xc0000000)
#define PSEL3_P1_31_MASK     (0xc0000000)

/* PINSEL 4 */
#define PSEL4_P2_0_GPIO     (0x00000000)
#define PSEL4_P2_0_PWM1_1   (0x00000001)
#define PSEL4_P2_0_TXD1     (0x00000002)
#define PSEL4_P2_0_TRACECLK (0x00000003)
#define PSEL4_P2_0_MASK     (0x00000003)

#define PSEL4_P2_1_GPIO     (0x00000000)
#define PSEL4_P2_1_PWM1_2   (0x00000004)
#define PSEL4_P2_1_RXD1     (0x00000008)
#define PSEL4_P2_1_PIPESTAT0 (0x0000000c)
#define PSEL4_P2_1_MASK     (0x0000000c)

#define PSEL4_P2_2_GPIO     (0x00000000)
#define PSEL4_P2_2_PWM1_3   (0x00000010)
#define PSEL4_P2_2_CTS1     (0x00000020)
#define PSEL4_P2_2_PIPESTAT1   (0x00000030)
#define PSEL4_P2_2_MASK     (0x00000030)

#define PSEL4_P2_3_GPIO     (0x00000000)
#define PSEL4_P2_3_PWM1_4    (0x00000040)
#define PSEL4_P2_3_DCD1     (0x00000080)
#define PSEL4_P2_3_PIPESTAT2 (0x000000c0)
#define PSEL4_P2_3_MASK     (0x000000c0)

#define PSEL4_P2_4_GPIO     (0x00000000)
#define PSEL4_P2_4_PWM1_5   (0x00000100)
#define PSEL4_P2_4_DSR1     (0x00000200)
#define PSEL4_P2_4_TRACESYNC (0x00000300)
#define PSEL4_P2_4_MASK     (0x00000300)

#define PSEL4_P2_5_GPIO     (0x00000000)
#define PSEL4_P2_5_PWM1_6   (0x00000400)
#define PSEL4_P2_5_DTR1     (0x00000800)
#define PSEL4_P2_5_TRACEPKT0 (0x00000c00)
#define PSEL4_P2_5_MASK     (0x00000c00)

#define PSEL4_P2_6_GPIO     (0x00000000)
#define PSEL4_P2_6_PCAP1_0  (0x00001000)
#define PSEL4_P2_6_RI1      (0x00002000)
#define PSEL4_P2_6_TRACEPKT1 (0x00003000)
#define PSEL4_P2_6_MASK     (0x00003000)

#define PSEL4_P2_7_GPIO    (0x00000000)
#define PSEL4_P2_7_RD2     (0x00004000)
#define PSEL4_P2_7_RTS1    (0x00008000)
#define PSEL4_P2_7_TRACEPKT2  (0x0000c000)
#define PSEL4_P2_7_MASK    (0x0000c000)

#define PSEL4_P2_8_GPIO     (0x00000000)
#define PSEL4_P2_8_TD2	  (0x00010000)
#define PSEL4_P2_8_TXD2     (0x00020000)
#define PSEL4_P2_8_TRACEPKT3 (0x00030000)
#define PSEL4_P2_8_MASK     (0x00030000)

#define PSEL4_P2_9_GPIO     (0x00000000)
#define PSEL4_P2_9_USB_CONNECT1 (0x00040000)
#define PSEL4_P2_9_RXD2     (0x00080000)
#define PSEL4_P2_9_EXTIN0   (0x000c0000)
#define PSEL4_P2_9_MASK     (0x000c0000)

#define PSEL4_P2_10_GPI0    (0x00000000)
#define PSEL4_P2_10_EINT0   (0x00100000)
#define PSEL4_P2_10_RSVD2   (0x00200000)
#define PSEL4_P2_10_RSVD3   (0x00300000)
#define PSEL4_P2_10_MASK    (0x00300000)

#define PSEL4_P2_11_GPIO     (0x00000000)
#define PSEL4_P2_11_EINT1    (0x00400000)
#define PSEL4_P2_11_MCIDAT1  (0x00800000)
#define PSEL4_P2_11_I2STX_CLK  (0x00c00000)
#define PSEL4_P2_11_MASK     (0x00c00000)

#define PSEL4_P2_12_GPIO     (0x00000000)
#define PSEL4_P2_12_EINT2    (0x01000000)
#define PSEL4_P2_12_MCIDAT2  (0x02000000)
#define PSEL4_P2_12_I2STX_WS (0x03000000)
#define PSEL4_P2_12_MASK     (0x03000000)

#define PSEL4_P2_13_GPIO     (0x00000000)
#define PSEL4_P2_13_EINT3	   (0x04000000)
#define PSEL4_P2_13_MCIDAT3  (0x08000000)
#define PSEL4_P2_13_I2STX_SDA (0x0c000000)
#define PSEL4_P2_13_MASK     (0x0c000000)

#define PSEL4_P2_14_RSVD0    (0x00000000)
#define PSEL4_P2_14_RSVD1    (0x10000000)
#define PSEL4_P2_14_RSVD2    (0x20000000)
#define PSEL4_P2_14_RSVD3    (0x30000000)
#define PSEL4_P2_14_MASK     (0x30000000)

#define PSEL4_P2_15_RSVD0    (0x00000000)
#define PSEL4_P2_15_RSVD1	   (0x40000000)
#define PSEL4_P2_15_RSVD2    (0x80000000)
#define PSEL4_P2_15_RSVD3    (0xc0000000)
#define PSEL4_P2_15_MASK     (0xc0000000)

/* PINSEL 5  All reserved */


/* PINSEL 6 */
#define PSEL6_P3_0_GPIO      (0x00000000)
#define PSEL6_P3_0_D0        (0x00000001)
#define PSEL6_P3_0_RSVD2     (0x00000002)
#define PSEL6_P3_0_RSVD3     (0x00000003)
#define PSEL6_P3_0_MASK      (0x00000003)

#define PSEL6_P3_1_GPIO      (0x00000000)
#define PSEL6_P3_1_D1        (0x00000004)
#define PSEL6_P3_1_RSVD2     (0x00000008)
#define PSEL6_P3_1_RSVD3     (0x0000000c)
#define PSEL6_P3_1_MASK      (0x0000000c)

#define PSEL6_P3_2_GPIO      (0x00000000)
#define PSEL6_P3_2_D2        (0x00000010)
#define PSEL6_P3_2_RSVD2     (0x00000020)
#define PSEL6_P3_2_RSVD3     (0x00000030)
#define PSEL6_P3_2_MASK      (0x00000030)

#define PSEL6_P3_3_GPIO      (0x00000000)
#define PSEL6_P3_3_D3        (0x00000040)
#define PSEL6_P3_3_RSVD2     (0x00000080)
#define PSEL6_P3_3_RSVD3     (0x000000c0)
#define PSEL6_P3_3_MASK      (0x000000c0)

#define PSEL6_P3_4_GPIO      (0x00000000)
#define PSEL6_P3_4_D4        (0x00000100)
#define PSEL6_P3_4_RSVD2     (0x00000200)
#define PSEL6_P3_4_RSVD3     (0x00000300)
#define PSEL6_P3_4_MASK      (0x00000400)

#define PSEL6_P3_5_GPIO      (0x00000000)
#define PSEL6_P3_5_D5        (0x00000400)
#define PSEL6_P3_5_RSVD2     (0x00000800)
#define PSEL6_P3_5_RSVD3     (0x00000c00)
#define PSEL6_P3_5_MASK      (0x00000c00)

#define PSEL6_P3_6_GPIO      (0x00000000)
#define PSEL6_P3_6_D6 		  (0x00001000)
#define PSEL6_P3_6_RSVD2     (0x00002000)
#define PSEL6_P3_6_RSVD3     (0x00003000)
#define PSEL6_P3_6_MASK      (0x00003000)

#define PSEL6_P3_7_GPIO      (0x00000000)
#define PSEL6_P3_7_D7        (0x00004000)
#define PSEL6_P3_7_RSVD2     (0x00008000)
#define PSEL6_P3_7_RSVD3     (0x0000c000)
#define PSEL6_P3_7_MASK      (0x0000c000)
/* Rest of PSEL 6 are reserved */

/* PINSEL 7 */
/* PINSEL 7 bit 0:13 are reserved */
#define PSEL7_P3_23_GPIO     (0x00000000)
#define PSEL7_P3_23_RSVD1    (0x00004000)
#define PSEL7_P3_23_CAP0_0   (0x00008000)
#define PSEL7_P3_23_PCAP1_0  (0x0000c000)
#define PSEL7_P3_23_MASK     (0x0000c000)

#define PSEL7_P3_24_GPIO     (0x00000000)
#define PSEL7_P3_24_RSVD1    (0x00010000)
#define PSEL7_P3_24_CAP0_1   (0x00020000)
#define PSEL7_P3_24_PWM1_1   (0x00030000)
#define PSEL7_P3_24_MASK     (0x00030000)

#define PSEL7_P3_25_GPIO     (0x00000000)
#define PSEL7_P3_25_RSVD1    (0x00040000)
#define PSEL7_P3_25_MAT0_0   (0x00080000)
#define PSEL7_P3_25_PWM1_2   (0x000c0000)
#define PSEL7_P3_25_MASK     (0x000c0000)

#define PSEL7_P3_26_GPI0     (0x00000000)
#define PSEL7_P3_26_RSVD1    (0x00100000)
#define PSEL7_P3_26_MAT0_1   (0x00200000)
#define PSEL7_P3_26_PWM1_3   (0x00300000)
#define PSEL7_P3_26_MASK     (0x00300000)
/* PINSEL 7 rest are reserved */


/* PINSEL 8 */
#define PSEL8_P4_0_GPIO      (0x00000000)
#define PSEL8_P4_0_A0        (0x00000001)
#define PSEL8_P4_0_RSVD2     (0x00000002)
#define PSEL8_P4_0_RSVD3     (0x00000003)
#define PSEL8_P4_0_MASK      (0x00000003)

#define PSEL8_P4_1_GPIO      (0x00000000)
#define PSEL8_P4_1_A1        (0x00000004)
#define PSEL8_P4_1_RXD3      (0x00000008)
#define PSEL8_P4_1_SCL1      (0x0000000c)
#define PSEL8_P4_1_MASK      (0x0000000c)

#define PSEL8_P4_2_GPIO      (0x00000000)
#define PSEL8_P4_2_A2        (0x00000010)
#define PSEL8_P4_2_RSVD2     (0x00000020)
#define PSEL8_P4_2_RSVD3     (0x00000030)
#define PSEL8_P4_2_MASK      (0x00000030)

#define PSEL8_P4_3_GPIO      (0x00000000)
#define PSEL8_P4_3_A3        (0x00000040)
#define PSEL8_P4_3_RSVD2     (0x00000080)
#define PSEL8_P4_3_RSVD3     (0x000000c0)
#define PSEL8_P4_3_MASK      (0x000000c0)

#define PSEL8_P4_4_GPIO      (0x00000000)
#define PSEL8_P4_4_A4        (0x00000100)
#define PSEL8_P4_4_RSVD2     (0x00000200)
#define PSEL8_P4_4_RSVD3     (0x00000300)
#define PSEL8_P4_4_MASK      (0x00000400)

#define PSEL8_P4_5_GPIO      (0x00000000)
#define PSEL8_P4_5_A5        (0x00000400)
#define PSEL8_P4_5_RSVD2     (0x00000800)
#define PSEL8_P4_5_RSVD3     (0x00000c00)
#define PSEL8_P4_5_MASK      (0x00000c00)

#define PSEL8_P4_6_GPIO      (0x00000000)
#define PSEL8_P4_6_A6        (0x00001000)
#define PSEL8_P4_6_RSVD2     (0x00002000)
#define PSEL8_P4_6_RSVD3     (0x00003000)
#define PSEL8_P4_6_MASK      (0x00003000)

#define PSEL8_P4_7_GPIO      (0x00000000)
#define PSEL8_P4_7_A7        (0x00004000)
#define PSEL8_P4_7_RSVD2     (0x00008000)
#define PSEL8_P4_7_RSVD3     (0x0000c000)
#define PSEL8_P4_7_MASK      (0x0000c000)

#define PSEL8_P4_8_GPIO      (0x00000000)
#define PSEL8_P4_8_A8        (0x00010000)
#define PSEL8_P4_8_RSVD2     (0x00020000)
#define PSEL8_P4_8_RSVD3     (0x00030000)
#define PSEL8_P4_8_MASK      (0x00030000)

#define PSEL8_P4_9_GPIO      (0x00000000)
#define PSEL8_P4_9_A9 		  (0x00040000)
#define PSEL8_P4_9_RSVD2     (0x00080000)
#define PSEL8_P4_9_RSVD3     (0x000c0000)
#define PSEL8_P4_9_MASK      (0x000c0000)

#define PSEL8_P4_10_GPIO     (0x00000000)
#define PSEL8_P4_10_A10      (0x00100000)
#define PSEL8_P4_10_RSVD2    (0x00200000)
#define PSEL8_P4_10_RSVD3    (0x00300000)
#define PSEL8_P4_10_MASK     (0x00300000)

#define PSEL8_P4_11_GPIO     (0x00000000)
#define PSEL8_P4_11_A11      (0x00400000)
#define PSEL8_P4_11_RSVD2    (0x00800000)
#define PSEL8_P4_11_RSVD3    (0x00c00000)
#define PSEL8_P4_11_MASK     (0x00c00000)

#define PSEL8_P4_12_GPIO     (0x00000000)
#define PSEL8_P4_12_A12      (0x01000000)
#define PSEL8_P4_12_RSVD2    (0x02000000)
#define PSEL8_P4_12_RSVD3    (0x03000000)
#define PSEL8_P4_12_MASK     (0x03000000)

#define PSEL8_P4_13_GPIO     (0x00000000)
#define PSEL8_P4_13_A13      (0x04000000)
#define PSEL8_P4_13_RSVD2    (0x08000000)
#define PSEL8_P4_13_RSVD3    (0x0c000000)
#define PSEL8_P4_13_MASK     (0x0c000000)

#define PSEL8_P4_14_GPIO     (0x00000000)
#define PSEL8_P4_14_A14      (0x10000000)
#define PSEL8_P4_14_RSVD2    (0x20000000)
#define PSEL8_P4_14_RSVD3    (0x30000000)
#define PSEL8_P4_14_MASK     (0x30000000)

#define PSEL8_P4_15_GPIO     (0x00000000)
#define PSEL8_P4_15_A15      (0x40000000)
#define PSEL8_P4_15_RSVD2    (0x80000000)
#define PSEL8_P4_15_RSVD3    (0xc0000000)
#define PSEL8_P4_15_MASK     (0xc0000000)

/* PINSEL 9 */
/* PINSEL 9 bit 0:15 are reserved */
#define PSEL9_P4_24_GPIO     (0x00000000)
#define PSEL9_P4_24_OE       (0x00010000)
#define PSEL9_P4_24_RSVD2    (0x00020000)
#define PSEL9_P4_24_RSVD3    (0x00030000)
#define PSEL9_P4_24_MASK     (0x00030000)

#define PSEL9_P4_25_GPIO     (0x00000000)
#define PSEL9_P4_25_         (0x00040000)
#define PSEL9_P4_25_BLS0     (0x00080000)
#define PSEL9_P4_25_RSVD3    (0x000c0000)
#define PSEL9_P4_25_MASK     (0x000c0000)

/* PINSEL 9 bit 26:27 are reserved */

#define PSEL9_P4_28_GPIO     (0x00000000)
#define PSEL9_P4_28_RSVD1    (0x01000000)
#define PSEL9_P4_28_MAT2_0   (0x02000000)
#define PSEL9_P4_28_TXD3     (0x03000000)
#define PSEL9_P4_28_MASK     (0x03000000)

#define PSEL9_P4_29_GPIO     (0x00000000)
#define PSEL9_P4_29_RSVD1    (0x04000000)
#define PSEL9_P4_29_MAT2_1   (0x08000000)
#define PSEL9_P4_29_RXD3     (0x0c000000)
#define PSEL9_P4_29_MASK     (0x0c000000)

#define PSEL9_P4_30_GPIO     (0x00000000)
#define PSEL9_P4_30_CS0      (0x10000000)
#define PSEL9_P4_30_RSVD2    (0x20000000)
#define PSEL9_P4_30_RSVD3    (0x30000000)
#define PSEL9_P4_30_MASK     (0x30000000)

#define PSEL9_P4_31_GPIO     (0x00000000)
#define PSEL9_P4_31_CS1      (0x40000000)
#define PSEL9_P4_31_RSVD2    (0x80000000)
#define PSEL9_P4_31_RSVD3    (0xc0000000)
#define PSEL9_P4_31_MASK     (0xc0000000)

/* PINSEL 10 */
#define PSEL10_ETM (0x00000002)

/* TODO PINMODE pullup/pulldown resistor configuration */
/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#endif  /* _ARCH_ARM_SRC_LPC23XX_PINSEL_H */
