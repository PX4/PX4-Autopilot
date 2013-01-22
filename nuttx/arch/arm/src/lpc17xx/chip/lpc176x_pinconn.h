/************************************************************************************
 * arch/arm/src/lpc17xx/chip/lpc176x_pinconn.h
 *
 *   Copyright (C) 2010, 2013 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LPC17XX_CHIP_LPC176X_PINCONN_H
#define __ARCH_ARM_SRC_LPC17XX_CHIP_LPC176X_PINCONN_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "chip/lpc17_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/

#define LPC17_PINCONN_PINSEL0_OFFSET    0x0000 /* Pin function select register 0 */
#define LPC17_PINCONN_PINSEL1_OFFSET    0x0004 /* Pin function select register 1 */
#define LPC17_PINCONN_PINSEL2_OFFSET    0x0008 /* Pin function select register 2 */
#define LPC17_PINCONN_PINSEL3_OFFSET    0x000c /* Pin function select register 3 */
#define LPC17_PINCONN_PINSEL4_OFFSET    0x0010 /* Pin function select register 4 */
#define LPC17_PINCONN_PINSEL7_OFFSET    0x001c /* Pin function select register 7 */
#define LPC17_PINCONN_PINSEL8_OFFSET    0x0020 /* Pin function select register 8 */
#define LPC17_PINCONN_PINSEL9_OFFSET    0x0024 /* Pin function select register 9 */
#define LPC17_PINCONN_PINSEL10_OFFSET   0x0028 /* Pin function select register 10 */
#define LPC17_PINCONN_PINMODE0_OFFSET   0x0040 /* Pin mode select register 0 */
#define LPC17_PINCONN_PINMODE1_OFFSET   0x0044 /* Pin mode select register 1 */
#define LPC17_PINCONN_PINMODE2_OFFSET   0x0048 /* Pin mode select register 2 */
#define LPC17_PINCONN_PINMODE3_OFFSET   0x004c /* Pin mode select register 3 */
#define LPC17_PINCONN_PINMODE4_OFFSET   0x0050 /* Pin mode select register 4 */
#define LPC17_PINCONN_PINMODE5_OFFSET   0x0054 /* Pin mode select register 5 */
#define LPC17_PINCONN_PINMODE6_OFFSET   0x0058 /* Pin mode select register 6 */
#define LPC17_PINCONN_PINMODE7_OFFSET   0x005c /* Pin mode select register 7 */
#define LPC17_PINCONN_PINMODE9_OFFSET   0x0064 /* Pin mode select register 9 */
#define LPC17_PINCONN_ODMODE0_OFFSET    0x0068 /* Open drain mode control register 0 */
#define LPC17_PINCONN_ODMODE1_OFFSET    0x006c /* Open drain mode control register 1 */
#define LPC17_PINCONN_ODMODE2_OFFSET    0x0070 /* Open drain mode control register 2 */
#define LPC17_PINCONN_ODMODE3_OFFSET    0x0074 /* Open drain mode control register 3 */
#define LPC17_PINCONN_ODMODE4_OFFSET    0x0078 /* Open drain mode control register 4 */
#define LPC17_PINCONN_I2CPADCFG_OFFSET  0x007c /* I2C Pin Configuration register */

/* Register addresses ***************************************************************/

#define LPC17_PINCONN_PINSEL0           (LPC17_PINCONN_BASE+LPC17_PINCONN_PINSEL0_OFFSET)
#define LPC17_PINCONN_PINSEL1           (LPC17_PINCONN_BASE+LPC17_PINCONN_PINSEL1_OFFSET)
#define LPC17_PINCONN_PINSEL2           (LPC17_PINCONN_BASE+LPC17_PINCONN_PINSEL2_OFFSET)
#define LPC17_PINCONN_PINSEL3           (LPC17_PINCONN_BASE+LPC17_PINCONN_PINSEL3_OFFSET)
#define LPC17_PINCONN_PINSEL4           (LPC17_PINCONN_BASE+LPC17_PINCONN_PINSEL4_OFFSET)
#define LPC17_PINCONN_PINSEL7           (LPC17_PINCONN_BASE+LPC17_PINCONN_PINSEL7_OFFSET)
#define LPC17_PINCONN_PINSEL8           (LPC17_PINCONN_BASE+LPC17_PINCONN_PINSEL8_OFFSET)
#define LPC17_PINCONN_PINSEL9           (LPC17_PINCONN_BASE+LPC17_PINCONN_PINSEL9_OFFSET)
#define LPC17_PINCONN_PINSEL10          (LPC17_PINCONN_BASE+LPC17_PINCONN_PINSEL10_OFFSET)
#define LPC17_PINCONN_PINMODE0          (LPC17_PINCONN_BASE+LPC17_PINCONN_PINMODE0_OFFSET)
#define LPC17_PINCONN_PINMODE1          (LPC17_PINCONN_BASE+LPC17_PINCONN_PINMODE1_OFFSET)
#define LPC17_PINCONN_PINMODE2          (LPC17_PINCONN_BASE+LPC17_PINCONN_PINMODE2_OFFSET)
#define LPC17_PINCONN_PINMODE3          (LPC17_PINCONN_BASE+LPC17_PINCONN_PINMODE3_OFFSET)
#define LPC17_PINCONN_PINMODE4          (LPC17_PINCONN_BASE+LPC17_PINCONN_PINMODE4_OFFSET)
#define LPC17_PINCONN_PINMODE5          (LPC17_PINCONN_BASE+LPC17_PINCONN_PINMODE5_OFFSET)
#define LPC17_PINCONN_PINMODE6          (LPC17_PINCONN_BASE+LPC17_PINCONN_PINMODE6_OFFSET)
#define LPC17_PINCONN_PINMODE7          (LPC17_PINCONN_BASE+LPC17_PINCONN_PINMODE7_OFFSET)
#define LPC17_PINCONN_PINMODE9          (LPC17_PINCONN_BASE+LPC17_PINCONN_PINMODE9_OFFSET)
#define LPC17_PINCONN_ODMODE0           (LPC17_PINCONN_BASE+LPC17_PINCONN_ODMODE0_OFFSET)
#define LPC17_PINCONN_ODMODE1           (LPC17_PINCONN_BASE+LPC17_PINCONN_ODMODE1_OFFSET)
#define LPC17_PINCONN_ODMODE2           (LPC17_PINCONN_BASE+LPC17_PINCONN_ODMODE2_OFFSET)
#define LPC17_PINCONN_ODMODE3           (LPC17_PINCONN_BASE+LPC17_PINCONN_ODMODE3_OFFSET)
#define LPC17_PINCONN_ODMODE4           (LPC17_PINCONN_BASE+LPC17_PINCONN_ODMODE4_OFFSET)
#define LPC17_PINCONN_I2CPADCFG         (LPC17_PINCONN_BASE+LPC17_PINCONN_I2CPADCFG_OFFSET)

/* Register bit definitions *********************************************************/
/* Pin Function Select register 0 (PINSEL0: 0x4002c000) */

#define PINCONN_PINSEL_GPIO             (0)
#define PINCONN_PINSEL_ALT1             (1)
#define PINCONN_PINSEL_ALT2             (2)
#define PINCONN_PINSEL_ALT3             (3)
#define PINCONN_PINSEL_MASK             (3)

#define PINCONN_PINSELL_SHIFT(n)        ((n) << 1) /* n=0,1,..,15 */
#define PINCONN_PINSELL_MASK(n)         (3 << PINCONN_PINSELL_SHIFT(n))
#define PINCONN_PINSELH_SHIFT(n)        (((n)-16) << 1) /* n=16,17,..31 */
#define PINCONN_PINSELH_MASK(n)         (3 << PINCONN_PINSELH_SHIFT(n))

#define PINCONN_PINSEL0_P0_SHIFT(n)     PINCONN_PINSELL_SHIFT(n) /* n=0,1,..,15 */
#define PINCONN_PINSEL0_P0_MASK(n)      PINCONN_PINSELL_MASK(n)  /* n=0,1,..,15 */

#define PINCONN_PINSEL0_P0p0_SHIFT      (0)       /* Bits 0-1: P0.0 00=GPIO 01=RD1 10=TXD3 11=SDA1 */
#define PINCONN_PINSEL0_P0p0_MASK       (3 << PINCONN_PINSEL0_P0p0_SHIFT)
#define PINCONN_PINSEL0_P0p1_SHIFT      (2)       /* Bits 2-3: P0.1 00=GPIO 01=TD1 10=RXD3 11=SCL1 */
#define PINCONN_PINSEL0_P0p1_MASK       (3 << PINCONN_PINSEL0_P0p1_SHIFT)
#define PINCONN_PINSEL0_P0p2_SHIFT      (4)       /* Bits 4-5: P0.2 00=GPIO 01=TXD0 10=AD0.7 11=Reserved */
#define PINCONN_PINSEL0_P0p2_MASK       (3 << PINCONN_PINSEL0_P0p2_SHIFT)
#define PINCONN_PINSEL0_P0p3_SHIFT      (6)       /* Bits 6-7: P0.3 00=GPIO 01=RXD0 10=AD0.6 11=Reserved */
#define PINCONN_PINSEL0_P0p3_MASK       (3 << PINCONN_PINSEL0_P0p3_SHIFT)
#define PINCONN_PINSEL0_P0p4_SHIFT      (8)       /* Bits 8-9: P0.4 00=GPIO 01=I2SRX_CLK 10=RD2 11=CAP2.0 */
#define PINCONN_PINSEL0_P0p4_MASK       (3 << PINCONN_PINSEL0_P0p4_SHIFT)
#define PINCONN_PINSEL0_P0p5_SHIFT      (10)      /* Bits 10-11: P0.5 00=GPIO 01=I2SRX_WS 10=TD2 11=CAP2.1 */
#define PINCONN_PINSEL0_P0p5_MASK       (3 << PINCONN_PINSEL0_P0p5_SHIFT)
#define PINCONN_PINSEL0_P0p6_SHIFT      (12)      /* Bits 12-13: P0.6 00=GPIO 01=I2SRX_SDA 10=SSEL1 11=MAT2.0 */
#define PINCONN_PINSEL0_P0p6_MASK       (3 << PINCONN_PINSEL0_P0p6_SHIFT)
#define PINCONN_PINSEL0_P0p7_SHIFT      (14)      /* Bits 14-15: P0.7 00=GPIO 01=I2STX_CLK 10=SCK1 11=MAT2.1 */
#define PINCONN_PINSEL0_P0p7_MASK       (3 << PINCONN_PINSEL0_P0p7_SHIFT)
#define PINCONN_PINSEL0_P0p8_SHIFT      (16)      /* Bits 16-17: P0.8 00=GPIO 01=I2STX_WS 10=MISO1 11=MAT2.2 */
#define PINCONN_PINSEL0_P0p8_MASK       (3 << PINCONN_PINSEL0_P0p8_SHIFT)
#define PINCONN_PINSEL0_P0p9_SHIFT      (18)      /* Bits 18-19: P0.9 00=GPIO 01=I2STX_SDA 10=MOSI1 11=MAT2.3 */
#define PINCONN_PINSEL0_P0p9_MASK       (3 << PINCONN_PINSEL0_P0p9_SHIFT)
#define PINCONN_PINSEL0_P0p10_SHIFT     (20)      /* Bits 20-21: P0.10 00=GPIO 01=TXD2 10=SDA2 11=MAT3.0 */
#define PINCONN_PINSEL0_P0p10_MASK      (3 << PINCONN_PINSEL0_P0p10_SHIFT)
#define PINCONN_PINSEL0_P0p11_SHIFT     (22)      /* Bits 22-23: P0.11 00=GPIO 01=RXD2 10=SCL2 11=MAT3.1 */
#define PINCONN_PINSEL0_P0p11_MASK      (3 << PINCONN_PINSEL0_P0p11_SHIFT)
                                                  /* Bits 24-29: Reserved */
#define PINCONN_PINSEL0_P0p15_SHIFT     (30)      /* Bits 30-31: P0.15 00=GPIO 01=TXD1 10=SCK0 11=SCK */
#define PINCONN_PINSEL0_P0p15_MASK      (3 << PINCONN_PINSEL0_P0p15_SHIFT)

/* Pin Function Select Register 1 (PINSEL1: 0x4002c004) */

#define PINCONN_PINSEL1_P0_SHIFT(n)     PINCONN_PINSELH_SHIFT(n) /* n=16,17,..31 */
#define PINCONN_PINSEL1_P0_MASK(n)      PINCONN_PINSELH_MASK(n)  /* n=16,17,..31 */

#define PINCONN_PINSEL1_P0p16_SHIFT     (0)       /* Bits 0-1: P0.16 00=GPIO 01=RXD1 10=SSEL0 11=SSEL */
#define PINCONN_PINSEL1_P0p16_MASK      (3 << PINCONN_PINSEL1_P0p16_SHIFT)
#define PINCONN_PINSEL1_P0p17_SHIFT     (2)       /* Bits 2-3: P0.17 00=GPIO 01=CTS1 10=MISO0 11=MISO */
#define PINCONN_PINSEL1_P0p17_MASK      (3 << PINCONN_PINSEL1_P0p17_SHIFT)
#define PINCONN_PINSEL1_P0p18_SHIFT     (4)       /* Bits 4-5: P0.18 00=GPIO 01=DCD1 10=MOSI0 11=MOSI */
#define PINCONN_PINSEL1_P0p18_MASK      (3 << PINCONN_PINSEL1_P0p18_SHIFT)
#define PINCONN_PINSEL1_P0p19_SHIFT     (6)       /* Bits 6-7: P0.19 00=GPIO 01=DSR1 10=Reserved 11=SDA1 */
#define PINCONN_PINSEL1_P0p19_MASK      (3 << PINCONN_PINSEL1_P0p19_SHIFT)
#define PINCONN_PINSEL1_P0p20_SHIFT     (8)       /* Bits 8-9: P0.20 00=GPIO 01=DTR1 10=Reserved 11=SCL1 */
#define PINCONN_PINSEL1_P0p20_MASK      (3 << PINCONN_PINSEL1_P0p20_SHIFT)
#define PINCONN_PINSEL1_P0p21_SHIFT     (10)      /* Bits 10-11: P0.21 00=GPIO 01=RI1 10=Reserved 11=RD1 */
#define PINCONN_PINSEL1_P0p21_MASK      (3 << PINCONN_PINSEL1_P0p21_SHIFT)
#define PINCONN_PINSEL1_P0p22_SHIFT     (12)      /* Bits 12-13: P0.22 00=GPIO 01=RTS1 10=Reserved 11=TD1 */
#define PINCONN_PINSEL1_P0p22_MASK      (3 << PINCONN_PINSEL1_P0p22_SHIFT)
#define PINCONN_PINSEL1_P0p23_SHIFT     (14)      /* Bits 14-15: P0.23 00=GPIO 01=AD0.0 10=I2SRX_CLK 11=CAP3.0 */
#define PINCONN_PINSEL1_P0p23_MASK      (3 << PINCONN_PINSEL1_P0p23_SHIFT)
#define PINCONN_PINSEL1_P0p24_SHIFT     (16)      /* Bits 16-17: P0.24 00=GPIO 01=AD0.1 10=I2SRX_WS 11=CAP3.1 */
#define PINCONN_PINSEL1_P0p24_MASK      (3 << PINCONN_PINSEL1_P0p24_SHIFT)
#define PINCONN_PINSEL1_P0p25_SHIFT     (18)      /* Bits 18-19: P0.25 00=GPIO 01=AD0.2 10=I2SRX_SDA 11=TXD3 */
#define PINCONN_PINSEL1_P0p25_MASK      (3 << PINCONN_PINSEL1_P0p25_SHIFT)
#define PINCONN_PINSEL1_P0p26_SHIFT     (20)      /* Bits 20-21: P0.26 00=GPIO 01=AD0.3 10=AOUT 11=RXD3 */
#define PINCONN_PINSEL1_P0p26_MASK      (3 << PINCONN_PINSEL1_P0p26_SHIFT)
#define PINCONN_PINSEL1_P0p27_SHIFT     (22)      /* Bits 22-23: P0.27 00=GPIO 01=SDA0 10=USB_SDA 11=Reserved */
#define PINCONN_PINSEL1_P0p27_MASK      (3 << PINCONN_PINSEL1_P0p27_SHIFT)
#define PINCONN_PINSEL1_P0p28_SHIFT     (24)      /* Bits 24-25: P0.28 00=GPIO 01=SCL0 10=USB_SCL 11=Reserved */
#define PINCONN_PINSEL1_P0p28_MASK      (3 << PINCONN_PINSEL1_P0p28_SHIFT)
#define PINCONN_PINSEL1_P0p29_SHIFT     (26)      /* Bits 26-27: P0.29 00=GPIO 01=USB_D+ 10=Reserved 11=Reserved */
#define PINCONN_PINSEL1_P0p29_MASK      (3 << PINCONN_PINSEL1_P0p29_SHIFT)
#define PINCONN_PINSEL1_P0p30_SHIFT     (28)      /* Bits 28-29: P0.30 00=GPIO 01=USB_D- 10=Reserved 11=Reserved */
#define PINCONN_PINSEL1_P0p30_MASK      (3 << PINCONN_PINSEL1_P0p30_SHIFT)
                                                  /* Bits 30-31: Reserved */
/* Pin Function Select register 2 (PINSEL2: 0x4002c008) */

#define PINCONN_PINSEL2_P1_SHIFT(n)     PINCONN_PINSELL_SHIFT(n) /* n=0,1,..,15 */
#define PINCONN_PINSEL2_P1_MASK(n)      PINCONN_PINSELL_MASK(n)  /* n=0,1,..,15 */

#define PINCONN_PINSEL2_P1p0_SHIFT      (0)       /* Bits 0-1: P1.0 00=GPIO 01=ENET_TXD0 10=Reserved 11=Reserved */
#define PINCONN_PINSEL2_P1p0_MASK       (3 << PINCONN_PINSEL2_P1p0_SHIFT)
#define PINCONN_PINSEL2_P1p1_SHIFT      (2)       /* Bits 2-3: P1.1 00=GPIO 01=ENET_TXD1 10=Reserved 11=Reserved */
#define PINCONN_PINSEL2_P1p1_MASK       (3 << PINCONN_PINSEL2_P1p1_SHIFT)
                                                  /* Bits 4-7: Reserved */
#define PINCONN_PINSEL2_P1p4_SHIFT      (8)       /* Bits 8-9: P1.4 00=GPIO 01=ENET_TX_EN 10=Reserved 11=Reserved */
#define PINCONN_PINSEL2_P1p4_MASK       (3 << PINCONN_PINSEL2_P1p4_SHIFT)
                                                  /* Bits 10-15: Reserved */
#define PINCONN_PINSEL2_P1p8_SHIFT      (16)      /* Bits 16-17: P1.8 00=GPIO 01=ENET_CRS 10=Reserved 11=Reserved */
#define PINCONN_PINSEL2_P1p8_MASK       (3 << PINCONN_PINSEL2_P1p8_SHIFT)
#define PINCONN_PINSEL2_P1p9_SHIFT      (18)      /* Bits 18-19: P1.9 00=GPIO 01=ENET_RXD0 10=Reserved 11=Reserved */
#define PINCONN_PINSEL2_P1p9_MASK       (3 << PINCONN_PINSEL2_P1p9_SHIFT)
#define PINCONN_PINSEL2_P1p10_SHIFT     (20)      /* Bits 20-21: P1.10 00=GPIO 01=ENET_RXD1 10=Reserved 11=Reserved */
#define PINCONN_PINSEL2_P1p10_MASK      (3 << PINCONN_PINSEL2_P1p10_SHIFT)
                                                  /* Bits 22-27: Reserved */
#define PINCONN_PINSEL2_P1p14_SHIFT     (28)      /* Bits 28-29: P1.14 00=GPIO 01=ENET_RX_ER 10=Reserved 11=Reserved */
#define PINCONN_PINSEL2_P1p14_MASK      (3 << PINCONN_PINSEL2_P1p14_SHIFT)
#define PINCONN_PINSEL2_P1p15_SHIFT     (30)      /* Bits 30-31: P1.15 00=GPIO 01=ENET_REF_CLK 10=Reserved 11=Reserved */
#define PINCONN_PINSEL2_P1p15_MASK      (3 << PINCONN_PINSEL2_P1p15_SHIFT)

/* Pin Function Select Register 3 (PINSEL3: 0x4002c00c) */

#define PINCONN_PINSEL3_P1_SHIFT(n)     PINCONN_PINSELH_SHIFT(n) /* n=16,17,..31 */
#define PINCONN_PINSEL3_P1_MASK(n)      PINCONN_PINSELH_MASK(n)  /* n=16,17,..31 */

#define PINCONN_PINSEL3_P1p16_SHIFT     (0)       /* Bits 0-1: P1.16 00=GPIO 01=ENET_MDC 10=Reserved 11=Reserved */
#define PINCONN_PINSEL3_P1p16_MASK      (3 << PINCONN_PINSEL3_P1p16_SHIFT)
#define PINCONN_PINSEL3_P1p17_SHIFT     (2)       /* Bits 2-3: P1.17 00=GPIO 01=ENET_MDIO 10=Reserved 11=Reserved */
#define PINCONN_PINSEL3_P1p17_MASK      (3 << PINCONN_PINSEL3_P1p17_SHIFT)
#define PINCONN_PINSEL3_P1p18_SHIFT     (4)       /* Bits 4-5: P1.18 00=GPIO 01=USB_UP_LED 10=PWM1.1 11=CAP1.0 */
#define PINCONN_PINSEL3_P1p18_MASK      (3 << PINCONN_PINSEL3_P1p18_SHIFT)
#define PINCONN_PINSEL3_P1p19_SHIFT     (6)       /* Bits 6-7: P1.19 00=GPIO 01=MCOA0 10=USB_PPWR 11=CAP1.1 */
#define PINCONN_PINSEL3_P1p19_MASK      (3 << PINCONN_PINSEL3_P1p19_SHIFT)
#define PINCONN_PINSEL3_P1p20_SHIFT     (8)       /* Bits 8-9: P1.20 00=GPIO 01=MCI0 10=PWM1.2 11=SCK0 */
#define PINCONN_PINSEL3_P1p20_MASK      (3 << PINCONN_PINSEL3_P1p20_SHIFT)
#define PINCONN_PINSEL3_P1p21_SHIFT     (10)      /* Bits 10-11: P1.21 00=GPIO 01=MCABORT 10=PWM1.3 11=SSEL0 */
#define PINCONN_PINSEL3_P1p21_MASK      (3 << PINCONN_PINSEL3_P1p21_SHIFT)
#define PINCONN_PINSEL3_P1p22_SHIFT     (12)      /* Bits 12-13: P1.22 00=GPIO 01=MCOB0 10=USB_PWRD 11=MAT1.0 */
#define PINCONN_PINSEL3_P1p22_MASK      (3 << PINCONN_PINSEL3_P1p22_SHIFT)
#define PINCONN_PINSEL3_P1p23_SHIFT     (14)      /* Bits 14-15: P1.23 00=GPIO 01=MCI1 10=PWM1.4 11=MISO0 */
#define PINCONN_PINSEL3_P1p23_MASK      (3 << PINCONN_PINSEL3_P1p23_SHIFT)
#define PINCONN_PINSEL3_P1p24_SHIFT     (16)      /* Bits 16-17: P1.24 00=GPIO 01=MCI2 10=PWM1.5 11=MOSI0 */
#define PINCONN_PINSEL3_P1p24_MASK      (3 << PINCONN_PINSEL3_P1p24_SHIFT)
#define PINCONN_PINSEL3_P1p25_SHIFT     (18)      /* Bits 18-19: P1.25 00=GPIO 01=MCOA1 10=Reserved 11=MAT1.1 */
#define PINCONN_PINSEL3_P1p25_MASK      (3 << PINCONN_PINSEL3_P1p25_SHIFT)
#define PINCONN_PINSEL3_P1p26_SHIFT     (20)      /* Bits 20-21: P1.26 00=GPIO 01=MCOB1 10=PWM1.6 11=CAP0.0 */
#define PINCONN_PINSEL3_P1p26_MASK      (3 << PINCONN_PINSEL3_P1p26_SHIFT)
#define PINCONN_PINSEL3_P1p27_SHIFT     (22)      /* Bits 22-23: P1.27 00=GPIO 01=CLKOUT 10=USB_OVRCR 11=CAP0.1 */
#define PINCONN_PINSEL3_P1p27_MASK      (3 << PINCONN_PINSEL3_P1p27_SHIFT)
#define PINCONN_PINSEL3_P1p28_SHIFT     (24)      /* Bits 24-25: P1.28 00=GPIO 01=MCOA2 10=PCAP1.0 11=MAT0.0 */
#define PINCONN_PINSEL3_P1p28_MASK      (3 << PINCONN_PINSEL3_P1p28_SHIFT)
#define PINCONN_PINSEL3_P1p29_SHIFT     (26)      /* Bits 26-27: P1.29 00=GPIO 01=MCOB2 10=PCAP1.1 11=MAT0.1 */
#define PINCONN_PINSEL3_P1p29_MASK      (3 << PINCONN_PINSEL3_P1p29_SHIFT)
#define PINCONN_PINSEL3_P1p30_SHIFT     (28)      /* Bits 28-29: P1.30 00=GPIO 01=Reserved 10=VBUS 11=AD0.4 */
#define PINCONN_PINSEL3_P1p30_MASK      (3 << PINCONN_PINSEL3_P1p30_SHIFT)
#define PINCONN_PINSEL3_P1p31_SHIFT     (30)      /* Bits 30-31: P1.31 00=GPIO 01=Reserved 10=SCK1 11=AD0.5 */
#define PINCONN_PINSEL3_P1p31_MASK      (3 << PINCONN_PINSEL3_P1p31_SHIFT)

/* Pin Function Select Register 4 (PINSEL4: 0x4002c010) */

#define PINCONN_PINSEL4_P2_SHIFT(n)     PINCONN_PINSELL_SHIFT(n) /* n=0,1,..,15 */
#define PINCONN_PINSEL4_P2_MASK(n)      PINCONN_PINSELL_MASK(n)  /* n=0,1,..,15 */

#define PINCONN_PINSEL4_P2p0_SHIFT     (0)        /* Bits 0-1: P2.0 00=GPIO 01=PWM1.1 10=TXD1 11=Reserved */
#define PINCONN_PINSEL4_P2p0_MASK      (3 << PINCONN_PINSEL4_P2p0_SHIFT)
#define PINCONN_PINSEL4_P2p1_SHIFT     (2)        /* Bits 2-3: P2.1 00=GPIO 01=PWM1.2 10=RXD1 11=Reserved */
#define PINCONN_PINSEL4_P2p1_MASK      (3 << PINCONN_PINSEL4_P2p1_SHIFT)
#define PINCONN_PINSEL4_P2p2_SHIFT     (4)        /* Bits 4-5: P2.2 00=GPIO 01=PWM1.3 10=CTS1 11=Reserved */
#define PINCONN_PINSEL4_P2p2_MASK      (3 << PINCONN_PINSEL4_P2p2_SHIFT)
#define PINCONN_PINSEL4_P2p3_SHIFT     (6)        /* Bits 6-7: P2.3 00=GPIO 01=PWM1.4 10=DCD1 11=Reserved */
#define PINCONN_PINSEL4_P2p3_MASK      (3 << PINCONN_PINSEL4_P2p3_SHIFT)
#define PINCONN_PINSEL4_P2p4_SHIFT     (8)        /* Bits 8-9: P2.4 00=GPIO 01=PWM1.5 10=DSR1 11=Reserved */
#define PINCONN_PINSEL4_P2p4_MASK      (3 << PINCONN_PINSEL4_P2p4_SHIFT)
#define PINCONN_PINSEL4_P2p5_SHIFT     (10)       /* Bits 10-11: P2.5 00=GPIO 01=PWM1.6 10=DTR1 11=Reserved */
#define PINCONN_PINSEL4_P2p5_MASK      (3 << PINCONN_PINSEL4_P2p5_SHIFT)
#define PINCONN_PINSEL4_P2p6_SHIFT     (12)       /* Bits 12-13: P2.6 00=GPIO 01=PCAP1.0 10=RI1 11=Reserved */
#define PINCONN_PINSEL4_P2p6_MASK      (3 << PINCONN_PINSEL4_P2p6_SHIFT)
#define PINCONN_PINSEL4_P2p7_SHIFT     (14)       /* Bits 14-15: P2.7 00=GPIO 01=RD2 10=RTS1 11=Reserved */
#define PINCONN_PINSEL4_P2p7_MASK      (3 << PINCONN_PINSEL4_P2p7_SHIFT)
#define PINCONN_PINSEL4_P2p8_SHIFT     (16)       /* Bits 16-17: P2.8 00=GPIO 01=TD2 10=TXD2 11=ENET_MDC */
#define PINCONN_PINSEL4_P2p8_MASK      (3 << PINCONN_PINSEL4_P2p8_SHIFT)
#define PINCONN_PINSEL4_P2p9_SHIFT     (18)       /* Bits 18-19: P2.9 00=GPIO 01=USB_CONNECT 10=RXD2 11=ENET_MDIO */
#define PINCONN_PINSEL4_P2p9_MASK      (3 << PINCONN_PINSEL4_P2p9_SHIFT)
#define PINCONN_PINSEL4_P2p10_SHIFT    (20)       /* Bits 20-21: P2.10 00=GPIO 01=EINT0 10=NMI 11=Reserved */
#define PINCONN_PINSEL4_P2p10_MASK     (3 << PINCONN_PINSEL4_P2p10_SHIFT)
#define PINCONN_PINSEL4_P2p11_SHIFT    (22)       /* Bits 22-23: P2.11 00=GPIO 01=EINT1 10=Reserved 11=I2STX_CLK */
#define PINCONN_PINSEL4_P2p11_MASK     (3 << PINCONN_PINSEL4_P2p11_SHIFT)
#define PINCONN_PINSEL4_P2p12_SHIFT    (24)       /* Bits 24-25: P2.12 00=GPIO 01=PEINT2 10=Reserved 11=I2STX_WS */
#define PINCONN_PINSEL4_P2p12_MASK     (3 << PINCONN_PINSEL4_P2p12_SHIFT)
#define PINCONN_PINSEL4_P2p13_SHIFT    (26)       /* Bits 26-27: P2.13 00=GPIO 01=EINT3 10=Reserved 11=I2STX_SDA */
#define PINCONN_PINSEL4_P2p13_MASK     (3 << PINCONN_PINSEL4_P2p13_SHIFT)
                                                  /* Bits 28-31: Reserved */
/* Pin Function Select Register 7 (PINSEL7: 0x4002c01c) */

#define PINCONN_PINSEL7_P3_SHIFT(n)     PINCONN_PINSELH_SHIFT(n) /* n=16,17,..31 */
#define PINCONN_PINSEL7_P3_MASK(n)      PINCONN_PINSELH_MASK(n)  /* n=16,17,..31 */

                                                  /* Bits 0-17: Reserved */
#define PINCONN_PINSEL7_P3p25_SHIFT    (18)       /* Bits 18-19: P3.25 00=GPIO 01=Reserved 10=MAT0.0 11=PWM1.2 */
#define PINCONN_PINSEL7_P3p25_MASK     (3 << PINCONN_PINSEL7_P3p25_SHIFT)
#define PINCONN_PINSEL7_P3p26_SHIFT    (20)       /* Bits 20-21: P3.26 00=GPIO 01=STCLK 10=MAT0.1 11=PWM1.3 */
#define PINCONN_PINSEL7_P3p26_MASK     (3 << PINCONN_PINSEL7_P3p26_SHIFT)
                                                  /* Bits 22-31: Reserved */

/* Pin Function Select Register 8 (PINSEL8: 0x4002c020) */
/* No description of bits -- Does this register exist? */

/* Pin Function Select Register 9 (PINSEL9: 0x4002c024) */

#define PINCONN_PINSEL9_P4_SHIFT(n)     PINCONN_PINSELH_SHIFT(n) /* n=16,17,..31 */
#define PINCONN_PINSEL9_P4_MASK(n)      PINCONN_PINSELH_MASK(n)  /* n=16,17,..31 */

                                                  /* Bits 0-23: Reserved */
#define PINCONN_PINSEL9_P4p28_SHIFT    (24)       /* Bits 24-25: P4.28 00=GPIO 01=RX_MCLK 10=MAT2.0 11=TXD3 */
#define PINCONN_PINSEL9_P4p28_MASK     (3 << PINCONN_PINSEL9_P4p28_SHIFT)
#define PINCONN_PINSEL9_P4p29_SHIFT    (26)       /* Bits 26-27: P4.29 00=GPIO 01=TX_MCLK 10=MAT2.1 11=RXD3 */
#define PINCONN_PINSEL9_P4p29_MASK     (3 << PINCONN_PINSEL9_P4p29_SHIFT)
                                                  /* Bits 28-31: Reserved */
/* Pin Function Select Register 10 (PINSEL10: 0x4002c028) */
                                                  /* Bits 0-2: Reserved */
#define PINCONN_PINSEL10_TPIU          (1 << 3)   /* Bit 3: 0=TPIU interface disabled; 1=TPIU interface enabled */
                                                  /* Bits 4-31: Reserved */
/* Pin Mode select register 0 (PINMODE0: 0x4002c040) */

#define PINCONN_PINMODE_PU              (0)       /* 00: pin has a pull-up resistor enabled */
#define PINCONN_PINMODE_RM              (1)       /* 01: pin has repeater mode enabled */
#define PINCONN_PINMODE_FLOAT           (2)       /* 10: pin has neither pull-up nor pull-down */
#define PINCONN_PINMODE_PD              (3)       /* 11: pin has a pull-down resistor enabled */
#define PINCONN_PINMODE_MASK            (3)

#define PINCONN_PINMODEL_SHIFT(n)       ((n) << 1) /* n=0,1,..,15 */
#define PINCONN_PINMODEL_MASK(n)        (3 << PINCONN_PINMODEL_SHIFT(n))
#define PINCONN_PINMODEH_SHIFT(n)       (((n)-16) << 1) /* n=16,17,..31 */
#define PINCONN_PINMODEH_MASK(n)        (3 << PINCONN_PINMODEH_SHIFT(n))

#define PINCONN_PINMODE0_P0_SHIFT(n)    PINCONN_PINMODEL_SHIFT(n) /* n=0,1,..,15 */
#define PINCONN_PINMODE0_P0_MASK(n)     PINCONN_PINMODEL_MASK(n)  /* n=0,1,..,15 */

#define PINCONN_PINMODE0_P0p0_SHIFT     (0)       /* Bits 0-1: P0.0 mode control */
#define PINCONN_PINMODE0_P0p0_MASK      (3 << PINCONN_PINMODE0_P0p0_SHIFT)
#define PINCONN_PINMODE0_P0p1_SHIFT     (2)       /* Bits 2-3: P0.1 mode control */
#define PINCONN_PINMODE0_P0p1_MASK      (3 << PINCONN_PINMODE0_P0p1_SHIFT)
#define PINCONN_PINMODE0_P0p2_SHIFT     (4)       /* Bits 4-5: P0.2 mode control */
#define PINCONN_PINMODE0_P0p2_MASK      (3 << PINCONN_PINMODE0_P0p2_SHIFT)
#define PINCONN_PINMODE0_P0p3_SHIFT     (6)       /* Bits 6-7: P0.3 mode control */
#define PINCONN_PINMODE0_P0p3_MASK      (3 << PINCONN_PINMODE0_P0p3_SHIFT)
#define PINCONN_PINMODE0_P0p4_SHIFT     (8)       /* Bits 8-9: P0.4 mode control */
#define PINCONN_PINMODE0_P0p4_MASK      (3 << PINCONN_PINMODE0_P0p4_SHIFT)
#define PINCONN_PINMODE0_P0p5_SHIFT     (10)      /* Bits 10-11: P0.5 mode control */
#define PINCONN_PINMODE0_P0p5_MASK      (3 << PINCONN_PINMODE0_P0p5_SHIFT)
#define PINCONN_PINMODE0_P0p6_SHIFT     (12)      /* Bits 12-13: P0.6 mode control */
#define PINCONN_PINMODE0_P0p6_MASK      (3 << PINCONN_PINMODE0_P0p6_SHIFT)
#define PINCONN_PINMODE0_P0p7_SHIFT     (14)      /* Bits 14-15: P0.7 mode control */
#define PINCONN_PINMODE0_P0p7_MASK      (3 << PINCONN_PINMODE0_P0p7_SHIFT)
#define PINCONN_PINMODE0_P0p8_SHIFT     (16)      /* Bits 16-17: P0.8 mode control */
#define PINCONN_PINMODE0_P0p8_MASK      (3 << PINCONN_PINMODE0_P0p8_SHIFT)
#define PINCONN_PINMODE0_P0p9_SHIFT     (18)      /* Bits 18-19: P0.9 mode control */
#define PINCONN_PINMODE0_P0p9_MASK      (3 << PINCONN_PINMODE0_P0p9_SHIFT)
#define PINCONN_PINMODE0_P0p10_SHIFT    (20)      /* Bits 20-21: P0.10 mode control */
#define PINCONN_PINMODE0_P0p10_MASK     (3 << PINCONN_PINMODE0_P0p10_SHIFT)
#define PINCONN_PINMODE0_P0p11_SHIFT    (22)      /* Bits 22-23: P0.11 mode control */
#define PINCONN_PINMODE0_P0p11_MASK     (3 << PINCONN_PINMODE0_P0p11_SHIFT)
                                                  /* Bits 24-29: Reserved */
#define PINCONN_PINMODE0_P0p15_SHIFT    (30)      /* Bits 30-31: P0.15 mode control */
#define PINCONN_PINMODE0_P0p15_MASK     (3 << PINCONN_PINMODE0_P0p15_SHIFT)

/* Pin Mode select register 1 (PINMODE1: 0x4002c044) */

#define PINCONN_PINMODE1_P0_SHIFT(n)    PINCONN_PINMODEH_SHIFT(n) /* n=16,17,..31 */
#define PINCONN_PINMODE1_P0_MASK(n)     PINCONN_PINMODEH_MASK(n)  /* n=16,17,..31 */

#define PINCONN_PINMODE1_P0p16_SHIFT    (0)       /* Bits 0-1: P0.16 mode control */
#define PINCONN_PINMODE1_P0p16_MASK     (3 << PINCONN_PINMODE1_P0p16_SHIFT)
#define PINCONN_PINMODE1_P0p17_SHIFT    (2)       /* Bits 2-3: P0.17 mode control */
#define PINCONN_PINMODE1_P0p17_MASK     (3 << PINCONN_PINMODE1_P0p17_SHIFT)
#define PINCONN_PINMODE1_P0p18_SHIFT    (4)       /* Bits 4-5: P0.18 mode control */
#define PINCONN_PINMODE1_P0p18_MASK     (3 << PINCONN_PINMODE1_P0p18_SHIFT)
#define PINCONN_PINMODE1_P0p19_SHIFT    (6)       /* Bits 6-7: P0.19 mode control */
#define PINCONN_PINMODE1_P0p19_MASK     (3 << PINCONN_PINMODE1_P0p19_SHIFT)
#define PINCONN_PINMODE1_P0p20_SHIFT    (8)       /* Bits 8-9: P0.20 mode control */
#define PINCONN_PINMODE1_P0p20_MASK     (3 << PINCONN_PINMODE1_P0p20_SHIFT)
#define PINCONN_PINMODE1_P0p21_SHIFT    (10)      /* Bits 10-11: P0.21 mode control */
#define PINCONN_PINMODE1_P0p21_MASK     (3 << PINCONN_PINMODE1_P0p21_SHIFT)
#define PINCONN_PINMODE1_P0p22_SHIFT    (12)      /* Bits 12-13: P0.22 mode control */
#define PINCONN_PINMODE1_P0p22_MASK     (3 << PINCONN_PINMODE1_P0p22_SHIFT)
#define PINCONN_PINMODE1_P0p23_SHIFT    (14)      /* Bits 14-15: P0.23 mode control */
#define PINCONN_PINMODE1_P0p23_MASK     (3 << PINCONN_PINMODE1_P0p23_SHIFT)
#define PINCONN_PINMODE1_P0p24_SHIFT    (16)      /* Bits 16-17: P0.24 mode control */
#define PINCONN_PINMODE1_P0p24_MASK     (3 << PINCONN_PINMODE1_P0p24_SHIFT)
#define PINCONN_PINMODE1_P0p25_SHIFT    (18)      /* Bits 18-19: P0.25 mode control */
#define PINCONN_PINMODE1_P0p25_MASK     (3 << PINCONN_PINMODE1_P0p25_SHIFT)
#define PINCONN_PINMODE1_P0p26_SHIFT    (20)      /* Bits 20-21: P0.26 mode control */
#define PINCONN_PINMODE1_P0p26_MASK     (3 << PINCONN_PINMODE1_P0p26_SHIFT)
                                                  /* Bits 22-31: Reserved */

/* Pin Mode select register 2 (PINMODE2: 0x4002c048) */

#define PINCONN_PINMODE2_P1_SHIFT(n)    PINCONN_PINMODEL_SHIFT(n) /* n=0,1,..,15 */
#define PINCONN_PINMODE2_P1_MASK(n)     PINCONN_PINMODEL_MASK(n)  /* n=0,1,..,15 */

#define PINCONN_PINMODE2_P1p0_SHIFT     (0)       /* Bits 2-1: P1.0 mode control */
#define PINCONN_PINMODE2_P1p0_MASK      (3 << PINCONN_PINMODE2_P1p0_SHIFT)
#define PINCONN_PINMODE2_P1p1_SHIFT     (2)       /* Bits 2-3: P1.1 mode control */
#define PINCONN_PINMODE2_P1p1_MASK      (3 << PINCONN_PINMODE2_P1p1_SHIFT)
                                                  /* Bits 4-7: Reserved */
#define PINCONN_PINMODE2_P1p4_SHIFT     (8)       /* Bits 8-9: P1.4 mode control */
#define PINCONN_PINMODE2_P1p4_MASK      (3 << PINCONN_PINMODE2_P1p4_SHIFT)
                                                  /* Bits 10-15: Reserved */
#define PINCONN_PINMODE2_P1p8_SHIFT     (16)      /* Bits 16-17: P1.8 mode control */
#define PINCONN_PINMODE2_P1p8_MASK      (3 << PINCONN_PINMODE2_P1p8_SHIFT)
#define PINCONN_PINMODE2_P1p9_SHIFT     (18)      /* Bits 18-19: P1.9 mode control */
#define PINCONN_PINMODE2_P1p9_MASK      (3 << PINCONN_PINMODE2_P1p9_SHIFT)
#define PINCONN_PINMODE2_P1p10_SHIFT    (20)      /* Bits 20-21: P1.10 mode control */
#define PINCONN_PINMODE2_P1p10_MASK     (3 << PINCONN_PINMODE2_P1p10_SHIFT)
                                                  /* Bits 22-27: Reserved */
#define PINCONN_PINMODE2_P1p14_SHIFT    (28)      /* Bits 28-29: P1.14 mode control */
#define PINCONN_PINMODE2_P1p14_MASK     (3 << PINCONN_PINMODE2_P1p14_SHIFT)
#define PINCONN_PINMODE2_P1p15_SHIFT    (30)      /* Bits 30-31: P1.15 mode control */
#define PINCONN_PINMODE2_P1p15_MASK     (3 << PINCONN_PINMODE2_P1p15_SHIFT)

/* Pin Mode select register 3 (PINMODE3: 0x4002c04c) */

#define PINCONN_PINMODE3_P1_SHIFT(n)    PINCONN_PINMODEH_SHIFT(n) /* n=16,17,..31 */
#define PINCONN_PINMODE3_P1_MASK(n)     PINCONN_PINMODEH_MASK(n)  /* n=16,17,..31 */

#define PINCONN_PINMODE3_P1p16_SHIFT    (0)       /* Bits 0-1: P1.16 mode control */
#define PINCONN_PINMODE3_P1p16_MASK     (3 << PINCONN_PINMODE3_P1p16_SHIFT)
#define PINCONN_PINMODE3_P1p17_SHIFT    (2)       /* Bits 2-3: P1.17 mode control */
#define PINCONN_PINMODE3_P1p17_MASK     (3 << PINCONN_PINMODE3_P1p17_SHIFT)
#define PINCONN_PINMODE3_P1p18_SHIFT    (4)       /* Bits 4-5: P1.18 mode control */
#define PINCONN_PINMODE3_P1p18_MASK     (3 << PINCONN_PINMODE3_P1p18_SHIFT)
#define PINCONN_PINMODE3_P1p19_SHIFT    (6)       /* Bits 6-7: P1.19 mode control */
#define PINCONN_PINMODE3_P1p19_MASK     (3 << PINCONN_PINMODE3_P1p19_SHIFT)
#define PINCONN_PINMODE3_P1p20_SHIFT    (8)       /* Bits 8-9: P1.20 mode control */
#define PINCONN_PINMODE3_P1p20_MASK     (3 << PINCONN_PINMODE3_P1p20_SHIFT)
#define PINCONN_PINMODE3_P1p21_SHIFT    (10)      /* Bits 10-11: P1.21 mode control */
#define PINCONN_PINMODE3_P1p21_MASK     (3 << PINCONN_PINMODE3_P1p21_SHIFT)
#define PINCONN_PINMODE3_P1p22_SHIFT    (12)      /* Bits 12-13: P1.22 mode control */
#define PINCONN_PINMODE3_P1p22_MASK     (3 << PINCONN_PINMODE3_P1p22_SHIFT)
#define PINCONN_PINMODE3_P1p23_SHIFT    (14)      /* Bits 14-15: P1.23 mode control */
#define PINCONN_PINMODE3_P1p23_MASK     (3 << PINCONN_PINMODE3_P1p23_SHIFT)
#define PINCONN_PINMODE3_P1p24_SHIFT    (16)      /* Bits 16-17: P1.24 mode control */
#define PINCONN_PINMODE3_P1p24_MASK     (3 << PINCONN_PINMODE3_P1p24_SHIFT)
#define PINCONN_PINMODE3_P1p25_SHIFT    (18)      /* Bits 18-19: P1.25 mode control */
#define PINCONN_PINMODE3_P1p25_MASK     (3 << PINCONN_PINMODE3_P1p25_SHIFT)
#define PINCONN_PINMODE3_P1p26_SHIFT    (20)      /* Bits 20-21: P1.26 mode control */
#define PINCONN_PINMODE3_P1p26_MASK     (3 << PINCONN_PINMODE3_P1p26_SHIFT)
#define PINCONN_PINMODE3_P1p27_SHIFT    (22)      /* Bits 22-23: P1.27 mode control */
#define PINCONN_PINMODE3_P1p27_MASK     (3 << PINCONN_PINMODE3_P1p27_SHIFT)
#define PINCONN_PINMODE3_P1p28_SHIFT    (24)      /* Bits 24-25: P1.28 mode control */
#define PINCONN_PINMODE3_P1p28_MASK     (3 << PINCONN_PINMODE3_P1p28_SHIFT)
#define PINCONN_PINMODE3_P1p29_SHIFT    (26)      /* Bits 26-27: P1.29 mode control */
#define PINCONN_PINMODE3_P1p29_MASK     (3 << PINCONN_PINMODE3_P1p29_SHIFT)
#define PINCONN_PINMODE3_P1p30_SHIFT    (28)      /* Bits 28-29: P1.30 mode control */
#define PINCONN_PINMODE3_P1p30_MASK     (3 << PINCONN_PINMODE3_P1p30_SHIFT)
#define PINCONN_PINMODE3_P1p31_SHIFT    (30)      /* Bits 30-31: P1.31 mode control */
#define PINCONN_PINMODE3_P1p31_MASK     (3 << PINCONN_PINMODE3_P1p31_SHIFT)

/* Pin Mode select register 4 (PINMODE4: 0x4002c050) */

#define PINCONN_PINMODE4_P2_SHIFT(n)    PINCONN_PINMODEL_SHIFT(n) /* n=0,1,..,15 */
#define PINCONN_PINMODE4_P2_MASK(n)     PINCONN_PINMODEL_MASK(n)  /* n=0,1,..,15 */

#define PINCONN_PINMODE4_P2p0_SHIFT     (0)       /* Bits 0-1: P2.0 mode control */
#define PINCONN_PINMODE4_P2p0_MASK      (3 << PINCONN_PINMODE4_P2p0_SHIFT)
#define PINCONN_PINMODE4_P2p1_SHIFT     (2)       /* Bits 2-3: P2.1 mode control */
#define PINCONN_PINMODE4_P2p1_MASK      (3 << PINCONN_PINMODE4_P2p1_SHIFT)
#define PINCONN_PINMODE4_P2p2_SHIFT     (4)       /* Bits 4-5: P2.2 mode control */
#define PINCONN_PINMODE4_P2p2_MASK      (3 << PINCONN_PINMODE4_P2p2_SHIFT)
#define PINCONN_PINMODE4_P2p3_SHIFT     (6)       /* Bits 6-7: P2.3 mode control */
#define PINCONN_PINMODE4_P2p3_MASK      (3 << PINCONN_PINMODE4_P2p3_SHIFT)
#define PINCONN_PINMODE4_P2p4_SHIFT     (8)       /* Bits 8-9: P2.4 mode control */
#define PINCONN_PINMODE4_P2p4_MASK      (3 << PINCONN_PINMODE4_P2p4_SHIFT)
#define PINCONN_PINMODE4_P2p5_SHIFT     (10)      /* Bits 10-11: P2.5 mode control */
#define PINCONN_PINMODE4_P2p5_MASK      (3 << PINCONN_PINMODE4_P2p5_SHIFT)
#define PINCONN_PINMODE4_P2p6_SHIFT     (12)      /* Bits 12-13: P2.6 mode control */
#define PINCONN_PINMODE4_P2p6_MASK      (3 << PINCONN_PINMODE4_P2p6_SHIFT)
#define PINCONN_PINMODE4_P2p7_SHIFT     (14)      /* Bits 14-15: P2.7 mode control */
#define PINCONN_PINMODE4_P2p7_MASK      (3 << PINCONN_PINMODE4_P2p7_SHIFT)
#define PINCONN_PINMODE4_P2p8_SHIFT     (16)      /* Bits 16-17: P2.8 mode control */
#define PINCONN_PINMODE4_P2p8_MASK      (3 << PINCONN_PINMODE4_P2p8_SHIFT)
#define PINCONN_PINMODE4_P2p9_SHIFT     (18)      /* Bits 18-19: P2.9 mode control */
#define PINCONN_PINMODE4_P2p9_MASK      (3 << PINCONN_PINMODE4_P2p9_SHIFT)
#define PINCONN_PINMODE4_P2p10_SHIFT    (20)      /* Bits 20-21: P2.10 mode control */
#define PINCONN_PINMODE4_P2p10_MASK     (3 << PINCONN_PINMODE4_P2p10_SHIFT)
#define PINCONN_PINMODE4_P2p11_SHIFT    (22)      /* Bits 22-23: P2.11 mode control */
#define PINCONN_PINMODE4_P2p11_MASK     (3 << PINCONN_PINMODE4_P2p11_SHIFT)
#define PINCONN_PINMODE4_P2p12_SHIFT    (24)      /* Bits 24-25: P2.12 mode control */
#define PINCONN_PINMODE4_P2p12_MASK     (3 << PINCONN_PINMODE4_P2p12_SHIFT)
#define PINCONN_PINMODE4_P2p13_SHIFT    (26)      /* Bits 26-27: P2.13 mode control */
#define PINCONN_PINMODE4_P2p13_MASK     (3 << PINCONN_PINMODE4_P2p13_SHIFT)
                                                  /* Bits 28-31: Reserved */
/* Pin Mode select register 5 (PINMODE5: 0x4002c054)
 * Pin Mode select register 6 (PINMODE6: 0x4002c058)
 * No bit definitions -- do these registers exist?
 */

#define PINCONN_PINMODE5_P2_SHIFT(n)    PINCONN_PINMODEH_SHIFT(n) /* n=16,17,..31 */
#define PINCONN_PINMODE5_P2_MASK(n)     PINCONN_PINMODEH_MASK(n)  /* n=16,17,..31 */

#define PINCONN_PINMODE6_P3_SHIFT(n)    PINCONN_PINMODEL_SHIFT(n) /* n=0,1,..,15 */
#define PINCONN_PINMODE6_P3_MASK(n)     PINCONN_PINMODEL_MASK(n)  /* n=0,1,..,15 */

/* Pin Mode select register 7 (PINMODE7: 0x4002c05c) */

#define PINCONN_PINMODE7_P3_SHIFT(n)    PINCONN_PINMODEH_SHIFT(n) /* n=16,17,..31 */
#define PINCONN_PINMODE7_P3_MASK(n)     PINCONN_PINMODEH_MASK(n)  /* n=16,17,..31 */
                                                  /* Bits 0-17: Reserved */
#define PINCONN_PINMODE7_P3p25_SHIFT    (18)      /* Bits 18-19: P3.25 mode control */
#define PINCONN_PINMODE7_P3p25_MASK     (3 << PINCONN_PINMODE7_P3p25_SHIFT)
#define PINCONN_PINMODE7_P3p26_SHIFT    (20)      /* Bits 20-21: P3.26 mode control */
#define PINCONN_PINMODE7_P3p26_MASK     (3 << PINCONN_PINMODE7_P3p26_SHIFT)
                                                  /* Bits 22-31: Reserved */
/* Pin Mode select register 9 (PINMODE9: 0x4002c064) */

#define PINCONN_PINMODE9_P4_SHIFT(n)    PINCONN_PINMODEH_SHIFT(n) /* n=16,17,..31 */
#define PINCONN_PINMODE9_P4_MASK(n)     PINCONN_PINMODEH_MASK(n)  /* n=16,17,..31 */
                                                  /* Bits 0-23: Reserved */
#define PINCONN_PINMODE9_P4p28_SHIFT    (24)      /* Bits 24-25: P4.28 mode control */
#define PINCONN_PINMODE9_P4p28_MASK     (3 << PINCONN_PINMODE9_P4p28_SHIFT)
#define PINCONN_PINMODE9_P4p29_SHIFT    (26)      /* Bits 26-27: P4.29 mode control */
#define PINCONN_PINMODE9_P4p29_MASK     (3 << PINCONN_PINMODE9_P4p29_SHIFT)
                                                  /* Bits 28-31: Reserved */
/* Open Drain Pin Mode select register 0 (PINMODE_OD0: 0x4002c068) */

#define PINCONN_ODMODE0_P0(n)           (1 << (n))

#define PINCONN_ODMODE0_P0p0            (1 << 0)  /* Bit  0:  P0.0 open drain mode */
#define PINCONN_ODMODE0_P0p1            (1 << 1)  /* Bit  1:  P0.1 open drain mode */
#define PINCONN_ODMODE0_P0p2            (1 << 2)  /* Bit  2:  P0.2 open drain mode */
#define PINCONN_ODMODE0_P0p3            (1 << 3)  /* Bit  3:  P0.3 open drain mode */
#define PINCONN_ODMODE0_P0p4            (1 << 4)  /* Bit  4:  P0.4 open drain mode */
#define PINCONN_ODMODE0_P0p5            (1 << 5)  /* Bit  5:  P0.5 open drain mode */
#define PINCONN_ODMODE0_P0p6            (1 << 6)  /* Bit  6:  P0.6 open drain mode */
#define PINCONN_ODMODE0_P0p7            (1 << 7)  /* Bit  7:  P0.7 open drain mode */
#define PINCONN_ODMODE0_P0p8            (1 << 8)  /* Bit  8:  P0.8 open drain mode */
#define PINCONN_ODMODE0_P0p9            (1 << 9)  /* Bit  9:  P0.9 open drain mode */
#define PINCONN_ODMODE0_P0p10           (1 << 10) /* Bit 10: P0.10 open drain mode */
#define PINCONN_ODMODE0_P0p11           (1 << 11) /* Bit 11: P0.11 open drain mode */
                                                  /* Bits 12-14: Reserved */
#define PINCONN_ODMODE0_P0p15           (1 << 15) /* Bit 15: P0.15 open drain mode */
#define PINCONN_ODMODE0_P0p16           (1 << 16) /* Bit 16: P0.16 open drain mode */
#define PINCONN_ODMODE0_P0p17           (1 << 17) /* Bit 17: P0.17 open drain mode */
#define PINCONN_ODMODE0_P0p18           (1 << 18) /* Bit 18: P0.18 open drain mode */
#define PINCONN_ODMODE0_P0p19           (1 << 19) /* Bit 19: P0.19 open drain mode */
#define PINCONN_ODMODE0_P0p20           (1 << 20) /* Bit 20: P0.20 open drain mode */
#define PINCONN_ODMODE0_P0p21           (1 << 21) /* Bit 21: P0.21 open drain mode */
#define PINCONN_ODMODE0_P0p22           (1 << 22) /* Bit 22: P0.22 open drain mode */
#define PINCONN_ODMODE0_P0p23           (1 << 23) /* Bit 23: P0.23 open drain mode */
#define PINCONN_ODMODE0_P0p24           (1 << 24) /* Bit 24: P0.24 open drain mode */
#define PINCONN_ODMODE0_P0p25           (1 << 25) /* Bit 25: P0.25 open drain mode */
#define PINCONN_ODMODE0_P0p26           (1 << 25) /* Bit 26: P0.26 open drain mode */
                                                  /* Bits 27-28: Reserved */
#define PINCONN_ODMODE0_P0p29           (1 << 29) /* Bit 29: P0.29 open drain mode */
#define PINCONN_ODMODE0_P0p30           (1 << 30) /* Bit 30: P0.30 open drain mode */
                                                  /* Bit 31: Reserved */
/* Open Drain Pin Mode select register 1 (PINMODE_OD1: 0x4002c06c) */

#define PINCONN_ODMODE1_P1(n)           (1 << (n))

#define PINCONN_ODMODE1_P1p0            (1 << 0)  /* Bit  0:  P1.0 open drain mode */
#define PINCONN_ODMODE1_P1p1            (1 << 1)  /* Bit  1:  P1.1 open drain mode */
                                                  /* Bits 2-3: Reserved */
#define PINCONN_ODMODE1_P1p4            (1 << 4)  /* Bit  4:  P1.4 open drain mode */
                                                  /* Bits 5-7: Reserved */
#define PINCONN_ODMODE1_P1p8            (1 << 8)  /* Bit  8:  P1.8 open drain mode */
#define PINCONN_ODMODE1_P1p9            (1 << 9)  /* Bit  9:  P1.9 open drain mode */
#define PINCONN_ODMODE1_P1p10           (1 << 10) /* Bit 10: P1.10 open drain mode */
                                                  /* Bits 11-13: Reserved */
#define PINCONN_ODMODE1_P1p14           (1 << 14) /* Bit 14: P1.14 open drain mode */
#define PINCONN_ODMODE1_P1p15           (1 << 15) /* Bit 15: P1.15 open drain mode */
#define PINCONN_ODMODE1_P1p16           (1 << 16) /* Bit 16: P1.16 open drain mode */
#define PINCONN_ODMODE1_P1p17           (1 << 17) /* Bit 17: P1.17 open drain mode */
#define PINCONN_ODMODE1_P1p18           (1 << 18) /* Bit 18: P1.18 open drain mode */
#define PINCONN_ODMODE1_P1p19           (1 << 19) /* Bit 19: P1.19 open drain mode */
#define PINCONN_ODMODE1_P1p20           (1 << 20) /* Bit 20: P1.20 open drain mode */
#define PINCONN_ODMODE1_P1p21           (1 << 21) /* Bit 21: P1.21 open drain mode */
#define PINCONN_ODMODE1_P1p22           (1 << 22) /* Bit 22: P1.22 open drain mode */
#define PINCONN_ODMODE1_P1p23           (1 << 23) /* Bit 23: P1.23 open drain mode */
#define PINCONN_ODMODE1_P1p24           (1 << 24) /* Bit 24: P1.24 open drain mode */
#define PINCONN_ODMODE1_P1p25           (1 << 25) /* Bit 25: P1.25 open drain mode */
#define PINCONN_ODMODE1_P1p26           (1 << 25) /* Bit 26: P1.26 open drain mode */
#define PINCONN_ODMODE1_P1p27           (1 << 27) /* Bit 27: P1.27 open drain mode */
#define PINCONN_ODMODE1_P1p28           (1 << 28) /* Bit 28: P1.28 open drain mode */
#define PINCONN_ODMODE1_P1p29           (1 << 29) /* Bit 29: P1.29 open drain mode */
#define PINCONN_ODMODE1_P1p30           (1 << 30) /* Bit 30: P1.30 open drain mode */
#define PINCONN_ODMODE1_P1p31           (1 << 31) /* Bit 31: P1.31 open drain mode */

/* Open Drain Pin Mode select register 2 (PINMODE_OD2: 0x4002c070) */

#define PINCONN_ODMODE2_P2(n)           (1 << (n))

#define PINCONN_ODMODE2_P2p0            (1 << 0)  /* Bit  0:  P2.0 open drain mode */
#define PINCONN_ODMODE2_P2p1            (1 << 1)  /* Bit  1:  P2.1 open drain mode */
#define PINCONN_ODMODE2_P2p2            (1 << 2)  /* Bit  2:  P2.2 open drain mode */
#define PINCONN_ODMODE2_P2p3            (1 << 3)  /* Bit  3:  P2.3 open drain mode */
#define PINCONN_ODMODE2_P2p4            (1 << 4)  /* Bit  4:  P2.4 open drain mode */
#define PINCONN_ODMODE2_P2p5            (1 << 5)  /* Bit  5:  P2.5 open drain mode */
#define PINCONN_ODMODE2_P2p6            (1 << 6)  /* Bit  6:  P2.6 open drain mode */
#define PINCONN_ODMODE2_P2p7            (1 << 7)  /* Bit  7:  P2.7 open drain mode */
#define PINCONN_ODMODE2_P2p8            (1 << 8)  /* Bit  8:  P2.8 open drain mode */
#define PINCONN_ODMODE2_P2p9            (1 << 9)  /* Bit  9:  P2.9 open drain mode */
#define PINCONN_ODMODE2_P2p10           (1 << 10) /* Bit 10: P2.10 open drain mode */
#define PINCONN_ODMODE2_P2p11           (1 << 11) /* Bit 11: P2.11 open drain mode */
#define PINCONN_ODMODE2_P2p12           (1 << 12) /* Bit 12: P2.12 open drain mode */
#define PINCONN_ODMODE2_P2p13           (1 << 13) /* Bit 13: P2.13 open drain mode */
                                                  /* Bits 14-31: Reserved */
/* Open Drain Pin Mode select register 3 (PINMODE_OD3: 0x4002c074) */

#define PINCONN_ODMODE3_P3(n)           (1 << (n))
                                                  /* Bits 0-24: Reserved */
#define PINCONN_ODMODE3_P3p25           (1 << 25) /* Bit 25: P3.25 open drain mode */
#define PINCONN_ODMODE3_P3p26           (1 << 25) /* Bit 26: P3.26 open drain mode */
                                                  /* Bits 17-31: Reserved */
/* Open Drain Pin Mode select register 4 (PINMODE_OD4: 0x4002c078) */

#define PINCONN_ODMODE4_P4(n)           (1 << (n))
                                                  /* Bits 0-27: Reserved */
#define PINCONN_ODMODE4_P4p28           (1 << 28) /* Bit 28: P4.28 open drain mode */
#define PINCONN_ODMODE4_P4p29           (1 << 29) /* Bit 29: P4.29 open drain mode */
                                                  /* Bits 30-31: Reserved */
/* I2C Pin Configuration register (I2CPADCFG: 0x4002c07c) */

#define PINCONN_I2CPADCFG_SDADRV0     (1 << 0)    /* Bit 0: SDA0 pin, P0.27 in Fast Mode Plus */
#define PINCONN_I2CPADCFG_SDAI2C0     (1 << 1)    /* Bit 1: SDA0 pin, P0.27 I2C glitch
                                                   * filtering/slew rate control */
#define PINCONN_I2CPADCFG_SCLDRV0     (1 << 2)    /* Bit 2: SCL0 pin, P0.28 in Fast Mode Plus */
#define PINCONN_I2CPADCFG_SCLI2C0     (1 << 3)    /* Bit 3: SCL0 pin, P0.28 I2C glitch
                                                   * filtering/slew rate control */
                                                  /* Bits 4-31: Reserved */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC17XX_CHIP_LPC176X_PINCONN_H */
