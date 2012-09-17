/****************************************************************************************************
 * arch/arm/src/lpc2378/lpc23xx_scb.h
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
 ****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC2378_LPC23XX_SCB_H
#define __ARCH_ARM_SRC_LPC2378_LPC23XX_SCB_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <chip.h>
#include "lpc23xx_vic.h"

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/
#define scb_getreg(o)    getreg32(LPC23XX_SCB_BASE + (o))
#define scb_putreg(v,o)  putreg32((v),LPC23XX_SCB_BASE + (o))

/* Memory Accelerator Mode */
#define MAMCR_OFF     0
#define MAMCR_PART    1
#define MAMCR_FULL    2

/* Memory Mapping */
#define MEMMAP2BBLK   0 /* Interrupt Vectors in Boot Block */
#define MEMMAP2FLASH  1 /* Interrupt Vectors in FLASH */
#define MEMMAP2SRAM   2 /* Interrupt Vectors in RAM */

/* Register bit settings */

/* PLL Control Register Bit Settings */

#define PLLCON_PLLE		(1 << 0) /* PLL Enable */
#define PLLCON_PLLC		(1 << 1) /* PLL Connect */

/* PLL Configuration Register Bit Settings */

#define PLLCFG_MSEL		(0x0000FFFF << 0)  /* PLL Multiplier (minus 1) */
#define PLLCFG_NSEL		(0x000000FF << 16) /* PLL Divider */


/* PLL Status Register Bit Settings */

#define PLLSTAT_MSEL	(0x7FFF << 0)  /* PLL Multiplier Readback */
#define PLLSTAT_NSEL	(0xFF << 16)   /* PLL Divider Readback */
#define PLLSTAT_PLLE	(1 << 24)      /* PLL Enable Readback */
#define PLLSTAT_PLLC	(1 << 25)      /* PLL Connect Readback */
#define PLLSTAT_PLOCK	(1 << 26)      /* PLL Lock Status */

/* PLL Feed Register values */

#define PLLFEED1	0xaa
#define PLLFEED2	0x55


/* Peripheral Power Control (PCONP) Register 0xE01FC0C4 */

#define PCTIM0		(1 << 1)	/* Timer/Counter 0 */
#define PCTIM1		(1 << 2)	/* Timer/Counter 1 */
#define PCUART0		(1 << 3)	/* UART0 power/clock */
#define PCUART1 	(1 << 4)	/* UART1 power/clock */
#define PCPWM1		(1 << 5)	/* Unused, always 0 */
#define PWM1		(1 << 6)	/* Pulse Width Modulation 1 */
#define PCI2C0		(1 << 7)	/* I2C0 interface */
#define PCSPI		(1 << 8)	/* SPI */
#define PCRTC		(1 << 9)	/* Real Time Clock*/
#define PCSSP1		(1 << 10)	/* SSP1 */
#define PCEMC		(1 << 11)	/* External Memory Controller */
#define PCAD		(1 << 12)	/* A/D converter (ADC) Note: Clear the PDN bit in the AD0CR before
											clearing this bit, and set this bit before setting PDN */
#define PCAN1		(1 << 13)	/* CAN Controller 1 */
#define PCAN2		(1 << 14)	/* CAN Controller 2 */
#define PCI2C1		(1 << 19)	/* The I2C1 interface power/clock control bit */
#define PCSSP0		(1 << 21)	/* The SSP0 interface power/clock control bit */
#define PCTIM2		(1 << 22)	/* Timer 2 */
#define PCTIM3		(1 << 23)	/* Timer 3 */
#define PCUART2		(1 << 24)	/* UART 2 */
#define PCUART3		(1 << 25)	/* UART 3 */
#define PCI2C2		(1 << 26)	/* I2C interface 2 */
#define PCI2S		(1 << 27)	/* I2S interface */
#define PCSDC		(1 << 28)	/* SD card interface */
#define PCGPDMA		(1 << 29)	/* GP DMA function */
#define PCENET		(1 << 30)	/* Ethernet block */
#define PCUSB		(1 << 31)	/* USB interface */
/****************************************************************************************************
 * Inline Functions
 ****************************************************************************************************/

/****************************************************************************************************
 * Global Function Prototypes
 ****************************************************************************************************/

#endif  /* __ARCH_ARM_SRC_LPC2378_LPC23XX_SCB_H */
