/************************************************************************************
 * arch/arm/src/kinetis/kinetis_sim.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_SIM_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_SIM_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define KINETIS_SIM_SOPT1_OFFSET      0x0000 /* System Options Register 1 */
#define KINETIS_SIM_SOPT2_OFFSET      0x0004 /* System Options Register 2 */
#define KINETIS_SIM_SOPT4_OFFSET      0x000c /* System Options Register 4 */
#define KINETIS_SIM_SOPT5_OFFSET      0x0010 /* System Options Register 5 */
#define KINETIS_SIM_SOPT6_OFFSET      0x0014 /* System Options Register 6 */
#define KINETIS_SIM_SOPT7_OFFSET      0x0018 /* System Options Register 7 */
#define KINETIS_SIM_SDID_OFFSET       0x0024 /* System Device Identification Register */
#define KINETIS_SIM_SCGC1_OFFSET      0x0028 /* System Clock Gating Control Register 1 */
#define KINETIS_SIM_SCGC2_OFFSET      0x002c /* System Clock Gating Control Register 2 */
#define KINETIS_SIM_SCGC3_OFFSET      0x0030 /* System Clock Gating Control Register 3 */
#define KINETIS_SIM_SCGC4_OFFSET      0x0034 /* System Clock Gating Control Register 4 */
#define KINETIS_SIM_SCGC5_OFFSET      0x0038 /* System Clock Gating Control Register 5 */
#define KINETIS_SIM_SCGC6_OFFSET      0x003c /* System Clock Gating Control Register 6 */
#define KINETIS_SIM_SCGC7_OFFSET      0x0040 /* System Clock Gating Control Register 7 */
#define KINETIS_SIM_CLKDIV1_OFFSET    0x0044 /* System Clock Divider Register 1 */
#define KINETIS_SIM_CLKDIV2_OFFSET    0x0048 /* System Clock Divider Register 2 */
#define KINETIS_SIM_FCFG1_OFFSET      0x004c /* Flash Configuration Register 1 */
#define KINETIS_SIM_FCFG2_OFFSET      0x0050 /* Flash Configuration Register 2 */
#define KINETIS_SIM_UIDH_OFFSET       0x0054 /* Unique Identification Register High */
#define KINETIS_SIM_UIDMH_OFFSET      0x0058 /* Unique Identification Register Mid-High */
#define KINETIS_SIM_UIDML_OFFSET      0x005c /* Unique Identification Register Mid Low */
#define KINETIS_SIM_UIDL_OFFSET       0x0060 /* Unique Identification Register Low */

/* Register Addresses ***************************************************************/
/* NOTE: The SIM_SOPT1 register is located at a different base address than the
 * other SIM registers.
 */

#define KINETIS_SIM_SOPT1             (KINETIS_SIMLP_BASE+KINETIS_SIM_SOPT1_OFFSET)
#define KINETIS_SIM_SOPT2             (KINETIS_SIM_BASE+KINETIS_SIM_SOPT2_OFFSET)
#define KINETIS_SIM_SOPT4             (KINETIS_SIM_BASE+KINETIS_SIM_SOPT4_OFFSET)
#define KINETIS_SIM_SOPT5             (KINETIS_SIM_BASE+KINETIS_SIM_SOPT5_OFFSET)
#define KINETIS_SIM_SOPT6             (KINETIS_SIM_BASE+KINETIS_SIM_SOPT6_OFFSET)
#define KINETIS_SIM_SOPT7             (KINETIS_SIM_BASE+KINETIS_SIM_SOPT7_OFFSET)
#define KINETIS_SIM_SDID              (KINETIS_SIM_BASE+KINETIS_SIM_SDID_OFFSET)
#define KINETIS_SIM_SCGC1             (KINETIS_SIM_BASE+KINETIS_SIM_SCGC1_OFFSET)
#define KINETIS_SIM_SCGC2             (KINETIS_SIM_BASE+KINETIS_SIM_SCGC2_OFFSET)
#define KINETIS_SIM_SCGC3             (KINETIS_SIM_BASE+KINETIS_SIM_SCGC3_OFFSET)
#define KINETIS_SIM_SCGC4             (KINETIS_SIM_BASE+KINETIS_SIM_SCGC4_OFFSET)
#define KINETIS_SIM_SCGC5             (KINETIS_SIM_BASE+KINETIS_SIM_SCGC5_OFFSET)
#define KINETIS_SIM_SCGC6             (KINETIS_SIM_BASE+KINETIS_SIM_SCGC6_OFFSET)
#define KINETIS_SIM_SCGC7             (KINETIS_SIM_BASE+KINETIS_SIM_SCGC7_OFFSET)
#define KINETIS_SIM_CLKDIV1           (KINETIS_SIM_BASE+KINETIS_SIM_CLKDIV1_OFFSET)
#define KINETIS_SIM_CLKDIV2           (KINETIS_SIM_BASE+KINETIS_SIM_CLKDIV2_OFFSET)
#define KINETIS_SIM_FCFG1             (KINETIS_SIM_BASE+KINETIS_SIM_FCFG1_OFFSET)
#define KINETIS_SIM_FCFG2             (KINETIS_SIM_BASE+KINETIS_SIM_FCFG2_OFFSET)
#define KINETIS_SIM_UIDH              (KINETIS_SIM_BASE+KINETIS_SIM_UIDH_OFFSET)
#define KINETIS_SIM_UIDMH             (KINETIS_SIM_BASE+KINETIS_SIM_UIDMH_OFFSET)
#define KINETIS_SIM_UIDML             (KINETIS_SIM_BASE+KINETIS_SIM_UIDML_OFFSET)
#define KINETIS_SIM_UIDL              (KINETIS_SIM_BASE+KINETIS_SIM_UIDL_OFFSET)

/* Register Bit Definitions *********************************************************/

/* System Options Register 1 */
                                                /* Bits 0-11: Reserved */
#define SIM_SOPT1_RAMSIZE_SHIFT       (12)      /* Bits 12-15: RAM size */
#define SIM_SOPT1_RAMSIZE_MASK        (15 << SIM_SOPT1_RAMSIZE_SHIFT)
#  define SIM_SOPT1_RAMSIZE_32KB      (5 << SIM_SOPT1_RAMSIZE_SHIFT) /* 32 KBytes */
#  define SIM_SOPT1_RAMSIZE_64KB      (7 << SIM_SOPT1_RAMSIZE_SHIFT) /* 64 KBytes */
#  define SIM_SOPT1_RAMSIZE_96KB      (8 << SIM_SOPT1_RAMSIZE_SHIFT) /* 96 KBytes */
#  define SIM_SOPT1_RAMSIZE_128KB     (9 << SIM_SOPT1_RAMSIZE_SHIFT) /* 128 KBytes */
                                                /* Bits 16-18: Reserved */
#define SIM_SOPT1_OSC32KSEL           (1 << 19) /* Bit 19: 32K oscillator clock select */
                                                /* Bits 20-22: Reserved */
#define SIM_SOPT1_MS                  (1 << 23) /* Bit 23: EzPort chip select pin state */
                                                /* Bits 24-29: Reserved */
#define SIM_SOPT1_USBSTBY             (1 << 30) /* Bit 30: USB voltage regulator in standby mode */
#define SIM_SOPT1_USBREGEN            (1 << 31) /* Bit 31: USB voltage regulator enable */

/* System Options Register 2 */

#define SIM_SOPT2_MCGCLKSEL           (1 << 0)  /* Bit 0:  MCG clock select */
                                                /* Bits 1-7: Reserved */
#define SIM_SOPT2_FBSL_SHIFT          (8)       /* Bits 8-9: FlexBus security level */
#define SIM_SOPT2_FBSL_MASK           (3 << SIM_SOPT2_FBSL_SHIFT)
#  define SIM_SOPT2_FBSL_NONE         (0 << SIM_SOPT2_FBSL_SHIFT) /* All off-chip accesses disallowed */
#  define SIM_SOPT2_FBSL_DATA         (2 << SIM_SOPT2_FBSL_SHIFT) /* Off-chip data accesses are allowed */
#  define SIM_SOPT2_FBSL_ALL          (3 << SIM_SOPT2_FBSL_SHIFT) /* All Off-chip accesses allowed */
                                                /* Bit 10: Reserved */
#define SIM_SOPT2_CMTUARTPAD          (1 << 11) /* Bit 11: CMT/UART pad drive strength */
#define SIM_SOPT2_TRACECLKSEL         (1 << 12) /* Bit 12: Debug trace clock select */
                                                /* Bits 13-15: Reserved */
#define SIM_SOPT2_PLLFLLSEL           (1 << 16) /* Bit 16: PLL/FLL clock select */
                                                /* Bit 17: Reserved */
#define SIM_SOPT2_USBSRC              (1 << 18) /* Bit 18: USB clock source select */
                                                /* Bit 19: Reserved */
#ifdef KINETIS_K60
#  define SIM_SOPT2_TIMESRC           (1 << 20) /* Bit 20: IEEE 1588 timestamp clock source select (K60) */
#endif
                                                /* Bits 12-23: Reserved */
#define SIM_SOPT2_I2SSRC_SHIFT        (24)      /* Bits 24-25: I2S master clock source select */
#define SIM_SOPT2_I2SSRC_MASK         (3 << SIM_SOPT2_I2SSRC_SHIFT)
#  define SIM_SOPT2_I2SCSRC_CORE      (0 << SIM_SOPT2_I2SSRC_SHIFT) /* Core/system clock / I2S fractional divider*/
#  define SIM_SOPT2_I2SCSRC_MCGCLK    (1 << SIM_SOPT2_I2SSRC_SHIFT) /* MCGPLLCLK/MCGFLLCLK clock/ I2S fractional divider */
#  define SIM_SOPT2_I2SCSRC_OCSERCLK  (2 << SIM_SOPT2_I2SSRC_SHIFT) /* OSCERCLK clock */
#  define SIM_SOPT2_I2SCSRC_EXTBYP    (3 << SIM_SOPT2_I2SSRC_SHIFT) /* External bypass clock (I2S0_CLKIN) */
                                                /* Bits 26-27: Reserved */
#define SIM_SOPT2_SDHCSRC_SHIFT       (28)      /* Bits 28-29: SDHC clock source select*/
#define SIM_SOPT2_SDHCSRC_MASK        (3 << SIM_SOPT2_SDHCSRC_SHIFT)
#  define SIM_SOPT2_SDHCSRC_CORE      (0 << SIM_SOPT2_SDHCSRC_SHIFT) /* Core/system clock */
#  define SIM_SOPT2_SDHCSRC_MCGCLK    (1 << SIM_SOPT2_SDHCSRC_SHIFT) /* MCGPLLCLK/MCGFLLCLK clock */
#  define SIM_SOPT2_SDHCSRC_OCSERCLK  (2 << SIM_SOPT2_SDHCSRC_SHIFT) /* OSCERCLK clock */
#  define SIM_SOPT2_SDHCSRC_EXTBYP    (3 << SIM_SOPT2_SDHCSRC_SHIFT) /* External bypass clock (SDHC0_CLKIN) */                                                /* Bits 30-31: Reserved */

/* System Options Register 4 */

#define SIM_SOPT4_FTM0FLT0            (1 << 0)  /* Bit 0:  FTM0 Fault 0 Select */
#define SIM_SOPT4_FTM0FLT1            (1 << 1)  /* Bit 1:  FTM0 Fault 1 Select */
#define SIM_SOPT4_FTM0FLT2            (1 << 2)  /* Bit 2:  FTM0 Fault 2 Select */
                                                /* Bit 3: Reserved */
#define SIM_SOPT4_FTM1FLT0            (1 << 4)  /* Bit 4:  FTM1 Fault 0 Select */
                                                /* Bits 5-7: Reserved */
#define SIM_SOPT4_FTM2FLT0            (1 << 8)  /* Bit 8:  FTM2 Fault 0 Select */
                                                /* Bits 9-17: Reserved */
#define SIM_SOPT4_FTM1CH0SRC_SHIFT    (18)      /* Bits 18-19: FTM1 channel 0 input capture source select */
#define SIM_SOPT4_FTM1CH0SRC_MASK     (3 << SIM_SOPT4_FTM1CH0SRC_SHIFT)
#  define SIM_SOPT4_FTM1CH0SRC_CH0    (0 << SIM_SOPT4_FTM1CH0SRC_SHIFT) /* FTM1_CH0 signal */
#  define SIM_SOPT4_FTM1CH0SRC_CMP0   (1 << SIM_SOPT4_FTM1CH0SRC_SHIFT) /* CMP0 output */
#  define SIM_SOPT4_FTM1CH0SRC_CMP1   (2 << SIM_SOPT4_FTM1CH0SRC_SHIFT) /* CMP1 output */
#define SIM_SOPT4_FTM2CH0SRC_SHIFT    (20)      /* Bits 20-21: FTM2 channel 0 input capture source select */
#define SIM_SOPT4_FTM2CH0SRC_MASK     (3 << SIM_SOPT4_FTM2CH0SRC_SHIFT)
#  define SIM_SOPT4_FTM2CH0SRC_CH0    (0 << SIM_SOPT4_FTM2CH0SRC_SHIFT) /* FTM2_CH0 signal */
#  define SIM_SOPT4_FTM2CH0SRC_CMP0   (1 << SIM_SOPT4_FTM2CH0SRC_SHIFT) /* CMP0 output */
#  define SIM_SOPT4_FTM2CH0SRC_CMP1   (2 << SIM_SOPT4_FTM2CH0SRC_SHIFT) /* CMP1 output */
                                                /* Bits 22-23: Reserved */
#define SIM_SOPT4_FTM0CLKSEL          (1 << 24) /* Bit 24:  FlexTimer 0 External Clock Pin Select */
#define SIM_SOPT4_FTM1CLKSEL          (1 << 25) /* Bit 25:  FTM1 External Clock Pin Select */
#define SIM_SOPT4_FTM2CLKSEL          (1 << 26) /* Bit 26:  FlexTimer 2 External Clock Pin Select */
                                                /* Bits 27-31: Reserved */

/* System Options Register 5 */

#define SIM_SOPT5_UART0TXSRC_SHIFT    (0)       /* Bits 0-1: UART 0 transmit data source select */
#define SIM_SOPT5_UART0TXSRC_MASK     (3 << SIM_SOPT5_UART0TXSRC_SHIFT)
#  define SIM_SOPT5_UART0TXSRC_TX     (0 << SIM_SOPT5_UART0TXSRC_SHIFT) /* UART0_TX pin */
#  define SIM_SOPT5_UART0TXSRC_FTM1   (1 << SIM_SOPT5_UART0TXSRC_SHIFT) /* UART0_TX modulated with FTM1 ch0 output */
#  define SIM_SOPT5_UART0TXSRC_FTM2   (2 << SIM_SOPT5_UART0TXSRC_SHIFT) /* UART0_TX modulated with FTM2 ch0 output */
#define SIM_SOPT5_UART0RXSRC_SHIFT    (2)       /* Bits 2-3: UART 0 receive data source select */
#define SIM_SOPT5_UART0RXSRC_MASK     (3 << SIM_SOPT5_UART0RXSRC_SHIFT)
#  define SIM_SOPT5_UART0RXSRC_RX     (0 << SIM_SOPT5_UART0RXSRC_SHIFT) /* UART0_RX pin */
#  define SIM_SOPT5_UART0RXSRC_CMP0   (1 << SIM_SOPT5_UART0RXSRC_SHIFT) /* CMP0 */
#  define SIM_SOPT5_UART0RXSRC_CMP1   (2 << SIM_SOPT5_UART0RXSRC_SHIFT) /* CMP1 */
#define SIM_SOPT5_UART1TXSRC_SHIFT    (4)       /* Bits 4-5: UART 1 transmit data source select */
#define SIM_SOPT5_UART1TXSRC_MASK     (3 << SIM_SOPT5_UART1TXSRC_SHIFT)
#  define SIM_SOPT5_UART1TXSRC_TX     (0 << SIM_SOPT5_UART1TXSRC_SHIFT) /* UART1_TX pin */
#  define SIM_SOPT5_UART1TXSRC_FTM1   (1 << SIM_SOPT5_UART1TXSRC_SHIFT) /* UART1_TX modulated with FTM1 ch0 output */
#  define SIM_SOPT5_UART1TXSRC_FTM2   (2 << SIM_SOPT5_UART1TXSRC_SHIFT) /* UART1_TX modulated with FTM2 ch0 output */
#define SIM_SOPT5_UART1RXSRC_SHIFT    (6)       /* Bits 6-7: UART 1 receive data source select */
#define SIM_SOPT5_UART1RXSRC_MASK     (3 << SIM_SOPT5_UART1RXSRC_SHIFT)
#  define SIM_SOPT5_UART1RXSRC_RX     (0 << SIM_SOPT5_UART1RXSRC_SHIFT) /* UART1_RX pin */
#  define SIM_SOPT5_UART1RXSRC_CMP0   (1 << SIM_SOPT5_UART1RXSRC_SHIFT) /* CMP0 */
#  define SIM_SOPT5_UART1RXSRC_CMP1   (2 << SIM_SOPT5_UART1RXSRC_SHIFT) /* CMP1 */
                                                /* Bits 8-31: Reserved */
/* System Options Register 6 */
                                                /* Bits 0-23: Reserved */
#define SIM_SOPT6_RSTFLTSEL_SHIFT     (24)      /* Bits 24-28: Reset pin filter select */
#define SIM_SOPT6_RSTFLTSEL_MASK      (31 << SIM_SOPT6_RSTFLTSEL_SHIFT)
#   define SIM_SOPT6_RSTFLTSEL(n)     (((n)-1) << SIM_SOPT6_RSTFLTSEL_SHIFT) /* Bux clock filter count n, n=1..32 */
#define SIM_SOPT6_RSTFLTEN_SHIFT      (29)      /* Bits 29-31: Reset pin filter enable */
#define SIM_SOPT6_RSTFLTEN_MASK       (7 << SIM_SOPT6_RSTFLTEN_SHIFT)
#define SIM_SOPT6_RSTFLTEN_DISABLED   (0 << SIM_SOPT6_RSTFLTEN_SHIFT) /* All filtering disabled */
#  define SIM_SOPT6_RSTFLTEN_BUSCLK1  (1 << SIM_SOPT6_RSTFLTEN_SHIFT) /* Bus clock filter enabled (normal); LPO clock filter enabled (stop) */
#  define SIM_SOPT6_RSTFLTEN_LPO1     (2 << SIM_SOPT6_RSTFLTEN_SHIFT) /* LPO clock filter enabled */
#  define SIM_SOPT6_RSTFLTEN_BUSCLK2  (3 << SIM_SOPT6_RSTFLTEN_SHIFT) /* Bus clock filter enabled (normal); All filtering disabled (stop) */
#  define SIM_SOPT6_RSTFLTEN_LPO2     (4 << SIM_SOPT6_RSTFLTEN_SHIFT) /* PO clock filter enabled (normal); All filtering disabled (stop) */

/* System Options Register 7 */

#define SIM_SOPT7_ADC0TRGSEL_SHIFT    (0)       /* Bits 0-3: ADC0 trigger select */
#define SIM_SOPT7_ADC0TRGSEL_MASK     (15 << SIM_SOPT7_ADC0TRGSEL_SHIFT)
#  define SIM_SOPT7_ADC0TRGSEL_PDB    (0 << SIM_SOPT7_ADC0TRGSEL_SHIFT)  /* PDB external trigger (PDB0_EXTRG) */
#  define SIM_SOPT7_ADC0TRGSEL_CMP0   (1 << SIM_SOPT7_ADC0TRGSEL_SHIFT)  /* High speed comparator 0 output */
#  define SIM_SOPT7_ADC0TRGSEL_CMP1   (2 << SIM_SOPT7_ADC0TRGSEL_SHIFT)  /* High speed comparator 1 output */
#  define SIM_SOPT7_ADC0TRGSEL_CMP2   (3 << SIM_SOPT7_ADC0TRGSEL_SHIFT)  /* High speed comparator 2 output */
#  define SIM_SOPT7_ADC0TRGSEL_PIT0   (4 << SIM_SOPT7_ADC0TRGSEL_SHIFT)  /* PIT trigger 0 */
#  define SIM_SOPT7_ADC0TRGSEL_PIT1   (5 << SIM_SOPT7_ADC0TRGSEL_SHIFT)  /* PIT trigger 1 */
#  define SIM_SOPT7_ADC0TRGSEL_PIT2   (6 << SIM_SOPT7_ADC0TRGSEL_SHIFT)  /* PIT trigger 2 */
#  define SIM_SOPT7_ADC0TRGSEL_PIT3   (7 << SIM_SOPT7_ADC0TRGSEL_SHIFT)  /* PIT trigger 3 */
#  define SIM_SOPT7_ADC0TRGSEL_FTM0   (8 << SIM_SOPT7_ADC0TRGSEL_SHIFT)  /* FTM0 trigger */
#  define SIM_SOPT7_ADC0TRGSEL_FTM1   (9 << SIM_SOPT7_ADC0TRGSEL_SHIFT)  /* FTM1 trigger */
#  define SIM_SOPT7_ADC0TRGSEL_FTM2   (10 << SIM_SOPT7_ADC0TRGSEL_SHIFT) /* FTM2 trigger */
#  define SIM_SOPT7_ADC0TRGSEL_ALARM  (12 << SIM_SOPT7_ADC0TRGSEL_SHIFT) /* RTC alarm */
#  define SIM_SOPT7_ADC0TRGSEL_SECS   (13 << SIM_SOPT7_ADC0TRGSEL_SHIFT) /* RTC seconds */
#  define SIM_SOPT7_ADC0TRGSEL_LPTMR  (14 << SIM_SOPT7_ADC0TRGSEL_SHIFT) /* Low-power timer trigger */
#define SIM_SOPT7_ADC0PRETRGSEL       (1 << 4)  /* Bit 4:  ADC0 pretrigger select */
                                                /* Bits 5-6: Reserved */
#define SIM_SOPT7_ADC0ALTTRGEN        (1 << 7)  /* Bit 7:  ADC0 alternate trigger enable */
#define SIM_SOPT7_ADC1TRGSEL_SHIFT    (8)       /* Bits 8-11: ADC1 trigger select */
#define SIM_SOPT7_ADC1TRGSEL_MASK     (15 << SIM_SOPT7_ADC1TRGSEL_SHIFT)
#  define SIM_SOPT7_ADC1TRGSEL_PDB    (0 << SIM_SOPT7_ADC1TRGSEL_SHIFT)  /* PDB external trigger (PDB0_EXTRG) */
#  define SIM_SOPT7_ADC1TRGSEL_CMP0   (1 << SIM_SOPT7_ADC1TRGSEL_SHIFT)  /* High speed comparator 0 output */
#  define SIM_SOPT7_ADC1TRGSEL_CMP1   (2 << SIM_SOPT7_ADC1TRGSEL_SHIFT)  /* High speed comparator 1 output */
#  define SIM_SOPT7_ADC1TRGSEL_CMP2   (3 << SIM_SOPT7_ADC1TRGSEL_SHIFT)  /* High speed comparator 2 output */
#  define SIM_SOPT7_ADC1TRGSEL_PIT0   (4 << SIM_SOPT7_ADC1TRGSEL_SHIFT)  /* PIT trigger 0 */
#  define SIM_SOPT7_ADC1TRGSEL_PIT1   (5 << SIM_SOPT7_ADC1TRGSEL_SHIFT)  /* PIT trigger 1 */
#  define SIM_SOPT7_ADC1TRGSEL_PIT2   (6 << SIM_SOPT7_ADC1TRGSEL_SHIFT)  /* PIT trigger 2 */
#  define SIM_SOPT7_ADC1TRGSEL_PIT3   (7 << SIM_SOPT7_ADC1TRGSEL_SHIFT)  /* PIT trigger 3 */
#  define SIM_SOPT7_ADC1TRGSEL_FTM0   (8 << SIM_SOPT7_ADC1TRGSEL_SHIFT)  /* FTM0 trigger */
#  define SIM_SOPT7_ADC1TRGSEL_FTM1   (9 << SIM_SOPT7_ADC1TRGSEL_SHIFT)  /* FTM1 trigger */
#  define SIM_SOPT7_ADC1TRGSEL_FTM2   (10 << SIM_SOPT7_ADC1TRGSEL_SHIFT) /* FTM2 trigger */
#  define SIM_SOPT7_ADC1TRGSEL_ALARM  (12 << SIM_SOPT7_ADC1TRGSEL_SHIFT) /* RTC alarm */
#  define SIM_SOPT7_ADC1TRGSEL_SECS   (13 << SIM_SOPT7_ADC1TRGSEL_SHIFT) /* RTC seconds */
#  define SIM_SOPT7_ADC1TRGSEL_LPTMR  (14 << SIM_SOPT7_ADC1TRGSEL_SHIFT) /* Low-power timer trigger */
#define SIM_SOPT7_ADC1PRETRGSEL       (1 << 12) /* Bit 12: ADC1 pre-trigger select */
                                                /* Bits 13-14: Reserved */
#define SIM_SOPT7_ADC1ALTTRGEN        (1 << 15) /* Bit 15: ADC1 alternate trigger enable */
                                                /* Bits 16-31: Reserved */
/* System Device Identification Register */

#define SIM_SDID_PINID_SHIFT          (0)       /* Bits 0-3: Pincount identification */
#define SIM_SDID_PINID_MASK           (15 << SIM_SDID_PINID_SHIFT)
#  define SIM_SDID_PINID_32PIN        (2 << SIM_SDID_PINID_SHIFT)  /* 32-pin */
#  define SIM_SDID_PINID_48PIN        (4 << SIM_SDID_PINID_SHIFT)  /* 48-pin */
#  define SIM_SDID_PINID_64PIN        (5 << SIM_SDID_PINID_SHIFT)  /* 64-pin */
#  define SIM_SDID_PINID_80PIN        (6 << SIM_SDID_PINID_SHIFT)  /* 80-pin */
#  define SIM_SDID_PINID_81PIN        (7 << SIM_SDID_PINID_SHIFT)  /* 81-pin */
#  define SIM_SDID_PINID_100PIN       (8 << SIM_SDID_PINID_SHIFT)  /* 100-pin */
#  define SIM_SDID_PINID_121PIN       (9 << SIM_SDID_PINID_SHIFT)  /* 121-pin */
#  define SIM_SDID_PINID_144PIN       (10 << SIM_SDID_PINID_SHIFT) /* 144-pin */
#  define SIM_SDID_PINID_196PIN       (12 << SIM_SDID_PINID_SHIFT) /* 196-pin */
#  define SIM_SDID_PINID_256PIN       (14 << SIM_SDID_PINID_SHIFT) /* 256-pin */
#define SIM_SDID_FAMID_SHIFT          (4)       /* Bits 4-6: Kinetis family identification */
#define SIM_SDID_FAMID_MASK           (7 << SIM_SDID_FAMID_SHIFT)
#  define SIM_SDID_FAMID_K10          (0 << SIM_SDID_FAMID_SHIFT) /* K10 */
#  define SIM_SDID_FAMID_K20          (1 << SIM_SDID_FAMID_SHIFT)) /* K20 */
#  define SIM_SDID_FAMID_K30          (2 << SIM_SDID_FAMID_SHIFT)) /* K30 */
#  define SIM_SDID_FAMID_K40          (3 << SIM_SDID_FAMID_SHIFT)) /* K40 */
#  define SIM_SDID_FAMID_K60          (4 << SIM_SDID_FAMID_SHIFT)) /* K60 */
#  define SIM_SDID_FAMID_K70          (5 << SIM_SDID_FAMID_SHIFT)) /* K70 */
#  define SIM_SDID_FAMID_K50          (6 << SIM_SDID_FAMID_SHIFT)) /* K50 and K52 */
#  define SIM_SDID_FAMID_K51          (7 << SIM_SDID_FAMID_SHIFT)) /* K51 and K53 */
                                                /* Bits 7-11: Reserved */
#define SIM_SDID_REVID_SHIFT          (12)      /* Bits 12-15: Device revision number */
#define SIM_SDID_REVID_MASK           (15 << SIM_SDID_REVID_SHIFT)
                                                /* Bits 16-31: Reserved */
/* System Clock Gating Control Register 1 */
                                                /* Bits 0-9: Reserved */
#define SIM_SCGC1_UART4               (1 << 10) /* Bit 10: UART4 Clock Gate Control */
#define SIM_SCGC1_UART5               (1 << 11) /* Bit 11: UART5 Clock Gate Control */
                                                /* Bits 12-31: Reserved */
/* System Clock Gating Control Register 2 */

#if defined(KINETIS_NENET) && KINETIS_NENET > 0
#  define SIM_SCGC2_ENET              (1 << 0)  /* Bit 0:  ENET Clock Gate Control (K60) */
#endif
                                                /* Bits 1-11: Reserved */
#define SIM_SCGC2_DAC0                (1 << 12) /* Bit 12: DAC0 Clock Gate Control */
#define SIM_SCGC2_DAC1                (1 << 13) /* Bit 13: DAC1 Clock Gate Control */
                                                /* Bits 14-31: Reserved */
/* System Clock Gating Control Register 3 */

#if defined(KINETIS_NRNG) && KINETIS_NRNG > 0
#  define SIM_SCGC3_RNGB              (1 << 0)  /* Bit 0:  RNGB Clock Gate Control (K60) */
#endif
                                                /* Bits 1-3: Reserved */
#define SIM_SCGC3_FLEXCAN1            (1 << 4)  /* Bit 4:  FlexCAN1 Clock Gate Control */
                                                /* Bits 5-11: Reserved */
#define SIM_SCGC3_SPI2                (1 << 12) /* Bit 12: SPI2 Clock Gate Control */
                                                /* Bits 13-16: Reserved */
#define SIM_SCGC3_SDHC                (1 << 17) /* Bit 17: SDHC Clock Gate Control */
                                                /* Bits 18-23: Reserved */
#define SIM_SCGC3_FTM2                (1 << 24) /* Bit 24: FTM2 Clock Gate Control */
                                                /* Bits 25-26: Reserved */
#define SIM_SCGC3_ADC1                (1 << 27) /* Bit 27: ADC1 Clock Gate Control */
                                                /* Bits 28-29: Reserved */
#if defined(KINETIS_NSLCD) && KINETIS_NSLCD > 0
#  define SIM_SCGC3_SLCD              (1 << 30) /* Bit 30: Segment LCD Clock Gate Control (K40) */
#endif
                                                /* Bit 31: Reserved */
/* System Clock Gating Control Register 4 */
                                                /* Bit 0:  Reserved */
#define SIM_SCGC4_EWM                 (1 << 1)  /* Bit 1:  EWM Clock Gate Control */
#define SIM_SCGC4_CMT                 (1 << 2)  /* Bit 2:  CMT Clock Gate Control */
                                                /* Bits 3-5: Reserved */
#define SIM_SCGC4_I2C0                (1 << 6)  /* Bit 6:  I2C0 Clock Gate Control */
#define SIM_SCGC4_I2C1                (1 << 7)  /* Bit 7:  I2C1 Clock Gate Control */
                                                /* Bits 8-9: Reserved */
#define SIM_SCGC4_UART0               (1 << 10) /* Bit 10: UART0 Clock Gate Control */
#define SIM_SCGC4_UART1               (1 << 11) /* Bit 11: UART1 Clock Gate Control */
#define SIM_SCGC4_UART2               (1 << 12) /* Bit 12: UART2 Clock Gate Control */
#define SIM_SCGC4_UART3               (1 << 13) /* Bit 13: UART3 Clock Gate Control */
                                                /* Bits 14-17: Reserved */
#define SIM_SCGC4_USBOTG              (1 << 18) /* Bit 18: USB Clock Gate Control */
#define SIM_SCGC4_CMP                 (1 << 19) /* Bit 19: Comparator Clock Gate Control */
#define SIM_SCGC4_VREF                (1 << 20) /* Bit 20: VREF Clock Gate Control */
                                                /* Bits 21-17: Reserved */
#define SIM_SCGC4_LLWU                (1 << 28) /* Bit 28: LLWU Clock Gate Control */
                                                /* Bits 29-31: Reserved */
/* System Clock Gating Control Register 5 */

#define SIM_SCGC5_LPTIMER             (1 << 0)  /* Bit 0:  Low Power Timer Clock Gate Control */
#define SIM_SCGC5_REGFILE             (1 << 1)  /* Bit 1:  Register File Clock Gate Control */
                                                /* Bits 2-4: Reserved */
#define SIM_SCGC5_TSI                 (1 << 5)  /* Bit 5:  TSI Clock Gate Control */
                                                /* Bits 6-8: Reserved */
#define SIM_SCGC5_PORTA               (1 << 9)  /* Bit 9:  Port A Clock Gate Control */
#define SIM_SCGC5_PORTB               (1 << 10) /* Bit 10: Port B Clock Gate Control */
#define SIM_SCGC5_PORTC               (1 << 11) /* Bit 11: Port C Clock Gate Control */
#define SIM_SCGC5_PORTD               (1 << 12) /* Bit 12: Port D Clock Gate Control */
#define SIM_SCGC5_PORTE               (1 << 13) /* Bit 13: Port E Clock Gate Control */
                                                /* Bits 14-31: Reserved */
/* System Clock Gating Control Register 6 */

#define SIM_SCGC6_FTFL                (1 << 0)  /* Bit 0:  Flash Memory Clock Gate Control */
#define SIM_SCGC6_DMAMUX              (1 << 1)  /* Bit 1:  DMA Mux Clock Gate Control */
                                                /* Bits 2-3: Reserved */
#define SIM_SCGC6_FLEXCAN0            (1 << 4)  /* Bit 4:  FlexCAN0 Clock Gate Control */
                                                /* Bits 5-11: Reserved */
#define SIM_SCGC6_SPI0                (1 << 12) /* Bit 12: SPI0 Clock Gate Control */
#define SIM_SCGC6_SPI1                (1 << 13) /* Bit 13: SPI1 Clock Gate Control */
                                                /* Bit 14: Reserved */
#define SIM_SCGC6_I2S                 (1 << 15) /* Bit 15: I2S Clock Gate Control */
                                                /* Bits 16-17: Reserved */
#define SIM_SCGC6_CRC                 (1 << 18) /* Bit 18: CRC Clock Gate Control */
                                                /* Bits 19-20: Reserved */
#define SIM_SCGC6_USBDCD              (1 << 21) /* Bit 21: USB DCD Clock Gate Control */
#define SIM_SCGC6_PDB                 (1 << 22) /* Bit 22: PDB Clock Gate Control */
#define SIM_SCGC6_PIT                 (1 << 23) /* Bit 23: PIT Clock Gate Control */
#define SIM_SCGC6_FTM0                (1 << 24) /* Bit 24: FTM0 Clock Gate Control */
#define SIM_SCGC6_FTM1                (1 << 25) /* Bit 25: FTM1 Clock Gate Control */
                                                /* Bit 26: Reserved */
#define SIM_SCGC6_ADC0                (1 << 27) /* Bit 27: ADC0 Clock Gate Control */
                                                /* Bit 28: Reserved */
#define SIM_SCGC6_RTC                 (1 << 29) /* Bit 29: RTC Clock Gate Control */
                                                /* Bits 30-31: Reserved */
/* System Clock Gating Control Register 7 */

#define SIM_SCGC7_FLEXBUS             (1 << 0)  /* Bit 0:  FlexBus Clock Gate Control */
#define SIM_SCGC7_DMA                 (1 << 1)  /* Bit 1:  DMA Clock Gate Control */
#define SIM_SCGC7_MPU                 (1 << 2)  /* Bit 2:  MPU Clock Gate Control */
                                                /* Bits 3-31: Reserved */
/* System Clock Divider Register 1 */
                                                /* Bits 0-15: Reserved */
#define SIM_CLKDIV1_OUTDIV4_SHIFT     (16)      /* Bits 16-19: Clock 4 output divider value */
#define SIM_CLKDIV1_OUTDIV4_MASK      (15 << SIM_CLKDIV1_OUTDIV4_SHIFT)
#  define SIM_CLKDIV1_OUTDIV4(n)      (((n)-1) << SIM_CLKDIV1_OUTDIV4_SHIFT) /* Divide by n, n=1..16 */
#  define SIM_CLKDIV1_OUTDIV4_1       (0 << SIM_CLKDIV1_OUTDIV4_SHIFT)  /* Divide by 1 */
#  define SIM_CLKDIV1_OUTDIV4_2       (1 << SIM_CLKDIV1_OUTDIV4_SHIFT)  /* Divide by 2 */
#  define SIM_CLKDIV1_OUTDIV4_3       (2 << SIM_CLKDIV1_OUTDIV4_SHIFT)  /* Divide by 3 */
#  define SIM_CLKDIV1_OUTDIV4_4       (3 << SIM_CLKDIV1_OUTDIV4_SHIFT)  /* Divide by 4 */
#  define SIM_CLKDIV1_OUTDIV4_5       (4 << SIM_CLKDIV1_OUTDIV4_SHIFT)  /* Divide by 5 */
#  define SIM_CLKDIV1_OUTDIV4_6       (5 << SIM_CLKDIV1_OUTDIV4_SHIFT)  /* Divide by 6 */
#  define SIM_CLKDIV1_OUTDIV4_7       (6 << SIM_CLKDIV1_OUTDIV4_SHIFT)  /* Divide by 7 */
#  define SIM_CLKDIV1_OUTDIV4_8       (7 << SIM_CLKDIV1_OUTDIV4_SHIFT)  /* Divide by 8 */
#  define SIM_CLKDIV1_OUTDIV4_9       (8 << SIM_CLKDIV1_OUTDIV4_SHIFT)  /* Divide by 9 */
#  define SIM_CLKDIV1_OUTDIV4_10      (9 << SIM_CLKDIV1_OUTDIV4_SHIFT)  /* Divide by 10 */
#  define SIM_CLKDIV1_OUTDIV4_11      (10 << SIM_CLKDIV1_OUTDIV4_SHIFT) /* Divide by 11 */
#  define SIM_CLKDIV1_OUTDIV4_12      (11 << SIM_CLKDIV1_OUTDIV4_SHIFT) /* Divide by 12 */
#  define SIM_CLKDIV1_OUTDIV4_13      (12 << SIM_CLKDIV1_OUTDIV4_SHIFT) /* Divide by 13 */
#  define SIM_CLKDIV1_OUTDIV4_14      (13 << SIM_CLKDIV1_OUTDIV4_SHIFT) /* Divide by 14 */
#  define SIM_CLKDIV1_OUTDIV4_15      (14 << SIM_CLKDIV1_OUTDIV4_SHIFT) /* Divide by 15 */
#  define SIM_CLKDIV1_OUTDIV4_16      (15 << SIM_CLKDIV1_OUTDIV4_SHIFT) /* Divide by 16 */
#define SIM_CLKDIV1_OUTDIV3_SHIFT     (20)      /* Bits 20-23: Clock 3 output divider value */
#define SIM_CLKDIV1_OUTDIV3_MASK      (15 << SIM_CLKDIV1_OUTDIV3_SHIFT)
#  define SIM_CLKDIV1_OUTDIV3(n)      (((n)-1) << SIM_CLKDIV1_OUTDIV3_SHIFT) /* Divide by n, n=1..16 */
#  define SIM_CLKDIV1_OUTDIV3_1       (0 << SIM_CLKDIV1_OUTDIV3_SHIFT)  /* Divide by 1 */
#  define SIM_CLKDIV1_OUTDIV3_2       (1 << SIM_CLKDIV1_OUTDIV3_SHIFT)  /* Divide by 2 */
#  define SIM_CLKDIV1_OUTDIV3_3       (2 << SIM_CLKDIV1_OUTDIV3_SHIFT)  /* Divide by 3 */
#  define SIM_CLKDIV1_OUTDIV3_4       (3 << SIM_CLKDIV1_OUTDIV3_SHIFT)  /* Divide by 4 */
#  define SIM_CLKDIV1_OUTDIV3_5       (4 << SIM_CLKDIV1_OUTDIV3_SHIFT)  /* Divide by 5 */
#  define SIM_CLKDIV1_OUTDIV3_6       (5 << SIM_CLKDIV1_OUTDIV3_SHIFT)  /* Divide by 6 */
#  define SIM_CLKDIV1_OUTDIV3_7       (6 << SIM_CLKDIV1_OUTDIV3_SHIFT)  /* Divide by 7 */
#  define SIM_CLKDIV1_OUTDIV3_8       (7 << SIM_CLKDIV1_OUTDIV3_SHIFT)  /* Divide by 8 */
#  define SIM_CLKDIV1_OUTDIV3_9       (8 << SIM_CLKDIV1_OUTDIV3_SHIFT)  /* Divide by 9 */
#  define SIM_CLKDIV1_OUTDIV3_10      (9 << SIM_CLKDIV1_OUTDIV3_SHIFT)  /* Divide by 10 */
#  define SIM_CLKDIV1_OUTDIV3_11      (10 << SIM_CLKDIV1_OUTDIV3_SHIFT) /* Divide by 11 */
#  define SIM_CLKDIV1_OUTDIV3_12      (11 << SIM_CLKDIV1_OUTDIV3_SHIFT) /* Divide by 12 */
#  define SIM_CLKDIV1_OUTDIV3_13      (12 << SIM_CLKDIV1_OUTDIV3_SHIFT) /* Divide by 13 */
#  define SIM_CLKDIV1_OUTDIV3_14      (13 << SIM_CLKDIV1_OUTDIV3_SHIFT) /* Divide by 14 */
#  define SIM_CLKDIV1_OUTDIV3_15      (14 << SIM_CLKDIV1_OUTDIV3_SHIFT) /* Divide by 15 */
#  define SIM_CLKDIV1_OUTDIV3_16      (15 << SIM_CLKDIV1_OUTDIV3_SHIFT) /* Divide by 16 */
#define SIM_CLKDIV1_OUTDIV2_SHIFT     (24)      /* Bits 24-27: Clock 2 output divider value */
#define SIM_CLKDIV1_OUTDIV2_MASK      (15 << SIM_CLKDIV1_OUTDIV2_SHIFT)
#  define SIM_CLKDIV1_OUTDIV2(n)      (((n)-1) << SIM_CLKDIV1_OUTDIV2_SHIFT) /* Divide by n, n=1..16 */
#  define SIM_CLKDIV1_OUTDIV2_1       (0 << SIM_CLKDIV1_OUTDIV2_SHIFT)  /* Divide by 1 */
#  define SIM_CLKDIV1_OUTDIV2_2       (1 << SIM_CLKDIV1_OUTDIV2_SHIFT)  /* Divide by 2 */
#  define SIM_CLKDIV1_OUTDIV2_3       (2 << SIM_CLKDIV1_OUTDIV2_SHIFT)  /* Divide by 3 */
#  define SIM_CLKDIV1_OUTDIV2_4       (3 << SIM_CLKDIV1_OUTDIV2_SHIFT)  /* Divide by 4 */
#  define SIM_CLKDIV1_OUTDIV2_5       (4 << SIM_CLKDIV1_OUTDIV2_SHIFT)  /* Divide by 5 */
#  define SIM_CLKDIV1_OUTDIV2_6       (5 << SIM_CLKDIV1_OUTDIV2_SHIFT)  /* Divide by 6 */
#  define SIM_CLKDIV1_OUTDIV2_7       (6 << SIM_CLKDIV1_OUTDIV2_SHIFT)  /* Divide by 7 */
#  define SIM_CLKDIV1_OUTDIV2_8       (7 << SIM_CLKDIV1_OUTDIV2_SHIFT)  /* Divide by 8 */
#  define SIM_CLKDIV1_OUTDIV2_9       (8 << SIM_CLKDIV1_OUTDIV2_SHIFT)  /* Divide by 9 */
#  define SIM_CLKDIV1_OUTDIV2_10      (9 << SIM_CLKDIV1_OUTDIV2_SHIFT)  /* Divide by 10 */
#  define SIM_CLKDIV1_OUTDIV2_11      (10 << SIM_CLKDIV1_OUTDIV2_SHIFT) /* Divide by 11 */
#  define SIM_CLKDIV1_OUTDIV2_12      (11 << SIM_CLKDIV1_OUTDIV2_SHIFT) /* Divide by 12 */
#  define SIM_CLKDIV1_OUTDIV2_13      (12 << SIM_CLKDIV1_OUTDIV2_SHIFT) /* Divide by 13 */
#  define SIM_CLKDIV1_OUTDIV2_14      (13 << SIM_CLKDIV1_OUTDIV2_SHIFT) /* Divide by 14 */
#  define SIM_CLKDIV1_OUTDIV2_15      (14 << SIM_CLKDIV1_OUTDIV2_SHIFT) /* Divide by 15 */
#  define SIM_CLKDIV1_OUTDIV2_16      (15 << SIM_CLKDIV1_OUTDIV2_SHIFT) /* Divide by 16 */
#define SIM_CLKDIV1_OUTDIV1_SHIFT     (28)      /* Bits 28-31: Clock 1 output divider value */
#define SIM_CLKDIV1_OUTDIV1_MASK      (15 << SIM_CLKDIV1_OUTDIV1_SHIFT)
#  define SIM_CLKDIV1_OUTDIV1(n)      (((n)-1) << SIM_CLKDIV1_OUTDIV1_SHIFT) /* Divide by n, n=1..16 */
#  define SIM_CLKDIV1_OUTDIV1_1       (0 << SIM_CLKDIV1_OUTDIV1_SHIFT)  /* Divide by 1 */
#  define SIM_CLKDIV1_OUTDIV1_2       (1 << SIM_CLKDIV1_OUTDIV1_SHIFT)  /* Divide by 2 */
#  define SIM_CLKDIV1_OUTDIV1_3       (2 << SIM_CLKDIV1_OUTDIV1_SHIFT)  /* Divide by 3 */
#  define SIM_CLKDIV1_OUTDIV1_4       (3 << SIM_CLKDIV1_OUTDIV1_SHIFT)  /* Divide by 4 */
#  define SIM_CLKDIV1_OUTDIV1_5       (4 << SIM_CLKDIV1_OUTDIV1_SHIFT)  /* Divide by 5 */
#  define SIM_CLKDIV1_OUTDIV1_6       (5 << SIM_CLKDIV1_OUTDIV1_SHIFT)  /* Divide by 6 */
#  define SIM_CLKDIV1_OUTDIV1_7       (6 << SIM_CLKDIV1_OUTDIV1_SHIFT)  /* Divide by 7 */
#  define SIM_CLKDIV1_OUTDIV1_8       (7 << SIM_CLKDIV1_OUTDIV1_SHIFT)  /* Divide by 8 */
#  define SIM_CLKDIV1_OUTDIV1_9       (8 << SIM_CLKDIV1_OUTDIV1_SHIFT)  /* Divide by 9 */
#  define SIM_CLKDIV1_OUTDIV1_10      (9 << SIM_CLKDIV1_OUTDIV1_SHIFT)  /* Divide by 10 */
#  define SIM_CLKDIV1_OUTDIV1_11      (10 << SIM_CLKDIV1_OUTDIV1_SHIFT) /* Divide by 11 */
#  define SIM_CLKDIV1_OUTDIV1_12      (11 << SIM_CLKDIV1_OUTDIV1_SHIFT) /* Divide by 12 */
#  define SIM_CLKDIV1_OUTDIV1_13      (12 << SIM_CLKDIV1_OUTDIV1_SHIFT) /* Divide by 13 */
#  define SIM_CLKDIV1_OUTDIV1_14      (13 << SIM_CLKDIV1_OUTDIV1_SHIFT) /* Divide by 14 */
#  define SIM_CLKDIV1_OUTDIV1_15      (14 << SIM_CLKDIV1_OUTDIV1_SHIFT) /* Divide by 15 */
#  define SIM_CLKDIV1_OUTDIV1_16      (15 << SIM_CLKDIV1_OUTDIV1_SHIFT) /* Divide by 16 */

/* System Clock Divider Register 2 */

#define SIM_CLKDIV2_USBFRAC           (1 << 0)  /* Bit 0:  USB clock divider fraction */
#define SIM_CLKDIV2_USBDIV_SHIFT      (1)       /* Bits 1-3: USB clock divider divisor */
#define SIM_CLKDIV2_USBDIV_MASK       (7 << SIM_CLKDIV2_USBDIV_SHIFT)
                                                /* Bits 4-7: Reserved */
#define SIM_CLKDIV2_I2SFRAC_SHIFT     (8)       /* Bits 8-15: I2S clock divider fraction */
#define SIM_CLKDIV2_I2SFRAC_MASK      (0xff << SIM_CLKDIV2_I2SFRAC_SHIFT)
                                                /* Bits 16-19: Reserved */
#define SIM_CLKDIV2_I2SDIV_SHIFT      (20)      /* Bits 20-31: I2S clock divider value */
#define SIM_CLKDIV2_I2SDIV_MASK       (0xfff << SIM_CLKDIV2_I2SDIV_SHIFT)

/* Flash Configuration Register 1 */
                                                /* Bits 0-7: Reserved */
#define SIM_FCFG1_DEPART_SHIFT        (8)       /* Bits 8-11: FlexNVM partition */
#define SIM_FCFG1_DEPART_MASK         (15 << SIM_FCFG1_DEPART_SHIFT)
                                                /* Bits 12-15: Reserved */
#define SIM_FCFG1_EESIZE_SHIFT        (16)      /* Bits 16-19: EEPROM size*/
#define SIM_FCFG1_EESIZE_MASK         (15 << SIM_FCFG1_EESIZE_SHIFT)
#  define SIM_FCFG1_EESIZE_4KB        (2 << SIM_FCFG1_EESIZE_SHIFT)  /* 4 KB */
#  define SIM_FCFG1_EESIZE_2KB        (3 << SIM_FCFG1_EESIZE_SHIFT)  /* 2 KB */
#  define SIM_FCFG1_EESIZE_1KB        (4 << SIM_FCFG1_EESIZE_SHIFT)  /* 1 KB */
#  define SIM_FCFG1_EESIZE_512B       (5 << SIM_FCFG1_EESIZE_SHIFT)  /* 512 Bytes */
#  define SIM_FCFG1_EESIZE_256B       (6 << SIM_FCFG1_EESIZE_SHIFT)  /* 256 Bytes */
#  define SIM_FCFG1_EESIZE_128B       (7 << SIM_FCFG1_EESIZE_SHIFT)  /* 128 Bytes */
#  define SIM_FCFG1_EESIZE_64B        (8 << SIM_FCFG1_EESIZE_SHIFT)  /* 64 Bytes */
#  define SIM_FCFG1_EESIZE_32B        (9 << SIM_FCFG1_EESIZE_SHIFT)  /* 32 Bytes */
#  define SIM_FCFG1_EESIZE_NONE       (15 << SIM_FCFG1_EESIZE_SHIFT) /* 0 Bytes */
                                                /* Bits 20-23: Reserved */
#ifdef KINETIS_K40
#define SIM_FCFG1_PFSIZE_SHIFT        (24)      /* Bits 24-27: Program flash size (K40) */
#define SIM_FCFG1_PFSIZE_MASK         (15 << SIM_FCFG1_PFSIZE_SHIFT)
#  define SIM_FCFG1_PFSIZE_128KB      (7 << SIM_FCFG1_PFSIZE_SHIFT)  /* 128KB program flash, 4KB protection region */
#  define SIM_FCFG1_PFSIZE_256KB      (9 << SIM_FCFG1_PFSIZE_SHIFT)  /* 256KB program flash, 8KB protection region */
#  define SIM_FCFG1_PFSIZE_512KB      (11 << SIM_FCFG1_PFSIZE_SHIFT) /* 512KB program flash, 16KB protection region */
#  define SIM_FCFG1_PFSIZE_512KB2     (15 << SIM_FCFG1_PFSIZE_SHIFT) /* 512KB program flash, 16KB protection region */
#define SIM_FCFG1_NVMSIZE_SHIFT       (28)      /* Bits 28-31: FlexNVM size (K40)*/
#define SIM_FCFG1_NVMSIZE_MASK        (15 << SIM_FCFG1_NVMSIZE_SHIFT)
#  define SIM_FCFG1_NVMSIZE_NONE      (0 << SIM_FCFG1_NVMSIZE_SHIFT)  /*  0KB FlexNVM */
#  define SIM_FCFG1_NVMSIZE_128KB     (7 << SIM_FCFG1_NVMSIZE_SHIFT)  /* 128KB FlexNVM, 16KB protection region */
#  define SIM_FCFG1_NVMSIZE_256KB     (9 << SIM_FCFG1_NVMSIZE_SHIFT)  /* 256KB FlexNVM, 32KB protection region */
#  define SIM_FCFG1_NVMSIZE_256KB2    (15 << SIM_FCFG1_NVMSIZE_SHIFT) /* 256KB FlexNVM, 32KB protection region */
#endif

#ifdef KINETIS_K60
#define SIM_FCFG1_FSIZE_SHIFT         (24)      /* Bits 24-31: Flash size (K60)*/
#define SIM_FCFG1_FSIZE_MASK          (0xff << SIM_FCFG1_FSIZE_SHIFT)
#  define SIM_FCFG1_FSIZE_32KB        (2 << SIM_FCFG1_FSIZE_SHIFT)  /* 32KB program flash, 1KB protection region */
#  define SIM_FCFG1_FSIZE_64KB        (4 << SIM_FCFG1_FSIZE_SHIFT)  /* 64KB program flash, 2KB protection region */
#  define SIM_FCFG1_FSIZE_128KB       (6 << SIM_FCFG1_FSIZE_SHIFT)  /* 128KB program flash, 4KB protection region */
#  define SIM_FCFG1_FSIZE_256KB       (8 << SIM_FCFG1_FSIZE_SHIFT)  /* 256KB program flash, 8KB protection region */
#  define SIM_FCFG1_FSIZE_512KB       (12 << SIM_FCFG1_FSIZE_SHIFT) /* 512KB program flash, 16KB protection region */
#endif

/* Flash Configuration Register 2 */
                                                /* Bits 0-15: Reserved */
#define SIM_FCFG2_MAXADDR1_SHIFT      (16)      /* Bits 16-21: Max address block 1 */
#define SIM_FCFG2_MAXADDR1_MASK       (nn << SIM_FCFG2_MAXADDR1_SHIFT)
                                                /* Bit 22: Reserved */
#define SIM_FCFG2_PFLSH               (1 << 23) /* Bit 23: Program flash */
#define SIM_FCFG2_MAXADDR0_SHIFT      (24)      /* Bits 24-29: Max address block 0 */
#define SIM_FCFG2_MAXADDR0_MASK       (nn << SIM_FCFG2_MAXADDR0_SHIFT)
                                                /* Bit 30: Reserved */
#define SIM_FCFG2_SWAPPFLSH           (1 << 31) /* Bit 31: Swap program flash */

/* Unique Identification Register High. 32-bit Unique Identification. */
/* Unique Identification Register Mid-High. 32-bit Unique Identification. */
/* Unique Identification Register Mid Low. 32-bit Unique Identification. */
/* Unique Identification Register Low. 32-bit Unique Identification. */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_KINETIS_SIM_H */
