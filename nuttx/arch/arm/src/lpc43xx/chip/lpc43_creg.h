/************************************************************************************
 * arch/arm/src/lpc43xx/chip/lpc43_creg.h
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

#ifndef __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_CREG_H
#define __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_CREG_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Register Offsets *****************************************************************/

#define LPC43_CREG0_OFFSET            0x0004 /* Chip configuration register 0 */
#define LPC43_CREG_M4MEMMAP_OFFSET    0x0100 /* ARM Cortex-M4 memory mapping */
#define LPC43_CREG1_OFFSET            0x0108 /* Chip configuration register 1 */
#define LPC43_CREG2_OFFSET            0x010c /* Chip configuration register 2 */
#define LPC43_CREG3_OFFSET            0x0110 /* Chip configuration register 3 */
#define LPC43_CREG4_OFFSET            0x0114 /* Chip configuration register 4 */
#define LPC43_CREG5_OFFSET            0x0118 /* Chip configuration register 5 */
#define LPC43_CREG_DMAMUX_OFFSET      0x011c /* DMA mux control */
#define LPC43_CREG_FLASHCFGA_OFFSET   0x0120 /* Flash accelerator bank A configuration */
#define LPC43_CREG_FLASHCFGB_OFFSET   0x0124 /* Flash accelerator bankd B configuration */
#define LPC43_CREG_ETBCFG_OFFSET      0x0128 /* ETB RAM configuration */
#define LPC43_CREG6_OFFSET            0x012c /* Chip configuration register 6 */
#define LPC43_CREG_M4TXEVENT_OFFSET   0x0130 /* Cortex-M4 TXEV event clear 0 */
#define LPC43_CREG_CHIPID_OFFSET      0x0200 /* Part ID */
#define LPC43_CREG_M0TXEVENT_OFFSET   0x0400 /* Cortex-M0 TXEV event clear */
#define LPC43_CREG_M0APPMEMMAP_OFFSET 0x0404 /* ARM Cortex-M0 memory mapping */
#define LPC43_CREG_USB0FLADJ_OFFSET   0x0500 /* USB0 frame length adjust */
#define LPC43_CREG_USB1FLADJ_OFFSET   0x0600 /* USB1 frame length adjust */

/* Register Addresses ***************************************************************/

#define LPC43_CREG0                   (LPC43_CREG_BASE+LPC43_CREG0_OFFSET)
#define LPC43_CREG_M4MEMMAP           (LPC43_CREG_BASE+LPC43_CREG_M4MEMMAP_OFFSET)
#define LPC43_CREG1                   (LPC43_CREG_BASE+LPC43_CREG1_OFFSET)
#define LPC43_CREG2                   (LPC43_CREG_BASE+LPC43_CREG2_OFFSET)
#define LPC43_CREG3                   (LPC43_CREG_BASE+LPC43_CREG3_OFFSET)
#define LPC43_CREG4                   (LPC43_CREG_BASE+LPC43_CREG4_OFFSET)
#define LPC43_CREG5                   (LPC43_CREG_BASE+LPC43_CREG5_OFFSET)
#define LPC43_CREG_DMAMUX             (LPC43_CREG_BASE+LPC43_CREG_DMAMUX_OFFSET)
#define LPC43_CREG_FLASHCFGA          (LPC43_CREG_BASE+LPC43_CREG_FLASHCFGA_OFFSET)
#define LPC43_CREG_FLASHCFGB          (LPC43_CREG_BASE+LPC43_CREG_FLASHCFGB_OFFSET)
#define LPC43_CREG_ETBCFG             (LPC43_CREG_BASE+LPC43_CREG_ETBCFG_OFFSET)
#define LPC43_CREG6                   (LPC43_CREG_BASE+LPC43_CREG6_OFFSET)
#define LPC43_CREG_M4TXEVENT          (LPC43_CREG_BASE+LPC43_CREG_M4TXEVENT_OFFSET)
#define LPC43_CREG_CHIPID             (LPC43_CREG_BASE+LPC43_CREG_CHIPID_OFFSET)
#define LPC43_CREG_M0TXEVENT          (LPC43_CREG_BASE+LPC43_CREG_M0TXEVENT_OFFSET)
#define LPC43_CREG_M0APPMEMMAP        (LPC43_CREG_BASE+LPC43_CREG_M0APPMEMMAP_OFFSET)
#define LPC43_CREG_USB0FLADJ          (LPC43_CREG_BASE+LPC43_CREG_USB0FLADJ_OFFSET)
#define LPC43_CREG_USB1FLADJ          (LPC43_CREG_BASE+LPC43_CREG_USB1FLADJ_OFFSET)

/* Register Bit Definitions *********************************************************/

/* Chip configuration register 0 */

#define CREG0_EN1KHZ                  (1 << 0)  /* Bit 0:  Enable 1 kHz output */
#define CREG0_EN32KHZ                 (1 << 1)  /* Bit 1:  Enable 32 kHz output */
#define CREG0_RESET32KHZ              (1 << 2)  /* Bit 2:  32 kHz oscillator reset */
#define CREG0_PD32KHZ                 (1 << 3)  /* Bit 3:  32 kHz power control */
                                                /* Bit 4:  Reserved */
#define CREG0_USB0PHY                 (1 << 5)  /* Bit 5:  USB0 PHY power control */
#define CREG0_ALARMCTRL_SHIFT         (6)       /* Bits 6-7: RTC_ALARM pin output control 0 R/W */
#define CREG0_ALARMCTRL_MASK          (3 << CREG0_ALARMCTRL_SHIFT)
#  define CREG0_ALARMCTRL_RTC         (0 << CREG0_ALARMCTRL_SHIFT) /* RTC alarm */
#  define CREG0_ALARMCTRL_EVENT       (1 << CREG0_ALARMCTRL_SHIFT) /* Event router event */
#  define CREG0_ALARMCTRL_INACTIVE    (3 << CREG0_ALARMCTRL_SHIFT) /* Inactive */
#define CREG0_BODLVL1_SHIFT           (8)       /* Bits 8-9: BOD trip level to generate an interrupt */
#define CREG0_BODLVL1_MASK            (3 << CREG0_BODLVL1_SHIFT)
#  define CREG0_BODLVL1_LEVEL0        (0 << CREG0_BODLVL1_SHIFT) /* Level 0 interrupt */
#  define CREG0_BODLVL1_LEVEL1        (1 << CREG0_BODLVL1_SHIFT) /* Level 1 interrupt */
#  define CREG0_BODLVL1_LEVEL2        (2 << CREG0_BODLVL1_SHIFT) /* Level 2 interrupt */
#  define CREG0_BODLVL1_LEVEL3        (3 << CREG0_BODLVL1_SHIFT) /* Level 3 interrupt */
#define CREG0_BODLVL2_SHIFT           (10)      /* Bits 10-11: BOD trip level to generate a reset */
#define CREG0_BODLVL2_MASK            (3 << CREG0_BODLVL2_SHIFT)
#  define CREG0_BODLVL2_LEVEL0        (0 << CREG0_BODLVL2_SHIFT) /* Level 0 reset */
#  define CREG0_BODLVL2_LEVEL1        (1 << CREG0_BODLVL2_SHIFT) /* Level 1 reset */
#  define CREG0_BODLVL2_LEVEL2        (2 << CREG0_BODLVL2_SHIFT) /* Level 2 reset */
#  define CREG0_BODLVL2_LEVEL3        (3 << CREG0_BODLVL2_SHIFT) /* Level 3 reset */
#define CREG0_SAMPLECTRL_SHIFT        (12)      /* Bits 12-13:  SAMPLE pin input/output control */
#define CREG0_SAMPLECTRL_MASK         (3 << CREG0_SAMPLECTRL_SHIFT)
#  define CREG0_SAMPLECTRL_MONITOR    (1 << CREG0_SAMPLECTRL_SHIFT) /* Output from event monitor/recorder */
#  define CREG0_SAMPLECTRL_EVNTRTR    (2 << CREG0_SAMPLECTRL_SHIFT) /* Output from the event router */
#define CREG0_WAKEUP0CTRL_SHIFT       (14)      /* Bits 14-15: WAKEUP0 pin input/output control */
#define CREG0_WAKEUP0CTRL_MASK        (3 << CREG0_WAKEUP0CTRL_SHIFT)
#  define CREG0_WAKEUP0CTRL_EVNTIN    (0 << CREG0_WAKEUP0CTRL_SHIFT) /* Input to the event router */
#  define CREG0_WAKEUP0CTRL_EVNTOUT   (1 << CREG0_WAKEUP0CTRL_SHIFT) /* Output from the event router */
#  define CREG0_WAKEUP0CTRL_EVNTIN2   (3 << CREG0_WAKEUP0CTRL_SHIFT) /* Input to the event router */
#define CREG0_WAKEUP1CTRL_SHIFT       (16)      /* Bits 16-17: WAKEUP1 pin input/output control */
#define CREG0_WAKEUP1CTRL_MASK        (3 << CREG0_WAKEUP1CTRL_SHIFT)
#  define CREG0_WAKEUP1CTRL_EVNTIN    (0 << CREG0_WAKEUP1CTRL_SHIFT) /* Input to the event router */
#  define CREG0_WAKEUP1CTRL_EVNTOUT   (1 << CREG0_WAKEUP1CTRL_SHIFT) /* Output from the event router */
#  define CREG0_WAKEUP1CTRL_EVNTIN2   (3 << CREG0_WAKEUP1CTRL_SHIFT) /* Input to the event router */
                                                /* Bits 18-31:  Reserved */
/* ARM Cortex-M4 memory mapping */
                                                /* Bits 0-11:  Reserved */
#define CREG_M4MEMMAP_SHIFT           (12)      /* Bits 12-31: M4MAP Shadow address */
#define CREG_M4MEMMAP_MASK            (0x000fffff << CREG_M4MEMMAP_SHIFT)

/* Chip configuration register 1-4.  Bit definitions not provided in the user manual */

/* Chip configuration register 5 */
                                                /* Bits 0-5:  Reserved */
#define CREG5_M4TAPSEL                (1 << 6)  /* Bit 6:  JTAG debug select for M4 core */
                                                /* Bits 7-8:  Reserved */
#define CREG5_M0APPTAPSEL             (1 << 9)  /* Bit 9:  JTAG debug select for M0 co-processor */
                                                /* Bits 10-31:  Reserved */
/* DMA mux control */

#define CREG_DMAMUX_PER0_SHIFT        (0)       /* Bits 0-1: Selection for DMA peripheral 0 */
#define CREG_DMAMUX_PER0_MASK         (3 << CREG_DMAMUX_PER0_SHIFT)
#  define CREG_DMAMUX_PER0 SPIFI      (0 << CREG_DMAMUX_PER0_SHIFT) /* SPIFI */
#  define CREG_DMAMUX_PER0_SCTM2      (1 << CREG_DMAMUX_PER0_SHIFT) /* SCT match 2 */
#  define CREG_DMAMUX_PER0_T3M1       (3 << CREG_DMAMUX_PER0_SHIFT) /* T3 match 1 */
#define CREG_DMAMUX_PER1_SHIFT        (2)       /* Bits 2-3: Selection for DMA peripheral 1 */
#define CREG_DMAMUX_PER1_MASK         (3 << CREG_DMAMUX_PER1_SHIFT)
#  define CREG_DMAMUX_PER1_T0M0       (0 << CREG_DMAMUX_PER1_SHIFT) /* Timer 0 match 0 */
#  define CREG_DMAMUX_PER1_U0TX       (1 << CREG_DMAMUX_PER1_SHIFT) /* USART0 transmit */
#define CREG_DMAMUX_PER2_SHIFT        (4)       /* Bits 4-5: Selection for DMA peripheral 2 */
#define CREG_DMAMUX_PER2_MASK         (3 << CREG_DMAMUX_PER2_SHIFT)
#  define CREG_DMAMUX_PER2_T0M1       (0 << CREG_DMAMUX_PER2_SHIFT) /* Timer 0 match 1 */
#  define CREG_DMAMUX_PER2_U0RX       (1 << CREG_DMAMUX_PER2_SHIFT) /* USART0 receive */
#define CREG_DMAMUX_PER3_SHIFT        (6)       /* Bits 6-7: Selection for DMA peripheral 3 */
#define CREG_DMAMUX_PER3_MASK         (3 << CREG_DMAMUX_PER3_SHIFT)
#  define CREG_DMAMUX_PER3_T1M0       (0 << CREG_DMAMUX_PER3_SHIFT) /* Timer 1 match 0 */
#  define CREG_DMAMUX_PER3_U1TX       (1 << CREG_DMAMUX_PER3_SHIFT) /* UART1 transmit */
#  define CREG_DMAMUX_PER3_I2S1D1     (2 << CREG_DMAMUX_PER3_SHIFT) /* I2S1 DMA request 1 */
#  define CREG_DMAMUX_PER3_SSP1TX     (3 << CREG_DMAMUX_PER3_SHIFT) /* SSP1 transmit */
#define CREG_DMAMUX_PER4_SHIFT        (8)       /* Bits 8-9: Selection for DMA peripheral 4 */
#define CREG_DMAMUX_PER4_MASK         (3 << CREG_DMAMUX_PER4_SHIFT)
#  define CREG_DMAMUX_PER4_T1M1       (0 << CREG_DMAMUX_PER4_SHIFT) /* Timer 1 match 1 */
#  define CREG_DMAMUX_PER4_U1RX       (1 << CREG_DMAMUX_PER4_SHIFT) /* UART1 receive */
#  define CREG_DMAMUX_PER4_I2S1D2     (2 << CREG_DMAMUX_PER4_SHIFT) /* I2S1 DMA request 2 */
#  define CREG_DMAMUX_PER4_SSP1RX     (3 << CREG_DMAMUX_PER4_SHIFT) /* SSP1 receive */
#define CREG_DMAMUX_PER5_SHIFT        (10)      /* Bits 10-11: Selection for DMA peripheral 5 */
#define CREG_DMAMUX_PER5_MASK         (3 << CREG_DMAMUX_PER5_SHIFT)
#  define CREG_DMAMUX_PER5_T2M0       (0 << CREG_DMAMUX_PER5_SHIFT) /* Timer 2 match 0 */
#  define CREG_DMAMUX_PER5_U2TX       (1 << CREG_DMAMUX_PER5_SHIFT) /* USART2 transmit */
#  define CREG_DMAMUX_PER5_SSP1TX     (2 << CREG_DMAMUX_PER5_SHIFT) /* SSP1 transmit */
#define CREG_DMAMUX_PER6_SHIFT        (12)      /* Bits 12-13: Selection for DMA peripheral 6 */
#define CREG_DMAMUX_PER6_MASK         (3 << CREG_DMAMUX_PER6_SHIFT)
#  define CREG_DMAMUX_PER6_T2M1       (0 << CREG_DMAMUX_PER6_SHIFT) /* Timer 2 match 1 */
#  define CREG_DMAMUX_PER6_U2RX       (1 << CREG_DMAMUX_PER6_SHIFT) /* USART2 receive */
#  define CREG_DMAMUX_PER6_SSP1RX     (2 << CREG_DMAMUX_PER6_SHIFT) /* SSP1 receive */
#define CREG_DMAMUX_PER7_SHIFT        (14)      /* Bits 14-15: Selection for DMA peripheral 7 */
#define CREG_DMAMUX_PER7_MASK         (3 << CREG_DMAMUX_PER7_SHIFT)
#  define CREG_DMAMUX_PER7_T3M1       (0 << CREG_DMAMUX_PER7_SHIFT) /* Timer 3 match l */
#  define CREG_DMAMUX_PER7_U3TX       (1 << CREG_DMAMUX_PER7_SHIFT) /* USART3 transmit */
#  define CREG_DMAMUX_PER7_SCTM0      (2 << CREG_DMAMUX_PER7_SHIFT) /* SCT match output 0 */
#define CREG_DMAMUX_PER8_SHIFT        (16)      /* Bits 16-17: Selection for DMA peripheral 8 */
#define CREG_DMAMUX_PER8_MASK         (3 << CREG_DMAMUX_PER8_SHIFT)
#  define CREG_DMAMUX_PER8_T3M1       (0 << CREG_DMAMUX_PER8_SHIFT) /* Timer 3 match 1 */
#  define CREG_DMAMUX_PER8_U3RX       (1 << CREG_DMAMUX_PER8_SHIFT) /* USART3 receive */
#  define CREG_DMAMUX_PER8_SCTM1      (2 << CREG_DMAMUX_PER8_SHIFT) /* SCT match output 1 */
#define CREG_DMAMUX_PER9_SHIFT        (18)      /* Bits 18-19: Selection for DMA peripheral 9 */
#define CREG_DMAMUX_PER9_MASK         (3 << CREG_DMAMUX_PER9_SHIFT)
#  define CREG_DMAMUX_PER9_SSP0RX     (0 << CREG_DMAMUX_PER9_SHIFT) /* SSP0 receive */
#  define CREG_DMAMUX_PER9_I2S0D1     (1 << CREG_DMAMUX_PER9_SHIFT) /* I2S0 DMA request 1 */
#  define CREG_DMAMUX_PER9_SCTM1      (2 << CREG_DMAMUX_PER9_SHIFT) /* SCT match output 1 */
#define CREG_DMAMUX_PER10_SHIFT       (20)      /* Bits 20-21: Selection for DMA peripheral 10 */
#define CREG_DMAMUX_PER10_MASK        (3 << CREG_DMAMUX_PER10_SHIFT)
#  define CREG_DMAMUX_PER10_SSP0TX    (0 << CREG_DMAMUX_PER10_SHIFT) /* SSP0 transmit */
#  define CREG_DMAMUX_PER10_I2S0D2    (1 << CREG_DMAMUX_PER10_SHIFT) /* I2S0 DMA request 2 */
#  define CREG_DMAMUX_PER10_SCTM0     (2 << CREG_DMAMUX_PER10_SHIFT) /* SCT match output 0 */
#define CREG_DMAMUX_PER11_SHIFT       (22)      /* Bits 22-23: Selection for DMA peripheral 11 */
#define CREG_DMAMUX_PER11_MASK        (3 << CREG_DMAMUX_PER11_SHIFT)
#  define CREG_DMAMUX_PER11_SSP1RX    (0 << CREG_DMAMUX_PER11_SHIFT) /* SSP1 receive */
#  define CREG_DMAMUX_PER11_U0TX      (2 << CREG_DMAMUX_PER11_SHIFT) /* USART0 transmit */
#define CREG_DMAMUX_PER12_SHIFT       (24)      /* Bits 24-25: Selection for DMA peripheral 12 */
#define CREG_DMAMUX_PER12_MASK        (3 << CREG_DMAMUX_PER12_SHIFT)
#  define CREG_DMAMUX_PER12_SSP1TX    (0 << CREG_DMAMUX_PER12_SHIFT) /* SSP1 transmit */
#  define CREG_DMAMUX_PER12_U0RX      (2 << CREG_DMAMUX_PER12_SHIFT) /* USART0 receive */
#define CREG_DMAMUX_PER13_SHIFT       (26)      /* Bits 26-27: Selection for DMA peripheral 13 */
#define CREG_DMAMUX_PER13_MASK        (3 << CREG_DMAMUX_PER13_SHIFT)
#  define CREG_DMAMUX_PER13_ADC0      (0 << CREG_DMAMUX_PER13_SHIFT) /* ADC0 */
#  define CREG_DMAMUX_PER13_SSP1RX    (2 << CREG_DMAMUX_PER13_SHIFT) /* SSP1 receive */
#  define CREG_DMAMUX_PER13_U3RX      (3 << CREG_DMAMUX_PER13_SHIFT) /* USART3 receive */
#define CREG_DMAMUX_PER14_SHIFT       (28)      /* Bits 28-29: Selection for DMA peripheral 14 */
#define CREG_DMAMUX_PER14_MASK        (3 << CREG_DMAMUX_PER12_SHIFT)
#  define CREG_DMAMUX_PER14_ADC1      (0 << CREG_DMAMUX_PER14_SHIFT) /* ADC1 */
#  define CREG_DMAMUX_PER14_SSP1TX    (2 << CREG_DMAMUX_PER14_SHIFT) /* SSP1 transmit */
#  define CREG_DMAMUX_PER14_U3TX      (3 << CREG_DMAMUX_PER14_SHIFT) /* USART3 transmit */
#define CREG_DMAMUX_PER15_SHIFT       (30)      /* Bits 30-31: Selection for DMA peripheral 15 */
#define CREG_DMAMUX_PER15_MASK        (3 << CREG_DMAMUX_PER15_SHIFT)
#  define CREG_DMAMUX_PER15_DAC       (0 << CREG_DMAMUX_PER15_SHIFT) /* DAC */
#  define CREG_DMAMUX_PER15_SCTM3     (1 << CREG_DMAMUX_PER15_SHIFT) /* SCT match output 3 */
#  define CREG_DMAMUX_PER15_T3M0      (3 << CREG_DMAMUX_PER15_SHIFT) /* Timer 3 match 0 */

/* Flash accelerator bank A/B configuration */
                                                /* Bits 0-11:  Reserved */
#define CREG_FLASHCFG_FLASHTIM_SHIFT  (12)      /* Bits 12-15: Flash access time */
#define CREG_FLASHCFG_FLASHTIM_MASK   (15 << CREG_FLASHCFG_FLASHTIM_SHIFT)
#  define CREG_FLASHCFG_FLASHTIM(n)   (((n)-1) << CREG_FLASHCFG_FLASHTIM_SHIFT) /* n BASE_M4_CLK clocks, n=1..10 */
                                                /* Bits 16-31:  Reserved */
#define CREG_FLASHCFG_POW             (1 << 31) /* Bit 31:  Flash bank A power control */

/* ETB RAM configuration */

#define CREG_ETBCFG                   (1 << 0)  /* Bit 0: Select SRAM interface */
                                                /* Bits 1-31:  Reserved */
/* Chip configuration register 6 */

#define CREG6_ETHMODE_SHIFT           (0)       /* Bits 0-2: Selects the Ethernet mode */
#define CREG6_ETHMODE_MASK            (7 << CREG6_ETHMODE_SHIFT)
#  define CREG6_ETHMODE_MII           (0 << CREG6_ETHMODE_SHIFT)
#  define CREG6_ETHMODE_RMII          (4 << CREG6_ETHMODE_SHIFT)
                                                /* Bit 3:  Reserved */
#define CREG6_CTOUTCTRL               (1 << 4)  /* Bit 4:  Selects the functionality of the SCT outputs */
                                                /* Bits 5-11:  Reserved */
#define CREG6_I2S0_TXSCK              (1 << 12) /* Bit 12:  I2S0_TX_SCK input select */
#define CREG6_I2S0_RXSCK              (1 << 13) /* Bit 13:  I2S0_RX_SCK input select */
#define CREG6_I2S1_TXSCK              (1 << 14) /* Bit 14:  I2S1_TX_SCK input select */
#define CREG6_I2S1_RXSCK              (1 << 15) /* Bit 15:  I2S1_RX_SCK input select */
#define CREG6_EMC_CLK                 (1 << 16) /* Bit 16:  EMC_CLK divided clock select */
                                                /* Bits 17-31:  Reserved */
/* Cortex-M4 TXEV event clear 0 */

#define CREG_M4TXEVENT                (1 << 0)  /* Bit 0: Cortex-M4 TXEV event */
                                                /* Bits 1-31:  Reserved */
/* Part ID (32-bit ID) */

#define CREG_CHIPID_FLASHLESS1        0x5906002b /* LPC4350/30/20/10 */
#define CREG_CHIPID_FLASHLESS2        0x6906002b /* LPC4350/30/20/10 */
#define CREG_CHIPID_FLASHPARTS        0x4906002b /* LPC4357/53 */

/* Cortex-M0 TXEV event clear */

#define CREG_M0TXEVENT                (1 << 0)  /* Bit 0: Cortex-M0 TXEV event */
                                                /* Bits 1-31:  Reserved */
/* ARM Cortex-M0 memory mapping */
                                                /* Bits 0-11:  Reserved */
#define CREG_M0APPMEMMAP_SHIFT        (12)      /* Bits 12-31: M4MAP Shadow address */
#define CREG_M0APPMEMMAP_MASK         (0x000fffff << CREG_M0APPMEMMAP_SHIFT)

/* USB0/1 frame length adjust */

#define CREG_USBFLADJ_SHIFT           (0)       /* Bits 0-5: FLTV Frame length timing value */
#define CREG_USBFLADJ_MASK            (0x3f << CREG_USBFLADJ_SHIFT)
#  define CREG_USBFLADJ(n)            ((((n)-59488) >> 4) << CREG_USBFLADJ_SHIFT)
                                                /* Bits 6-31:  Reserved */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_CREG_H */
