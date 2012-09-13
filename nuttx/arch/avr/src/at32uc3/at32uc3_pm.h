/************************************************************************************
 * arch/avr/src/at32uc3/at32uc3_pm.h
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

#ifndef __ARCH_AVR_SRC_AT32UC3_AT32UC3_PM_H
#define __ARCH_AVR_SRC_AT32UC3_AT32UC3_PM_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/

#define AVR32_PM_MCCTRL_OFFSET     0x0000 /* Main Clock Control Register */
#define AVR32_PM_CKSEL_OFFSET      0x0004 /* Clock Select Register */
#define AVR32_PM_CPUMASK_OFFSET    0x0008 /* CPU Mask Register */
#define AVR32_PM_HSBMASK_OFFSET    0x000c /* HSB Mask Register */
#define AVR32_PM_PBAMASK_OFFSET    0x0010 /* PBA Mask Register */
#define AVR32_PM_PBBMASK_OFFSET    0x0014 /* PBB Mask Register */
#define AVR32_PM_PLL0_OFFSET       0x0020 /* PLL0 Control Register */
#define AVR32_PM_PLL1_OFFSET       0x0024 /* PLL1 Control Register */
#define AVR32_PM_OSCCTRL0_OFFSET   0x0028 /* Oscillator 0 Control Register */
#define AVR32_PM_OSCCTRL1_OFFSET   0x002c /* Oscillator 1 Control Register */
#define AVR32_PM_OSCCTRL32_OFFSET  0x0030 /* Oscillator 32 Control Register */
#define AVR32_PM_IER_OFFSET        0x0040 /* Interrupt Enable Register */
#define AVR32_PM_IDR_OFFSET        0x0044 /* Interrupt Disable Register */
#define AVR32_PM_IMR_OFFSET        0x0048 /* Interrupt Mask Register */
#define AVR32_PM_ISR_OFFSET        0x004c /* Interrupt Status Register */
#define AVR32_PM_ICR_OFFSET        0x0050 /* Interrupt Clear Register */
#define AVR32_PM_POSCSR_OFFSET     0x0054 /* Power and Oscillators Status Register */
#define AVR32_PM_GCCTRL_OFFSET(n)  (0x0060+((n)<<2)) /* 0x0060-0x070 Generic Clock Control Register */
#define AVR32_PM_RCCR_OFFSET       0x00c0 /* RC Oscillator Calibration Register */
#define AVR32_PM_BGCR_OFFSET       0x00c4 /* Bandgap Calibration Register */
#define AVR32_PM_VREGCR_OFFSET     0x00c8 /* Linear Regulator Calibration Register */
#define AVR32_PM_BOD_OFFSET        0x00d0 /* BOD Level Register BOD Read/Write */
#define AVR32_PM_RCAUSE_OFFSET     0x0140 /* Reset Cause Register */
#define AVR32_PM_AWEN_OFFSET       0x0144 /* Asynchronous Wake Up Enable Register */
#define AVR32_PM_GPLP0_OFFSET      0x0200 /* General Purpose Low-Power Register 0 */
#define AVR32_PM_GPLP1_OFFSET      0x0204 /* General Purpose Low-Power Register 1 */

/* Register Addresses ***************************************************************/

#define AVR32_PM_MCCTRL            (AVR32_PM_BASE+AVR32_PM_MCCTRL_OFFSET)
#define AVR32_PM_CKSEL             (AVR32_PM_BASE+AVR32_PM_CKSEL_OFFSET)
#define AVR32_PM_CPUMASK           (AVR32_PM_BASE+AVR32_PM_CPUMASK_OFFSET)
#define AVR32_PM_HSBMASK           (AVR32_PM_BASE+AVR32_PM_HSBMASK_OFFSET)
#define AVR32_PM_PBAMASK           (AVR32_PM_BASE+AVR32_PM_PBAMASK_OFFSET)
#define AVR32_PM_PBBMASK           (AVR32_PM_BASE+AVR32_PM_PBBMASK_OFFSET)
#define AVR32_PM_PLL0              (AVR32_PM_BASE+AVR32_PM_PLL0_OFFSET)
#define AVR32_PM_PLL1              (AVR32_PM_BASE+AVR32_PM_PLL1_OFFSET)
#define AVR32_PM_OSCCTRL0          (AVR32_PM_BASE+AVR32_PM_OSCCTRL0_OFFSET)
#define AVR32_PM_OSCCTRL1          (AVR32_PM_BASE+AVR32_PM_OSCCTRL1_OFFSET)
#define AVR32_PM_OSCCTRL32         (AVR32_PM_BASE+AVR32_PM_OSCCTRL32_OFFSET)
#define AVR32_PM_IER               (AVR32_PM_BASE+AVR32_PM_IER_OFFSET)
#define AVR32_PM_IDR               (AVR32_PM_BASE+AVR32_PM_IDR_OFFSET)
#define AVR32_PM_IMR               (AVR32_PM_BASE+AVR32_PM_IMR_OFFSET)
#define AVR32_PM_ISR               (AVR32_PM_BASE+AVR32_PM_ISR_OFFSET)
#define AVR32_PM_ICR               (AVR32_PM_BASE+AVR32_PM_ICR_OFFSET)
#define AVR32_PM_POSCSR            (AVR32_PM_BASE+AVR32_PM_POSCSR_OFFSET)
#define AVR32_PM_GCCTRL(n)         (AVR32_PM_BASE+AVR32_PM_GCCTRL_OFFSET(n))
#define AVR32_PM_RCCR              (AVR32_PM_BASE+AVR32_PM_RCCR_OFFSET)
#define AVR32_PM_BGCR              (AVR32_PM_BASE+AVR32_PM_BGCR_OFFSET)
#define AVR32_PM_VREGCR            (AVR32_PM_BASE+AVR32_PM_VREGCR_OFFSET)
#define AVR32_PM_BOD               (AVR32_PM_BASE+AVR32_PM_BOD_OFFSET)
#define AVR32_PM_RCAUSE            (AVR32_PM_BASE+AVR32_PM_RCAUSE_OFFSET)
#define AVR32_PM_AWEN              (AVR32_PM_BASE+AVR32_PM_AWEN_OFFSET)
#define AVR32_PM_GPLP0             (AVR32_PM_BASE+AVR32_PM_GPLP0_OFFSET)
#define AVR32_PM_GPLP1             (AVR32_PM_BASE+AVR32_PM_GPLP1_OFFSET)

/* Register Bit-field Definitions ***************************************************/

/* Main Clock Control Register Bit-field Definitions */

#define PM_MCCTRL_MCSEL_SHIFT      (0)       /* Bits 0-1: Main Clock Select */
#define PM_MCCTRL_MCSEL_MASK       (3 << PM_MCCTRL_MCSEL_SHIFT)
#  define PM_MCCTRL_MCSEL_SLOW     (0 << PM_MCCTRL_MCSEL_SHIFT) /* slow clock is source */
#  define PM_MCCTRL_MCSEL_OSC0     (1 << PM_MCCTRL_MCSEL_SHIFT) /* Oscillator 0 is source */
#  define PM_MCCTRL_MCSEL_PLL0     (2 << PM_MCCTRL_MCSEL_SHIFT) /* PLL0 is source */
#define PM_MCCTRL_OSC0EN           (1 << 2)  /* Bit 2:  Oscillator 0 Enable */
#define PM_MCCTRL_OSC1EN           (1 << 3)  /* Bit 3:  Oscillator 1 Enable */

/* Clock Select Register Bit-field Definitions */

#define PM_CKSEL_CPUSEL_SHIFT      (0)       /* Bits 0-2: CPU Clock Select */
#define PM_CKSEL_CPUSEL_MASK       (7 << PM_CKSEL_CPUSEL_SHIFT)
#define PM_CKSEL_CPUDIV            (1 << 7)  /* Bit 7:  CPU Division */
#define PM_CKSEL_HSBSEL_SHIFT      (8)       /* Bits 8-10: HSB Clock Select */
#define PM_CKSEL_HSBSEL_MASK       (7 << PM_CKSEL_HSBSEL_SHIFT)
#define PM_CKSEL_HSBDIV            (1 << 15) /* Bit 15: HSB Division */
#define PM_CKSEL_PBASEL_SHIFT      (16)      /* Bits 16-28: BA Clock Select */
#define PM_CKSEL_PBASEL_MASK       (7 << PM_CKSEL_PBASEL_SHIFT)
#define PM_CKSEL_PBADIV            (1 << 23) /* Bit 23: PBA Division */
#define PM_CKSEL_PBBSEL_SHIFT      (24)      /* Bits 24-26: PBB Clock Select */
#define PM_CKSEL_PBBSEL_MASK       (7 << PM_CKSEL_PBBSEL_SHIFT)
#define PM_CKSEL_PBBDIV            (1 << 31) /* Bit 31: PBB Division */

/* CPU Mask Register Bit-field Definitions */

#define PM_CPUMASK_OCD             (1 << 1)  /* Bit 1:  OCD */

/* HSB Mask Register Bit-field Definitions */

#define PM_HSBMASK_FLASHC          (1 << 0)  /* Bit 0:  FLASHC */
#define PM_HSBMASK_PBA             (1 << 1)  /* Bit 1:  PBA bridge */
#define PM_HSBMASK_PBB             (1 << 2)  /* Bit 2:  PBB bridge */
#define PM_HSBMASK_USBB            (1 << 3)  /* Bit 3:  USBB */
#define PM_HSBMASK_PDCA            (1 << 4)  /* Bit 4:  PDCA */

/* PBA Mask Register Bit-field Definitions */

#define PM_PBAMASK_INTC            (1 << 0)  /* Bit 0:  INTC */
#define PM_PBAMASK_GPIO            (1 << 1)  /* Bit 1:  GPIO */
#define PM_PBAMASK_PDCA            (1 << 2)  /* Bit 2:  PDCA */
#define PM_PBAMASK_PMRTCEIC        (1 << 3)  /* Bit 3:  PM/RTC/EIC */
#define PM_PBAMASK_ADC             (1 << 4)  /* Bit 4:  ADC */
#define PM_PBAMASK_SPI             (1 << 5)  /* Bit 5:  SPI */
#define PM_PBAMASK_TWI             (1 << 6)  /* Bit 6:  TWI */
#define PM_PBAMASK_USART0          (1 << 7)  /* Bit 7:  USART0 */
#define PM_PBAMASK_USART1          (1 << 8)  /* Bit 8:  USART1 */
#define PM_PBAMASK_USART2          (1 << 9)  /* Bit 9:  USART2 */
#define PM_PBAMASK_PWM             (1 << 10) /* Bit 10: PWM */
#define PM_PBAMASK_SSC             (1 << 11) /* Bit 11: SSC */
#define PM_PBAMASK_TC              (1 << 12) /* Bit 12: TC */
#define PM_PBAMASK_ABDAC           (1 << 13) /* Bit 13: ABDAC */

/* PBB Mask Register Bit-field Definitions */

#define PM_PBBMASK_HMATRIX         (1 << 0)  /* Bit 0:  HMATRIX */
#define PM_PBBMASK_USBB            (1 << 2)  /* Bit 2:  USBB */
#define PM_PBBMASK_FLASHC          (1 << 3)  /* Bit 3:  FLASHC */

/* PLL0/1 Control Register Bit-field Definitions */

#define PM_PLL_PLLEN               (1 << 0)  /* Bit 0:  PLL Enable */
#define PM_PLL_PLLOSC              (1 << 1)  /* Bit 1:  PLL Oscillator Select */
#define PM_PLL_PLLOPT_SHIFT        (2)       /* Bits 2-3: PLL Option */
#define PM_PLL_PLLOPT_MASK         (7 << PM_PLL_PLLOPT_SHIFT)
#  define PM_PLL_PLLOPT_VCO        (1 << PM_PLL_PLLOPT_SHIFT) /* Select the VCO frequency range */
#  define PM_PLL_PLLOPT_XTRADIV    (2 << PM_PLL_PLLOPT_SHIFT) /* Enable the extra output divider */
#  define PM_PLL_PLLOPT_WBWDIS     (4 << PM_PLL_PLLOPT_SHIFT) /* Disable the Wide-Bandwidth mode */
#define PM_PLL_PLLDIV_SHIFT        (8)       /* Bits 8-11: PLL Division Factor */
#define PM_PLL_PLLDIV_MASK         (15 << PM_PLL_PLLDIV_SHIFT)
#define PM_PLL_PLLMUL_SHIFT        (16)      /* Bits 16-19: PLL Multiply Factor */
#define PM_PLL_PLLMUL_MASK         (15 << PM_PLL_PLLMUL_SHIFT)
#define PM_PLL_PLLCOUNT_SHIFT      (24)      /* Bits 24-29: PLL Count */
#define PM_PLL_PLLCOUNT_MASK       (0x3f << PM_PLL_PLLCOUNT_SHIFT)

/* Oscillator 0/1 Control Register Bit-field Definitions */

#define PM_OSCCTRL_MODE_SHIFT      (0)       /* Bits 0-2: Oscillator Mode */
#define PM_OSCCTRL_MODE_MASK       (7 << PM_OSCCTRL_MODE_SHIFT)
#  define PM_OSCCTRL_MODE_EXT      (0 << PM_OSCCTRL_MODE_SHIFT) /* External clock */
#  define PM_OSCCTRL_MODE_XTALp9   (4 << PM_OSCCTRL_MODE_SHIFT) /* Crystal XIN 0.4-0.9MHz */
#  define PM_OSCCTRL_MODE_XTAL3    (5 << PM_OSCCTRL_MODE_SHIFT) /* Crystal XIN 0.9-3.0MHz */
#  define PM_OSCCTRL_MODE_XTAL8    (6 << PM_OSCCTRL_MODE_SHIFT) /* Crystal XIN 3.0-8.0MHz */
#  define PM_OSCCTRL_MODE_XTALHI   (7 << PM_OSCCTRL_MODE_SHIFT) /* Crystal XIN above 8.0MHz */
#define PM_OSCCTRL_STARTUP_SHIFT   (8)       /* Bits 8-10: Oscillator Startup Time */
#define PM_OSCCTRL_STARTUP_MASK    (7 << PM_OSCCTRL_STARTUP_SHIFT)
#  define PM_OSCCTRL_STARTUP_0     (0 << PM_OSCCTRL_STARTUP_SHIFT) /* Num RCOsc cycles */
#  define PM_OSCCTRL_STARTUP_64    (1 << PM_OSCCTRL_STARTUP_SHIFT) /* " " "   " "    " */
#  define PM_OSCCTRL_STARTUP_128   (2 << PM_OSCCTRL_STARTUP_SHIFT) /* " " "   " "    " */
#  define PM_OSCCTRL_STARTUP_2K    (3 << PM_OSCCTRL_STARTUP_SHIFT) /* " " "   " "    " */
#  define PM_OSCCTRL_STARTUP_4K    (4 << PM_OSCCTRL_STARTUP_SHIFT) /* " " "   " "    " */
#  define PM_OSCCTRL_STARTUP_8K    (5 << PM_OSCCTRL_STARTUP_SHIFT) /* " " "   " "    " */
#  define PM_OSCCTRL_STARTUP_16K   (6 << PM_OSCCTRL_STARTUP_SHIFT) /* " " "   " "    " */

/* Oscillator 32 Control Register Bit-field Definitions */

#define PM_OSCCTRL32_EN             (1 << 0)  /* Bit 0: Enable the 32KHz oscillator */
#define PM_OSCCTRL32_MODE_SHIFT     (8)       /* Bits 8-10: Oscillator Mode */
#define PM_OSCCTRL32_MODE_MASK      (7 << PM_OSCCTRL32_MODE_SHIFT)
#  define PM_OSCCTRL32_MODE_EXT     (0 << PM_OSCCTRL32_MODE_SHIFT) /* External clock */
#  define PM_OSCCTRL32_MODE_XTAL    (1 << PM_OSCCTRL32_MODE_SHIFT) /* Crystal */
#define PM_OSCCTRL32_STARTUP_SHIFT  (16)      /* Bits 16-18: Oscillator Startup Time */
#define PM_OSCCTRL32_STARTUP_MASK   (7 << PM_OSCCTRL32_STARTUP_SHIFT)
#  define PM_OSCCTRL32_STARTUP_0    (0 << PM_OSCCTRL32_STARTUP_SHIFT) /* Num RCOsc cycles */
#  define PM_OSCCTRL32_STARTUP_128  (1 << PM_OSCCTRL32_STARTUP_SHIFT) /* " " "   " "    " */
#  define PM_OSCCTRL32_STARTUP_8K   (2 << PM_OSCCTRL32_STARTUP_SHIFT) /* " " "   " "    " */
#  define PM_OSCCTRL32_STARTUP_16K  (3 << PM_OSCCTRL32_STARTUP_SHIFT) /* " " "   " "    " */
#  define PM_OSCCTRL32_STARTUP_64K  (4 << PM_OSCCTRL32_STARTUP_SHIFT) /* " " "   " "    " */
#  define PM_OSCCTRL32_STARTUP_128K (5 << PM_OSCCTRL32_STARTUP_SHIFT) /* " " "   " "    " */
#  define PM_OSCCTRL32_STARTUP_512K (6 << PM_OSCCTRL32_STARTUP_SHIFT) /* " " "   " "    " */

/* Interrupt Enable Register Bit-field Definitions */
/* Interrupt Disable Register Bit-field Definitions */
/* Interrupt Mask Register Bit-field Definitions */
/* Interrupt Status Register Bit-field Definitions */
/* Interrupt Clear Register Bit-field Definitions */

#define PM_INT_LOCK0               (1 << 0)  /* Bit 0:  PLL0 locked */
#define PM_INT_LOCK1               (1 << 1)  /* Bit 1:  PLL1 locked */
#define PM_INT_CKRDY               (1 << 5)  /* Bit 5:  Clock Ready */
#define PM_INT_MSKRDY              (1 << 6)  /* Bit 6:  Mask Ready */
#define PM_INT_OSC0RDY             (1 << 7)  /* Bit 7:  Oscillator 0 Ready */
#define PM_INT_OSC1RDY             (1 << 8)  /* Bit 8:  Oscillator 1 Ready */
#define PM_INT_OSC32RDY            (1 << 9)  /* Bit 9:  32 KHz oscillator Ready */
#define PM_INT_BODDET              (1 << 16) /* Bit 16: Brown out detection */

/* Power and Oscillators Status Register Bit-field Definitions */

#define PM_POSCSR_LOCK0               (1 << 0)  /* Bit 0:  PLL0 locked */
#define PM_POSCSR_LOCK1               (1 << 1)  /* Bit 1:  PLL1 locked */
#define PM_POSCSR_WAKE                (1 << 2)  /* Bit 1:  PLL1 locked */
#define PM_POSCSR_CKRDY               (1 << 5)  /* Bit 5:  Clock Ready */
#define PM_POSCSR_MSKRDY              (1 << 6)  /* Bit 6:  Mask Ready */
#define PM_POSCSR_OSC0RDY             (1 << 7)  /* Bit 7:  Oscillator 0 Ready */
#define PM_POSCSR_OSC1RDY             (1 << 8)  /* Bit 8:  Oscillator 1 Ready */
#define PM_POSCSR_OSC32RDY            (1 << 9)  /* Bit 9:  32 KHz oscillator Ready */
#define PM_POSCSR_BODDET              (1 << 16) /* Bit 16: Brown out detection */

/* 0x0060-0x070 Generic Clock Control Register Bit-field Definitions */

#define PM_GCCTRL_OSCSEL              (1 << 0)  /* Bit 0:  Oscillator Select */
#define PM_GCCTRL_PLLSEL              (1 << 1)  /* Bit 1:  PLL Select */
#define PM_GCCTRL_CEN                 (1 << 2)  /* Bit 2:  Clock Enable */
#define PM_GCCTRL_DIVEN               (1 << 4)  /* Bit 4:  Divide Enable */
#define PM_GCCTRL_DIV_SHIFT           (8)       /* Bits 8-15: Division Factor */
#define PM_GCCTRL_DIV_MASK            (0xff << PM_GCCTRL_DIV_SHIFT)

/* RC Oscillator Calibration Register Bit-field Definitions */

#define PM_RCCR_CALIB_SHIFT           (0)       /* Bits 0-9: Calibration Value */
#define PM_RCCR_CALIB_MASK            (0x3ff << PM_RCCR_CALIB_SHIFT)
#define PM_RCCR_FCD                   (1 << 16) /* Bit 16: Flash Calibration Done */
#define PM_RCCR_KEY_SHIFT             (24)      /* Bits 24-31: Register Write protection */
#define PM_RCCR_KEY_MASK              (0xff << PM_RCCR_KEY_SHIFT)

/* Bandgap Calibration Register Bit-field Definitions */

#define PM_BGCR_CALIB_SHIFT           (0)       /* Bits 0-3: Calibration Value */
#define PM_BGCR_CALIB_MASK            (7 << PM_BGCR_CALIB_SHIFT)
#define PM_BGCR_FCD                   (1 << 16) /* Bit 16: Flash Calibration Done */
#define PM_BGCR_KEY_SHIFT             (24)      /* Bits 24-31: Register Write protection */
#define PM_BGCR_KEY_MASK              (0xff << PM_BGCR_KEY_SHIFT)

/* Linear Regulator Calibration Register Bit-field Definitions */

#define PM_VREGCR_CALIB_SHIFT         (0)       /* Bits 0-3: Calibration Value */
#define PM_VREGCR_CALIB_MASK          (7 << PM_VREGCR_CALIB_SHIFT)
#define PM_VREGCR_FCD                 (1 << 16) /* Bit 16: Flash Calibration Done */
#define PM_VREGCR_KEY_SHIFT           (24)      /* Bits 24-31: Register Write protection */
#define PM_VREGCR_KEY_MASK            (0xff << PM_VREGCR_KEY_SHIFT)

/* BOD Level Register BOD Read/Write Bit-field Definitions */

#define PM_BOD_LEVEL_SHIFT            (0)       /* Bits 0-5: BOD Level */
#define PM_BOD_LEVEL_MASK             (0x3f << PM_BOD_LEVEL_SHIFT)
#define PM_BOD_HYST                   (1 << 6)  /* Bit 6:  BOD Hysteresis */
#define PM_BOD_CTRL_SHIFT             (8)       /* Bits 8-9: BOD Control */
#define PM_BOD_CTRL_MASK              (3 << PM_BOD_CTRL_SHIFT)
#  define PM_BOD_CTRL_OFF             (xxx << PM_BOD_CTRL_SHIFT) /* BOD is off */
#  define PM_BOD_CTRL_RESET           (xxx << PM_BOD_CTRL_SHIFT) /* BOD enabled/can reset */
#  define PM_BOD_CTRL_NORESET         (xxx << PM_BOD_CTRL_SHIFT) /* BOD enabled/cannot reset */
#define PM_BOD_FCD                    (1 << 16) /* Bit 16: BOD Fuse calibration done */
#define PM_BOD_KEY_SHIFT              (24)      /* Bits 24-31: Register Write protection */
#define PM_BOD_KEY_MASK               (0xff << PM_BOD_KEY_SHIFT)

/* Reset Cause Register */

#define PM_RCAUSE_POR                 (1 << 0)  /* Bit 0:  Power-on Reset */
#define PM_RCAUSE_BOD                 (1 << 1)  /* Bit 1:  Brown-out Reset */
#define PM_RCAUSE_EXT                 (1 << 2)  /* Bit 2:  External Reset Pin */
#define PM_RCAUSE_WDT                 (1 << 3)  /* Bit 3:  Watchdog Reset */
#define PM_RCAUSE_JTAG                (1 << 4)  /* Bit 4:  JTAG reset */
#define PM_RCAUSE_SLEEP               (1 << 6)  /* Bit 6:  Sleep */
#define PM_RCAUSE_CPUERR              (1 << 7)  /* Bit 7:  CPU Error */
#define PM_RCAUSE_OCDRST              (1 << 8)  /* Bit 8:  OCD Reset */

/* Asynchronous Wake Up Enable Register Bit-field Definitions */

#define PM_AWEN_USBWAKEN              (1 << 0)  /* Bit 0:  USB Wake Up Enable */

/* General Purpose Low-Power Register 0/1 Bit-field Definitions */

/* These registers contain a 32-bit value with no smaller bit-field */

/* GCLK Allocation ******************************************************************/

#define AVR32_PM_GCLK0                (0)       /* GCLK0 pin */
#define AVR32_PM_GCLK1                (1)       /* GCLK2 pin */
#define AVR32_PM_GCLK2                (2)       /* GCLK2 pin */
#define AVR32_PM_GCLK_USBB            (3)       /* USBB */
#define AVR32_PM_GCLK_ABDAC           (4)       /* ABDAC */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_AVR_SRC_AT32UC3_AT32UC3_PM_H */

