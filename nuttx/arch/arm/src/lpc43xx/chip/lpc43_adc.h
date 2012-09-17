/************************************************************************************
 * arch/arm/src/lpc43xx/lpc43_adc.h
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

#ifndef __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_ADC_H
#define __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_ADC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/

#define LPC43_ADC_CR_OFFSET      0x0000  /* A/D Control Register */
#define LPC43_ADC_GDR_OFFSET     0x0004  /* A/D Global Data Register */
#define LPC43_ADC_INTEN_OFFSET   0x000c  /* A/D Interrupt Enable Register */

#define LPC43_ADC_DR_OFFSET(n)   (0x0010+((n) << 2))
#define LPC43_ADC_DR0_OFFSET     0x0010  /* A/D Channel 0 Data Register */
#define LPC43_ADC_DR1_OFFSET     0x0014  /* A/D Channel 1 Data Register */
#define LPC43_ADC_DR2_OFFSET     0x0018  /* A/D Channel 2 Data Register */
#define LPC43_ADC_DR3_OFFSET     0x001c  /* A/D Channel 3 Data Register */
#define LPC43_ADC_DR4_OFFSET     0x0020  /* A/D Channel 4 Data Register */
#define LPC43_ADC_DR5_OFFSET     0x0024  /* A/D Channel 5 Data Register */
#define LPC43_ADC_DR6_OFFSET     0x0028  /* A/D Channel 6 Data Register */
#define LPC43_ADC_DR7_OFFSET     0x002c  /* A/D Channel 7 Data Register */

#define LPC43_ADC_STAT_OFFSET    0x0030  /* A/D Status Register */

/* Register addresses ***************************************************************/

#define LPC43_ADC0_CR            (LPC43_ADC0_BASE+LPC43_ADC_CR_OFFSET)
#define LPC43_ADC0_GDR           (LPC43_ADC0_BASE+LPC43_ADC_GDR_OFFSET)
#define LPC43_ADC0_INTEN         (LPC43_ADC0_BASE+LPC43_ADC_INTEN_OFFSET)
#define LPC43_ADC0_DR(n)         (LPC43_ADC0_BASE+LPC43_ADC_DR_OFFSET(n))
#define LPC43_ADC0_DR0           (LPC43_ADC0_BASE+LPC43_ADC_DR0_OFFSET)
#define LPC43_ADC0_DR1           (LPC43_ADC0_BASE+LPC43_ADC_DR1_OFFSET)
#define LPC43_ADC0_DR2           (LPC43_ADC0_BASE+LPC43_ADC_DR2_OFFSET)
#define LPC43_ADC0_DR3           (LPC43_ADC0_BASE+LPC43_ADC_DR3_OFFSET)
#define LPC43_ADC0_DR4           (LPC43_ADC0_BASE+LPC43_ADC_DR4_OFFSET)
#define LPC43_ADC0_DR5           (LPC43_ADC0_BASE+LPC43_ADC_DR5_OFFSET)
#define LPC43_ADC0_DR6           (LPC43_ADC0_BASE+LPC43_ADC_DR6_OFFSET)
#define LPC43_ADC0_DR7           (LPC43_ADC0_BASE+LPC43_ADC_DR7_OFFSET)
#define LPC43_ADC0_STAT          (LPC43_ADC0_BASE+LPC43_ADC_STAT_OFFSET)

#define LPC43_ADC1_CR            (LPC43_ADC1_BASE+LPC43_ADC_CR_OFFSET)
#define LPC43_ADC1_GDR           (LPC43_ADC1_BASE+LPC43_ADC_GDR_OFFSET)
#define LPC43_ADC1_INTEN         (LPC43_ADC1_BASE+LPC43_ADC_INTEN_OFFSET)
#define LPC43_ADC1_DR(n)         (LPC43_ADC1_BASE+LPC43_ADC_DR_OFFSET(n))
#define LPC43_ADC1_DR0           (LPC43_ADC1_BASE+LPC43_ADC_DR0_OFFSET)
#define LPC43_ADC1_DR1           (LPC43_ADC1_BASE+LPC43_ADC_DR1_OFFSET)
#define LPC43_ADC1_DR2           (LPC43_ADC1_BASE+LPC43_ADC_DR2_OFFSET)
#define LPC43_ADC1_DR3           (LPC43_ADC1_BASE+LPC43_ADC_DR3_OFFSET)
#define LPC43_ADC1_DR4           (LPC43_ADC1_BASE+LPC43_ADC_DR4_OFFSET)
#define LPC43_ADC1_DR5           (LPC43_ADC1_BASE+LPC43_ADC_DR5_OFFSET)
#define LPC43_ADC1_DR6           (LPC43_ADC1_BASE+LPC43_ADC_DR6_OFFSET)
#define LPC43_ADC1_DR7           (LPC43_ADC1_BASE+LPC43_ADC_DR7_OFFSET)
#define LPC43_ADC1_STAT          (LPC43_ADC1_BASE+LPC43_ADC_STAT_OFFSET)

/* Register bit definitions *********************************************************/

/* A/D Control Register */

#define ADC_CR_SEL_SHIFT         (0)       /* Bits 0-7: Selects pins to be sampled */
#define ADC_CR_SEL_MASK          (0xff << ADC_CR_SEL_MASK)
#define ADC_CR_CLKDIV_SHIFT      (8)       /* Bits 8-15: APB clock (PCLK_ADC0) divisor */
#define ADC_CR_CLKDIV_MASK       (0xff << ADC_CR_CLKDIV_SHIFT)
#define ADC_CR_BURST             (1 << 16) /* Bit 16: A/D Repeated conversions */

#define ADC_CR_CLKS_SHIFT        (17)      /* Bits 17-19: Number of clocks in conversion */
#define ADC_CR_CLKS_MASK         (7 << ADC_CR_CLKS_SHIFT)
#  define ADC_CR_CLKS_11         (0 << ADC_CR_CLKS_SHIFT) /* 11 clocks / 10 bits */
#  define ADC_CR_CLKS_10         (1 << ADC_CR_CLKS_SHIFT) /* 10 clocks / 9 bits */
#  define ADC_CR_CLKS_9          (2 << ADC_CR_CLKS_SHIFT) /* 9 clocks / 8 bits */
#  define ADC_CR_CLKS_8          (3 << ADC_CR_CLKS_SHIFT) /* 8 clocks / 7 bits */
#  define ADC_CR_CLKS_7          (4 << ADC_CR_CLKS_SHIFT) /* 7 clocks / 6 bits */
#  define ADC_CR_CLKS_6          (5 << ADC_CR_CLKS_SHIFT) /* 6 clocks / 5 bits */
#  define ADC_CR_CLKS_5          (6 << ADC_CR_CLKS_SHIFT) /* 5 clocks / 4 bits */
#  define ADC_CR_CLKS_4          (7 << ADC_CR_CLKS_SHIFT) /* 4 clocks / 3 bits */
                                           /* Bit 20: Reserved */
#define ADC_CR_PDN               (1 << 21) /* Bit 21: A/D converter power-down mode */
                                           /* Bits 22-23: Reserved */
#define ADC_CR_START_SHIFT       (24)      /* Bits 24-26: Control A/D conversion start */
#define ADC_CR_START_MASK        (7 << ADC_CR_START_SHIFT)
#  define ADC_CR_START_NOSTART   (0 << ADC_CR_START_SHIFT) /* No start */
#  define ADC_CR_START_NOW       (1 << ADC_CR_START_SHIFT) /* Start now */
#  define ADC_CR_START_CTOUT15   (2 << ADC_CR_START_SHIFT) /* Start when edge on CTOUT_15 */
#  define ADC_CR_START_CTOUT8    (3 << ADC_CR_START_SHIFT) /* Start when edge on CTOUT_8 */
#  define ADC_CR_START_ADCTRIG0  (4 << ADC_CR_START_SHIFT) /* Start when edge on ADCTRIG0 */
#  define ADC_CR_START_ADCTRIG1  (5 << ADC_CR_START_SHIFT) /* Start when edge on ADCTRIG1 */
#  define ADC_CR_START_MCPWM     (6 << ADC_CR_START_SHIFT) /* Start when edge on MCPWM */
#define ADC_CR_EDGE              (1 << 27) /* Bit 27: Start on falling edge  */
                                           /* Bits 28-31: Reserved */
/* A/D Global Data Register */
                                           /* Bits 0-3: Reserved */
#define ADC_GDR_VVREF_SHIFT      (6)       /* Bits 6-15: Result of conversion (DONE==1) */
#define ADC_GDR_VVREF_MASK       (0x03ff << ADC_GDR_VVREF_SHIFT)
                                           /* Bits 16-23: Reserved */
#define ADC_GDR_CHAN_SHIFT       (24)      /* Bits 24-26: Channel converted */
#define ADC_GDR_CHAN_MASK        (3 << ADC_GDR_CHN_SHIFT)
                                           /* Bits 27-29: Reserved */
#define ADC_GDR_OVERRUN          (1 << 30) /* Bit 30: Conversion(s) lost/overwritten*/
#define ADC_GDR_DONE             (1 << 31) /* Bit 31: A/D conversion complete*/

/* A/D Interrupt Enable Register */

#define ADC_INTEN_CHAN(n)        (1 << (n))
#define ADC_INTEN_CHAN0          (1 << 0)  /* Bit 0:  Enable ADC chan 0 complete intterrupt */
#define ADC_INTEN_CHAN1          (1 << 1)  /* Bit 1:  Enable ADC chan 1 complete interrupt */
#define ADC_INTEN_CHAN2          (1 << 2)  /* Bit 2:  Enable ADC chan 2 complete interrupt */
#define ADC_INTEN_CHAN3          (1 << 3)  /* Bit 3:  Enable ADC chan 3 complete interrupt */
#define ADC_INTEN_CHAN4          (1 << 4)  /* Bit 4:  Enable ADC chan 4 complete interrupt */
#define ADC_INTEN_CHAN5          (1 << 5)  /* Bit 5:  Enable ADC chan 5 complete interrupt */
#define ADC_INTEN_CHAN6          (1 << 6)  /* Bit 6:  Enable ADC chan 6 complete interrupt */
#define ADC_INTEN_CHAN7          (1 << 7)  /* Bit 7:  Enable ADC chan 7 complete interrupt */
#define ADC_INTEN_GLOBAL         (1 << 8)  /* Bit 8:  Only the global DONE generates interrupt */
                                           /* Bits 9-31: Reserved */
/* Channel 0-7 A/D Data Register */
                                           /* Bits 0-3: Reserved */
#define ADC_DR_VVREF_SHIFT       (6)       /* Bits 6-15: Result of conversion (DONE==1) */
#define ADC_DR_VVREF_MASK        (0x03ff << ADC_DR_VVREF_SHIFT)
                                           /* Bits 16-29: Reserved */
#define ADC_DR_OVERRUN           (1 << 30) /* Bit 30: Conversion(s) lost/overwritten*/
#define ADC_DR_DONE              (1 << 31) /* Bit 31: A/D conversion complete*/

/* A/D Status Register */

#define ADC_STAT_DONE(n)         (1 << (n))
#define ADC_STAT_DONE0           (1 << 0)  /* Bit 0:  A/D chan 0 DONE */
#define ADC_STAT_DONE1           (1 << 1)  /* Bit 1:  A/D chan 1 DONE */
#define ADC_STAT_DONE2           (1 << 2)  /* Bit 2:  A/D chan 2 DONE */
#define ADC_STAT_DONE3           (1 << 3)  /* Bit 3:  A/D chan 3 DONE */
#define ADC_STAT_DONE4           (1 << 4)  /* Bit 4:  A/D chan 4 DONE */
#define ADC_STAT_DONE5           (1 << 5)  /* Bit 5:  A/D chan 5 DONE */
#define ADC_STAT_DONE6           (1 << 6)  /* Bit 6:  A/D chan 6 DONE */
#define ADC_STAT_DONE7           (1 << 7)  /* Bit 7:  A/D chan 7 DONE */
#define ADC_STAT_OVERRUN(n)      ((1 << (n)) + 8)
#define ADC_STAT_OVERRUN0        (1 << 8)  /* Bit 8:  A/D chan 0 OVERRUN */
#define ADC_STAT_OVERRUN1        (1 << 9)  /* Bit 9:  A/D chan 1 OVERRUN */
#define ADC_STAT_OVERRUN2        (1 << 10) /* Bit 10: A/D chan 2 OVERRUN */
#define ADC_STAT_OVERRUN3        (1 << 11) /* Bit 11: A/D chan 3 OVERRUN */
#define ADC_STAT_OVERRUN4        (1 << 12) /* Bit 12: A/D chan 4 OVERRUN */
#define ADC_STAT_OVERRUN5        (1 << 13) /* Bit 13: A/D chan 5 OVERRUN */
#define ADC_STAT_OVERRUN6        (1 << 14) /* Bit 14: A/D chan 6 OVERRUN */
#define ADC_STAT_OVERRUN7        (1 << 15) /* Bit 15: A/D chan 7 OVERRUN */
#define ADC_STAT_INT             (1 << 16) /* Bit 15: A/D interrupt */
                                           /* Bits 17-31: Reserved */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_ADC_H */
