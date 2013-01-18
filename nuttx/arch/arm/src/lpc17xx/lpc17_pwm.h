/************************************************************************************
 * arch/arm/src/lpc17xx/lpc17_pwm.h
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

#ifndef __ARCH_ARM_SRC_LPC17XX_LPC17_PWM_H
#define __ARCH_ARM_SRC_LPC17XX_LPC17_PWM_H

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

#define LPC17_PWM_IR_OFFSET     0x0000 /* Interrupt Register */
#define LPC17_PWM_TCR_OFFSET    0x0004 /* Timer Control Register */
#define LPC17_PWM_TC_OFFSET     0x0008 /* Timer Counter */
#define LPC17_PWM_PR_OFFSET     0x000c /* Prescale Register */
#define LPC17_PWM_PC_OFFSET     0x0010 /* Prescale Counter */
#define LPC17_PWM_MCR_OFFSET    0x0014 /* Match Control Register */
#define LPC17_PWM_MR0_OFFSET    0x0018 /* Match Register 0 */
#define LPC17_PWM_MR1_OFFSET    0x001c /* Match Register 1 */
#define LPC17_PWM_MR2_OFFSET    0x0020 /* Match Register 2 */
#define LPC17_PWM_MR3_OFFSET    0x0024 /* Match Register 3 */
#define LPC17_PWM_CCR_OFFSET    0x0028 /* Capture Control Register */
#define LPC17_PWM_CR0_OFFSET    0x002c /* Capture Register 0 */
#define LPC17_PWM_CR1_OFFSET    0x0030 /* Capture Register 1 */
#define LPC17_PWM_CR2_OFFSET    0x0034 /* Capture Register 2 */
#define LPC17_PWM_CR3_OFFSET    0x0038 /* Capture Register 3 */
#define LPC17_PWM_MR4_OFFSET    0x0040 /* Match Register 4 */
#define LPC17_PWM_MR5_OFFSET    0x0044 /* Match Register 5 */
#define LPC17_PWM_MR6_OFFSET    0x0048 /* Match Register 6 */
#define LPC17_PWM_PCR_OFFSET    0x004c /* PWM Control Register */
#define LPC17_PWM_LER_OFFSET    0x0050 /* Load Enable Register */
#define LPC17_PWM_CTCR_OFFSET   0x0070 /* Counter/Timer Control Register */

/* Register addresses ***************************************************************/

#define LPC17_PWM1_IR          (LPC17_PWM1_BASE+LPC17_PWM_IR_OFFSET)
#define LPC17_PWM1_TCR         (LPC17_PWM1_BASE+LPC17_PWM_TCR_OFFSET)
#define LPC17_PWM1_TC          (LPC17_PWM1_BASE+LPC17_PWM_TC_OFFSET)
#define LPC17_PWM1_PR          (LPC17_PWM1_BASE+LPC17_PWM_PR_OFFSET)
#define LPC17_PWM1_PC          (LPC17_PWM1_BASE+LPC17_PWM_PC_OFFSET)
#define LPC17_PWM1_MCR         (LPC17_PWM1_BASE+LPC17_PWM_MCR_OFFSET)
#define LPC17_PWM1_MR0         (LPC17_PWM1_BASE+LPC17_PWM_MR0_OFFSET)
#define LPC17_PWM1_MR1         (LPC17_PWM1_BASE+LPC17_PWM_MR1_OFFSET)
#define LPC17_PWM1_MR2         (LPC17_PWM1_BASE+LPC17_PWM_MR2_OFFSET)
#define LPC17_PWM1_MR3         (LPC17_PWM1_BASE+LPC17_PWM_MR3_OFFSET)
#define LPC17_PWM1_MR4         (LPC17_PWM1_BASE+LPC17_PWM_MR4_OFFSET)
#define LPC17_PWM1_MR5         (LPC17_PWM1_BASE+LPC17_PWM_MR5_OFFSET)
#define LPC17_PWM1_MR6         (LPC17_PWM1_BASE+LPC17_PWM_MR6_OFFSET)
#define LPC17_PWM1_CCR         (LPC17_PWM1_BASE+LPC17_PWM_CCR_OFFSET)
#define LPC17_PWM1_CR0         (LPC17_PWM1_BASE+LPC17_PWM_CR0_OFFSET)
#define LPC17_PWM1_CR1         (LPC17_PWM1_BASE+LPC17_PWM_CR1_OFFSET)
#define LPC17_PWM1_CR2         (LPC17_PWM1_BASE+LPC17_PWM_CR2_OFFSET)
#define LPC17_PWM1_CR3         (LPC17_PWM1_BASE+LPC17_PWM_CR3_OFFSET)
#define LPC17_PWM1_PCR         (LPC17_PWM1_BASE+LPC17_PWM_PCR_OFFSET)
#define LPC17_PWM1_LER         (LPC17_PWM1_BASE+LPC17_PWM_LER_OFFSET)
#define LPC17_PWM1_CTCR        (LPC17_PWM1_BASE+LPC17_PWM_CTCR_OFFSET)

/* Register bit definitions *********************************************************/
/* Registers holding 32-bit numeric values (no bit field definitions):
 *
 *   Timer Counter (TC)
 *   Prescale Register (PR)
 *   Prescale Counter (PC)
 *   Match Register 0 (MR0)
 *   Match Register 1 (MR1)
 *   Match Register 2 (MR2)
 *   Match Register 3 (MR3)
 *   Match Register 4 (MR3)
 *   Match Register 5 (MR3)
 *   Match Register 6 (MR3)
 *   Capture Register 0 (CR0)
 *   Capture Register 1 (CR1)
 *   Capture Register 1 (CR2)
 *   Capture Register 1 (CR3)
 */

/* Interrupt Register */

#define PWM_IR_MR0                (1 << 0)  /* Bit 0:  PWM match channel 0 interrrupt */
#define PWM_IR_MR1                (1 << 1)  /* Bit 1:  PWM match channel 1 interrrupt */
#define PWM_IR_MR2                (1 << 2)  /* Bit 2:  PWM match channel 2 interrrupt */
#define PWM_IR_MR3                (1 << 3)  /* Bit 3:  PWM match channel 3 interrrupt */
#define PWM_IR_CAP0               (1 << 4)  /* Bit 4:  Capture input 0 interrrupt */
#define PWM_IR_CAP1               (1 << 5)  /* Bit 5:  Capture input 1 interrrupt */
                                            /* Bits 6-7: Reserved */
#define PWM_IR_MR4                (1 << 8)  /* Bit 8:  PWM match channel 4 interrrupt */
#define PWM_IR_MR5                (1 << 9)  /* Bit 9:  PWM match channel 5 interrrupt */
#define PWM_IR_MR6                (1 << 10) /* Bit 10: PWM match channel 6 interrrupt */
                                            /* Bits 11-31: Reserved */
/* Timer Control Register */

#define PWM_TCR_CNTREN            (1 << 0)  /* Bit 0:  Counter Enable */
#define PWM_TCR_CNTRRST           (1 << 1)  /* Bit 1:  Counter Reset */
                                            /* Bit 2: Reserved */
#define PWM_TCR_PWMEN             (1 << 3)  /* Bit 3:  PWM Enable */
                                            /* Bits 4-31: Reserved */
/* Match Control Register */

#define PWM_MCR_MR0I              (1 << 0)  /* Bit 0:  Interrupt on MR0 */
#define PWM_MCR_MR0R              (1 << 1)  /* Bit 1:  Reset on MR0 */
#define PWM_MCR_MR0S              (1 << 2)  /* Bit 2:  Stop on MR0 */
#define PWM_MCR_MR1I              (1 << 3)  /* Bit 3:  Interrupt on MR1 */
#define PWM_MCR_MR1R              (1 << 4)  /* Bit 4:  Reset on MR1 */
#define PWM_MCR_MR1S              (1 << 5)  /* Bit 5:  Stop on MR1 */
#define PWM_MCR_MR2I              (1 << 6)  /* Bit 6:  Interrupt on MR2 */
#define PWM_MCR_MR2R              (1 << 7)  /* Bit 7:  Reset on MR2 */
#define PWM_MCR_MR2S              (1 << 8)  /* Bit 8:  Stop on MR2 */
#define PWM_MCR_MR3I              (1 << 9)  /* Bit 9:  Interrupt on MR3 */
#define PWM_MCR_MR3R              (1 << 10) /* Bit 10: Reset on MR3 */
#define PWM_MCR_MR3S              (1 << 11) /* Bit 11: Stop on MR3 */
#define PWM_MCR_MR4I              (1 << 12) /* Bit 12:  Interrupt on MR4 */
#define PWM_MCR_MR4R              (1 << 13) /* Bit 13: Reset on MR4 */
#define PWM_MCR_MR4S              (1 << 14) /* Bit 14: Stop on MR4 */
#define PWM_MCR_MR5I              (1 << 15) /* Bit 15:  Interrupt on MR5 */
#define PWM_MCR_MR5R              (1 << 16) /* Bit 16: Reset on MR5*/
#define PWM_MCR_MR5S              (1 << 17) /* Bit 17: Stop on MR5 */
#define PWM_MCR_MR6I              (1 << 18) /* Bit 18:  Interrupt on MR6 */
#define PWM_MCR_MR6R              (1 << 19) /* Bit 19: Reset on MR6 */
#define PWM_MCR_MR6S              (1 << 20) /* Bit 20: Stop on MR6 */
                                            /* Bits 21-31: Reserved */
/* Capture Control Register (Where are CAP2 and 3?) */

#define PWM_CCR_CAP0RE            (1 << 0)  /* Bit 0: Capture on CAPn.0 rising edge */
#define PWM_CCR_CAP0FE            (1 << 1)  /* Bit 1: Capture on CAPn.0 falling edg */
#define PWM_CCR_CAP0I             (1 << 2)  /* Bit 2: Interrupt on CAPn.0 */
#define PWM_CCR_CAP1RE            (1 << 3)  /* Bit 3: Capture on CAPn.1 rising edge */
#define PWM_CCR_CAP1FE            (1 << 4)  /* Bit 4: Capture on CAPn.1 falling edg */
#define PWM_CCR_CAP1I             (1 << 5)  /* Bit 5: Interrupt on CAPn.1 */
                                            /* Bits 6-31: Reserved */
/* PWM Control Register */
                                            /* Bits 0-1: Reserved */
#define PWM_PCR_SEL2              (1 << 2)  /* Bit 2:  PWM2 single edge controlled mode */
#define PWM_PCR_SEL3              (1 << 3)  /* Bit 3:  PWM3 single edge controlled mode */
#define PWM_PCR_SEL4              (1 << 4)  /* Bit 4:  PWM4 single edge controlled mode */
#define PWM_PCR_SEL5              (1 << 5)  /* Bit 5:  PWM5 single edge controlled mode */
#define PWM_PCR_SEL6              (1 << 6)  /* Bit 6:  PWM6 single edge controlled mode */
                                            /* Bits 7-8: Reserved */
#define PWM_PCR_ENA1              (1 << 9)  /* Bit 9:  Enable PWM1 output */
#define PWM_PCR_ENA2              (1 << 10) /* Bit 10: Enable PWM2 output */
#define PWM_PCR_ENA3              (1 << 11) /* Bit 11: Enable PWM3 output */
#define PWM_PCR_ENA4              (1 << 12) /* Bit 12: Enable PWM4 output */
#define PWM_PCR_ENA5              (1 << 13) /* Bit 13: Enable PWM5 output */
#define PWM_PCR_ENA6              (1 << 14) /* Bit 14: Enable PWM6 output */
                                            /* Bits 15-31: Reserved */
/* Load Enable Register */

#define PWM_LER_M0EN              (1 << 0)  /* Bit 0:  Enable PWM Match 0 Latch */
#define PWM_LER_M1EN              (1 << 1)  /* Bit 1:  Enable PWM Match 1 Latch */
#define PWM_LER_M2EN              (1 << 2)  /* Bit 2:  Enable PWM Match 2 Latch */
#define PWM_LER_M3EN              (1 << 3)  /* Bit 3:  Enable PWM Match 3 Latch */
#define PWM_LER_M4EN              (1 << 4)  /* Bit 4:  Enable PWM Match 4 Latch */
#define PWM_LER_M5EN              (1 << 5)  /* Bit 5:  Enable PWM Match 5 Latch */
#define PWM_LER_M6EN              (1 << 6)  /* Bit 6:  Enable PWM Match 6 Latch */
                                            /* Bits 7-31: Reserved */
/* Counter/Timer Control Register */

#define PWM_CTCR_MODE_SHIFT       (0)       /* Bits 0-1: Counter/Timer Mode */
#define PWM_CTCR_MODE_MASK        (3 << PWM_CTCR_MODE_SHIFT)
#  define PWM_CTCR_MODE_TIMER     (0 << PWM_CTCR_MODE_SHIFT) /* Timer Mode, prescal match */
#  define PWM_CTCR_MODE_CNTRRE    (1 << PWM_CTCR_MODE_SHIFT) /* Counter Mode, CAP rising edge */
#  define PWM_CTCR_MODE_CNTRFE    (2 << PWM_CTCR_MODE_SHIFT) /* Counter Mode, CAP falling edge */
#  define PWM_CTCR_MODE_CNTRBE    (3 << PWM_CTCR_MODE_SHIFT) /* Counter Mode, CAP both edges */
#define PWM_CTCR_INPSEL_SHIFT     (2)       /* Bits 2-3: Count Input Select */
#define PWM_CTCR_INPSEL_MASK      (3 << PWM_CTCR_INPSEL_SHIFT)
#  define PWM_CTCR_INPSEL_CAPNp0  (0 << PWM_CTCR_INPSEL_SHIFT) /* CAPn.0 for TIMERn */
#  define PWM_CTCR_INPSEL_CAPNp1  (1 << PWM_CTCR_INPSEL_SHIFT) /* CAPn.0 for TIMERn */
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

#endif /* __ARCH_ARM_SRC_LPC17XX_LPC17_PWM_H */
