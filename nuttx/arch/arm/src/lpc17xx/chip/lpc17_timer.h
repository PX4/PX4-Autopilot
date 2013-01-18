/************************************************************************************
 * arch/arm/src/lpc17xx/chip/lpc17_timer.h
 *
 *   Copyright (C) 2010, 2012-2013 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LPC17XX_CHIP_LPC17_TIMER_H
#define __ARCH_ARM_SRC_LPC17XX_CHIP_LPC17_TIMER_H

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

#define LPC17_TMR_IR_OFFSET       0x0000 /* Interrupt Register */
#define LPC17_TMR_TCR_OFFSET      0x0004 /* Timer Control Register */
#define LPC17_TMR_TC_OFFSET       0x0008 /* Timer Counter */
#define LPC17_TMR_PR_OFFSET       0x000c /* Prescale Register */
#define LPC17_TMR_PC_OFFSET       0x0010 /* Prescale Counter */
#define LPC17_TMR_MCR_OFFSET      0x0014 /* Match Control Register */
#define LPC17_TMR_MR0_OFFSET      0x0018 /* Match Register 0 */
#define LPC17_TMR_MR1_OFFSET      0x001c /* Match Register 1 */
#define LPC17_TMR_MR2_OFFSET      0x0020 /* Match Register 2 */
#define LPC17_TMR_MR3_OFFSET      0x0024 /* Match Register 3 */
#define LPC17_TMR_CCR_OFFSET      0x0028 /* Capture Control Register */
#define LPC17_TMR_CR0_OFFSET      0x002c /* Capture Register 0 */
#define LPC17_TMR_CR1_OFFSET      0x0030 /* Capture Register 1 */
#define LPC17_TMR_EMR_OFFSET      0x003c /* External Match Register */
#define LPC17_TMR_CTCR_OFFSET     0x0070 /* Count Control Register */

/* Register addresses ***************************************************************/

#define LPC17_TMR0_IR             (LPC17_TMR0_BASE+LPC17_TMR_IR_OFFSET)
#define LPC17_TMR0_TCR            (LPC17_TMR0_BASE+LPC17_TMR_TCR_OFFSET)
#define LPC17_TMR0_TC             (LPC17_TMR0_BASE+LPC17_TMR_TC_OFFSET)
#define LPC17_TMR0_PR             (LPC17_TMR0_BASE+LPC17_TMR_PR_OFFSET)
#define LPC17_TMR0_PC             (LPC17_TMR0_BASE+LPC17_TMR_PC_OFFSET)
#define LPC17_TMR0_MCR            (LPC17_TMR0_BASE+LPC17_TMR_MCR_OFFSET)
#define LPC17_TMR0_MR0            (LPC17_TMR0_BASE+LPC17_TMR_MR0_OFFSET)
#define LPC17_TMR0_MR1            (LPC17_TMR0_BASE+LPC17_TMR_MR1_OFFSET)
#define LPC17_TMR0_MR2            (LPC17_TMR0_BASE+LPC17_TMR_MR2_OFFSET)
#define LPC17_TMR0_MR3            (LPC17_TMR0_BASE+LPC17_TMR_MR3_OFFSET)
#define LPC17_TMR0_CCR            (LPC17_TMR0_BASE+LPC17_TMR_CCR_OFFSET)
#define LPC17_TMR0_CR0            (LPC17_TMR0_BASE+LPC17_TMR_CR0_OFFSET)
#define LPC17_TMR0_CR1            (LPC17_TMR0_BASE+LPC17_TMR_CR1_OFFSET)
#define LPC17_TMR0_EMR            (LPC17_TMR0_BASE+LPC17_TMR_EMR_OFFSET)
#define LPC17_TMR0_CTCR           (LPC17_TMR0_BASE+LPC17_TMR_CTCR_OFFSET)

#define LPC17_TMR1_IR             (LPC17_TMR1_BASE+LPC17_TMR_IR_OFFSET)
#define LPC17_TMR1_TCR            (LPC17_TMR1_BASE+LPC17_TMR_TCR_OFFSET)
#define LPC17_TMR1_TC             (LPC17_TMR1_BASE+LPC17_TMR_TC_OFFSET)
#define LPC17_TMR1_PR             (LPC17_TMR1_BASE+LPC17_TMR_PR_OFFSET)
#define LPC17_TMR1_PC             (LPC17_TMR1_BASE+LPC17_TMR_PC_OFFSET)
#define LPC17_TMR1_MCR            (LPC17_TMR1_BASE+LPC17_TMR_MCR_OFFSET)
#define LPC17_TMR1_MR0            (LPC17_TMR1_BASE+LPC17_TMR_MR0_OFFSET)
#define LPC17_TMR1_MR1            (LPC17_TMR1_BASE+LPC17_TMR_MR1_OFFSET)
#define LPC17_TMR1_MR2            (LPC17_TMR1_BASE+LPC17_TMR_MR2_OFFSET)
#define LPC17_TMR1_MR3            (LPC17_TMR1_BASE+LPC17_TMR_MR3_OFFSET)
#define LPC17_TMR1_CCR            (LPC17_TMR1_BASE+LPC17_TMR_CCR_OFFSET)
#define LPC17_TMR1_CR0            (LPC17_TMR1_BASE+LPC17_TMR_CR0_OFFSET)
#define LPC17_TMR1_CR1            (LPC17_TMR1_BASE+LPC17_TMR_CR1_OFFSET)
#define LPC17_TMR1_EMR            (LPC17_TMR1_BASE+LPC17_TMR_EMR_OFFSET)
#define LPC17_TMR1_CTCR           (LPC17_TMR1_BASE+LPC17_TMR_CTCR_OFFSET)

#define LPC17_TMR2_IR             (LPC17_TMR2_BASE+LPC17_TMR_IR_OFFSET)
#define LPC17_TMR2_TCR            (LPC17_TMR2_BASE+LPC17_TMR_TCR_OFFSET)
#define LPC17_TMR2_TC             (LPC17_TMR2_BASE+LPC17_TMR_TC_OFFSET)
#define LPC17_TMR2_PR             (LPC17_TMR2_BASE+LPC17_TMR_PR_OFFSET)
#define LPC17_TMR2_PC             (LPC17_TMR2_BASE+LPC17_TMR_PC_OFFSET)
#define LPC17_TMR2_MCR            (LPC17_TMR2_BASE+LPC17_TMR_MCR_OFFSET)
#define LPC17_TMR2_MR0            (LPC17_TMR2_BASE+LPC17_TMR_MR0_OFFSET)
#define LPC17_TMR2_MR1            (LPC17_TMR2_BASE+LPC17_TMR_MR1_OFFSET)
#define LPC17_TMR2_MR2            (LPC17_TMR2_BASE+LPC17_TMR_MR2_OFFSET)
#define LPC17_TMR2_MR3            (LPC17_TMR2_BASE+LPC17_TMR_MR3_OFFSET)
#define LPC17_TMR2_CCR            (LPC17_TMR2_BASE+LPC17_TMR_CCR_OFFSET)
#define LPC17_TMR2_CR0            (LPC17_TMR2_BASE+LPC17_TMR_CR0_OFFSET)
#define LPC17_TMR2_CR1            (LPC17_TMR2_BASE+LPC17_TMR_CR1_OFFSET)
#define LPC17_TMR2_EMR            (LPC17_TMR2_BASE+LPC17_TMR_EMR_OFFSET)
#define LPC17_TMR2_CTCR           (LPC17_TMR2_BASE+LPC17_TMR_CTCR_OFFSET)

#define LPC17_TMR3_IR             (LPC17_TMR3_BASE+LPC17_TMR_IR_OFFSET)
#define LPC17_TMR3_TCR            (LPC17_TMR3_BASE+LPC17_TMR_TCR_OFFSET)
#define LPC17_TMR3_TC             (LPC17_TMR3_BASE+LPC17_TMR_TC_OFFSET)
#define LPC17_TMR3_PR             (LPC17_TMR3_BASE+LPC17_TMR_PR_OFFSET)
#define LPC17_TMR3_PC             (LPC17_TMR3_BASE+LPC17_TMR_PC_OFFSET)
#define LPC17_TMR3_MCR            (LPC17_TMR3_BASE+LPC17_TMR_MCR_OFFSET)
#define LPC17_TMR3_MR0            (LPC17_TMR3_BASE+LPC17_TMR_MR0_OFFSET)
#define LPC17_TMR3_MR1            (LPC17_TMR3_BASE+LPC17_TMR_MR1_OFFSET)
#define LPC17_TMR3_MR2            (LPC17_TMR3_BASE+LPC17_TMR_MR2_OFFSET)
#define LPC17_TMR3_MR3            (LPC17_TMR3_BASE+LPC17_TMR_MR3_OFFSET)
#define LPC17_TMR3_CCR            (LPC17_TMR3_BASE+LPC17_TMR_CCR_OFFSET)
#define LPC17_TMR3_CR0            (LPC17_TMR3_BASE+LPC17_TMR_CR0_OFFSET)
#define LPC17_TMR3_CR1            (LPC17_TMR3_BASE+LPC17_TMR_CR1_OFFSET)
#define LPC17_TMR3_EMR            (LPC17_TMR3_BASE+LPC17_TMR_EMR_OFFSET)
#define LPC17_TMR3_CTCR           (LPC17_TMR3_BASE+LPC17_TMR_CTCR_OFFSET)

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
 *   Capture Register 0 (CR0)
 *   Capture Register 1 (CR1)
 */

/* Interrupt Register */

#define TMR_IR_MR0                (1 << 0)  /* Bit 0:  Match channel 0 interrupt */
#define TMR_IR_MR1                (1 << 1)  /* Bit 1:  Match channel 1 interrupt */
#define TMR_IR_MR2                (1 << 2)  /* Bit 2:  Match channel 2 interrupt */
#define TMR_IR_MR3                (1 << 3)  /* Bit 3:  Match channel 3 interrupt */
#define TMR_IR_CR0                (1 << 4)  /* Bit 4:  Capture channel 0 interrupt */
#define TMR_IR_CR1                (1 << 5)  /* Bit 5:  Capture channel 1 interrupt */
                                            /* Bits 6-31: Reserved */
/* Timer Control Register */

#define TMR_TCR_EN                (1 << 0)  /* Bit 0:  Counter Enable */
#define TMR_TCR_RESET             (1 << 1)  /* Bit 1:  Counter Reset */
                                            /* Bits 2-31: Reserved */
/* Match Control Register */

#define TMR_MCR_MR0I              (1 << 0)  /* Bit 0:  Interrupt on MR0 */
#define TMR_MCR_MR0R              (1 << 1)  /* Bit 1:  Reset on MR0 */
#define TMR_MCR_MR0S              (1 << 2)  /* Bit 2:  Stop on MR0 */
#define TMR_MCR_MR1I              (1 << 3)  /* Bit 3:  Interrupt on MR1 */
#define TMR_MCR_MR1R              (1 << 4)  /* Bit 4:  Reset on MR1 */
#define TMR_MCR_MR1S              (1 << 5)  /* Bit 5:  Stop on MR1 */
#define TMR_MCR_MR2I              (1 << 6)  /* Bit 6:  Interrupt on MR2 */
#define TMR_MCR_MR2R              (1 << 7)  /* Bit 7:  Reset on MR2 */
#define TMR_MCR_MR2S              (1 << 8)  /* Bit 8:  Stop on MR2 */
#define TMR_MCR_MR3I              (1 << 9)  /* Bit 9:  Interrupt on MR3 */
#define TMR_MCR_MR3R              (1 << 10) /* Bit 10: Reset on MR3 */
#define TMR_MCR_MR3S              (1 << 11) /* Bit 11: Stop on MR3 */
                                            /* Bits 12-31: Reserved */
/* Capture Control Register */

#define TMR_CCR_CAP0RE            (1 << 0)  /* Bit 0: Capture on CAPn.0 rising edge */
#define TMR_CCR_CAP0FE            (1 << 1)  /* Bit 1: Capture on CAPn.0 falling edge */
#define TMR_CCR_CAP0I             (1 << 2)  /* Bit 2: Interrupt on CAPn.0 */
#define TMR_CCR_CAP1RE            (1 << 3)  /* Bit 3: Capture on CAPn.1 rising edge */
#define TMR_CCR_CAP1FE            (1 << 4)  /* Bit 4: Capture on CAPn.1 falling edge */
#define TMR_CCR_CAP1I             (1 << 5)  /* Bit 5: Interrupt on CAPn.1 */
                                            /* Bits 6-31: Reserved */
/* External Match Register */

#define TMR_EMR_NOTHING           (0)       /* Do Nothing */
#define TMR_EMR_CLEAR             (1)       /* Clear external match bit MATn.m */
#define TMR_EMR_SET               (2)       /* Set external match bit MATn.m */
#define TMR_EMR_TOGGLE            (3)       /* Toggle external match bit MATn.m */

#define TMR_EMR_EM0               (1 << 0)  /* Bit 0:  External Match 0 */
#define TMR_EMR_EM1               (1 << 1)  /* Bit 1:  External Match 1 */
#define TMR_EMR_EM2               (1 << 2)  /* Bit 2:  External Match 2 */
#define TMR_EMR_EM3               (1 << 3)  /* Bit 3:  External Match 3 */
#define TMR_EMR_EMC0_SHIFT        (4)       /* Bits 4-5: External Match Control 0 */
#define TMR_EMR_EMC0_MASK         (3 << TMR_EMR_EMC0_SHIFTy)
#  define TMR_EMR_EMC0_NOTHING    (TMR_EMR_NOTHING << TMR_EMR_EMC0_SHIFT)
#  define TMR_EMR_EMC0_CLEAR      (TMR_EMR_CLEAR << TMR_EMR_EMC0_SHIFT)
#  define TMR_EMR_EMC0_SET        (TMR_EMR_SET << TMR_EMR_EMC0_SHIFT)
#  define TMR_EMR_EMC0_TOGGLE     (TMR_EMR_TOGGLE << TMR_EMR_EMC0_SHIFT)
#define TMR_EMR_EMC1_SHIFT        (6)       /* Bits 6-7: External Match Control 1 */
#define TMR_EMR_EMC1_MASK         (3 << TMR_EMR_EMC1_SHIFT)
#  define TMR_EMR_EMC1_NOTHING    (TMR_EMR_NOTHING << TMR_EMR_EMC1_SHIFT)
#  define TMR_EMR_EMC1_CLEAR      (TMR_EMR_CLEAR << TMR_EMR_EMC1_SHIFT)
#  define TMR_EMR_EMC1_SET        (TMR_EMR_SET << TMR_EMR_EMC1_SHIFT)
#  define TMR_EMR_EMC1_TOGGLE     (TMR_EMR_TOGGLE << TMR_EMR_EMC1_SHIFT)
#define TMR_EMR_EMC2_SHIFT        (8)       /* Bits 8-9: External Match Control 2 */
#define TMR_EMR_EMC2_MASK         (3 << TMR_EMR_EMC2_SHIFT)
#  define TMR_EMR_EMC2_NOTHING    (TMR_EMR_NOTHING << TMR_EMR_EMC2_SHIFT)
#  define TMR_EMR_EMC2_CLEAR      (TMR_EMR_CLEAR << TMR_EMR_EMC2_SHIFT)
#  define TMR_EMR_EMC2_SET        (TMR_EMR_SET << TMR_EMR_EMC2_SHIFT)
#  define TMR_EMR_EMC2_TOGGLE     (TMR_EMR_TOGGLE << TMR_EMR_EMC2_SHIFT)
#define TMR_EMR_EMC3_SHIFT        (10)      /* Bits 10-11: External Match Control 3 */
#define TMR_EMR_EMC3_MASK         (3 << TMR_EMR_EMC3_SHIFT)
#  define TMR_EMR_EMC3_NOTHING    (TMR_EMR_NOTHING << TMR_EMR_EMC3_SHIFT)
#  define TMR_EMR_EMC3_CLEAR      (TMR_EMR_CLEAR << TMR_EMR_EMC3_SHIFT)
#  define TMR_EMR_EMC3_SET        (TMR_EMR_SET << TMR_EMR_EMC3_SHIFT)
#  define TMR_EMR_EMC3_TOGGLE     (TMR_EMR_TOGGLE << TMR_EMR_EMC3_SHIFT)
                                            /* Bits 12-31: Reserved */
/* Count Control Register */

#define TMR_CTCR_MODE_SHIFT       (0)       /* Bits 0-1: Counter/Timer Mode */
#define TMR_CTCR_MODE_MASK        (3 << TMR_CTCR_MODE_SHIFT)
#  define TMR_CTCR_MODE_TIMER     (0 << TMR_CTCR_MODE_SHIFT) /* Timer Mode, prescale match */
#  define TMR_CTCR_MODE_CNTRRE    (1 << TMR_CTCR_MODE_SHIFT) /* Counter Mode, CAP rising edge */
#  define TMR_CTCR_MODE_CNTRFE    (2 << TMR_CTCR_MODE_SHIFT) /* Counter Mode, CAP falling edge */
#  define TMR_CTCR_MODE_CNTRBE    (3 << TMR_CTCR_MODE_SHIFT) /* Counter Mode, CAP both edges */
#define TMR_CTCR_INPSEL_SHIFT     (2)       /* Bits 2-3: Count Input Select */
#define TMR_CTCR_INPSEL_MASK      (3 << TMR_CTCR_INPSEL_SHIFT)
#  define TMR_CTCR_INPSEL_CAPNp0  (0 << TMR_CTCR_INPSEL_SHIFT) /* CAPn.0 for TIMERn */
#  define TMR_CTCR_INPSEL_CAPNp1  (1 << TMR_CTCR_INPSEL_SHIFT) /* CAPn.1 for TIMERn */
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

#endif /* __ARCH_ARM_SRC_LPC17XX_CHIP_LPC17_TIMER_H */
