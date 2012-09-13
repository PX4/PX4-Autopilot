/********************************************************************************************
 * arch/arm/src/kinetis/kinetis_cmp.h
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
 ********************************************************************************************/

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_CMP_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_CMP_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

/* Register Offsets *************************************************************************/

#define KINESIS_CMP_OFFSET(n)     ((n) << 3)
#define KINESIS_CMP0_OFFSET       0x0000
#define KINESIS_CMP1_OFFSET       0x0008
#define KINESIS_CMP2_OFFSET       0x0010

#define KINETIS_CMP_CR0_OFFSET    0x0000 /* CMP Control Register 0 */
#define KINETIS_CMP_CR1_OFFSET    0x0001 /* CMP Control Register 1 */
#define KINETIS_CMP_FPR_OFFSET    0x0002 /* CMP Filter Period Register */
#define KINETIS_CMP_SCR_OFFSET    0x0003 /* CMP Status and Control Register */
#define KINETIS_CMP_DACCR_OFFSET  0x0004 /* DAC Control Register */
#define KINETIS_CMP_MUXCR_OFFSET  0x0005 /* MUX Control Register */

/* Register Addresses ***********************************************************************/

#define KINESIS_CMP_BASE(n)       (KINETIS_CMP_BASE+KINESIS_CMP_OFFSET(n))
#define KINESIS_CMP0_BASE         (KINETIS_CMP_BASE+KINESIS_CMP0_OFFSET)
#define KINESIS_CMP1_BASE         (KINETIS_CMP_BASE+KINESIS_CMP1_OFFSET)
#define KINESIS_CMP2_BASE         (KINETIS_CMP_BASE+KINESIS_CMP2_OFFSET)

#define KINETIS_CMP_CR0(n)        (KINESIS_CMP_BASE(n)+KINETIS_CMP_CR0_OFFSET)
#define KINETIS_CMP_CR1(n)        (KINESIS_CMP_BASE(n)+KINETIS_CMP_CR1_OFFSET)
#define KINETIS_CMP_FPR(n)        (KINESIS_CMP_BASE(n)+KINETIS_CMP_FPR_OFFSET)
#define KINETIS_CMP_SCR(n)        (KINESIS_CMP_BASE(n)+KINETIS_CMP_SCR_OFFSET)
#define KINETIS_CMP_DACCR(n)      (KINESIS_CMP_BASE(n)+KINETIS_CMP_DACCR_OFFSET)
#define KINETIS_CMP_MUXCR(n)      (KINESIS_CMP_BASE(n)+KINETIS_CMP_MUXCR_OFFSET)

#define KINETIS_CMP0_CR0          (KINETIS_CMP0_BASE+KINETIS_CMP_CR0_OFFSET)
#define KINETIS_CMP0_CR1          (KINETIS_CMP0_BASE+KINETIS_CMP_CR1_OFFSET)
#define KINETIS_CMP0_FPR          (KINETIS_CMP0_BASE+KINETIS_CMP_FPR_OFFSET)
#define KINETIS_CMP0_SCR          (KINETIS_CMP0_BASE+KINETIS_CMP_SCR_OFFSET)
#define KINETIS_CMP0_DACCR        (KINETIS_CMP0_BASE+KINETIS_CMP_DACCR_OFFSET)
#define KINETIS_CMP0_MUXCR        (KINETIS_CMP0_BASE+KINETIS_CMP_MUXCR_OFFSET)

#define KINETIS_CMP1_CR0          (KINETIS_CMP1_BASE+KINETIS_CMP_CR0_OFFSET)
#define KINETIS_CMP1_CR1          (KINETIS_CMP1_BASE+KINETIS_CMP_CR1_OFFSET)
#define KINETIS_CMP1_FPR          (KINETIS_CMP1_BASE+KINETIS_CMP_FPR_OFFSET)
#define KINETIS_CMP1_SCR          (KINETIS_CMP1_BASE+KINETIS_CMP_SCR_OFFSET)
#define KINETIS_CMP1_DACCR        (KINETIS_CMP1_BASE+KINETIS_CMP_DACCR_OFFSET)
#define KINETIS_CMP1_MUXCR        (KINETIS_CMP1_BASE+KINETIS_CMP_MUXCR_OFFSET)

#define KINETIS_CMP2_CR0          (KINETIS_CMP2_BASE+KINETIS_CMP_CR0_OFFSET)
#define KINETIS_CMP2_CR1          (KINETIS_CMP2_BASE+KINETIS_CMP_CR1_OFFSET)
#define KINETIS_CMP2_FPR          (KINETIS_CMP2_BASE+KINETIS_CMP_FPR_OFFSET)
#define KINETIS_CMP2_SCR          (KINETIS_CMP2_BASE+KINETIS_CMP_SCR_OFFSET)
#define KINETIS_CMP2_DACCR        (KINETIS_CMP2_BASE+KINETIS_CMP_DACCR_OFFSET)
#define KINETIS_CMP2_MUXCR        (KINETIS_CMP2_BASE+KINETIS_CMP_MUXCR_OFFSET)

/* Register Bit Definitions *****************************************************************/

/* CMP Control Register 0 (8-bit) */

#define CMP_CR0_HYSTCTR_SHIFT     (0)       /* Bits 0-1: Comparator hard block hysteresis control */
#define CMP_CR0_HYSTCTR_MASK      (3 << CMP_CR0_HYSTCTR_SHIFT)
#  define CMP_CR0_HYSTCTR_LVL0    (0 << CMP_CR0_HYSTCTR_SHIFT)
#  define CMP_CR0_HYSTCTR_LVL1    (1 << CMP_CR0_HYSTCTR_SHIFT)
#  define CMP_CR0_HYSTCTR_LVL2    (2 << CMP_CR0_HYSTCTR_SHIFT)
#  define CMP_CR0_HYSTCTR_LVL3    (3 << CMP_CR0_HYSTCTR_SHIFT)
                                            /* Bits 2-3: Reserved */
#define CMP_CR0_FILTER_CNT_SHIFT  (4)       /* Bits 4-6: Filter Sample Count */
#define CMP_CR0_FILTER_CNT_MASK   (7 << CMP_CR0_FILTER_CNT_SHIFT)
#  define CMP_CR0_FILTER_DISABLED (0 << CMP_CR0_FILTER_CNT_SHIFT) /* Filter is disabled */
#  define CMP_CR0_FILTER_CNT1     (1 << CMP_CR0_FILTER_CNT_SHIFT) /* 1 consecutive sample must agree */
#  define CMP_CR0_FILTER_CNT2     (2 << CMP_CR0_FILTER_CNT_SHIFT) /* 2 consecutive samples must agree */
#  define CMP_CR0_FILTER_CNT3     (3 << CMP_CR0_FILTER_CNT_SHIFT) /* 3 consecutive samples must agree */
#  define CMP_CR0_FILTER_CNT4     (4 << CMP_CR0_FILTER_CNT_SHIFT) /* 4 consecutive samples must agree */
#  define CMP_CR0_FILTER_CNT5     (5 << CMP_CR0_FILTER_CNT_SHIFT) /* 5 consecutive samples must agree */
#  define CMP_CR0_FILTER_CNT6     (6 << CMP_CR0_FILTER_CNT_SHIFT) /* 6 consecutive samples must agree */
#  define CMP_CR0_FILTER_CNT7     (7 << CMP_CR0_FILTER_CNT_SHIFT) /* 7 consecutive samples must agree */
                                            /* Bit 7:  Reserved */
/* CMP Control Register 1 (8-bit) */

#define CMP_CR1_EN                (1 << 0)  /* Bit 0:  Comparator Module Enable */
#define CMP_CR1_OPE               (1 << 1)  /* Bit 1:  Comparator Output Pin Enable */
#define CMP_CR1_COS               (1 << 2)  /* Bit 2:  Comparator Output Select */
#define CMP_CR1_INV               (1 << 3)  /* Bit 3:  Comparator INVERT */
#define CMP_CR1_PMODE             (1 << 4)  /* Bit 4:  Power Mode Select */
                                            /* Bit 5:  Reserved */
#define CMP_CR1_WE                (1 << 6)  /* Bit 6:  Windowing Enable */
#define CMP_CR1_SE                (1 << 7)  /* Bit 7:  Sample Enable */

/* CMP Filter Period Register (8-bit Filter Sample Period) */


/* CMP Status and Control Register (8-bit) */

#define CMP_SCR_COUT              (1 << 0)  /* Bit 0:  Analog Comparator Output */
#define CMP_SCR_CFF               (1 << 1)  /* Bit 1:  Analog Comparator Flag Falling */
#define CMP_SCR_CFR               (1 << 2)  /* Bit 2:  Analog Comparator Flag Rising */
#define CMP_SCR_IEF               (1 << 3)  /* Bit 3:  Comparator Interrupt Enable Falling */
#define CMP_SCR_IER               (1 << 4)  /* Bit 4:  Comparator Interrupt Enable Rising */
#define CMP_SCR_SMELB             (1 << 5)  /* Bit 5:  Stop Mode Edge/Level Interrupt Control */
#define CMP_SCR_DMAEN             (1 << 6)  /* Bit 6:  DMA Enable Control */
                                            /* Bit 7:  Reserved */
/* DAC Control Register (8-bit) */

#define CMP_DACCR_VOSEL_SHIFT     (0)       /* Bits 0-5: DAC Output Voltage Select */
#define CMP_DACCR_VOSEL_MASK      (0x3f << CMP_DACCR_VOSEL_SHIFT)
#define CMP_DACCR_VRSEL           (1 << 6)  /* Bit 6:  Supply Voltage Reference Source Select */
#define CMP_DACCR_DACEN           (1 << 7)  /* Bit 7:  DAC Enable */

/* MUX Control Register (8-bit) */

#define CMP_MUXCR_MSEL_SHIFT      (0)       /* Bits 0-2: Minus Input MUX Control */
#define CMP_MUXCR_MSEL_MASK       (7 << CMP_MUXCR_MSEL_SHIFT)
#  define CMP_MUXCR_MSEL_IN0      (0 << CMP_MUXCR_MSEL_SHIFT)
#  define CMP_MUXCR_MSEL_IN1      (1 << CMP_MUXCR_MSEL_SHIFT)
#  define CMP_MUXCR_MSEL_IN2      (2 << CMP_MUXCR_MSEL_SHIFT)
#  define CMP_MUXCR_MSEL_IN3      (3 << CMP_MUXCR_MSEL_SHIFT)
#  define CMP_MUXCR_MSEL_IN4      (4 << CMP_MUXCR_MSEL_SHIFT)
#  define CMP_MUXCR_MSEL_IN5      (5 << CMP_MUXCR_MSEL_SHIFT)
#  define CMP_MUXCR_MSEL_IN6      (6 << CMP_MUXCR_MSEL_SHIFT)
#  define CMP_MUXCR_MSEL_IN7      (7 << CMP_MUXCR_MSEL_SHIFT)
#define CMP_MUXCR_PSEL_SHIFT      (3)       /* Bits 3-5: Plus Input MUX Control */
#define CMP_MUXCR_PSEL_MASK       (7 << CMP_MUXCR_PSEL_SHIFT)
#  define CMP_MUXCR_PSEL_IN0      (0 << CMP_MUXCR_PSEL_SHIFT)
#  define CMP_MUXCR_PSEL_IN1      (1 << CMP_MUXCR_PSEL_SHIFT)
#  define CMP_MUXCR_PSEL_IN2      (2 << CMP_MUXCR_PSEL_SHIFT)
#  define CMP_MUXCR_PSEL_IN3      (3 << CMP_MUXCR_PSEL_SHIFT)
#  define CMP_MUXCR_PSEL_IN4      (4 << CMP_MUXCR_PSEL_SHIFT)
#  define CMP_MUXCR_PSEL_IN5      (5 << CMP_MUXCR_PSEL_SHIFT)
#  define CMP_MUXCR_PSEL_IN6      (6 << CMP_MUXCR_PSEL_SHIFT)
#  define CMP_MUXCR_PSEL_IN7      (7 << CMP_MUXCR_PSEL_SHIFT)
#define CMP_MUXCR_MEN             (1 << 6)  /* Bit 6:  MMUX Enable */
#define CMP_MUXCR_PEN             (1 << 7)  /* Bit 7:  PMUX Enable */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_KINETIS_CMP_H */
