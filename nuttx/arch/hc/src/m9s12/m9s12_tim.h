/************************************************************************************
 * arch/hc/src/m9s12/m9s12_tim.h (TIM16b4c v1)
 *
 *   Copyright (C) 2010-2011 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_HC_SRC_M9S12_M9S12_TIM_H
#define __ARCH_ARM_HC_SRC_M9S12_M9S12_TIM_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define HCS12_TIM_TIOS_OFFSET         0x0000 /* Timer Input Capture/Output Compare Select */
#define HCS12_TIM_CFORC_OFFSET        0x0001 /* Timer Compare Force Register */
#define HCS12_TIM_OC7M_OFFSET         0x0002 /* Output Compare 7 Mask Register */
#define HCS12_TIM_OC7D_OFFSET         0x0003 /* Output Compare 7 Data Register */
#define HCS12_TIM_TCNTHI2_OFFSET      0x0004 /* Timer Count Register */
#define HCS12_TIM_TCNTLO2_OFFSET      0x0005 /* Timer Count Register */
#define HCS12_TIM_TSCR1_OFFSET        0x0006 /* Timer System Control Register 1 */
#define HCS12_TIM_TTOV_OFFSET         0x0007 /* Timer Toggle Overflow Register */
#define HCS12_TIM_TCTL1_OFFSET        0x0008 /* Timer Control Register1 */
#define HCS12_TIM_TCTL3_OFFSET        0x000a /* Timer Control Register3 */
#define HCS12_TIM_TIE_OFFSET          0x000c /* Timer Interrupt Enable Register */
#define HCS12_TIM_TSCR2_OFFSET        0x000d /* Timer System Control Register 2 */
#define HCS12_TIM_TFLG1_OFFSET        0x000e /* Main Timer Interrupt Flag 1 */
#define HCS12_TIM_TFLG2_OFFSET        0x000f /* Main Timer Interrupt Flag 2 */
#define HCS12_TIM_TC4HI_OFFSET        0x0018 /* Timer Input Capture/Output Compare Register 4 */
#define HCS12_TIM_TC4LO_OFFSET        0x0019 /* Timer Input Capture/Output Compare Register 4 */
#define HCS12_TIM_TC5HI_OFFSET        0x001a /* Timer Input Capture/Output Compare Register 5 */
#define HCS12_TIM_TC5LO_OFFSET        0x001b /* Timer Input Capture/Output Compare Register 5 */
#define HCS12_TIM_TC6HI_OFFSET        0x001c /* Timer Input Capture/Output Compare Register 6 */
#define HCS12_TIM_TC6LO_OFFSET        0x001d /* Timer Input Capture/Output Compare Register 6 */
#define HCS12_TIM_TC7HI_OFFSET        0x001e /* Timer Input Capture/Output Compare Register 7 */
#define HCS12_TIM_TC7LO_OFFSET        0x001f /* Timer Input Capture/Output Compare Register 7 */
#define HCS12_TIM_PACTL_OFFSET        0x0020 /* 16-Bit Pulse Accumulator Control Register */
#define HCS12_TIM_PAFLG_OFFSET        0x0021 /* Pulse Accumulator Flag Register */
#define HCS12_TIM_PACNTHI_OFFSET      0x0022 /* Pulse Accumulator Count Register */
#define HCS12_TIM_PACNTLO_OFFSET      0x0023 /* Pulse Accumulator Count Register */
#define HCS12_TIM_TIMTST2_OFFSET      0x002d /* Timer Test Register */

/* Register Addresses ***************************************************************/

#define HCS12_TIM_TIOS                (HCS12_REG_BASE+HCS12_TIM_BASE+HCS12_TIM_TIOS_OFFSET)
#define HCS12_TIM_CFORC               (HCS12_REG_BASE+HCS12_TIM_BASE+HCS12_TIM_CFORC_OFFSET)
#define HCS12_TIM_OC7M                (HCS12_REG_BASE+HCS12_TIM_BASE+HCS12_TIM_OC7M_OFFSET)
#define HCS12_TIM_OC7D                (HCS12_REG_BASE+HCS12_TIM_BASE+HCS12_TIM_OC7D_OFFSET)
#define HCS12_TIM_TCNTHI2             (HCS12_REG_BASE+HCS12_TIM_BASE+HCS12_TIM_TCNTHI2_OFFSET)
#define HCS12_TIM_TCNTLO2             (HCS12_REG_BASE+HCS12_TIM_BASE+HCS12_TIM_TCNTLO2_OFFSET)
#define HCS12_TIM_TSCR1               (HCS12_REG_BASE+HCS12_TIM_BASE+HCS12_TIM_TSCR1_OFFSET)
#define HCS12_TIM_TTOV                (HCS12_REG_BASE+HCS12_TIM_BASE+HCS12_TIM_TTOV_OFFSET)
#define HCS12_TIM_TCTL1               (HCS12_REG_BASE+HCS12_TIM_BASE+HCS12_TIM_TCTL1_OFFSET)
#define HCS12_TIM_TCTL3               (HCS12_REG_BASE+HCS12_TIM_BASE+HCS12_TIM_TCTL3_OFFSET)
#define HCS12_TIM_TIE                 (HCS12_REG_BASE+HCS12_TIM_BASE+HCS12_TIM_TIE_OFFSET)
#define HCS12_TIM_TSCR2               (HCS12_REG_BASE+HCS12_TIM_BASE+HCS12_TIM_TSCR2_OFFSET)
#define HCS12_TIM_TFLG1               (HCS12_REG_BASE+HCS12_TIM_BASE+HCS12_TIM_TFLG1_OFFSET)
#define HCS12_TIM_TFLG2               (HCS12_REG_BASE+HCS12_TIM_BASE+HCS12_TIM_TFLG2_OFFSET)
#define HCS12_TIM_TC4HI               (HCS12_REG_BASE+HCS12_TIM_BASE+HCS12_TIM_TC4HI_OFFSET)
#define HCS12_TIM_TC4LO               (HCS12_REG_BASE+HCS12_TIM_BASE+HCS12_TIM_TC4LO_OFFSET)
#define HCS12_TIM_TC5HI               (HCS12_REG_BASE+HCS12_TIM_BASE+HCS12_TIM_TC5HI_OFFSET)
#define HCS12_TIM_TC5LO               (HCS12_REG_BASE+HCS12_TIM_BASE+HCS12_TIM_TC6HI_OFFSET)
#define HCS12_TIM_TC6HI               (HCS12_REG_BASE+HCS12_TIM_BASE+HCS12_TIM_TC6LO_OFFSET)
#define HCS12_TIM_TC6LO               (HCS12_REG_BASE+HCS12_TIM_BASE+HCS12_TIM_TC7HI_OFFSET)
#define HCS12_TIM_TC7HI               (HCS12_REG_BASE+HCS12_TIM_BASE+HCS12_TIM_TC7LO_OFFSET)
#define HCS12_TIM_TC7LO               (HCS12_REG_BASE+HCS12_TIM_BASE+HCS12_TIM_PACTL_OFFSET)
#define HCS12_TIM_PACTL               (HCS12_REG_BASE+HCS12_TIM_BASE+HCS12_TIM_PAFLG_OFFSET)
#define HCS12_TIM_PAFLG               (HCS12_REG_BASE+HCS12_TIM_BASE+HCS12_TIM_PACNTHI_OFFSET)
#define HCS12_TIM_PACNTHI             (HCS12_REG_BASE+HCS12_TIM_BASE+HCS12_TIM_PACNTLO_OFFSET)
#define HCS12_TIM_PACNTLO             (HCS12_REG_BASE+HCS12_TIM_BASE+HCS12_TIM_TIMTST2_OFFSET)
#define HCS12_TIM_TIMTST2             (HCS12_REG_BASE+HCS12_TIM_BASE+HCS12_TIM_TIMTST2_OFFSET)

/* Register Bit-Field Definitions ***************************************************/

/* Timer Input Capture/Output Compare Select Bit-Field Definitions */

#define TIM_TIOS(n)                   (1 << (n)) /* 0:Input capture, 1:Output compare */

/* Timer Compare Force Register Bit-Field Definitions */

#define TIM_CFORC(n)                  (1 << (n)) /* Force Output Compare Action */

/* Output Compare 7 Mask Register Bit-Field Definitions */

#define TIM_OC7M(n)                   (1 << (n)) /* Output Compare 7 Mask */

/* Output Compare 7 Data Register Bit-Field Definitions */

#define TIM_OC7D(n)                   (1 << (n)) /* Output Compare 7 Data */

/* Timer Count HI/LO Register Bit-Field Definitions */
/* These two registers form a 16-bit timer up counter and have no internal bit-fields */

/* Timer System Control Register 1 Bit-Field Definitions */

#define TIM_TSCR1_TFFCA               (1 << 4)   /* Timer Fast Flag Clear All */
#define TIM_TSCR1_TSFRA               (1 << 5)   /* Timer Stops While in Freeze Mode */
#define TIM_TSCR1_TSWAI               (1 << 6)   /* Timer Module Stops While in Wait */
#define TIM_TSCR1_TEN                 (1 << 7)   /* Timer Enable */

/* Timer Toggle Overflow Register Bit-Field Definitions */

#define TIM_TTOV(n)                   (1 << (n)) /* Toggle On Overflow Bits

/* Timer Control Register1 Bit-Field Definitions */

#define TIM_TCTL1_SHIFT(n)           (((n)-4) << 1)
#define TIM_TCTL1_MASK (n)           (3 << TIM_TCTL1_SHIFT(n))

#  define TIM_TCTL1_OL(n)            (1 << TIM_TCTL1_SHIFT(n)) /* Output Level */
#  define TIM_TCTL1_OM(n)            (2 << TIM_TCTL1_SHIFT(n)) /* Output Mode */

#  define TIM_TCTL1_DISABLED(n)      (0 << TIM_TCTL1_SHIFT(n)) /* Timer disconnected from output pin logic */
#  define TIM_TCTL1_TOGGLE(n)        (1 << TIM_TCTL1_SHIFT(n)) /* Toggle OCx output line */
#  define TIM_TCTL1_CLEAR(n)         (2 << TIM_TCTL1_SHIFT(n)) /* Clear OCx output line to zero */
#  define TIM_TCTL1_SET(n)           (3 << TIM_TCTL1_SHIFT(n)) /* Set OCx output line to one */

/* Timer Control Register3 Bit-Field Definitions */

#define TIM_TCTL3_EDG_SHIFT(n)       (((n)-4) << 1) /* Input Capture Edge Control */
#define TIM_TCTL3_EDG_MASK (n)       (3 << TIM_TCTL3_EDG_SHIFT(n))

#  define TIM_TCTL3_EDGA(n)          (1 << TIM_TCTL3_EDG_SHIFT(n))
#  define TIM_TCTL3_EDGB(n)          (2 << TIM_TCTL3_EDG_SHIFT(n))

#  define TIM_TCTL3_DISABLED(n)      (0 << TIM_TCTL3_EDG_SHIFT(n)) /* Capture disabled */
#  define TIM_TCTL3_RISING(n)        (1 << TIM_TCTL3_EDG_SHIFT(n)) /* Capture on rising edges only */
#  define TIM_TCTL3_FALLING(n)       (2 << TIM_TCTL3_EDG_SHIFT(n)) /* Capture on falling edges only */
#  define TIM_TCTL3_BOTH(n)          (3 << TIM_TCTL3_EDG_SHIFT(n)) /* Capture on any edge */

/* Timer Interrupt Enable Register Bit-Field Definitions */

#define TIM_TIE(n)                   (1 << (n)) /* Input Capture/Output Compare n Interrupt Enable */

/* Timer System Control Register 2 Bit-Field Definitions */

#define TIM_TSCR2_PR_SHIFT           (0)        /* Timer Prescaler Select */
#define TIM_TSCR2_PR_MASK            (7 << TIM_TSCR2_PR_SHIFT)
#  define TIM_TSCR2_PR0              (1 << TIM_TSCR2_PR_SHIFT)
#  define TIM_TSCR2_PR1              (2 << TIM_TSCR2_PR_SHIFT)
#  define TIM_TSCR2_PR2              (4 << TIM_TSCR2_PR_SHIFT)
#  define TIM_TSCR2_PR_DIV1          (0 << TIM_TSCR2_PR_SHIFT) /* Bus Clock/1 */
#  define TIM_TSCR2_PR_DIV2          (1 << TIM_TSCR2_PR_SHIFT) /* Bus Clock/2 */
#  define TIM_TSCR2_PR_DIV4          (2 << TIM_TSCR2_PR_SHIFT) /* Bus Clock/4 */
#  define TIM_TSCR2_PR_DIV8          (3 << TIM_TSCR2_PR_SHIFT) /* Bus Clock/8 */
#  define TIM_TSCR2_PR_DIV16         (4 << TIM_TSCR2_PR_SHIFT) /* Bus Clock/16 */
#  define TIM_TSCR2_PR_DIV32         (5 << TIM_TSCR2_PR_SHIFT) /* Bus Clock/32 */
#  define TIM_TSCR2_PR_DIV64         (6 << TIM_TSCR2_PR_SHIFT) /* Bus Clock/64 */
#  define TIM_TSCR2_PR_DIV128        (7 << TIM_TSCR2_PR_SHIFT) /* Bus Clock/128 */
#define TIM_TSCR2_TCRE               (1 << 3)  /* Timer Counter Reset Enable */
#define TIM_TSCR2_TOI                (1 << 7)  /* Timer Overflow Interrupt Enable */

/* Main Timer Interrupt Flag 1 Bit-Field Definitions */

#define TIM_TFLG1(n)                 (1 << (n)) /* Input Capture/Output Compare Channel n Flag */

/* Main Timer Interrupt Flag 2 Bit-Field Definitions */

#define TIM_TFLG2_TOF                (1 << 7)   /* Timer Overflow Flag */

/* Timer Input Capture/Output Compare HI/LO Register 4-7 Bit-Field Definitions */
/* These register pairs form a 16-bit timer compare values and have no internal bit-fields */

/* 16-Bit Pulse Accumulator Control Register Bit-Field Definitions */

#define TIM_PACTL_PAI                (1 << 0)   /* Pulse Accumulator Input Interrupt Enable*/
#define TIM_PACTL_PAOVI              (1 << 1)   /* Pulse Accumulator Overflow Interrupt Enable */
#define TIM_PACTL_CLK_SHIFT          (2)        /* Clock Select Bits */
#define TIM_PACTL_CLK_MASK           (3 << TIM_PACTL_CLK_SHIFT)
#  define TIM_PACTL_CLK0             (1 << TIM_PACTL_CLK_SHIFT)
#  define TIM_PACTL_CLK1             (1 << TIM_PACTL_CLK_SHIFT)
#  define TIM_PACTL_PRESCAL          (0 << TIM_PACTL_CLK_SHIFT) /* Use timer prescaler clock as timer counter clock */
#  define TIM_PACTL_PACLK            (1 << TIM_PACTL_CLK_SHIFT) /* Use PACLK as input to timer counter clock */
#  define TIM_PACTL_DIV256           (2 << TIM_PACTL_CLK_SHIFT) /* Use PACLK/256 as timer counter clock frequency */
#  define TIM_PACTL_DIV64K           (3 << TIM_PACTL_CLK_SHIFT) /* Use PACLK/65536 as timer counter clock frequency */
#define TIM_PACTL_PIN_SHIFT          (4)          /* Pin action */
#define TIM_PACTL_PIN_MASK           (3 << TIM_PACTL_PIN_SHIFT)
#  define TIM_PACTL_PEDGE            (1 << TIM_PACTL_PIN_SHIFT) /* Pulse Accumulator Edge Control */
#  define TIM_PACTL_PAMOD            (2 << TIM_PACTL_PIN_SHIFT) /* Pulse Accumulator Mode */
#  define TIM_PACTL_FALLING          (0 << TIM_PACTL_PIN_SHIFT) /* Falling edge */
#  define TIM_PACTL_RISING           (1 << TIM_PACTL_PIN_SHIFT) /* Rising edge */
#  define TIM_PACTL_DIV64HI          (2 << TIM_PACTL_PIN_SHIFT) /* Div. by 64 clock enabled with pin high level */
#  define TIM_PACTL_DIV64LO          (3 << TIM_PACTL_PIN_SHIFT) /* Div. by 64 clock enabled with pin low level */
#define TIM_PACTL_PAEN               (1 << 6)   /* Pulse Accumulator System Enable */

/* Pulse Accumulator Flag Register Bit-Field Definitions */

#define TIM_PAFLG_PAIF               (1 << 0)   /* Pulse Accumulator Input edge Flag */
#define TIM_PAFLG_PAOVF              (1 << 1)   /* Pulse Accumulator Overflow Flag */

/* Pulse Accumulator Count HI/LO Register Bit-Field Definitions */
/* This register pair forms a 16-bit pulse accumulator value with no internal bit-fields */

/* Timer Test Register Bit-Field Definitions */
/* Not documented */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_HC_SRC_M9S12_M9S12_TIM_H */
