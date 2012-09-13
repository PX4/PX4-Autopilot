/************************************************************************************
 * arch/arm/src/str71x/str71x_timer.h
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_STR71X_STR71X_TIMER_H
#define __ARCH_ARM_SRC_STR71X_STR71X_TIMER_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "str71x_map.h"

/************************************************************************************
 * Pre-procesor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/

#define STR71X_TIMER_ICAR_OFFSET    (0x0000)                  /* 16-bits wide */
#define STR71X_TIMER_ICBR_OFFSET    (0x0004)                  /* 16-bits wide */
#define STR71X_TIMER_OCAR_OFFSET    (0x0008)                  /* 16-bits wide */
#define STR71X_TIMER_OCBR_OFFSET    (0x000c)                  /* 16-bits wide */
#define STR71X_TIMER_CNTR_OFFSET    (0x0010)                  /* 16-bits wide */
#define STR71X_TIMER_CR1_OFFSET     (0x0014)                  /* 16-bits wide */
#define STR71X_TIMER_CR2_OFFSET     (0x0018)                  /* 16-bits wide */
#define STR71X_TIMER_SR_OFFSET      (0x001c)                  /* 16-bits wide */

/* Register Addresses ***************************************************************/

#define STR71X_TIMER_ICAR(b)        ((b) + STR71X_TIMER_ICAR_OFFSET)
#define STR71X_TIMER_ICBR(b)        ((b) + STR71X_TIMER_ICBR_OFFSET)
#define STR71X_TIMER_OCAR(b)        ((b) + STR71X_TIMER_OCAR_OFFSET)
#define STR71X_TIMER_OCBR(b)        ((b) + STR71X_TIMER_OCBR_OFFSET)
#define STR71X_TIMER_CNTR(b)        ((b) + STR71X_TIMER_CNTR_OFFSET)
#define STR71X_TIMER_CR1(b)         ((b) + STR71X_TIMER_CR1_OFFSET)
#define STR71X_TIMER_CR2(b)         ((b) + STR71X_TIMER_CR2_OFFSET)
#define STR71X_TIMER_SR(b)          ((b) + STR71X_TIMER_SR_OFFSET)

#define STR71X_TIMER0_ICAR          (STR71X_TIMER0_BASE + STR71X_TIMER_ICAR_OFFSET)
#define STR71X_TIMER0_ICBR          (STR71X_TIMER0_BASE + STR71X_TIMER_ICBR_OFFSET)
#define STR71X_TIMER0_OCAR          (STR71X_TIMER0_BASE + STR71X_TIMER_OCAR_OFFSET)
#define STR71X_TIMER0_OCBR          (STR71X_TIMER0_BASE + STR71X_TIMER_OCBR_OFFSET)
#define STR71X_TIMER0_CNTR          (STR71X_TIMER0_BASE + STR71X_TIMER_CNTR_OFFSET)
#define STR71X_TIMER0_CR1           (STR71X_TIMER0_BASE + STR71X_TIMER_CR1_OFFSET)
#define STR71X_TIMER0_CR2           (STR71X_TIMER0_BASE + STR71X_TIMER_CR2_OFFSET)
#define STR71X_TIMER0_SR            (STR71X_TIMER0_BASE + STR71X_TIMER_SR_OFFSET)

#define STR71X_TIMER1_ICAR          (STR71X_TIMER1_BASE + STR71X_TIMER_ICAR_OFFSET)
#define STR71X_TIMER1_ICBR          (STR71X_TIMER1_BASE + STR71X_TIMER_ICBR_OFFSET)
#define STR71X_TIMER1_OCAR          (STR71X_TIMER1_BASE + STR71X_TIMER_OCAR_OFFSET)
#define STR71X_TIMER1_OCBR          (STR71X_TIMER1_BASE + STR71X_TIMER_OCBR_OFFSET)
#define STR71X_TIMER1_CNTR          (STR71X_TIMER1_BASE + STR71X_TIMER_CNTR_OFFSET)
#define STR71X_TIMER1_CR1           (STR71X_TIMER1_BASE + STR71X_TIMER_CR1_OFFSET)
#define STR71X_TIMER1_CR2           (STR71X_TIMER1_BASE + STR71X_TIMER_CR2_OFFSET)
#define STR71X_TIMER1_SR            (STR71X_TIMER1_BASE + STR71X_TIMER_SR_OFFSET)

#define STR71X_TIMER2_ICAR          (STR71X_TIMER2_BASE + STR71X_TIMER_ICAR_OFFSET)
#define STR71X_TIMER2_ICBR          (STR71X_TIMER2_BASE + STR71X_TIMER_ICBR_OFFSET)
#define STR71X_TIMER2_OCAR          (STR71X_TIMER2_BASE + STR71X_TIMER_OCAR_OFFSET)
#define STR71X_TIMER2_OCBR          (STR71X_TIMER2_BASE + STR71X_TIMER_OCBR_OFFSET)
#define STR71X_TIMER2_CNTR          (STR71X_TIMER2_BASE + STR71X_TIMER_CNTR_OFFSET)
#define STR71X_TIMER2_CR1           (STR71X_TIMER2_BASE + STR71X_TIMER_CR1_OFFSET)
#define STR71X_TIMER2_CR2           (STR71X_TIMER2_BASE + STR71X_TIMER_CR2_OFFSET)
#define STR71X_TIMER2_SR            (STR71X_TIMER2_BASE + STR71X_TIMER_SR_OFFSET)

#define STR71X_TIMER3_ICAR          (STR71X_TIMER3_BASE + STR71X_TIMER_ICAR_OFFSET)
#define STR71X_TIMER3_ICBR          (STR71X_TIMER3_BASE + STR71X_TIMER_ICBR_OFFSET)
#define STR71X_TIMER3_OCAR          (STR71X_TIMER3_BASE + STR71X_TIMER_OCAR_OFFSET)
#define STR71X_TIMER3_OCBR          (STR71X_TIMER3_BASE + STR71X_TIMER_OCBR_OFFSET)
#define STR71X_TIMER3_CNTR          (STR71X_TIMER3_BASE + STR71X_TIMER_CNTR_OFFSET)
#define STR71X_TIMER3_CR1           (STR71X_TIMER3_BASE + STR71X_TIMER_CR1_OFFSET)
#define STR71X_TIMER3_CR2           (STR71X_TIMER3_BASE + STR71X_TIMER_CR2_OFFSET)
#define STR71X_TIMER3_SR            (STR71X_TIMER3_BASE + STR71X_TIMER_SR_OFFSET)

/* Register bit settings ***********************************************************/

/* Timer control register (CR1 and CR2) */

#define STR71X_TIMERCR1_ECKEN       (0x0001) /* Bit 0:  External clock enable */
#define STR71X_TIMERCR1_EXEDG       (0x0002) /* Bit 1:  External clock edge */
#define STR71X_TIMERCR1_IEDGA       (0x0004) /* Bit 2:  Input edge A */
#define STR71X_TIMERCR1_IEDGB       (0x0008) /* Bit 3:  Input edge B */
#define STR71X_TIMERCR1_PWM         (0x0010) /* Bit 4:  Pulse width modulation */
#define STR71X_TIMERCR1_OPM         (0x0020) /* Bit 5:  One pulse mode */
#define STR71X_TIMERCR1_OCAE        (0x0040) /* Bit 6:  Output compare A enable */
#define STR71X_TIMERCR1_OCBE        (0x0080) /* Bit 7:  Output compare B enable */
#define STR71X_TIMERCR1_OLVLA       (0x0100) /* Bit 8:  Output level A */
#define STR71X_TIMERCR1_OLVLB       (0x0200) /* Bit 9:  Output level B */
#define STR71X_TIMERCR1_FOLVA       (0x0400) /* Bit 10: Forced output compare A */
#define STR71X_TIMERCR1_FOLVB       (0x0800) /* Bit 11: Forced output compare B */
#define STR71X_TIMERCR1_PWMI        (0x4000) /* Bit 14: Pulse width modulation input */
#define STR71X_TIMERCR1_EN          (0x8000) /* Bit 15: Timer count enable */

#define STR71X_TIMERCR2_DIVMASK     (0x00ff) /* Bits 0-7: Timer prescaler value */
#define STR71X_TIMERCR2_OCBIE       (0x0800) /* Bit 11: Output capture B enable */
#define STR71X_TIMERCR2_ICBIE       (0x1000) /* Bit 12: Input capture B enable */
#define STR71X_TIMERCR2_TOIE        (0x2000) /* Bit 13: Timer overflow enable */
#define STR71X_TIMERCR2_OCAIE       (0x4000) /* Bit 14: Output capture A enable */
#define STR71X_TIMERCR2_ICAIE       (0x8000) /* Bit 15: Input capture B enable */

/* Timer status register (SR) */

#define STR71X_TIMERSR_OCFB         (0x0800) /* Bit 11: Output capture flag B */
#define STR71X_TIMERSR_ICFB         (0x1000) /* Bit 12: Input capture flag B */
#define STR71X_TIMERSR_TOF          (0x2000) /* Bit 13: Timer overflow */
#define STR71X_TIMERSR_OCFA         (0x4000) /* Bit 14: Output capture flag A */
#define STR71X_TIMERSR_ICFA         (0x8000) /* Bit 15: Input capture flag A */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_STR71X_STR71X_TIMER_H */
