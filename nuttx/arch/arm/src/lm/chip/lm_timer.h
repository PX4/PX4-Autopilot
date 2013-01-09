/************************************************************************************
 * arch/arm/src/lm/chip/lm_timer.h
 *
 *   Copyright (C) 2012 Max Nekludov. All rights reserved.
 *   Author: Max Nekludov <macscomp@gmail.com>
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

#ifndef __ARCH_ARM_SRC_LM_CHIP_LM_TIMER_H
#define __ARCH_ARM_SRC_LM_CHIP_LM_TIMER_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Timer register offsets ***********************************************************/

#define LM_TIMER_GPTMCFG_OFFSET          0x000
#define LM_TIMER_GPTMTAMR_OFFSET         0x004
#define LM_TIMER_GPTMCTL_OFFSET          0x00c
#define LM_TIMER_GPTMIMR_OFFSET          0x018
#define LM_TIMER_GPTMRIS_OFFSET          0x01c
#define LM_TIMER_GPTMICR_OFFSET          0x024
#define LM_TIMER_GPTMTAILR_OFFSET        0x028
#define LM_TIMER_GPTMTAR_OFFSET          0x048

/* SSI register addresses ***********************************************************/

#define LM_TIMER_BASE(n)                 (LM_TIMER0_BASE + (n)*0x01000)

#define LM_TIMER_GPTMCFG(n)              (LM_TIMER_BASE(n) + LM_TIMER_GPTMCFG_OFFSET)
#define LM_TIMER_GPTMTAMR(n)             (LM_TIMER_BASE(n) + LM_TIMER_GPTMTAMR_OFFSET)
#define LM_TIMER_GPTMCTL(n)              (LM_TIMER_BASE(n) + LM_TIMER_GPTMCTL_OFFSET)
#define LM_TIMER_GPTMIMR(n)              (LM_TIMER_BASE(n) + LM_TIMER_GPTMIMR_OFFSET)
#define LM_TIMER_GPTMRIS(n)              (LM_TIMER_BASE(n) + LM_TIMER_GPTMRIS_OFFSET)
#define LM_TIMER_GPTMICR(n)              (LM_TIMER_BASE(n) + LM_TIMER_GPTMICR_OFFSET)
#define LM_TIMER_GPTMTAILR(n)            (LM_TIMER_BASE(n) + LM_TIMER_GPTMTAILR_OFFSET)
#define LM_TIMER_GPTMTAR(n)              (LM_TIMER_BASE(n) + LM_TIMER_GPTMTAR_OFFSET)

/* SSI register bit defitiions ******************************************************/

/* GPTM Configuration (GPTMCFG), offset 0x000 */

#define TIMER_GPTMCFG_CFG_SHIFT          0    /* Bits 2-0: GPTM Configuration */
#define TIMER_GPTM_CFG_MASK              (0x07 << TIMER_GPTMCFG_CFG_SHIFT)
#define   TIMER_GPTMCFG_CFG_32           (0 << TIMER_GPTMCFG_CFG_SHIFT)       /* 32-bit timer configuration */
#define   TIMER_GPTMCFG_CFG_RTC          (1 << TIMER_GPTMCFG_CFG_SHIFT)       /* 32-bit real-time clock (RTC) counter configuration */
#define   TIMER_GPTMCFG_CFG_16           (1 << TIMER_GPTMCFG_CFG_SHIFT)       /* 16-bit timer configuration */

/* GPTM Timer A Mode (GPTMTAMR), offset 0x004 */

#define TIMER_GPTMTAMR_TAMR_SHIFT        0    /* Bits 1-0: GPTM Timer A Mode */
#define TIMER_GPTMTAMR_TAMR_MASK         (0x03 << TIMER_GPTMTAMR_TAMR_SHIFT)
#define   TIMER_GPTMTAMR_TAMR_ONESHOT    (1 << TIMER_GPTMTAMR_TAMR_SHIFT)     /* One-Shot Timer mode */
#define   TIMER_GPTMTAMR_TAMR_PERIODIC   (2 << TIMER_GPTMTAMR_TAMR_SHIFT)     /* Periodic Timer mode */
#define   TIMER_GPTMTAMR_TAMR_CAPTURE    (3 << TIMER_GPTMTAMR_TAMR_SHIFT)     /* Capture mode */
#define TIMER_GPTMTAMR_TACMR_SHIFT       2    /* Bits 2:   GPTM Timer A Capture Mode */
#define TIMER_GPTMTAMR_TACMR_MASK        (0x01 << TIMER_GPTMTAMR_TACMR_SHIFT)
#define   TIMER_GPTMTAMR_TACMR_EDGECOUNT (0 << TIMER_GPTMTAMR_TACMR_SHIFT)    /* Edge-Count mode */
#define   TIMER_GPTMTAMR_TACMR_EDGETIME  (1 << TIMER_GPTMTAMR_TACMR_SHIFT)    /* Edge-Time mode */
#define TIMER_GPTMTAMR_TAAMS_SHIFT       3    /* Bits 3:   GPTM Timer A Alternate Mode Select */
#define TIMER_GPTMTAMR_TAAMS_MASK        (0x01 << TIMER_GPTMTAMR_TAAMS_SHIFT)
#define   TIMER_GPTMTAMR_TAAMS_CAPTURE   (0 << TIMER_GPTMTAMR_TAAMS_SHIFT)    /* Capture mode is enabled */
#define   TIMER_GPTMTAMR_TAAMS_PWM       (1 << TIMER_GPTMTAMR_TAAMS_SHIFT)    /* PWM mode is enabled */
#define TIMER_GPTMTAMR_TACDIR_SHIFT      4    /* Bits 4:   GPTM Timer A Count Direction */
#define TIMER_GPTMTAMR_TACDIR_MASK       (0x01 << TIMER_GPTMTAMR_TACDIR_SHIFT)
#define   TIMER_GPTMTAMR_TACDIR_DOWN     (0 << TIMER_GPTMTAMR_TACDIR_SHIFT)   /* The timer counts down */
#define   TIMER_GPTMTAMR_TACDIR_UP       (1 << TIMER_GPTMTAMR_TACDIR_SHIFT)   /* When in one-shot or periodic mode, the timer counts up */
#define TIMER_GPTMTAMR_TAMIE_SHIFT       5    /* Bits 5:   GPTM Timer A Match Interrupt Enable */
#define TIMER_GPTMTAMR_TAMIE_MASK        (0x01 << TIMER_GPTMTAMR_TAMIE_SHIFT)

/* GPTM Control (GPTMCTL), offset 0x00C */

#define TIMER_GPTMCTL_TAEN_SHIFT         0    /* Bits 0:   GPTM Timer A Enable */
#define TIMER_GPTMCTL_TAEN_MASK          (0x01 << TIMER_GPTMCTL_TAEN_SHIFT)
#define TIMER_GPTMCTL_TASTALL_SHIFT      1    /* Bits 1:   GPTM Timer A Stall Enable */
#define TIMER_GPTMCTL_TASTALL_MASK       (0x01 << TIMER_GPTMCTL_TASTALL_SHIFT)

/* GPTM Interrupt Mask (GPTMIMR), offset 0x018 */

#define TIMER_GPTMIMR_TATOIM_SHIFT       0    /* Bits 0:   GPTM Timer A Time-Out Interrupt Mask */
#define TIMER_GPTMIMR_TATOIM_MASK        (0x01 << TIMER_GPTMIMR_TATOIM_SHIFT)

/* GPTM Raw Interrupt Status (GPTMRIS), offset 0x01C */

#define TIMER_GPTMRIS_TATORIS_SHIFT      0    /* Bits 0:   GPTM Timer A Time-Out Raw Interrupt */
#define TIMER_GPTMRIS_TATORIS_MASK       (0x01 << TIMER_GPTMRIS_TATORIS_SHIFT)

/* GPTM Interrupt Clear (GPTMICR), offset 0x024 */

#define TIMER_GPTMICR_TATOCINT_SHIFT     0    /* Bits 0:   GPTM Timer A Time-Out Raw Interrupt Clear*/
#define TIMER_GPTMICR_TATOCINT_MASK      (0x01 << TIMER_GPTMICR_TATOCINT_SHIFT)

#endif /* __ARCH_ARM_SRC_LM_CHIP_LM_TIMER_H */
