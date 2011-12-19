/************************************************************************************
 * arch/mips/src/pic32mx/pic32mx-timer.h
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

#ifndef __ARCH_MIPS_SRC_PIC32MX_PIC32MX_TIMER_H
#define __ARCH_MIPS_SRC_PIC32MX_PIC32MX_TIMER_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "pic32mx-memorymap.h"

#if CHIP_NTIMERS > 0

/************************************************************************************
 * Pre-Processor Definitions
 ************************************************************************************/
/* Register Offsets *****************************************************************/

#define PIC32MX_TIMER_CON_OFFSET    0x0000 /* Timer control register */
#define PIC32MX_TIMER_CONCLR_OFFSET 0x0004 /* Timer control clear register */
#define PIC32MX_TIMER_CONSET_OFFSET 0x0008 /* Timer control set register */
#define PIC32MX_TIMER_CONINV_OFFSET 0x000c /* Timer control invert register */
#define PIC32MX_TIMER_CNT_OFFSET    0x0010 /* Timer count register */
#define PIC32MX_TIMER_CNTCLR_OFFSET 0x0014 /* Timer count clear register */
#define PIC32MX_TIMER_CNTSET_OFFSET 0x0018 /* Timer count set register */
#define PIC32MX_TIMER_CNTINV_OFFSET 0x001c /* Timer count invert register */
#define PIC32MX_TIMER_PR_OFFSET     0x0020 /* Timer period register */
#define PIC32MX_TIMER_PRCLR_OFFSET  0x0024 /* Timer period clear register */
#define PIC32MX_TIMER_PRSET_OFFSET  0x0028 /* Timer period set register */
#define PIC32MX_TIMER_PRINV_OFFSET  0x002c /* Timer period invert register */

/* Register Addresses ***************************************************************/

#define PIC32MX_TIMER_CON(n)        (PIC32MX_TIMER_K1BASE(n)+PIC32MX_TIMER_CON_OFFSET)
#define PIC32MX_TIMER_CONCLR(n)     (PIC32MX_TIMER_K1BASE(n)+PIC32MX_TIMER_CONCLR_OFFSET)
#define PIC32MX_TIMER_CONSET(n)     (PIC32MX_TIMER_K1BASE(n)+PIC32MX_TIMER_CONSET_OFFSET)
#define PIC32MX_TIMER_CONINV(n)     (PIC32MX_TIMER_K1BASE(n)+PIC32MX_TIMER_CONINV_OFFSET)
#define PIC32MX_TIMER_CNT(n)        (PIC32MX_TIMER_K1BASE(n)+PIC32MX_TIMER_CNT_OFFSET)
#define PIC32MX_TIMER_CNTCLR(n)     (PIC32MX_TIMER_K1BASE(n)+PIC32MX_TIMER_CNTCLR_OFFSET)
#define PIC32MX_TIMER_CNTSET(n)     (PIC32MX_TIMER_K1BASE(n)+PIC32MX_TIMER_CNTSET_OFFSET)
#define PIC32MX_TIMER_CNTINV(n)     (PIC32MX_TIMER_K1BASE(n)+PIC32MX_TIMER_CNTINV_OFFSET)
#define PIC32MX_TIMER_PR(n)         (PIC32MX_TIMER_K1BASE(n)+PIC32MX_TIMER_PR_OFFSET)
#define PIC32MX_TIMER_PRCLR(n)      (PIC32MX_TIMER_K1BASE(n)+PIC32MX_TIMER_PRCLR_OFFSET)
#define PIC32MX_TIMER_PRSET(n)      (PIC32MX_TIMER_K1BASE(n)+PIC32MX_TIMER_PRSET_OFFSET)
#define PIC32MX_TIMER_PRINV(n)      (PIC32MX_TIMER_K1BASE(n)+PIC32MX_TIMER_PRINV_OFFSET)

#define PIC32MX_TIMER1_CON          (PIC32MX_TIMER1_K1BASE+PIC32MX_TIMER_CON_OFFSET)
#define PIC32MX_TIMER1_CONCLR       (PIC32MX_TIMER1_K1BASE+PIC32MX_TIMER_CONCLR_OFFSET)
#define PIC32MX_TIMER1_CONSET       (PIC32MX_TIMER1_K1BASE+PIC32MX_TIMER_CONSET_OFFSET)
#define PIC32MX_TIMER1_CONINV       (PIC32MX_TIMER1_K1BASE+PIC32MX_TIMER_CONINV_OFFSET)
#define PIC32MX_TIMER1_CNT          (PIC32MX_TIMER1_K1BASE+PIC32MX_TIMER_CNT_OFFSET)
#define PIC32MX_TIMER1_CNTCLR       (PIC32MX_TIMER1_K1BASE+PIC32MX_TIMER_CNTCLR_OFFSET)
#define PIC32MX_TIMER1_CNTSET       (PIC32MX_TIMER1_K1BASE+PIC32MX_TIMER_CNTSET_OFFSET)
#define PIC32MX_TIMER1_CNTINV       (PIC32MX_TIMER1_K1BASE+PIC32MX_TIMER_CNTINV_OFFSET)
#define PIC32MX_TIMER1_PR           (PIC32MX_TIMER1_K1BASE+PIC32MX_TIMER_PR_OFFSET)
#define PIC32MX_TIMER1_PRCLR        (PIC32MX_TIMER1_K1BASE+PIC32MX_TIMER_PRCLR_OFFSET)
#define PIC32MX_TIMER1_PRSET        (PIC32MX_TIMER1_K1BASE+PIC32MX_TIMER_PRSET_OFFSET)
#define PIC32MX_TIMER1_PRINV        (PIC32MX_TIMER1_K1BASE+PIC32MX_TIMER_PRINV_OFFSET)

#if CHIP_NTIMERS > 1
#  define PIC32MX_TIMER2_CON        (PIC32MX_TIMER2_K1BASE+PIC32MX_TIMER_CON_OFFSET)
#  define PIC32MX_TIMER2_CONCLR     (PIC32MX_TIMER2_K1BASE+PIC32MX_TIMER_CONCLR_OFFSET)
#  define PIC32MX_TIMER2_CONSET     (PIC32MX_TIMER2_K1BASE+PIC32MX_TIMER_CONSET_OFFSET)
#  define PIC32MX_TIMER2_CONINV     (PIC32MX_TIMER2_K1BASE+PIC32MX_TIMER_CONINV_OFFSET)
#  define PIC32MX_TIMER2_CNT        (PIC32MX_TIMER2_K1BASE+PIC32MX_TIMER_CNT_OFFSET)
#  define PIC32MX_TIMER2_CNTCLR     (PIC32MX_TIMER2_K1BASE+PIC32MX_TIMER_CNTCLR_OFFSET)
#  define PIC32MX_TIMER2_CNTSET     (PIC32MX_TIMER2_K1BASE+PIC32MX_TIMER_CNTSET_OFFSET)
#  define PIC32MX_TIMER2_CNTINV     (PIC32MX_TIMER2_K1BASE+PIC32MX_TIMER_CNTINV_OFFSET)
#  define PIC32MX_TIMER2_PR         (PIC32MX_TIMER2_K1BASE+PIC32MX_TIMER_PR_OFFSET)
#  define PIC32MX_TIMER2_PRCLR      (PIC32MX_TIMER2_K1BASE+PIC32MX_TIMER_PRCLR_OFFSET)
#  define PIC32MX_TIMER2_PRSET      (PIC32MX_TIMER2_K1BASE+PIC32MX_TIMER_PRSET_OFFSET)
#  define PIC32MX_TIMER2_PRINV      (PIC32MX_TIMER2_K1BASE+PIC32MX_TIMER_PRINV_OFFSET)
#endif

#if CHIP_NTIMERS > 2
#  define PIC32MX_TIMER3_CON        (PIC32MX_TIMER3_K1BASE+PIC32MX_TIMER_CON_OFFSET)
#  define PIC32MX_TIMER3_CONCLR     (PIC32MX_TIMER3_K1BASE+PIC32MX_TIMER_CONCLR_OFFSET)
#  define PIC32MX_TIMER3_CONSET     (PIC32MX_TIMER3_K1BASE+PIC32MX_TIMER_CONSET_OFFSET)
#  define PIC32MX_TIMER3_CONINV     (PIC32MX_TIMER3_K1BASE+PIC32MX_TIMER_CONINV_OFFSET)
#  define PIC32MX_TIMER3_CNT        (PIC32MX_TIMER3_K1BASE+PIC32MX_TIMER_CNT_OFFSET)
#  define PIC32MX_TIMER3_CNTCLR     (PIC32MX_TIMER3_K1BASE+PIC32MX_TIMER_CNTCLR_OFFSET)
#  define PIC32MX_TIMER3_CNTSET     (PIC32MX_TIMER3_K1BASE+PIC32MX_TIMER_CNTSET_OFFSET)
#  define PIC32MX_TIMER3_CNTINV     (PIC32MX_TIMER3_K1BASE+PIC32MX_TIMER_CNTINV_OFFSET)
#  define PIC32MX_TIMER3_PR         (PIC32MX_TIMER3_K1BASE+PIC32MX_TIMER_PR_OFFSET)
#  define PIC32MX_TIMER3_PRCLR      (PIC32MX_TIMER3_K1BASE+PIC32MX_TIMER_PRCLR_OFFSET)
#  define PIC32MX_TIMER3_PRSET      (PIC32MX_TIMER3_K1BASE+PIC32MX_TIMER_PRSET_OFFSET)
#  define PIC32MX_TIMER3_PRINV      (PIC32MX_TIMER3_K1BASE+PIC32MX_TIMER_PRINV_OFFSET)
#endif

#if CHIP_NTIMERS > 3
#  define PIC32MX_TIMER4_CON        (PIC32MX_TIMER4_K1BASE+PIC32MX_TIMER_CON_OFFSET)
#  define PIC32MX_TIMER4_CONCLR     (PIC32MX_TIMER4_K1BASE+PIC32MX_TIMER_CONCLR_OFFSET)
#  define PIC32MX_TIMER4_CONSET     (PIC32MX_TIMER4_K1BASE+PIC32MX_TIMER_CONSET_OFFSET)
#  define PIC32MX_TIMER4_CONINV     (PIC32MX_TIMER4_K1BASE+PIC32MX_TIMER_CONINV_OFFSET)
#  define PIC32MX_TIMER4_CNT        (PIC32MX_TIMER4_K1BASE+PIC32MX_TIMER_CNT_OFFSET)
#  define PIC32MX_TIMER4_CNTCLR     (PIC32MX_TIMER4_K1BASE+PIC32MX_TIMER_CNTCLR_OFFSET)
#  define PIC32MX_TIMER4_CNTSET     (PIC32MX_TIMER4_K1BASE+PIC32MX_TIMER_CNTSET_OFFSET)
#  define PIC32MX_TIMER4_CNTINV     (PIC32MX_TIMER4_K1BASE+PIC32MX_TIMER_CNTINV_OFFSET)
#  define PIC32MX_TIMER4_PR         (PIC32MX_TIMER4_K1BASE+PIC32MX_TIMER_PR_OFFSET)
#  define PIC32MX_TIMER4_PRCLR      (PIC32MX_TIMER4_K1BASE+PIC32MX_TIMER_PRCLR_OFFSET)
#  define PIC32MX_TIMER4_PRSET      (PIC32MX_TIMER4_K1BASE+PIC32MX_TIMER_PRSET_OFFSET)
#  define PIC32MX_TIMER4_PRINV      (PIC32MX_TIMER4_K1BASE+PIC32MX_TIMER_PRINV_OFFSET)
#endif

#if CHIP_NTIMERS > 4
#  define PIC32MX_TIMER5_CON        (PIC32MX_TIMER5_K1BASE+PIC32MX_TIMER_CON_OFFSET)
#  define PIC32MX_TIMER5_CONCLR     (PIC32MX_TIMER5_K1BASE+PIC32MX_TIMER_CONCLR_OFFSET)
#  define PIC32MX_TIMER5_CONSET     (PIC32MX_TIMER5_K1BASE+PIC32MX_TIMER_CONSET_OFFSET)
#  define PIC32MX_TIMER5_CONINV     (PIC32MX_TIMER5_K1BASE+PIC32MX_TIMER_CONINV_OFFSET)
#  define PIC32MX_TIMER5_CNT        (PIC32MX_TIMER5_K1BASE+PIC32MX_TIMER_CNT_OFFSET)
#  define PIC32MX_TIMER5_CNTCLR     (PIC32MX_TIMER5_K1BASE+PIC32MX_TIMER_CNTCLR_OFFSET)
#  define PIC32MX_TIMER5_CNTSET     (PIC32MX_TIMER5_K1BASE+PIC32MX_TIMER_CNTSET_OFFSET)
#  define PIC32MX_TIMER5_CNTINV     (PIC32MX_TIMER5_K1BASE+PIC32MX_TIMER_CNTINV_OFFSET)
#  define PIC32MX_TIMER5_PR         (PIC32MX_TIMER5_K1BASE+PIC32MX_TIMER_PR_OFFSET)
#  define PIC32MX_TIMER5_PRCLR      (PIC32MX_TIMER5_K1BASE+PIC32MX_TIMER_PRCLR_OFFSET)
#  define PIC32MX_TIMER5_PRSET      (PIC32MX_TIMER5_K1BASE+PIC32MX_TIMER_PRSET_OFFSET)
#  define PIC32MX_TIMER5_PRINV      (PIC32MX_TIMER5_K1BASE+PIC32MX_TIMER_PRINV_OFFSET)
#endif

/* Register Bit-Field Definitions ***************************************************/

/* Timer control register */

#define TIMER_CON_TCS               (1 << 1)  /* Bit 1: Timer clock source select (all) */
#define TIMER1_CON_TSYNC            (1 << 2)  /* Bit 2: Timer external clock input synchronization selection (timer 1 only) */
#define TIMER_CON_T32               (1 << 3)  /* Bit 2: 32-bit timer mode select (even timers only) */
#define TIMER_CON_TCKPS_SHIFT       (4)       /* Bits 4-6:  Timer input clock prescale select (all except timer 1) */
#define TIMER_CON_TCKPS_MASK        (7 << TIMER_CON_TCKPS_SHIFT)
#  define TIMER_CON_TCKPS_1         (0 << TIMER_CON_TCKPS_SHIFT) /* 1:1 prescale value */
#  define TIMER_CON_TCKPS_2         (1 << TIMER_CON_TCKPS_SHIFT) /* 1:2 prescale value */
#  define TIMER_CON_TCKPS_4         (2 << TIMER_CON_TCKPS_SHIFT) /* 1:4 prescale value */
#  define TIMER_CON_TCKPS_8         (3 << TIMER_CON_TCKPS_SHIFT) /* 1:8 prescale value */
#  define TIMER_CON_TCKPS_16        (4 << TIMER_CON_TCKPS_SHIFT) /* 1:16 prescale value */
#  define TIMER_CON_TCKPS_32        (5 << TIMER_CON_TCKPS_SHIFT) /* 1:32 prescale value */
#  define TIMER_CON_TCKPS_64        (6 << TIMER_CON_TCKPS_SHIFT) /* 1:64 prescale value */
#  define TIMER_CON_TCKPS_256       (7 << TIMER_CON_TCKPS_SHIFT) /* 1:256 prescale value */
#define TIMER1_CON_TCKPS_SHIFT      (4)       /* Bits 4-5:  Timer input clock prescale select (timer 1 only) */
#define TIMER1_CON_TCKPS_MASK       (3 << TIMER1_CON_TCKPS_SHIFT)
#  define TIMER1_CON_TCKPS_1        (0 << TIMER1_CON_TCKPS_SHIFT) /* 1:1 prescale value */
#  define TIMER1_CON_TCKPS_8        (1 << TIMER1_CON_TCKPS_SHIFT) /* 1:8 prescale value */
#  define TIMER1_CON_TCKPS_64       (2 << TIMER1_CON_TCKPS_SHIFT) /* 1:64 prescale value */
#  define TIMER1_CON_TCKPS_256      (3 << TIMER1_CON_TCKPS_SHIFT) /* 1:256 prescale value */
#define TIMER_CON_TGATE             (1 << 7)  /* Bit 7: Timer gated time accumulation enable (all) */
#define TIMER1_CON_TWIP             (1 << 11) /* Bit 11: Asynchronous timer write in progress (timer 1 only) */
#define TIMER1_CON_TWDIS            (1 << 12) /* Bit 12: Asynchronous timer write disable (timer 1 only) */
#define TIMER_CON_SIDL              (1 << 13) /* Bit 13: Stop in idle mode (all) */
#define TIMER_CON_FRZ               (1 << 14) /* Bit 14: Freeze in debug exception mode (all) */
#define TIMER_CON_ON                (1 << 15) /* Bit 15: Timer on (all) */

/* Timer count register */

#define TIMER_CNT_MASK  0xffff /* 16-bit timer counter value */

/* Timer period register */

#define TIMER_PR_MASK    0xffff /* 16-bit timer period value */

/************************************************************************************
 * Public Types
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CHIP_NTIMERS > 0 */
#endif /* __ARCH_MIPS_SRC_PIC32MX_PIC32MX_TIMER_H */
