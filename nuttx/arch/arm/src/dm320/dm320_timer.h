/************************************************************************************
 * dm320/dm320_timer.h
 *
 *   Copyright (C) 2007, 2009 Gregory Nutt. All rights reserved.
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

#ifndef __DM320_TIMER_H
#define __DM320_TIMER_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Timer Registers */

#define DM320_TIMER0_TMMD         (DM320_PERIPHERALS_VADDR + 0x0000) /* Timer 0 Mode */
#define DM320_TIMER0_TMPRSCL      (DM320_PERIPHERALS_VADDR + 0x0004) /* Timer 0 Prescalar */
#define DM320_TIMER0_TMDIV        (DM320_PERIPHERALS_VADDR + 0x0006) /* Timer 0 Divisor (count) */
#define DM320_TIMER0_TMTRG        (DM320_PERIPHERALS_VADDR + 0x0008) /* Timer 0 One-Shot Trigger */
#define DM320_TIMER0_TMCNT        (DM320_PERIPHERALS_VADDR + 0x000A) /* Timer 0 Count */

#define DM320_TIMER1_TMMD         (DM320_PERIPHERALS_VADDR + 0x0080) /* Timer 1 Mode */
#define DM320_TIMER1_TMPRSCL      (DM320_PERIPHERALS_VADDR + 0x0084) /* Timer 1 Prescalar */
#define DM320_TIMER1_TMDIV        (DM320_PERIPHERALS_VADDR + 0x0086) /* Timer 1 Divisor (count) */
#define DM320_TIMER1_TMTRG        (DM320_PERIPHERALS_VADDR + 0x0088) /* Timer 1 One-Shot Trigger */
#define DM320_TIMER1_TMCNT        (DM320_PERIPHERALS_VADDR + 0x008A) /* Timer 1 Count */

#define DM320_TIMER2_TMMD         (DM320_PERIPHERALS_VADDR + 0x0100) /* Timer 2 Mode */
#define DM320_TIMER2_TMPRSCL      (DM320_PERIPHERALS_VADDR + 0x0104) /* Timer 2 Prescalar */
#define DM320_TIMER2_TMDIV        (DM320_PERIPHERALS_VADDR + 0x0106) /* Timer 2 Divisor (count) */
#define DM320_TIMER2_TMTRG        (DM320_PERIPHERALS_VADDR + 0x0108) /* Timer 2 One-Shot Trigger */
#define DM320_TIMER2_TMCNT        (DM320_PERIPHERALS_VADDR + 0x010A) /* Timer 2 Count */

#define DM320_TIMER3_TMMD         (DM320_PERIPHERALS_VADDR + 0x0180) /* Timer 2 Mode */
#define DM320_TIMER3_TMPRSCL      (DM320_PERIPHERALS_VADDR + 0x0184) /* Timer 2 Prescalar */
#define DM320_TIMER3_TMDIV        (DM320_PERIPHERALS_VADDR + 0x0186) /* Timer 2 Divisor (count) */
#define DM320_TIMER3_TMTRG        (DM320_PERIPHERALS_VADDR + 0x0188) /* Timer 2 One-Shot Trigger */
#define DM320_TIMER3_TMCNT        (DM320_PERIPHERALS_VADDR + 0x018A) /* Timer 2 Count */

/* Timer 0,1,2,3 Mode Register Bits: */

#define  DM320_TMR_MODE_TEST_MASK 0x00fc /* Bits 7:2=Test */
#define  DM320_TMR_MODE_MODE_MASK 0x0003 /* Bits 1:0=timer mode */

# define DM320_TMR_MODE_STOP      0x0000 /* Stop Timer */
# define DM320_TMR_MODE_ONESHOT   0x0001 /* Start one-shot timer */
# define DM320_TMR_MODE_FREERUN   0x0002 /* Start free-running timer */

/* Timer 0,1,2,3 Clock Select Register Bits: */

#define  DM320_TMR_PRSCL_MASK     0x03ff /* Bits 0:9=Timer prescale value */

/* Timer 0,1,2,3 Clock Divisor (Count) Register Bits: */

#define  DM320_TMR_DIV_MASK       0xffff /* Bits 0:15=Timer divisor value */

/* Timer 0,1,2,3 Timer One-Short Trigger Register Bits: */

#define  DM320_TMR_TMTRG_MASK     0x0001 /* Bit 0=One short trigger */

# define DM320_TMR_TMTRG_START    0x0001 /* 1 starts one shot timer */

/* Timer 0,1,2,3 Timer Counter Register Bits: */

#define DM320_TMR_COUNT_MASK      0xffff /* Bits 0:15=Current counter value */

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

#ifndef __ASSEMBLY__

#endif

#endif  /* __DM320_TIMER_H */
