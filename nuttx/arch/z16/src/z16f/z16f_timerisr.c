/***************************************************************************
 * z16f/z16f_timerisr.c
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
 ***************************************************************************/

/***************************************************************************
 * Included Files
 ***************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <debug.h>

#include <nuttx/arch.h>

#include "chip/chip.h"
#include "clock_internal.h"
#include "up_internal.h"

/***************************************************************************
 * Definitions
 ***************************************************************************/

/* System clock frequency value from ZDS target settings */

extern _Erom unsigned long SYS_CLK_FREQ;
#define _DEFCLK ((unsigned long)&SYS_CLK_FREQ)

/***************************************************************************
 * Private Types
 ***************************************************************************/

/***************************************************************************
 * Private Function Prototypes
 ***************************************************************************/

/***************************************************************************
 * Global Functions
 ***************************************************************************/

/***************************************************************************
 * Function:  up_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the system.
 *
 ***************************************************************************/

int up_timerisr(int irq, uint32_t *regs)
{
   /* Process timer interrupt */

   sched_process_timer();
   return 0;
}

/***************************************************************************
 * Function:  up_timerinit
 *
 * Description:
 *   This function is called during start-up to initialize
 *   the timer interrupt.
 *
 ***************************************************************************/

void up_timerinit(void)
{
  up_disable_irq(Z16F_IRQ_SYSTIMER);

  /* Disable the timer and configure for divide by 1 and continuous mode. */

  putreg8( Z16F_TIMERSCTL1_DIV1 | Z16F_TIMERSCTL1_CONT, Z16F_TIMER0_CTL1);

  /* Assign an intial timer value */

  putreg16(0x0001, Z16F_TIMER0_HL);

  /* Set the timer reload value.
   *
   * In continuous mode:
   *
   *   timer_frequency = system_clock_freqency / divisor / reload_value
   * or
   *   reload_value = (system_clock_frequency / timer_frequency / divisor
   *
   * For system_clock_frequency=200MHz, timer_frequency=100KHz, and divisor=1,
   * this yields 200.
   */

  putreg16(((uint32_t)_DEFCLK / 100000), Z16F_TIMER0_R);

  /* Enable the timer */

  putreg8((getreg8(Z16F_TIMER0_CTL1) | Z16F_TIMERCTL1_TEN), Z16F_TIMER0_CTL1);

  /* Set the timer priority */

  /* Attach and enable the timer interrupt (leaving at priority 0 */

  irq_attach(Z16F_IRQ_SYSTIMER, (xcpt_t)up_timerisr);
  up_enable_irq(Z16F_IRQ_SYSTIMER);
}

