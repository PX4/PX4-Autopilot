/***************************************************************************
 * arch/z80/src/ez80/ez80_timerisr.c
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

#include <arch/io.h>
#include <nuttx/arch.h>

#include "chip/chip.h"
#include "clock_internal.h"
#include "up_internal.h"

/***************************************************************************
 * Definitions
 ***************************************************************************/

/***************************************************************************
 * Private Types
 ***************************************************************************/

/***************************************************************************
 * Private Functions
 ***************************************************************************/

/***************************************************************************
 * Public Functions
 ***************************************************************************/

/***************************************************************************
 * Function:  up_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the system.
 *
 ***************************************************************************/

int up_timerisr(int irq, chipreg_t *regs)
{
  volatile uint8_t reg;

  /* Read the appropropriate timer0 registr to clear the interrupt */
  
#ifdef _EZ80F91
  reg = inp(EZ80_TMR0_IIR);
#else
  /* _EZ80190, _EZ80L92, _EZ80F92, _EZ80F93 */

  reg = inp(EZ80_TMR0_CTL);
#endif

  /* Process timer interrupt */

  sched_process_timer();
   
 /* Architecture specific hook into the timer interrupt handler */

#ifdef CONFIG_ARCH_TIMERHOOK
  up_timerhook();
#endif

  return 0;
}

/***************************************************************************
 * Function:  up_timerinit
 *
 * Description:
 *   This function is called during start-up to initialize the timer
 *   interrupt.
 *
 ***************************************************************************/

void up_timerinit(void)
{
  uint16_t reload;
  uint8_t  reg;

  /* Disable the timer */

  outp(EZ80_TMR0_CTL, 0x00);

  /* Attach system timer interrupts */

  irq_attach(EZ80_IRQ_SYSTIMER, (xcpt_t)up_timerisr);

  /* Set up the timer reload value */
  /* Write to the timer reload register to set the reload value.
   *
   * In continuous mode:
   *
   *   timer_period = reload_value x clock_divider / system_clock_frequency
   * or
   *   reload_value = (timer_period * system_clock_frequency) / clock_divider
   *
   * For timer_period=10mS, and clock_divider=16, that would yield:
   *
   *   reload_value = system_clock_frequency / 1600
   *
   * For a system timer of 50,000,000 that would result in a reload value of
   * 31,250
   *
   * NOTE: The system clock frequency value is defined in the board.h file
   */

  reload = (uint16_t)(ez80_systemclock / 1600);
  outp(EZ80_TMR0_RRH, (uint8_t)(reload >> 8));
  outp(EZ80_TMR0_RRL, (uint8_t)(reload));

  /* Clear any pending timer interrupts */
  
#if defined(_EZ80F91)
  reg = inp(EZ80_TMR0_IIR);
#elif defined(_EZ80L92) || defined(_EZ80F92) ||defined(_EZ80F93)
  reg = inp(EZ80_TMR0_CTL);
#endif

  /* Configure and enable the timer */

#if defined(_EZ80190)
  outp(EZ80_TMR0_CTL, 0x5f);
#elif defined(_EZ80F91)
  outp(EZ80_TMR0_CTL, (EZ80_TMRCLKDIV_16|EZ80_TMRCTL_TIMCONT|EZ80_TMRCTL_RLD|EZ80_TMRCTL_TIMEN));
#elif defined(_EZ80L92) || defined(_EZ80F92) ||defined(_EZ80F93)
  outp(EZ80_TMR0_CTL, 0x57);
#endif

/* Enable timer end-of-count interrupts */

#if defined(_EZ80F91)
  outp(EZ80_TMR0_IER, EZ80_TMRIER_EOCEN);
#endif
}
