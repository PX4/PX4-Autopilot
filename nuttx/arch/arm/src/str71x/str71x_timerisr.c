/****************************************************************************
 * arch/arm/src/str71x/str71x_timerisr.c
 *
 *   Copyright (C) 2007-2009 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <time.h>
#include <debug.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "clock_internal.h"
#include "up_internal.h"

#include "str71x_internal.h"

/****************************************************************************
 * Pre-procesor Definitions
 ****************************************************************************/

/* Configuration */

#ifndef CONFIG_TIM_PRI
#  define CONFIG_TIM_PRI 1
#elif CONFIG_TIM_PRI <= 1 || CONFIG_TIM_PRI > 15
#  error "CONFIG_TIM_PRI is out of range"
#endif

/* The desired timer interrupt frequency is provided by the definition
 * CLK_TCK (see include/time.h).  CLK_TCK defines the desired number of
 * system clock ticks per second.  That value is a user configurable setting
 * that defaults to 100 (100 ticks per second = 10 MS interval).
 *
 * The best accuracy would be obtained by using the largest value in the
 * the output compare register (OCAR), i.e., 0xffff = 65,535:
 */

#define MAX_OCAR    65535

 /* In this case, the desired, maximum clocking would be MAX_TIM0CLK.  For
  * example if CLK_TCK is the default of 100Hz, then the ideal clocking for
  * timer0 would be 6,553,500 */

#define MAX_TIM0CLK (MAX_OCAR * CLK_TCK)

 /* The best divider then would be the one that reduces PCLK2 to MAX_TIM0CLK.
  * Note that the following calculation forces an integer divisor to the next
  * integer above the optimal.  So, for example, if MAX_TIM0CLK is 6,553,500
  * and PCLK2 is 32MHz, then ideal PCLK2_DIVIDER would be 4.88 but 5 is used
  * instead.  The value 5 would give an actual TIM0CLK of 6,400,000, less
  * than the maximum.
  */

#if STR71X_PCLK2 > MAX_TIM0CLK
#  define PCLK2_DIVIDER (((STR71X_PCLK2) + (MAX_TIM0CLK+1)) / MAX_TIM0CLK)
#else
#  define PCLK2_DIVIDER (1)
#endif

#if PCLK2_DIVIDER > 255
#  error "PCLK2 is too fast for any divisor"
#endif

  /* Then we can get the actual OCAR value from the selected divider value.
   * For example, if PCLK2 is 32MHz and PCLK2_DIVIDER is 5, then the actual
   * TIM0CLK would 6,4000,000 and the final OCAR_VALUE would be 64,000.
   */

#define ACTUAL_TIM0CLK (STR71X_PCLK2 / PCLK2_DIVIDER)
#define OCAR_VALUE     (ACTUAL_TIM0CLK / CLK_TCK)

#if OCAR_VALUE > 65535
#  error "PCLK2 is too fast for the configured CLK_TCK"
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  up_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

int up_timerisr(int irq, uint32_t *regs)
{
  uint16_t ocar;

  /* Clear all the output compare A interrupt status bit */

  putreg16(~STR71X_TIMERSR_OCFA, STR71X_TIMER0_SR);

  /* Set up for the next compare match.  We could either reset
   * the OCAR and CNTR to restart, or simply update the OCAR as
   * follows to that the match occurs later without resetting:
   */

  ocar = getreg16(STR71X_TIMER0_OCAR);
  ocar += OCAR_VALUE;
  putreg16(ocar, STR71X_TIMER0_OCAR);

  /* Process timer interrupt */

  sched_process_timer();
  return 0;
}

/****************************************************************************
 * Function:  up_timerinit
 *
 * Description:
 *   This function is called during start-up to initialize
 *   the timer interrupt.
 *
 ****************************************************************************/

void up_timerinit(void)
{
  irqstate_t flags;

  /* Make sure that timer0 is disabled */

  flags = irqsave();
  putreg16(0x0000, STR71X_TIMER0_CR1);
  putreg16(0x0000, STR71X_TIMER0_CR2);
  putreg16(0x0000, STR71X_TIMER0_SR);

 /* Configure TIM0 so that it is clocked by the internal APB2 frequency (PCLK2)
  * divided by the above prescaler value (1) -- versus an external Clock.
  * -- Nothing to do because  STR71X_TIMERCR1_ECKEN is already cleared.
  *
  * Select a divisor to reduce the frequency of clocking.  This must be
  * done so that the entire timer interval can fit in the 16-bit OCAR register.
  * (see the discussion above).
  */

  putreg16(STR71X_TIMERCR2_OCAIE | (PCLK2_DIVIDER - 1), STR71X_TIMER0_CR2);

  /* Start The TIM0 Counter and enable the output comparison A */

  putreg16(STR71X_TIMERCR1_EN | STR71X_TIMERCR1_OCAE, STR71X_TIMER0_CR1);

  /* Setup output compare A for desired interrupt frequency.  Note that
   * the OCAE and OCBE bits are cleared and the pins are available for other
   * functions.
   */

  putreg16(OCAR_VALUE, STR71X_TIMER0_OCAR);
  putreg16(0xfffc, STR71X_TIMER0_CNTR);

  /* Set the timer interrupt priority */

  up_prioritize_irq(STR71X_IRQ_SYSTIMER, CONFIG_TIM_PRI);

  /* Attach the timer interrupt vector */

  (void)irq_attach(STR71X_IRQ_SYSTIMER, (xcpt_t)up_timerisr);

  /* And enable the timer interrupt */

  up_enable_irq(STR71X_IRQ_SYSTIMER);
  irqrestore(flags);
}
