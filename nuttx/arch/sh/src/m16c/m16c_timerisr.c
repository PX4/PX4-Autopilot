/****************************************************************************
 * arch/sh/src/m16c/m16c_timerisr.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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

#include "clock_internal.h"
#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"
#include "m16c_timer.h"

/****************************************************************************
 * Preprocessor Definitions
 ****************************************************************************/
/* Configuration */

#ifndef M16C_TA0_PRIO              /* Timer A0 interrupt priority */
#  define M16C_TA0_PRIO   5
#endif

/* Determine the ideal preload value for the timer.
 *
 * For example, given a 20MHz input frequency and a desired 100 Hz, clock,
 * the ideal reload value would be:
 *
 *  20,000,000 / 100 = 200,000
 *
 * The ideal prescaler value would be the one, then that drops this to exactly
 * 66535:
 *
 *  M16C_IDEAL_PRESCALER = 200,000 / 65535 = 3.05
 *
 * And any value greater than 3.05 would also work with less and less precision.
 * The following calculation will produce the ideal prescaler as the next integer
 * value above any fractional values:
 */

#define M16C_DIVISOR (65535 * CLK_TCK)
#define M16C_IDEAL_PRESCALER \
	((M16C_XIN_FREQ + M16C_DIVISOR - 1) / M16C_DIVISOR)

/* Now, given this idel prescaler value, pick between available choices: 1, 8, and 32 */

#if M16C_IDEAL_PRESCALER > 8
#  define M16C_PRESCALE_VALUE 32
#  define M16C_PRESCALE_BITS TAnMR_TCK_TMF32
#elif M16C_IDEL_PRESCALER > 1
#  define M16C_PRESCALE_VALUE 8
#  define M16C_PRESCALE_BITS TAnMR_TCK_TMF8
#else
#  define M16C_PRESCALE_VALUE 1
#  define M16C_PRESCALE_BITS TAnMR_TCK_TMF1
#endif

/* Timer 0 Mode Settings */

#define M16C_TA0MODE_CONFIG \
	(TAnMR_TMOD_TIMER|TAnMR_MR_TMNOOUT|TAnMR_MR_TMNOGATE|M16C_PRESCALE_BITS)

/* The actual reload value matching the selected prescaler value */

#define M16C_RELOAD_VALUE \
	((M16C_XIN_FREQ / M16C_PRESCALE_VALUE / CLK_TCK)  - 1)

/****************************************************************************
 * Private Type Definitions
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
  /* Make sure that no timers are running and that all timer interrupts are
   * disabled.
   */

  putreg8(0, M16C_TABSR);
  putreg8(0, M16C_TA0IC);
  putreg8(0, M16C_TA1IC);
  putreg8(0, M16C_TA2IC);
  putreg8(0, M16C_TA3IC);
  putreg8(0, M16C_TA4IC);
  putreg8(0, M16C_TB0IC);
  putreg8(0, M16C_TB1IC);
  putreg8(0, M16C_TB2IC);

  /* Set up timer 0 mode register for timer mode with the calculated prescaler value */
 
  putreg8(M16C_TA0MODE_CONFIG, M16C_TA0MR);

  /* Set the calculated reload value */

  putreg16(M16C_RELOAD_VALUE, M16C_TA0);
 
  /* Attach the interrupt handler */

  irq_attach(M16C_SYSTIMER_IRQ, (xcpt_t)up_timerisr);

  /* Enable timer interrupts */

  putreg8(1, M16C_TA0IC);
  
  /* Set the interrupt priority */

  putreg8(M16C_TA0_PRIO, M16C_TA0IC);

  /* Start the timer */

  putreg8(TABSR_TA0S, M16C_TABSR);
}
