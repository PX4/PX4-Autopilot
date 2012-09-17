/****************************************************************************
 * arch/arm/src/dm320/dm320_timerisr.c
 * arch/arm/src/chip/dm320_timerisr.c
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <debug.h>
#include <nuttx/arch.h>

#include "chip.h"
#include "up_arch.h"
#include "clock_internal.h"
#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DM320 Timers
 *
 * Each of the general-purpose timers can run in one of two modes:  one-
 * shot mode and free-run mode.  In one-shot mode, an interrupt only 
 * occurs once and then the timer must be explicitly reset to begin the 
 * timing operation again.  In free-run mode, when the timer generates an 
 * interrupt, the timer counter is automatically reloaded to start the count 
 * operation again.  Use the bit field MODE in TMMDx to configure the 
 * timer for one-shot more or free-run mode. The bit field MODE in TMMDx
 * also allows you to stop the timer.
 * 
 * Either the ARM clock divided by 2 (CLK_ARM/2) or an external clock 
 * connected to the M27XI pin can be selected as the clock source of the 
 * timer.
 *
 * The actual clock frequency used in the timer count operation is the input 
 * clock divided by: 1 plus the value set in the bit field PRSCL of the 
 * register TMPRSCLx (10 bits).  The timer expires when it reaches the 
 * value set in the bit field DIV of the register TMDIVx (16 bits) plus 1. 
 * PRSCL+1 is the source clock frequency divide factor and DIV+1 is the 
 * timer count value.  The frequency of a timer interrupt is given by the 
 * following equation:
 *
 * Interrupt Frequency = (Source Clock Frequency) / (PRSCL+1) / (DIV+1)
 */

/* System Timer
 *
 * Timer0 is dedicated as the system timer.  The rate of system timer
 * interrupts is assumed to to 10MS per tick / 100Hz. The following
 * register settings are used for timer 0
 *
 * System clock formula:
 *   Interrupt Frequency = (Source Clock Frequency) / (PRSCL+1) / (DIV+1)
 *   Source Clock Frequency = 27MHz  (PLL clock)
 *   DIV                    = 26,999 (Yields 1Khz timer clock)
 *   PRSCL                  = 9      (Produces 100Hz interrupts)
 */

#define DM320_TMR0_MODE  DM320_TMR_MODE_FREERUN /* Free running */
#define DM320_TMR0_DIV   26999                  /* (see above) */
#define DM320_TMR0_PRSCL 9                      /* (see above) */

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
   /* Process timer interrupt */

   sched_process_timer();
   return 0;
}

/****************************************************************************
 * Function:  up_timerinit
 *
 * Description:
 *   This function is called during start-up to initialize the timer
 *   interrupt.
 *
 ****************************************************************************/

void up_timerinit(void)
{
  up_disable_irq(DM320_IRQ_SYSTIMER);

  /* Start timer0 running so that an interrupt is generated at
   * the rate MSEC_PER_TICK.
   */

  putreg16(DM320_TMR0_PRSCL, DM320_TIMER0_TMPRSCL); /* Timer 0 Prescalar */
  putreg16(DM320_TMR0_DIV, DM320_TIMER0_TMDIV);     /* Timer 0 Divisor (count) */

  /* Start the timer */

  putreg16(DM320_TMR0_MODE, DM320_TIMER0_TMMD); /* Timer 0 Mode */

  /* Attach and enable the timer interrupt */

  irq_attach(DM320_IRQ_SYSTIMER, (xcpt_t)up_timerisr);
  up_enable_irq(DM320_IRQ_SYSTIMER);
}

