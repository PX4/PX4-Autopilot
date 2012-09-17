/****************************************************************************
 * arch/arm/src/lpc214x/lpc214x_timerisr.c
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
#include <debug.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "clock_internal.h"
#include "up_internal.h"

#include "lpc214x_timer.h"
#include "lpc214x_vic.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* The timers count at the rate of PCLK which is determined by PLL_M and
 * and APBDIV:
 */

#define LPC214X_CCLKFREQ  (LPC214X_FOSC*LPC214X_PLL_M)
#define LPC214X_PCLKFREQ  (LPC214X_CCLKFREQ/LPC214X_APB_DIV)

#define tmr_getreg8(o)    getreg8(LPC214X_TMR0_BASE+(o))
#define tmr_getreg16(o)   getreg16(LPC214X_TMR0_BASE+(o))
#define tmr_getreg32(o)   getreg32(LPC214X_TMR0_BASE+(o))

#define tmr_putreg8(v,o)  putreg8((v), LPC214X_TMR0_BASE+(o))
#define tmr_putreg16(v,o) putreg16((v), LPC214X_TMR0_BASE+(o))
#define tmr_putreg32(v,o) putreg32((v), LPC214X_TMR0_BASE+(o))

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
 *   The timer ISR will perform a variety of services for
 *   various portions of the systems.
 *
 ****************************************************************************/

#ifdef CONFIG_VECTORED_INTERRUPTS
int up_timerisr(uint32_t *regs)
#else
int up_timerisr(int irq, uint32_t *regs)
#endif
{
   /* Process timer interrupt */

   sched_process_timer();

   /* Clear the MR0 match interrupt */

   tmr_putreg8(LPC214X_TMR_IR_MR0I, LPC214X_TMR_IR_OFFSET);

   /* Reset the VIC as well */

#ifdef CONFIG_VECTORED_INTERRUPTS
   vic_putreg(0, LPC214X_VIC_VECTADDR_OFFSET);
#endif
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
  uint16_t mcr;

  /* Clear all match and capture event interrupts */

  tmr_putreg8(LPC214X_TMR_IR_ALLI, LPC214X_TMR_IR_OFFSET);

  /* Clear the timer counter */

  tmr_putreg32(0, LPC214X_TMR_TC_OFFSET);

  /* No pre-scaler */

  tmr_putreg32(0, LPC214X_TMR_PR_OFFSET);

  /* Set timer match registger to get a TICK_PER_SEC rate
   * See arch/board.h and sched/os_internal.h
   */

  tmr_putreg32(LPC214X_PCLKFREQ/TICK_PER_SEC, LPC214X_TMR_MR0_OFFSET);

  /* Reset timer counter regiser and interrupt on match */

  mcr = tmr_getreg16(LPC214X_TMR_MCR_OFFSET);
  mcr &= ~LPC214X_TMR_MCR_MR1I;
  mcr |= (LPC214X_TMR_MCR_MR0I | LPC214X_TMR_MCR_MR0R);
  tmr_putreg16(mcr, LPC214X_TMR_MCR_OFFSET);

  /* Enable counting */

  tmr_putreg8(LPC214X_TMR_CR_ENABLE, LPC214X_TMR_TCR_OFFSET);

  /* Attach the timer interrupt vector */

#ifdef CONFIG_VECTORED_INTERRUPTS
  up_attach_vector(LPC214X_IRQ_SYSTIMER, LPC214X_SYSTIMER_VEC, (vic_vector_t)up_timerisr);
#else
  (void)irq_attach(LPC214X_IRQ_SYSTIMER, (xcpt_t)up_timerisr);
#endif

  /* And enable the timer interrupt */

  up_enable_irq(LPC214X_IRQ_SYSTIMER);
}
