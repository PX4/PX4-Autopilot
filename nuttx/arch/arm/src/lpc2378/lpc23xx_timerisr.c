/****************************************************************************
 * arch/arm/src/lpc2378/lpc23xx_timerisr.c
 *
 *   Copyright (C) 2010 Rommel Marcelo. All rights reserved.
 *   Author: Rommel Marcelo
 *
 * This file is part of the NuttX RTOS and based on the lpc2148 port:
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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
#include <sys/types.h>
#include <debug.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "clock_internal.h"
#include "internal.h"
#include "up_arch.h"

#include "lpc23xx_scb.h"
#include "lpc23xx_vic.h"
#include "lpc23xx_timer.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* T0_PCLKDIV valid values are 1,2,4 */

#define T0_PCLK_DIV		1

/* PCKLSEL0 bits 3:2, 00=CCLK/4, 01=CCLK/1 , 10=CCLK/2  */

#ifdef T0_PCLK_DIV
#  if T0_PCLK_DIV == 1
#    	define TIMER0_PCLKSEL	(0x00000004)
#  elif T0_PCLK_DIV == 2
#    	 define TIMER0_PCLKSEL	(0x00000008)
#  elif T0_PCLK_DIV == 4
#    	 define TIMER0_PCLKSEL	(0x00000000)
#  endif
#endif

#define T0_PCLKSEL_MASK		(0x0000000C)

#define T0_TICKS_COUNT	( (CCLK / T0_PCLK_DIV ) / TICK_PER_SEC )

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
int up_timerisr(uint32_t * regs)
#else
int up_timerisr(int irq, uint32_t * regs)
#endif
{
  static uint32_t tick;

  /* Process timer interrupt */

  sched_process_timer();

  /* Clear the MR0 match interrupt */

  tmr_putreg8(TMR_IR_MR0I, TMR_IR_OFFSET);

  /* Reset the VIC as well */

#ifdef CONFIG_VECTORED_INTERRUPTS
  /* write any value to VICAddress to acknowledge the interrupt */
  vic_putreg(0, VIC_ADDRESS_OFFSET);
#endif

  if (tick++ > 100)
    {
      tick = 0;
      up_statledoff();
    }
  else
    up_statledon();

  return 0;
}

/****************************************************************************
 * Function:  up_timerinit
 *
 * Description:
 *   This function is called during start-up to initialize
 *   the system timer interrupt.
 *
 ****************************************************************************/

void up_timerinit(void)
{
  uint16_t mcr;

  /* Power up Timer0 */

  SCB_PCONP |= PCTIM0;

  /* Timer0 clock input frequency = CCLK / TO_PCLKDIV */

  SCB_PCLKSEL0 = (SCB_PCLKSEL0 & ~T0_PCLKSEL_MASK) | TIMER0_PCLKSEL;

  /* Clear all match and capture event interrupts */

  tmr_putreg8(TMR_IR_ALLI, TMR_IR_OFFSET);

  /* Clear the timer counter */

  tmr_putreg32(0, TMR_TC_OFFSET);

  /* No pre-scaler */

  tmr_putreg32(0, TMR_PR_OFFSET);
  tmr_putreg32(0, TMR_PC_OFFSET);

  /* Set timer match register to get a TICK_PER_SEC rate See arch/board.h and
   * sched/os_internal.h
   */

  tmr_putreg32(T0_TICKS_COUNT, TMR_MR0_OFFSET); /* 10ms Intterrupt */

  /* Reset timer counter register and interrupt on match */

  mcr = tmr_getreg16(TMR_MCR_OFFSET);
  mcr &= ~TMR_MCR_MR1I;
  mcr |= (TMR_MCR_MR0I | TMR_MCR_MR0R);
  tmr_putreg16(mcr, TMR_MCR_OFFSET);    /* -- bit 0=1 -int on MR0, bit 1=1 - Reset on MR0 */

  /* Enable counting */
  /* ~ tmr_putreg32(1, TMR_TCR_OFFSET); */

  tmr_putreg8(TMR_CR_ENABLE, TMR_TCR_OFFSET);

  /* Attach the timer interrupt vector */

#ifdef CONFIG_VECTORED_INTERRUPTS
  up_attach_vector(IRQ_SYSTIMER, PRIORITY_HIGHEST, (vic_vector_t) up_timerisr);
#else
  (void)irq_attach(IRQ_SYSTIMER, (xcpt_t) up_timerisr);
  up_prioritize_irq(IRQ_SYSTIMER, PRIORITY_HIGHEST);
#endif

  /* And enable the system timer interrupt */

  up_enable_irq(IRQ_SYSTIMER);
}
