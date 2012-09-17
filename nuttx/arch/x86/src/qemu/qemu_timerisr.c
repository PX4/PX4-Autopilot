/****************************************************************************
 * arch/x86/src/qemu/qemu_timerisr.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 *   Based on Bran's kernel development tutorials. Rewritten for JamesM's
 *   kernel development tutorials.
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
#include <arch/irq.h>
#include <arch/io.h>
#include <arch/board/board.h>

#include "clock_internal.h"
#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"
#include "qemu_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Programmable interval timer (PIT)
 *
 *   Fpit = Fin / divisor
 *   divisor = Fin / divisor
 *
 * Where:
 *
 *   Fpit = The desired interrupt frequency.
 *   Fin  = PIT input frequency (PIT_CLOCK provided in board.h)
 *
 * The desired timer interrupt frequency is provided by the definition CLK_TCK
 * (see include/time.h).  CLK_TCK defines the desired number of system clock
 * ticks per second.  That value is a user configurable setting that defaults
 * to 100 (100 ticks per second = 10 MS interval).
 */

#define PIT_DIVISOR  ((uint32_t)PIT_CLOCK/(uint32_t)CLK_TCK)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  up_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

static int up_timerisr(int irq, uint32_t *regs)
{
   /* Process timer interrupt */

   sched_process_timer();
   return 0;
}

/****************************************************************************
 * Global Functions
 ****************************************************************************/

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
  /* uint32_t to avoid compile time overflow errors */

  uint32_t divisor = PIT_DIVISOR;
  DEBUGASSERT(divisor <= 0xffff);

  /* Attach to the timer interrupt handler */

  (void)irq_attach(IRQ0, (xcpt_t)up_timerisr);

  /* Send the command byte to configure counter 0 */

  outb(PIT_OCW_MODE_SQUARE|PIT_OCW_RL_DATA|PIT_OCW_COUNTER_0, PIT_REG_COMMAND);

  /* Set the PIT input frequency divisor */

  outb((uint8_t)(divisor & 0xff),  PIT_REG_COUNTER0);
  outb((uint8_t)((divisor >> 8) & 0xff), PIT_REG_COUNTER0);

  /* And enable IRQ0 */

  up_enable_irq(IRQ0);
}
