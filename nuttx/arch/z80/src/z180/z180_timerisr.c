/****************************************************************************
 * arch/z80/src/z180/z180_timerisr.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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

#include <arch/board/board.h>

#include "clock_internal.h"
#include "up_internal.h"

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* "The Z180 contains a two channel 16-bit Programmable Reload Timer. Each
 * PRT channel contains a 16-bit down counter and a 16-bit reload register."
 * Channel 0 is dedicated as the system timer.
 */

/* "The PRT input clock for both channels is equal to the system clock
 * divided by 20."
 */

#define Z180_PRT_CLOCK   (Z180_SYSCLOCK / 20)

/* The data Register "(TMDR) is decremented once every twenty clocks. When
 * TMDR counts down to 0, it is automatically reloaded with the value
 * contained in the Reload Register (RLDR)."
 */

#define A180_PRT0_RELOAD (Z180_PRT_CLOCK / CLK_TCK)

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
 * Function: up_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

int up_timerisr(int irq, chipreg_t *regs)
{
  volatile uint8_t regval;

  /* "When TMDR0 decrements to 0, TIF0 is set to 1. This generates an interrupt
   * request if enabled by TIE0 = 1. TIF0 is reset to 0 when TCR is read and
   * the higher or lower byte of TMDR0 is read." 
   */

  regval = inp(Z180_PRT_TCR);
  regval = inp(Z180_PRT0_DRL);
  regval = inp(Z180_PRT0_DRH);

  /* Process timer interrupt */

  sched_process_timer();
  return 0;
}

/****************************************************************************
 * Function: up_timerinit
 *
 * Description:
 *   This function is called during start-up to initialize the timer
 *   interrupt.
 *
 ****************************************************************************/

void up_timerinit(void)
{
  uint8_t regval;

  /* Configure PRT0 to interrupt at the requested rate */
  /* First stop PRT0 and disable interrupts */

  regval  = inp(Z180_PRT_TCR);
  regval &= (PRT_TCR_TIF0|PRT_TCR_TIE0|PRT_TCR_TDE0);
  outp(Z180_PRT_TCR, regval);

  /* Set the timer reload value so that the timer will interrupt at the
   * desired frequency.  "For writing, the TMDR down counting must be
   * inhibited using the TDE (Timer Down Count Enable) bits in the TCR
   * (Timer Control Register). Then, any or both higher and lower bytes of
   * TMDR can be freely written (and read) in any order."
   */

  outp(Z180_PRT0_RLDRL, (A180_PRT0_RELOAD & 0xff));
  outp(Z180_PRT0_RLDRH, (A180_PRT0_RELOAD >> 8));

  /* Enable down-counting */

  regval |= PRT_TCR_TDE0;
  outp(Z180_PRT_TCR, regval);

  /* Attach the timer interrupt vector */

  (void)irq_attach(Z180_PRT0, (xcpt_t)up_timerisr);

  /* And enable the timer interrupt */

  regval |= PRT_TCR_TIE0;
  outp(Z180_PRT_TCR, regval);
}
