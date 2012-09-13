/****************************************************************************
 * arch/hc/src/m9s12/m9s12_timerisr.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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
#include "m9s12_internal.h"
#include "m9s12_crg.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* The desired timer interrupt frequency is provided by the definition
 * CLK_TCK (see include/time.h).  CLK_TCK defines the desired number of
 * system clock ticks per second.  That value is a user configurable setting
 * that defaults to 100 (100 ticks per second = 10 MS interval).
 */

/* The timer frequency is the OSCCLK divided down.  The divisor is
 * (A+1)*(2**(B+9)) where:
 *
 *   A = MODCNT RTR[0:3]
 *   B = PREP   RTR[4:6]
 *
 * Maximum and minimum values:
 */

#define MIN_PRER    1024 /* 2**10, B=1 */
#define MAX_PRER   65536 /* 2**16, B=7 */
 
#define MIN_MODCNT     1 /* A=0 */
#define MAX_MODCNT    16 /* A=15 */

/* Pick the smallest value of B for which:
 *
 *   OSCCLK/(MAX_MODCNT*(2**(B+9))) >= CLK_TCK >= OSCCLK/(MIN_MODCNT*(2**(B+9)))
 */

#if CLK_TCK >= HCS12_OSCCLK/(MAX_MODCNT*1024) && HCS12_OSCCLK/(MIN_MODCNT*1024)
#  define PRER_VALUE      1
#  define PRER_DIV     1024
#elif CLK_TCK >= HCS12_OSCCLK/(MAX_MODCNT*2048) && HCS12_OSCCLK/(MIN_MODCNT*2048)
#  define PRER_VALUE      2
#  define PRER_DIV     2048
#elif CLK_TCK >= HCS12_OSCCLK/(MAX_MODCNT*4096) && HCS12_OSCCLK/(MIN_MODCNT*4096)
#  define PRER_VALUE      3
#  define PRER_DIV     4096
#elif CLK_TCK >= HCS12_OSCCLK/(MAX_MODCNT*8192) && HCS12_OSCCLK/(MIN_MODCNT*8192)
#  define PRER_VALUE      4
#  define PRER_DIV     8192
#elif CLK_TCK >= HCS12_OSCCLK/(MAX_MODCNT*16384) && HCS12_OSCCLK/(MIN_MODCNT*16384)
#  define PRER_VALUE      5
#  define PRER_DIV    16384
#elif CLK_TCK >= HCS12_OSCCLK/(MAX_MODCNT*32768) && HCS12_OSCCLK/(MIN_MODCNT*32768)
#  define PRER_VALUE      6
#  define PRER_DIV    32768
#elif CLK_TCK >= HCS12_OSCCLK/(MAX_MODCNT*65536) && HCS12_OSCCLK/(MIN_MODCNT*65536)
#  define PRER_VALUE      7
#  define PRER_DIV    65536
#else
#  error "Cannot generate CLK_TCK from HCSCLK_OSCCLK"
#endif

/* Now we can simply calculate A from:
 *
 * CLK_TCK = OSCCLK/((A+1)*PRER_DIV)
 * OSCCLK / (CLK_TCK * PRER_DIV) - 1
 */

 #define MODCNT_DENOM  ((uint32_t)CLK_TCK * (uint32_t)PRER_DIV)
 #define MODCNT_VALUE  ((((uint32_t)HCS12_OSCCLK  + (MODCNT_DENOM/2))/ MODCNT_DENOM) - 1)

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
  /* Clear real time interrupt flag */

  putreg8(CRG_CRGFLG_RTIF, HCS12_CRG_CRGFLG);

  /* Process timer interrupt */

  sched_process_timer();
  return 0;
}

/****************************************************************************
 * Function:  up_timerinit
 *
 * Description:
 *   This function is called during start-up to initialize the system timer
 *   interrupt.
 *
 ****************************************************************************/

void up_timerinit(void)
{
  uint32_t tmp;
  uint8_t  regval;

  /* Configure hardware RTI timer (with a hack to avoid and integer overflow
   * error at compile time.. hopefully the optimizer will eliminate these
   * uint32_t operations).
   */

  tmp = MODCNT_VALUE << CRG_RTICTL_MODCNT_SHIFT | PRER_VALUE << CRG_RTICTL_PRER_SHIFT;
  putreg8((uint8_t)tmp, HCS12_CRG_RTICTL);

  /* Attach the timer interrupt vector */

  (void)irq_attach(HCS12_IRQ_VRTI, (xcpt_t)up_timerisr);

  /* Enable RTI interrupt by setting the RTIE bit */

  regval  = getreg8(HCS12_CRG_CRGINT);
  regval |= CRG_CRGINT_RTIE;
  putreg8(regval, HCS12_CRG_CRGINT);
}
