/****************************************************************************
 * arch/avr/src/at32uc3/at32uc3_timerisr.c
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

#include <stdint.h>
#include <debug.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "up_arch.h"

#include "chip.h"
#include "at32uc3_internal.h"
#include "at32uc3_pm.h"
#include "at32uc3_rtc.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* The desired timer interrupt frequency is normally provided by the
 * definition CLK_TCK (see include/time.h).  CLK_TCK defines the desired
 * number of system clock ticks per second.  That value is a user configurable
 * setting that defaults to 100 (100 ticks per second = 10 MS interval).
 *
 * However, the AVR RTC does not support that default value well and so, we
 * will insist that default is over-ridden by CONFIG_MSEC_PER_TICK in the
 * configuration file.  Further, we will insist that CONFIG_MSEC_PER_TICK
 * have the value 8 (see reasoning below).
 */

#if defined(CONFIG_MSEC_PER_TICK) && CONFIG_MSEC_PER_TICK != 10
#  error "Only a 100KHz system clock is supported"
#endif

/* The frequency of the RTC is given by:
 *
 * fRTC = fINPUT / 2**(PSEL + 1)
 *
 * Using the 32KHz (actually 32786Hz) clock, various RTC counting can
 * be obtained:
 *
 * fRTC = 32768 / 2**16 = 32768/65536 = 0.5Hz  -> 2000 ms per tick
 * fRTC = 32768 / 2**15 = 32768/32768 = 1.0Hz  -> 1000 ms per tick
 * fRTC = 32768 / 2**14 = 32768/16384 = 2.0Hz  ->  500 ms per tick
 * fRTC = 32768 / 2**13 = 32768/8192  = 4.0Hz  ->  250 ms per tick
 * fRTC = 32768 / 2**12 = 32768/4096  = 8.0Hz  ->  125 ms per tick
 * fRTC = 32768 / 2**11 = 32768/2048  = 16.0Hz ->  62.5 ms per tick
 * fRTC = 32768 / 2**10 = 32768/1024  = 32.0Hz ->  31.25 ms per tick
 * fRTC = 32768 / 2**9  = 32768/512   = 64.0Hz ->  15.63 ms per tick
 * fRTC = 32768 / 2**8  = 32768/256   = 125Hz  ->   7.81 ms per tick
 * fRTC = 32768 / 2**7  = 32768/128   = 250Hz  ->   3.91 ms per tick
 * fRTC = 32768 / 2**6  = 32768/64    = 500Hz  ->   1.95 ms per tick
 * fRTC = 32768 / 2**5  = 32768/32    = 1KHz   ->   0.98 ms per tick
 * fRTC = 32768 / 2**4  = 32768/16    = 2KHz   -> 488.28 us per tick
 * fRTC = 32768 / 2**3  = 32768/8     = 4KHz   -> 244.14 us per tick
 * fRTC = 32768 / 2**2  = 32768/4     = 8KHz   -> 122.07 us per tick
 * fRTC = 32768 / 2                   = 16KHz  ->  61.03 us per tick
 *
 * We'll use PSEL == 1 (fRTC == 122.07us) and we will set TOP to 81.
 * Therefore, the TOP interrupt should occur after 81+1=82 counts
 * at a rate of 122.07us x 82 = 10.01 ms
 *
 * Using the RCOSC at a nominal 115KHz, we can do he following:
 *
 * fRTC = 115000 / 2**16 = 115000/65536 = 1.754Hz  -> 569.9 ms per tick
 * fRTC = 115000 / 2**15 = 115000/32768 = 3.509Hz  -> 284.9 ms per tick
 * fRTC = 115000 / 2**14 = 115000/16384 = 7.019Hz  -> 142.47 ms per tick
 * fRTC = 115000 / 2**13 = 115000/8192  = 14.04Hz  ->  71.23 ms per tick
 * fRTC = 115000 / 2**12 = 115000/4096  = 28.08Hz  ->  35.62 ms per tick
 * fRTC = 115000 / 2**11 = 115000/2048  = 56.15Hz  ->  17.81 ms per tick
 * fRTC = 115000 / 2**10 = 115000/1024  = 112.3Hz  ->   8.904 ms per tick
 * fRTC = 115000 / 2**9  = 115000/512   = 224.6Hz  ->   4.452 ms per tick
 * fRTC = 115000 / 2**8  = 115000/256   = 449.2Hz  ->   2.227 ms per tick
 * fRTC = 115000 / 2**7  = 115000/128   = 898.4Hz  ->   1.113 ms per tick
 * fRTC = 115000 / 2**6  = 115000/64    = 1.796KHz ->  556.5 us per tick
 * fRTC = 115000 / 2**5  = 115000/32    = 3.594KHz ->  278.3 us per tick
 * fRTC = 115000 / 2**4  = 115000/16    = 7.188KHz ->  139.1 us per tick
 * fRTC = 115000 / 2**3  = 115000/8     = 14.38KHz ->  69.57 us per tick
 * fRTC = 115000 / 2**2  = 115000/4     = 28.75KHz ->  34.78 us per tick
 * fRTC = 115000 / 2                    = 57.50KHz -> 17l.39 us per tick
 *
 * We'll use PSEL == 3 (fRTC == 69.57ns) and we will set TOP to 79.
 * Therefore, the TOP interrupt should occur after 143+1=144 counts
 * at a rate of 69.57us x 144 = 10.02 ms
 */
 
#ifdef AVR32_CLOCK_OSC32
#  define AV32_PSEL 1
#  define AV32_TOP (82-1)
#else
#  define AV32_PSEL 2
#  define AV32_TOP (144-1)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  rtc_busy
 *
 * Description:
 *   Make sure that the RTC is no busy before trying to operate on it.  If
 *   the RTC is busy, it will discard writes to TOP, VAL, and CTRL.
 *
 ****************************************************************************/

static void rtc_waitnotbusy(void)
{
  while ((getreg32(AVR32_RTC_CTRL) & RTC_CTRL_BUSY) != 0);
}

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
   /* Clear the pending timer interrupt */
 
   putreg32(RTC_INT_TOPI, AVR32_RTC_ICR);

   /* Process timer interrupt */

   sched_process_timer();
   return 0;
}

/****************************************************************************
 * Function:  up_timerinit
 *
 * Description:
 *   This function is called during start-up to initialize the timer
 *   interrupt.  NOTE:  This function depends on setup of OSC32 by
 *   up_clkinitialize().
 *
 ****************************************************************************/

void up_timerinit(void)
{
  uint32_t regval;

  /* Enable clocking: "The clock for the RTC bus interface (CLK_RTC) is generated
   * by the Power Manager. This clock is enabled at reset, and can be disabled
   * in the Power Manager. It is recommended to disable the RTC before disabling
   * the clock, to avoid freezing the RTC in an undefined state."
   */

#if 0
  regval = getreg32(AVR32_PM_PBAMASK);
  regval |= PM_PBAMASK_PMRTCEIC;
  putreg32(regval, AVR32_PM_PBAMASK);
#endif

  /* Configure the RTC.  Source == 32KHz OSC32 or RC OSC */

  rtc_waitnotbusy();
#ifdef AVR32_CLOCK_OSC32
  putreg32((RTC_CTRL_CLK32 | (AV32_PSEL << RTC_CTRL_PSEL_SHIFT) | RTC_CTRL_CLKEN),
           AVR32_RTC_CTRL);
#else
  putreg32(((AV32_PSEL << RTC_CTRL_PSEL_SHIFT) | RTC_CTRL_CLKEN),
           AVR32_RTC_CTRL);
#endif

  /* Set the counter value to zero and the TOP value to AVR32_TOP (see above) */

  rtc_waitnotbusy();
  putreg32(AV32_TOP, AVR32_RTC_TOP);
  rtc_waitnotbusy();
  putreg32(0, AVR32_RTC_VAL);

  /* Attach the timer interrupt vector */

  (void)irq_attach(AVR32_IRQ_RTC, (xcpt_t)up_timerisr);

  /* Enable RTC interrupts */

  putreg32(RTC_INT_TOPI, AVR32_RTC_IER);
  
  /* Enable the RTC */

  rtc_waitnotbusy();
  regval = getreg32(AVR32_RTC_CTRL);
  regval |= RTC_CTRL_EN;
  putreg32(regval, AVR32_RTC_CTRL);
}
