/****************************************************************************
 * arch/avr/src/at90usb/at90usb_timerisr.c
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
#include <avr/io.h>

#include "up_arch.h"

#include "at90usb_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* The CPU frequency is given by BOARD_CPU_CLOCK (defined in board.h).  The
 * desired interrupt frequency is given by CONFIG_MSEC_PER_TICK.  An unscaled
 * ideal match is given by:
 *
 *   CLOCK = CPU_CLOCK / DIVISOR                      # CPU clocks/sec
 *   MATCH = CLOCK / CLOCKS_PER_SEC                   # CPU clocks/timer tick
 *   MATCH = CPU_CLOCK / DIVISOR / CLOCKS_PER_SEC     # CPU clocks/timer tick
 *
 * But we only have 16-bits of accuracy so we need to pick the smallest
 * divisor using the following brute force calculation:
 */

#define MATCH1      (( BOARD_CPU_CLOCK                + CLOCKS_PER_SEC/2) / CLOCKS_PER_SEC)
#define MATCH8     ((((BOARD_CPU_CLOCK + 4) / 8)      + CLOCKS_PER_SEC/2) / CLOCKS_PER_SEC)
#define MATCH64    ((((BOARD_CPU_CLOCK + 8) / 64)     + CLOCKS_PER_SEC/2) / CLOCKS_PER_SEC)
#define MATCH256   ((((BOARD_CPU_CLOCK + 128) / 256)  + CLOCKS_PER_SEC/2) / CLOCKS_PER_SEC)
#define MATCH1024  ((((BOARD_CPU_CLOCK + 512) / 1024) + CLOCKS_PER_SEC/2) / CLOCKS_PER_SEC)

#if MATCH1 <= 65536
#  define MATCH   (MATCH1-1)
#  define PRESCALE 1
#elif MATCH8 <= 65536
#  define MATCH    (MATCH8-1)
#  define PRESCALE 2
#elif MATCH64 <= 65536
#  define MATCH    (MATCH64-1)
#  define PRESCALE 3
#elif MATCH256 <= 65536
#  define MATCH    (MATCH256-1)
#  define PRESCALE 4
#elif MATCH1024 <= 65536
#  define MATCH    (MATCH1024-1)
#  define PRESCALE 5
#else
#  error "Cannot represent this timer frequency"
#endif

/*
 * Eg. CPU_CLOCK = 8MHz, CLOCKS_PER_SEC = 100
 *
 *   MATCH1      ((8000000 + 50) / 100) = 80,000  FREQ=100.0Hz
 *   MATCH8      ((1000000 + 50) / 100) = 10,000  FREQ=100.0Hz <-- this one
 *   MATCH64     (( 125000 + 50) / 100) =  1,250  FREQ=100.0Hz
 *   MATCH256    ((  31250 + 50) / 100) =    313  FREQ=99.8Hz
 *   MATCH1024   ((   7804 + 50) / 100) =     78  FREQ=100.1Hz
 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Functions
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
 *   interrupt.  NOTE:  This function depends on setup of OSC32 by
 *   up_clkinitialize().
 *
 ****************************************************************************/

void up_timerinit(void)
{
  /* Setup timer 1 compare match A to generate a tick interrupt.
   *
   * First, setup the match value for compare match A.
   */

  OCR1AH = (uint8_t)((uint16_t)MATCH >> 8);
  OCR1AL = (uint8_t)((uint16_t)MATCH & 0xff);

  /* Setup clock source and compare match behaviour.
   *
   * TCRR1A:
   *   COM1A 0:1 = 00  -> Normal port operation
   *   COM1B 0:1 = 00  -> Normal port operation
   *   COM1C 0:1 = 00  -> Normal port operation
   *   WGM1  0:1 = 00  -> Clear Timer on Compare (CTC) modes of operation
   */

  TCCR1A = 0;

  /* TCCR1B:
   *   ICNC1     = 0   -> Input Capture Noise Canceler disabled
   *   ICES1     = 0   -> Input Capture Edge Select
   *   WGM   2:3 = 01  -> Clear Timer on Compare (CTC) modes of operation
   *   CS1   0:2 = xxx ->Selected pre-scaler.
   */

  TCCR1B = (1 << WGM12) | PRESCALE;

  /* Attach the timer interrupt vector */

  (void)irq_attach(AT90USB_IRQ_T1COMPA, (xcpt_t)up_timerisr);

  /* Enable the interrupt on compare match A */

  TIMSK1 |= (1 << OCIE1A);
}
