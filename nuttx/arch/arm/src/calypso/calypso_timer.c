/****************************************************************************
 * arch/arm/src/calypso/calypso_timer.c
 * Calypso DBB internal Timer Driver
 *
 * (C) 2010 by Harald Welte <laforge@gnumonks.org>
 * (C) 2011 by Stefan Richter <ichgeh@l--putt.de>
 *
 * This source code is derivated from Osmocom-BB project and was
 * relicensed as BSD with permission from original authors.
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
 **************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <nuttx/arch.h>

#include <arch/calypso/defines.h>
#include <arch/calypso/memory.h>
#include <arch/calypso/timer.h>

#include "up_arch.h"

#define BASE_ADDR_TIMER		0xfffe3800
#define TIMER2_OFFSET		0x3000

#define TIMER_REG(n, m)		(((n)-1) ? (BASE_ADDR_TIMER + TIMER2_OFFSET + (m)) : (BASE_ADDR_TIMER + (m)))

enum timer_reg {
	CNTL_TIMER	= 0x00,
	LOAD_TIMER	= 0x02,
	READ_TIMER	= 0x04,
};

enum timer_ctl {
	CNTL_START		= (1 << 0),
	CNTL_AUTO_RELOAD	= (1 << 1),
	CNTL_CLOCK_ENABLE	= (1 << 5),
};

/* Regular Timers (1 and 2) */

void hwtimer_enable(int num, int on)
{
	uint8_t ctl;

	if (num < 1 || num > 2) {
		printf("Unknown timer %u\n", num);
		return;
	}

	ctl = getreg8(TIMER_REG(num, CNTL_TIMER));
	if (on)
		ctl |= CNTL_START|CNTL_CLOCK_ENABLE;
	else
		ctl &= ~CNTL_START;
	putreg8(ctl, TIMER_REG(num, CNTL_TIMER));
}

void hwtimer_config(int num, uint8_t pre_scale, int auto_reload)
{
	uint8_t ctl;

	ctl = (pre_scale & 0x7) << 2;
	if (auto_reload)
		ctl |= CNTL_AUTO_RELOAD;

	putreg8(ctl, TIMER_REG(num, CNTL_TIMER));
}

void hwtimer_load(int num, uint16_t val)
{
	putreg16(val, TIMER_REG(num, LOAD_TIMER));
}

uint16_t hwtimer_read(int num)
{
	uint8_t ctl = getreg8(TIMER_REG(num, CNTL_TIMER));

	/* somehow a read results in an abort */
	if ((ctl & (CNTL_START|CNTL_CLOCK_ENABLE)) != (CNTL_START|CNTL_CLOCK_ENABLE))
		return 0xFFFF;
	return getreg16(TIMER_REG(num, READ_TIMER));
}

/************************************************************
 * Watchdog Timer
 ************************************************************/

#define BASE_ADDR_WDOG		0xfffff800
#define WDOG_REG(m)		(BASE_ADDR_WDOG + m)

enum wdog_reg {
	WD_CNTL_TIMER	= CNTL_TIMER,
	WD_LOAD_TIMER	= LOAD_TIMER,
	WD_READ_TIMER	= 0x02,
	WD_MODE		= 0x04,
};

enum wdog_ctl {
	WD_CTL_START = (1 << 7),
	WD_CTL_AUTO_RELOAD = (1 << 8)
};

enum wdog_mode {
	WD_MODE_DIS_ARM = 0xF5,
	WD_MODE_DIS_CONFIRM = 0xA0,
	WD_MODE_ENABLE = (1 << 15)
};

#define WD_CTL_PRESCALE(value) (((value)&0x07) << 9)

static void wdog_irq(__unused enum irq_nr nr)
{
	puts("=> WATCHDOG\n");
}

void wdog_enable(int on)
{
	if (!on) {
		putreg16(WD_MODE_DIS_ARM, WDOG_REG(WD_MODE));
		putreg16(WD_MODE_DIS_CONFIRM, WDOG_REG(WD_MODE));
	}
}

void wdog_reset(void)
{
	// enable watchdog
	putreg16(WD_MODE_ENABLE, WDOG_REG(WD_MODE));
	// force expiration
	putreg16(0x0000, WDOG_REG(WD_LOAD_TIMER));
	putreg16(0x0000, WDOG_REG(WD_LOAD_TIMER));
}

/************************************************************
 * Global Functions
 ************************************************************/

/************************************************************
 * Function:  up_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for
 *   various portions of the systems.
 *
 ************************************************************/

int up_timerisr(int irq, uint32_t *regs)
{
   /* Process timer interrupt */

   sched_process_timer();
   return 0;
}

/************************************************************
 * Function:  up_timerinit
 *
 * Description:
 *   Setup Calypso HW timer 2 to cause system ticks.
 *
 *   This function is called during start-up to initialize
 *   the timer interrupt.
 *
 ************************************************************/

void up_timerinit(void)
{
  up_disable_irq(IRQ_SYSTIMER);

  /* The timer runs at 13MHz / 32, i.e. 406.25kHz */
  /* 4062 ticks until expiry yields 100Hz interrupt */
  hwtimer_load(2, 4062);
  hwtimer_config(2, 0, 1);
  hwtimer_enable(2, 1);

  /* Attach and enable the timer interrupt */
  irq_attach(IRQ_SYSTIMER, (xcpt_t)up_timerisr);
  up_enable_irq(IRQ_SYSTIMER);
}

