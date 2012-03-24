/****************************************************************************
 * calypso_heap.c
 * Initialize memory interfaces of Calypso MCU
 *
 *   (C) 2010 by Harald Welte <laforge@gnumonks.org>
 *   (C) 2011 Stefan Richter <ichgeh@l--putt.de>
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
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include <arch/calypso/clock.h>
#include <arch/calypso/timer.h>

#include "up_arch.h"
#include "up_internal.h"

/****************************************************************************
 * Name: up_addregion
 *
 * Description:
 *   This function is called right after basics are initialized and right
 *   before IRQ system setup.
 *
 ****************************************************************************/

void up_addregion(void)
{
#ifdef CONFIG_ARCH_BOARD_COMPALE99
	/* Disable watchdog in first non-common function */
	wdog_enable(0);
#endif
	// XXX: change to initialization of extern memory with save defaults
	/* Configure memory interface */
	calypso_mem_cfg(CALYPSO_nCS0, 3, CALYPSO_MEM_16bit, 1);
	calypso_mem_cfg(CALYPSO_nCS1, 3, CALYPSO_MEM_16bit, 1);
	calypso_mem_cfg(CALYPSO_nCS2, 5, CALYPSO_MEM_16bit, 1);
	calypso_mem_cfg(CALYPSO_nCS3, 5, CALYPSO_MEM_16bit, 1);
	calypso_mem_cfg(CALYPSO_CS4, 0, CALYPSO_MEM_8bit, 1);
	calypso_mem_cfg(CALYPSO_nCS6, 0, CALYPSO_MEM_32bit, 1);
	calypso_mem_cfg(CALYPSO_nCS7, 0, CALYPSO_MEM_32bit, 0);

	/* Set VTCXO_DIV2 = 1, configure PLL for 104 MHz and give ARM half of that */
	calypso_clock_set(2, CALYPSO_PLL13_104_MHZ, ARM_MCLK_DIV_2);

	/* Configure the RHEA bridge with some sane default values */
	calypso_rhea_cfg(0, 0, 0xff, 0, 1, 0, 0);
}
