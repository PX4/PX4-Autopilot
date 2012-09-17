/****************************************************************************
 * Driver for shared features of ARMIO modules
 *
 *   Copyright (C) 2011 Stefan Richter. All rights reserved.
 *   Author: Stefan Richter <ichgeh@l--putt.de>
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

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <arch/calypso/memory.h>
#include <arch/calypso/armio.h>

#include "up_arch.h"

/****************************************************************************
 * HW access
 ****************************************************************************/

#define BASE_ADDR_ARMIO	0xfffe4800
#define ARMIO_REG(x)	((void *)BASE_ADDR_ARMIO + (x))

enum armio_reg {
	LATCH_IN	= 0x00,
	LATCH_OUT	= 0x02,
	IO_CNTL		= 0x04,
	CNTL_REG	= 0x06,
	LOAD_TIM	= 0x08,
	KBR_LATCH_REG	= 0x0a,
	KBC_REG		= 0x0c,
	BUZZ_LIGHT_REG	= 0x0e,
	LIGHT_LEVEL	= 0x10,
	BUZZER_LEVEL	= 0x12,
	GPIO_EVENT_MODE	= 0x14,
	KBD_GPIO_INT	= 0x16,
	KBD_GPIO_MASKIT	= 0x18,
	GPIO_DEBOUNCING	= 0x1a,
	GPIO_LATCH	= 0x1c,
};

#define KBD_INT		(1<<0)
#define GPIO_INT	(1<<1)


/****************************************************************************
 * ARMIO interrupt handler
 *   forward keypad events
 *   forward GPIO events
 ****************************************************************************/

static int kbd_gpio_irq(int irq, uint32_t *regs)
{
	calypso_kbd_irq();

	return 0;
}


/****************************************************************************
 * Initialize ARMIO
 ****************************************************************************/

void calypso_armio(void)
{
	/* Enable ARMIO clock */
	putreg16(1<<5, ARMIO_REG(CNTL_REG));

	/* Mask GPIO interrupt and keypad interrupt */
	putreg16(KBD_INT|GPIO_INT, ARMIO_REG(KBD_GPIO_MASKIT));

	/* Attach and enable the interrupt */
	irq_attach(IRQ_KEYPAD_GPIO, (xcpt_t)kbd_gpio_irq);
	up_enable_irq(IRQ_KEYPAD_GPIO);
}
