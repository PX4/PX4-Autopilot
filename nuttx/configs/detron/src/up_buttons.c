/****************************************************************************
 * configs/detron/src/up_buttons.c
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#include <nuttx/irq.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "lpc17_internal.h"
#include "detron_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static xcpt_t g_irq_pin_panel_pulse_up;
static xcpt_t g_irq_pin_panel_pulse_down;
static xcpt_t g_irq_pin_panel_center;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_buttoninit
 ****************************************************************************/

void up_buttoninit(void)
{
	lpc17_configgpio(pin_panel_pulse_up);
	lpc17_configgpio(pin_panel_pulse_down);
	lpc17_configgpio(pin_panel_center);
}

/****************************************************************************
 * Name: up_buttons
 ****************************************************************************/

uint8_t up_buttons(void)
{
  uint8_t retval;

  retval  = lpc17_gpioread(pin_panel_pulse_up) ? 0 : pin_panel_pulse_up;
  retval |= lpc17_gpioread(pin_panel_pulse_down) ? 0 : pin_panel_pulse_down;
  retval |= lpc17_gpioread(pin_panel_center) ? 0 : pin_panel_center;
  return retval;
}

/****************************************************************************
 * Name: up_irq_pin_panel_pulse_up
 ****************************************************************************/

xcpt_t up_irq_pin_panel_pulse_up(xcpt_t irqhandler)
{
	xcpt_t oldhandler;
	irqstate_t flags;

	/* Get the old button interrupt handler and save the new one */

	oldhandler   = g_irq_pin_panel_pulse_up;
	g_irq_pin_panel_pulse_up = irqhandler;

	/* Disable interrupts until we are done */

	flags = irqsave();

	/* Configure the interrupt */

	(void)irq_attach(IRQ_pin_panel_pulse_up, irqhandler);
	up_enable_irq(IRQ_pin_panel_pulse_up);

	irqrestore(flags);

	return oldhandler;
}

/****************************************************************************
 * Name: up_irq_pin_panel_pulse_down
 *  ****************************************************************************/

xcpt_t up_irq_pin_panel_pulse_down(xcpt_t irqhandler)
{
	xcpt_t oldhandler;
	irqstate_t flags;

	/* Get the old button interrupt handler and save the new one */

	oldhandler   = g_irq_pin_panel_pulse_down;
	g_irq_pin_panel_pulse_down = irqhandler;

	/* Disable interrupts until we are done */

	flags = irqsave();

	/* Configure the interrupt */

	(void)irq_attach(IRQ_pin_panel_pulse_down, irqhandler);
	up_enable_irq(IRQ_pin_panel_pulse_down);

	irqrestore(flags);

	return oldhandler;
}

/****************************************************************************
 * Name: up_irq_pin_panel_center
 *  ****************************************************************************/

xcpt_t up_irq_pin_panel_center(xcpt_t irqhandler)
{
	xcpt_t oldhandler;
	irqstate_t flags;

	/* Get the old button interrupt handler and save the new one */

	oldhandler   = g_irq_pin_panel_center;
	g_irq_pin_panel_center = irqhandler;

	/* Disable interrupts until we are done */

	flags = irqsave();

	/* Configure the interrupt */

    (void)irq_attach(IRQ_pin_panel_center, irqhandler);
	up_enable_irq(IRQ_pin_panel_center);

	irqrestore(flags);

	return oldhandler;
}

