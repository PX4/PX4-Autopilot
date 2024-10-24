/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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

#include <px4_platform_common/px4_config.h>
#include <systemlib/px4_macros.h>

#include <arch/board/board.h>

#include <errno.h>

#include "chip.h"
#include "imxrt_irq.h"
#include "hardware/imxrt_gpio.h"

typedef struct {
	int low;
	int hi;
} lh_t;


const lh_t port_to_irq[13] = {
	{_IMXRT_GPIO1_0_15_BASE, _IMXRT_GPIO1_16_31_BASE},
	{_IMXRT_GPIO2_0_15_BASE, _IMXRT_GPIO2_16_31_BASE},
	{_IMXRT_GPIO3_0_15_BASE, _IMXRT_GPIO3_16_31_BASE},
	{_IMXRT_GPIO4_0_15_BASE, _IMXRT_GPIO4_16_31_BASE},
	{_IMXRT_GPIO5_0_15_BASE, _IMXRT_GPIO5_16_31_BASE},
	{_IMXRT_GPIO6_0_15_BASE, _IMXRT_GPIO6_16_31_BASE},
	{0, 0}, // GPIO7 Not on CM7
	{0, 0}, // GPIO8 Not on CM7
	{0, 0}, // GPIO9 Not on CM7
	{0, 0}, // GPIO10 Not on CM7
	{0, 0}, // GPIO11 Not on CM7
	{0, 0}, // GPIO12 Not on CM7
	{_IMXRT_GPIO13_BASE, _IMXRT_GPIO13_BASE},
};

static bool imxrt_pin_irq_valid(gpio_pinset_t pinset)
{
	int port   = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
	lh_t irqlh = port_to_irq[port];
	return (irqlh.low != 0 && irqlh.hi != 0);
}
/****************************************************************************
 * Name: imxrt_pin_irqattach
 *
 * Description:
 *   Sets/clears GPIO based event and interrupt triggers.
 *
 * Input Parameters:
 *  - pinset: gpio pin configuration
 *  - rising/falling edge: enables
 *  - func:   when non-NULL, generate interrupt
 *  - arg:    Argument passed to the interrupt callback
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure indicating the
 *   nature of the failure.
 *
 ****************************************************************************/

static int imxrt_pin_irqattach(gpio_pinset_t pinset, xcpt_t func, void *arg)
{
	int rv = -EINVAL;
	volatile int port   = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
	volatile int pin    = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
	volatile int irq;
	lh_t irqlh = port_to_irq[port];

	if (irqlh.low != 0 && irqlh.hi != 0) {
		rv = OK;
		irq = (pin < 16) ? irqlh.low : irqlh.hi;
		irq += pin % 16;
		irq_attach(irq, func, arg);
		up_enable_irq(irq);
	}

	return rv;
}

/****************************************************************************
 * Name: imxrt_gpiosetevent
 *
 * Description:
 *   Sets/clears GPIO based event and interrupt triggers.
 *
 * Input Parameters:
 *  - pinset: gpio pin configuration
 *  - rising/falling edge: enables
 *  - event:  generate event when set
 *  - func:   when non-NULL, generate interrupt
 *  - arg:    Argument passed to the interrupt callback
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure indicating the
 *   nature of the failure.
 *
 ****************************************************************************/
#if defined(CONFIG_IMXRT_GPIO_IRQ)
int imxrt_gpiosetevent(uint32_t pinset, bool risingedge, bool fallingedge,
		       bool event, xcpt_t func, void *arg)
{
	int ret = -ENOSYS;

	if (imxrt_pin_irq_valid(pinset)) {
		if (func == NULL) {
			imxrt_gpioirq_disable(pinset);
			pinset &= ~GPIO_INTCFG_MASK;
			ret = imxrt_config_gpio(pinset);

		} else {

			pinset &= ~GPIO_INTCFG_MASK;

			if (risingedge & fallingedge) {
				pinset |= GPIO_INTBOTH_EDGES;

			} else if (risingedge) {
				pinset |= GPIO_INT_RISINGEDGE;

			} else if (fallingedge) {
				pinset |= GPIO_INT_FALLINGEDGE;
			}

			imxrt_gpioirq_configure(pinset);
			ret = imxrt_pin_irqattach(pinset, func, arg);
		}
	}

	return ret;
}
#endif /* CONFIG_IMXRT_GPIO_IRQ */
