/****************************************************************************
 *
 *   Copyright (C) 2020, 2023 PX4 Development Team. All rights reserved.
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
#include "hardware/imx9_gpio.h"

/****************************************************************************
 * Name: imx9_gpiosetevent
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
#if defined(CONFIG_IMX9_GPIO_IRQ)
int imx9_gpiosetevent(uint32_t pinset, bool risingedge, bool fallingedge,
		      bool event, xcpt_t func, void *arg)
{
	int ret = -ENOSYS;

	pinset &= ~(GPIO_INTCFG_MASK | GPIO_INTBOTHCFG_MASK);

	if (func == NULL || !(risingedge || fallingedge)) {
		imx9_gpioirq_disable(pinset);
		imx9_gpioirq_attach(pinset, NULL, NULL);
		ret = imx9_config_gpio(pinset);

	} else {

		if (risingedge & fallingedge) {
			pinset |= GPIO_INTBOTH_EDGES;

		} else if (risingedge) {
			pinset |= GPIO_INT_RISINGEDGE;

		} else if (fallingedge) {
			pinset |= GPIO_INT_FALLINGEDGE;
		}

		ret = imx9_config_gpio(pinset);

		if (ret == OK) {
			ret = imx9_gpioirq_configure(pinset);
		}

		if (ret == OK) {
			ret = imx9_gpioirq_attach(pinset, func, arg);
		}

		if (ret == OK) {
			ret = imx9_gpioirq_enable(pinset);
		}

	}

	return ret;
}
#endif /* CONFIG_IMX9_GPIO_IRQ */
