/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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

/*
 * GPIO driver for PX4FMU.
 *
 */

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/spi.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "chip.h"
#include "stm32_internal.h"
#include "px4fmu-internal.h"

#include <arch/board/drv_gpio.h>

static int	px4fmu_gpio_ioctl(struct file *filep, int cmd, unsigned long arg);

static const struct file_operations px4fmu_gpio_fops = {
	.ioctl = px4fmu_gpio_ioctl,
};

static struct {
	uint32_t	input;
	uint32_t	output;
	uint32_t	alt;
} gpio_tab[] = {
	{GPIO_GPIO0_INPUT, GPIO_GPIO0_OUTPUT, 0},
	{GPIO_GPIO1_INPUT, GPIO_GPIO1_OUTPUT, 0},
	{GPIO_GPIO2_INPUT, GPIO_GPIO2_OUTPUT, GPIO_USART2_CTS_1},
	{GPIO_GPIO3_INPUT, GPIO_GPIO3_OUTPUT, GPIO_USART2_RTS_1},
	{GPIO_GPIO4_INPUT, GPIO_GPIO4_OUTPUT, GPIO_USART2_TX_1},
	{GPIO_GPIO5_INPUT, GPIO_GPIO5_OUTPUT, GPIO_USART2_RX_1},
	{GPIO_GPIO6_INPUT, GPIO_GPIO6_OUTPUT, GPIO_CAN2_TX_2},
	{GPIO_GPIO7_INPUT, GPIO_GPIO7_OUTPUT, GPIO_CAN2_RX_2},
};

#define NGPIO	(sizeof(gpio_tab) / sizeof(gpio_tab[0]))


static void
px4fmu_gpio_reset(void)
{
	/*
	 * Setup default GPIO config - all pins as GPIOs, GPIO driver chip 
	 * to input mode.
	 */
	for (unsigned i = 0; i < NGPIO; i++)
		stm32_configgpio(gpio_tab[i].input);

	stm32_gpiowrite(GPIO_GPIO_DIR, 0);
	stm32_configgpio(GPIO_GPIO_DIR);
}

static void
px4fmu_gpio_set_function(uint32_t gpios, int function)
{
	/*
	 * GPIOs 0 and 1 must have the same direction as they are buffered
	 * by a shared 2-port driver.  Any attempt to set either sets both.
	 */
	if (gpios & 3) {
		gpios |= 3;

		/* flip the buffer to output mode if required */
		if (GPIO_SET_OUTPUT == function)
			stm32_gpiowrite(GPIO_GPIO_DIR, 1);
	}

	/* configure selected GPIOs as required */
	for (unsigned i = 0; i < NGPIO; i++) {
		if (gpios & (1<<i)) {
			switch (function) {
			case GPIO_SET_INPUT:
				stm32_configgpio(gpio_tab[i].input);
				break;
			case GPIO_SET_OUTPUT:
				stm32_configgpio(gpio_tab[i].output);
				break;
			case GPIO_SET_ALT_1:
				if (gpio_tab[i].alt != 0)
					stm32_configgpio(gpio_tab[i].alt);
				break;
			}
		}
	}

	/* flip buffer to input mode if required */
	if ((GPIO_SET_INPUT == function) && (gpios & 3))
		stm32_gpiowrite(GPIO_GPIO_DIR, 0);
}

static void
px4fmu_gpio_write(uint32_t gpios, int function)
{
	int value = (function == GPIO_SET) ? 1 : 0;

	for (unsigned i = 0; i < NGPIO; i++)
		if (gpios & (1<<i))
			stm32_gpiowrite(gpio_tab[i].output, value);
}

static uint32_t
px4fmu_gpio_read(void)
{
	uint32_t bits = 0;

	for (unsigned i = 0; i < NGPIO; i++)
		if (stm32_gpioread(gpio_tab[i].input))
			bits |= (1 << i);

	return bits;
}

void
px4fmu_gpio_init(void)
{
	/* reset all GPIOs to default state */
	px4fmu_gpio_reset();

	/* register the driver */
	register_driver(GPIO_DEVICE_PATH, &px4fmu_gpio_fops, 0666, NULL);
}

static int
px4fmu_gpio_ioctl(struct file *filep, int cmd, unsigned long arg)
{
	int	result = OK;

	switch (cmd) {
		
	case GPIO_RESET:
		px4fmu_gpio_reset();
		break;

	case GPIO_SET_OUTPUT:
	case GPIO_SET_INPUT:
	case GPIO_SET_ALT_1:
		px4fmu_gpio_set_function(arg, cmd);
		break;

	case GPIO_SET:
	case GPIO_CLEAR:
		px4fmu_gpio_write(arg, cmd);
		break;

	case GPIO_GET:
		*(uint32_t *)arg = px4fmu_gpio_read();
		break;

	default:
		result = -ENOTTY;
	}
	return result;
}

