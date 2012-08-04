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

/**
 * @file GPIO driver for PX4IO.
 */

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdbool.h>

#include <errno.h>

#include <arch/board/board.h>
#include <arch/board/drv_gpio.h>

#include "px4io_internal.h"
#include "stm32_gpio.h"

static int	gpio_ioctl(struct file *filep, int cmd, unsigned long arg);

static const struct file_operations gpio_fops = {
	.ioctl = gpio_ioctl
};

/*
 * Order of initialisers in this array must match the order of
 * GPIO_ definitions in drv_gpio.h
 */
static const uint32_t gpios[] = {
	/* settable */
	GPIO_ACC1_PWR_EN,
	GPIO_ACC2_PWR_EN,
	GPIO_SERVO_PWR_EN,
	GPIO_RELAY1_EN,
	GPIO_RELAY2_EN,
	GPIO_LED1,
	GPIO_LED2,
	GPIO_LED3,

	/* readonly */
	GPIO_ACC_OC_DETECT,
	GPIO_SERVO_OC_DETECT,
	GPIO_BTN_SAFETY
};

int
gpio_drv_init(void)
{
	int i;

	/* initialise GPIOs */
	for (i = 0; i < GPIO_MAX; i++)
		if (gpios[i])
			stm32_configgpio(gpios[i]);

	/* register the device */
	return register_driver("/dev/gpio", &gpio_fops, 0666, NULL);
}

static int
gpio_ioctl(struct file *filep, int cmd, unsigned long arg)
{
	/* attempt to set a GPIO? */
	if ((cmd >= GPIO_SET(0)) && (cmd <= GPIO_SET(GPIO_MAX_SETTABLE))) {
		uint32_t gpio = gpios[cmd - GPIO_SET(0)];

		if (gpio != 0) {
			stm32_gpiowrite(gpio, arg ? true : false);
			return 0;
		}
	} else if ((cmd >= GPIO_GET(0)) && (cmd <= GPIO_GET(GPIO_MAX))) {
		uint32_t gpio = gpios[cmd - GPIO_GET(0)];

		if (gpio != 0)
			return stm32_gpioread(gpio) ? 1 : 0;
	}
	return -ENOTTY;
}