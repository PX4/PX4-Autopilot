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
 * led driver for PX4FMU
 *
 * This is something of an experiment currently (ha, get it?)
 */

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/spi.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "chip.h"
#include "stm32_internal.h"
#include "px4fmu-internal.h"

#include <arch/board/drv_led.h>

static int	px4fmu_led_ioctl(struct file *filep, int cmd, unsigned long arg);
static ssize_t	px4fmu_led_pseudoread(struct file *filp, FAR char *buffer, size_t buflen);

static const struct file_operations px4fmu_led_fops = {
	.read  = px4fmu_led_pseudoread,
	.ioctl = px4fmu_led_ioctl,
};

int
px4fmu_led_init(void)
{
	/* register the driver */
	return register_driver("/dev/led", &px4fmu_led_fops, 0666, NULL);
}

static ssize_t
px4fmu_led_pseudoread(struct file *filp, FAR char *buffer, size_t buflen)
{
	return 0;
}

static int
px4fmu_led_ioctl(struct file *filep, int cmd, unsigned long arg)
{
	int	result = 0;

	switch (cmd) {

	case LED_ON:
		switch (arg) {
		case 0:
		case 1:
			up_ledon(arg);
			break;
		default:
			result = -1;
			break;
		}
		break;

	case LED_OFF:
		switch (arg) {
		case 0:
		case 1:
			up_ledoff(arg);
			break;
		default:
			result = -1;
			break;
		}
		break;
		default:
			result = -1;
			break;
	}
	return result;
}

