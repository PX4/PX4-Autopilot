/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: Anton Babushkin <anton.babushkin@me.com>
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
 * @file gpio_led.c
 *
 * Drive status LED via GPIO.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <stdbool.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>
#include <systemlib/systemlib.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_status.h>
#include <poll.h>
#include <drivers/drv_gpio.h>


struct gpio_led_s
{
	struct work_s work;
	int gpio_fd;
	struct vehicle_status_s status;
	int vehicle_status_sub;
	bool led_state;
	int counter;
};

static struct gpio_led_s gpio_led;

__EXPORT int gpio_led_main(int argc, char *argv[]);

void gpio_led_start(FAR void *arg);

void gpio_led_cycle(FAR void *arg);

int gpio_led_main(int argc, char *argv[])
{
	memset(&gpio_led, 0, sizeof(gpio_led));
	int ret = work_queue(LPWORK, &gpio_led.work, gpio_led_start, &gpio_led, 0);
	if (ret != 0) {
		printf("[gpio_led] Failed to queue work: %d\n", ret);
		exit(1);
	}
	exit(0);
}

void gpio_led_start(FAR void *arg)
{
	FAR struct gpio_led_s *priv = (FAR struct gpio_led_s *)arg;

	/* open GPIO device */
	priv->gpio_fd = open(GPIO_DEVICE_PATH, 0);
	if (priv->gpio_fd < 0) {
		printf("[gpio_led] GPIO: open fail\n");
		return;
	}

	/* configure GPIO pin */
	ioctl(priv->gpio_fd, GPIO_SET_OUTPUT, GPIO_EXT_1);

	/* subscribe to vehicle status topic */
	memset(&priv->status, 0, sizeof(priv->status));
	priv->vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));

	/* add worker to queue */
	int ret = work_queue(LPWORK, &priv->work, gpio_led_cycle, priv, 0);
	if (ret != 0) {
		printf("[gpio_led] Failed to queue work: %d\n", ret);
		return;
	}

	printf("[gpio_led] Started\n");
}

void gpio_led_cycle(FAR void *arg)
{
	FAR struct gpio_led_s *priv = (FAR struct gpio_led_s *)arg;

	/* check for status updates*/
	bool status_updated;
	orb_check(priv->vehicle_status_sub, &status_updated);
	if (status_updated)
		orb_copy(ORB_ID(vehicle_status), priv->vehicle_status_sub, &priv->status);

	/* select pattern for current status */
	int pattern = 0;
	if (priv->status.flag_system_armed) {
		if (priv->status.battery_warning == VEHICLE_BATTERY_WARNING_NONE) {
			pattern = 0x3f;	// ****** solid (armed)
		} else {
			pattern = 0x2A;	// *_*_*_ fast blink (armed, battery warning)
		}
	} else {
		if (priv->status.state_machine == SYSTEM_STATE_PREFLIGHT) {
			pattern = 0x00;	// ______ off (disarmed, preflight check)
		} else if (priv->status.state_machine == SYSTEM_STATE_STANDBY &&
				   priv->status.battery_warning == VEHICLE_BATTERY_WARNING_NONE) {
			pattern = 0x38;	// ***___ slow blink (disarmed, ready)
		} else {
			pattern = 0x28;	// *_*___ slow double blink (disarmed, not good to arm)
		}
	}

	/* blink pattern */
	bool led_state_new = (pattern & (1 << priv->counter)) != 0;
	if (led_state_new != priv->led_state) {
		priv->led_state = led_state_new;
		if (led_state_new) {
			ioctl(priv->gpio_fd, GPIO_SET, GPIO_EXT_1);
		} else {
			ioctl(priv->gpio_fd, GPIO_CLEAR, GPIO_EXT_1);
		}
	}

	priv->counter++;
	if (priv->counter > 5)
		priv->counter = 0;

	/* repeat cycle at 5 Hz*/
	work_queue(LPWORK, &priv->work, gpio_led_cycle, priv, USEC2TICK(200000));
}
