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
 * Status LED via GPIO driver.
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
#include <systemlib/err.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_safety.h>
#include <poll.h>
#include <drivers/drv_gpio.h>

struct gpio_led_s {
	struct work_s work;
	int gpio_fd;
	int pin;
	struct vehicle_status_s status;
	int vehicle_status_sub;
	struct actuator_safety_s safety;
	int actuator_safety_sub;
	bool led_state;
	int counter;
};

static struct gpio_led_s gpio_led_data;
static bool gpio_led_started = false;

__EXPORT int gpio_led_main(int argc, char *argv[]);

void gpio_led_start(FAR void *arg);

void gpio_led_cycle(FAR void *arg);

int gpio_led_main(int argc, char *argv[])
{
	int pin = GPIO_EXT_1;

	if (argc < 2) {
		errx(1, "no argument provided. Try 'start' or 'stop' [-p 1/2]");

	} else {

		/* START COMMAND HANDLING */
		if (!strcmp(argv[1], "start")) {

			if (argc > 2) {
				if (!strcmp(argv[1], "-p")) {
					if (!strcmp(argv[2], "1")) {
						pin = GPIO_EXT_1;

					} else if (!strcmp(argv[2], "2")) {
						pin = GPIO_EXT_2;

					} else {
						warnx("[gpio_led] Unsupported pin: %s\n", argv[2]);
						exit(1);
					}
				}
			}

			memset(&gpio_led_data, 0, sizeof(gpio_led_data));
			gpio_led_data.pin = pin;
			int ret = work_queue(LPWORK, &gpio_led_data.work, gpio_led_start, &gpio_led_data, 0);

			if (ret != 0) {
				warnx("[gpio_led] Failed to queue work: %d\n", ret);
				exit(1);

			} else {
				gpio_led_started = true;
			}

			exit(0);

			/* STOP COMMAND HANDLING */

		} else if (!strcmp(argv[1], "stop")) {
			gpio_led_started = false;

			/* INVALID COMMAND */

		} else {
			errx(1, "unrecognized command '%s', only supporting 'start' or 'stop'", argv[1]);
		}
	}
}

void gpio_led_start(FAR void *arg)
{
	FAR struct gpio_led_s *priv = (FAR struct gpio_led_s *)arg;

	/* open GPIO device */
	priv->gpio_fd = open(GPIO_DEVICE_PATH, 0);

	if (priv->gpio_fd < 0) {
		warnx("[gpio_led] GPIO: open fail\n");
		return;
	}

	/* configure GPIO pin */
	ioctl(priv->gpio_fd, GPIO_SET_OUTPUT, priv->pin);

	/* subscribe to vehicle status topic */
	memset(&priv->status, 0, sizeof(priv->status));
	priv->vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));

	/* add worker to queue */
	int ret = work_queue(LPWORK, &priv->work, gpio_led_cycle, priv, 0);

	if (ret != 0) {
		warnx("[gpio_led] Failed to queue work: %d\n", ret);
		return;
	}

	warnx("[gpio_led] Started, using pin GPIO_EXT%i\n", priv->pin);
}

void gpio_led_cycle(FAR void *arg)
{
	FAR struct gpio_led_s *priv = (FAR struct gpio_led_s *)arg;

	/* check for status updates*/
	bool status_updated;
	orb_check(priv->vehicle_status_sub, &status_updated);

	if (status_updated)
		orb_copy(ORB_ID(vehicle_status), priv->vehicle_status_sub, &priv->status);

	orb_check(priv->vehicle_status_sub, &status_updated);

	if (status_updated)
		orb_copy(ORB_ID(actuator_safety), priv->actuator_safety_sub, &priv->safety);

	/* select pattern for current status */
	int pattern = 0;

	if (priv->safety.armed) {
		if (priv->status.battery_warning == VEHICLE_BATTERY_WARNING_NONE) {
			pattern = 0x3f;	// ****** solid (armed)

		} else {
			pattern = 0x2A;	// *_*_*_ fast blink (armed, battery warning)
		}

	} else {
		if (priv->safety.ready_to_arm) {
			pattern = 0x00;	// ______ off (disarmed, preflight check)

		} else if (priv->safety.ready_to_arm && priv->status.battery_warning == VEHICLE_BATTERY_WARNING_NONE) {
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
			ioctl(priv->gpio_fd, GPIO_SET, priv->pin);

		} else {
			ioctl(priv->gpio_fd, GPIO_CLEAR, priv->pin);
		}
	}

	priv->counter++;

	if (priv->counter > 5)
		priv->counter = 0;

	/* repeat cycle at 5 Hz*/
	if (gpio_led_started)
		work_queue(LPWORK, &priv->work, gpio_led_cycle, priv, USEC2TICK(200000));
}
