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
#include <uORB/topics/actuator_armed.h>
#include <poll.h>
#include <drivers/drv_gpio.h>
#include <modules/px4iofirmware/protocol.h>

struct gpio_led_s {
	struct work_s work;
	int gpio_fd;
	bool use_io;
	int pin;
	struct vehicle_status_s status;
	int vehicle_status_sub;
	struct actuator_armed_s armed;
	int actuator_armed_sub;
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
	if (argc < 2) {
		errx(1, "usage: gpio_led {start|stop} [-p <1|2|a1|a2|r1|r2>]\n"
		     "\t-p\tUse pin:\n"
		     "\t\t1\tPX4FMU GPIO_EXT1 (default)\n"
		     "\t\t2\tPX4FMU GPIO_EXT2\n"
		     "\t\ta1\tPX4IO ACC1\n"
		     "\t\ta2\tPX4IO ACC2\n"
		     "\t\tr1\tPX4IO RELAY1\n"
		     "\t\tr2\tPX4IO RELAY2");

	} else {

		if (!strcmp(argv[1], "start")) {
			if (gpio_led_started) {
				errx(1, "already running");
			}

			bool use_io = false;
			int pin = GPIO_EXT_1;

			if (argc > 2) {
				if (!strcmp(argv[2], "-p")) {
					if (!strcmp(argv[3], "1")) {
						use_io = false;
						pin = GPIO_EXT_1;

					} else if (!strcmp(argv[3], "2")) {
						use_io = false;
						pin = GPIO_EXT_2;

					} else if (!strcmp(argv[3], "a1")) {
						use_io = true;
						pin = PX4IO_P_SETUP_RELAYS_ACC1;

					} else if (!strcmp(argv[3], "a2")) {
						use_io = true;
						pin = PX4IO_P_SETUP_RELAYS_ACC2;

					} else if (!strcmp(argv[3], "r1")) {
						use_io = true;
						pin = PX4IO_P_SETUP_RELAYS_POWER1;

					} else if (!strcmp(argv[3], "r2")) {
						use_io = true;
						pin = PX4IO_P_SETUP_RELAYS_POWER2;

					} else {
						errx(1, "unsupported pin: %s", argv[3]);
					}
				}
			}

			memset(&gpio_led_data, 0, sizeof(gpio_led_data));
			gpio_led_data.use_io = use_io;
			gpio_led_data.pin = pin;
			int ret = work_queue(LPWORK, &gpio_led_data.work, gpio_led_start, &gpio_led_data, 0);

			if (ret != 0) {
				errx(1, "failed to queue work: %d", ret);

			} else {
				gpio_led_started = true;
				char pin_name[24];

				if (use_io) {
					if (pin & (PX4IO_P_SETUP_RELAYS_ACC1 | PX4IO_P_SETUP_RELAYS_ACC2)) {
						sprintf(pin_name, "PX4IO ACC%i", (pin >> 3));

					} else {
						sprintf(pin_name, "PX4IO RELAY%i", pin);
					}

				} else {
					sprintf(pin_name, "PX4FMU GPIO_EXT%i", pin);

				}

				warnx("start, using pin: %s", pin_name);
			}

			exit(0);


		} else if (!strcmp(argv[1], "stop")) {
			if (gpio_led_started) {
				gpio_led_started = false;
				warnx("stop");

			} else {
				errx(1, "not running");
			}

		} else {
			errx(1, "unrecognized command '%s', only supporting 'start' or 'stop'", argv[1]);
		}
	}
}

void gpio_led_start(FAR void *arg)
{
	FAR struct gpio_led_s *priv = (FAR struct gpio_led_s *)arg;

	char *gpio_dev;

	if (priv->use_io) {
		gpio_dev = PX4IO_DEVICE_PATH;
	} else {
		gpio_dev = PX4FMU_DEVICE_PATH;
	}

	/* open GPIO device */
	priv->gpio_fd = open(gpio_dev, 0);

	if (priv->gpio_fd < 0) {
		// TODO find way to print errors
		//printf("gpio_led: GPIO device \"%s\" open fail\n", gpio_dev);
		gpio_led_started = false;
		return;
	}

	/* configure GPIO pin */
	/* px4fmu only, px4io doesn't support GPIO_SET_OUTPUT and will ignore */
	ioctl(priv->gpio_fd, GPIO_SET_OUTPUT, priv->pin);

	/* subscribe to vehicle status topic */
	memset(&priv->status, 0, sizeof(priv->status));
	priv->vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));

	/* add worker to queue */
	int ret = work_queue(LPWORK, &priv->work, gpio_led_cycle, priv, 0);

	if (ret != 0) {
		// TODO find way to print errors
		//printf("gpio_led: failed to queue work: %d\n", ret);
		gpio_led_started = false;
		return;
	}
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
		orb_copy(ORB_ID(actuator_armed), priv->actuator_armed_sub, &priv->armed);

	/* select pattern for current status */
	int pattern = 0;

	if (priv->armed.armed) {
		if (priv->status.battery_warning == VEHICLE_BATTERY_WARNING_NONE) {
			pattern = 0x3f;	// ****** solid (armed)

		} else {
			pattern = 0x2A;	// *_*_*_ fast blink (armed, battery warning)
		}

	} else {
		if (priv->armed.ready_to_arm) {
			pattern = 0x00;	// ______ off (disarmed, preflight check)

		} else if (priv->armed.ready_to_arm && priv->status.battery_warning == VEHICLE_BATTERY_WARNING_NONE) {
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

	/* repeat cycle at 5 Hz */
	if (gpio_led_started) {
		work_queue(LPWORK, &priv->work, gpio_led_cycle, priv, USEC2TICK(200000));

	} else {
		/* switch off LED on stop */
		ioctl(priv->gpio_fd, GPIO_CLEAR, priv->pin);
	}
}
