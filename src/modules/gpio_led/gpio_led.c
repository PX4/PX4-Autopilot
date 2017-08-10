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
#include <uORB/topics/battery_status.h>
#include <poll.h>
#include <drivers/drv_gpio.h>
#include <modules/px4iofirmware/protocol.h>

#define PIN_NAME "AUX OUT 1"

/* Minimum pin number */
#define GPIO_MIN_SERVO_PIN 1

/* Maximum */
#if defined(GPIO_SERVO_16)
#  define GPIO_MAX_SERVO_PIN 16
#elif defined(GPIO_SERVO_15)
#  define GPIO_MAX_SERVO_PIN 15
#elif defined(GPIO_SERVO_14)
#  define GPIO_MAX_SERVO_PIN 14
#elif defined(GPIO_SERVO_13)
#  define GPIO_MAX_SERVO_PIN 13
#elif defined(GPIO_SERVO_12)
#  define GPIO_MAX_SERVO_PIN 12
#elif defined(GPIO_SERVO_11)
#  define GPIO_MAX_SERVO_PIN 11
#elif defined(GPIO_SERVO_10)
#  define GPIO_MAX_SERVO_PIN 10
#elif defined(GPIO_SERVO_9)
#  define GPIO_MAX_SERVO_PIN 9
#elif defined(GPIO_SERVO_8)
#  define GPIO_MAX_SERVO_PIN 8
#elif defined(GPIO_SERVO_7)
#  define GPIO_MAX_SERVO_PIN 7
#elif defined(GPIO_SERVO_6)
#  define GPIO_MAX_SERVO_PIN 6
#else
#  error "Board must define GPIO_SERVO_1 and GPIO_SERVO_n where n is 6-16"

#endif

struct gpio_led_s {
	struct work_s work;
	int gpio_fd;
	int pin;
	struct vehicle_status_s vehicle_status;
	struct battery_status_s battery_status;
	int vehicle_status_sub;
	int battery_status_sub;
	bool led_state;
	int counter;
};


static struct gpio_led_s *gpio_led_data;
static bool gpio_led_started = false;

__EXPORT int gpio_led_main(int argc, char *argv[]);

void gpio_led_start(FAR void *arg);

void gpio_led_cycle(FAR void *arg);

int gpio_led_main(int argc, char *argv[])
{
	if (argc < 2) {
		errx(1, "usage: gpio_led {start|stop} [-p <n>]\n"
		     "\t-p <n>\tUse specified AUX OUT pin number (default: 1)"
		    );

	} else {

		if (!strcmp(argv[1], "start")) {
			if (gpio_led_started) {
				errx(1, "already running");
			}

			/* by default GPIO_SERVO_1 on FMUv2 */
			int pin = 1;

			/* pin name to display */
			char pin_name[sizeof(PIN_NAME) + 2] = PIN_NAME;

			if (argc > 2) {
				if (!strcmp(argv[2], "-p")) {

					unsigned int n = strtoul(argv[3], NULL, 10);

					if (n >= GPIO_MIN_SERVO_PIN && n <= GPIO_MAX_SERVO_PIN) {
						pin = 1 << (n - 1);
						snprintf(pin_name, sizeof(pin_name), "AUX OUT %d", n);

					} else {
						errx(1, "unsupported pin: %s", argv[3]);
					}

				}
			}

			gpio_led_data = malloc(sizeof(struct gpio_led_s));
			memset(gpio_led_data, 0, sizeof(struct gpio_led_s));
			gpio_led_data->pin = pin;
			int ret = work_queue(LPWORK, &(gpio_led_data->work), gpio_led_start, gpio_led_data, 0);

			if (ret != 0) {
				errx(1, "failed to queue work: %d", ret);

			} else {
				gpio_led_started = true;
				warnx("start, using pin: %s", pin_name);
				exit(0);
			}

		} else if (!strcmp(argv[1], "stop")) {
			if (gpio_led_started) {
				gpio_led_started = false;
				warnx("stop");
				exit(0);

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

	char *gpio_dev = PX4FMU_DEVICE_PATH;

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

	/* initialize vehicle status structure */
	memset(&priv->vehicle_status, 0, sizeof(priv->vehicle_status));

	/* initialize battery status structure */
	memset(&priv->battery_status, 0, sizeof(priv->battery_status));

	/* subscribe to vehicle status topic */
	priv->vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));

	/* subscribe to battery status topic */
	priv->battery_status_sub = orb_subscribe(ORB_ID(battery_status));

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

	/* check for vehicle status updates*/
	bool updated;
	orb_check(priv->vehicle_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_status), priv->vehicle_status_sub, &priv->vehicle_status);
	}

	orb_check(priv->battery_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(battery_status), priv->battery_status_sub, &priv->battery_status);
	}

	/* select pattern for current vehiclestatus */
	int pattern = 0;

	if (priv->vehicle_status.arming_state == ARMING_STATE_ARMED_ERROR) {
		pattern = 0x2A;	// *_*_*_ fast blink (armed, error)

	} else if (priv->vehicle_status.arming_state == ARMING_STATE_ARMED) {
		if (priv->battery_status.warning == BATTERY_WARNING_NONE
		    && !priv->vehicle_status.failsafe) {
			pattern = 0x3f;	// ****** solid (armed)

		} else {
			pattern = 0x3e;	// *****_ slow blink (armed, battery low or failsafe)
		}

	} else if (priv->vehicle_status.arming_state == ARMING_STATE_STANDBY) {
		pattern = 0x38;	// ***___ slow blink (disarmed, ready)

	} else if (priv->vehicle_status.arming_state == ARMING_STATE_STANDBY_ERROR) {
		pattern = 0x28;	// *_*___ slow double blink (disarmed, error)

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

	if (priv->counter > 5) {
		priv->counter = 0;
	}

	/* repeat cycle at 5 Hz */
	if (gpio_led_started) {
		work_queue(LPWORK, &priv->work, gpio_led_cycle, priv, USEC2TICK(200000));

	} else {
		/* switch off LED on stop */
		ioctl(priv->gpio_fd, GPIO_CLEAR, priv->pin);
	}
}
