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
#include <px4_module.h>
#include <px4_log.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>
#include <systemlib/err.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/battery_status.h>
#include <poll.h>
#include <drivers/drv_gpio.h>
#include <modules/px4iofirmware/protocol.h>


#define CYCLE_RATE_HZ  5

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
#elif defined(GPIO_SERVO_5)
#  define GPIO_MAX_SERVO_PIN 5
#elif defined(GPIO_SERVO_4)
#  define GPIO_MAX_SERVO_PIN 4
#else
#  error "Board must define GPIO_SERVO_1 and GPIO_SERVO_n where n is 4-16"
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
static volatile enum {
	Stopped =  0,
	Running =  1,
	Falied  =  2,
	Stopping = 3
} gpio_led_state = Stopped;

__EXPORT int gpio_led_main(int argc, char *argv[]);

void gpio_led_start(FAR void *arg);

void gpio_led_cycle(FAR void *arg);

static void print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module is responsible for drving a single LED on one of the FMU AUX pins.

It listens on the vehicle_status and battery_status topics and provides visual annunciation on the LED.

### Implementation
The module runs on the work queue. It schedules at a fixed frequency of 5 Hz

### Examples
It is started with:
$  gpio_led start
To drive an LED connected AUX1 pin.

OR with any of the avaliabel AUX pins
$  gpio_led start -p 5
To drive an LED connected AUX5 pin.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("gpio_led", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "annunciation on AUX OUT pin");
	PRINT_MODULE_USAGE_PARAM_FLAG('p', "Use specified AUX OUT pin number (default: 1)", true);
	PRINT_MODULE_USAGE_COMMAND("stop");
}

int gpio_led_main(int argc, char *argv[])
{
	if (argc < 2) {
		print_usage(NULL);
		exit(1);
	} else {

		if (!strcmp(argv[1], "start")) {
			if (gpio_led_state != Stopped) {
				PX4_WARN("already running");
				exit(1);
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
						PX4_ERR("unsupported pin: %s (valid values are %d-%d)", argv[3], GPIO_MIN_SERVO_PIN, GPIO_MAX_SERVO_PIN);
						exit(1);
					}

				}
			}

			gpio_led_data = malloc(sizeof(struct gpio_led_s));
			if (gpio_led_data == NULL) {
				PX4_ERR("failed to allocate memory!");
				exit(1);
			} else {
				memset(gpio_led_data, 0, sizeof(struct gpio_led_s));
				gpio_led_data->pin = pin;
				int ret = work_queue(LPWORK, &(gpio_led_data->work), gpio_led_start, gpio_led_data, 0);

				if (ret != 0) {
					PX4_ERR("failed to queue work: %d", ret);
					goto out;
				} else {
					usleep(1000000/CYCLE_RATE_HZ);
					if (gpio_led_state != Running) {
						gpio_led_state = Stopped;
						goto out;
					}
					PX4_INFO("start, using pin: %s", pin_name);
					exit(0);
				}
			}

		} else if (!strcmp(argv[1], "stop")) {
			if (gpio_led_state == Running) {
				gpio_led_state = Stopping;
				while(gpio_led_state != Stopped) {
					usleep(1000000/CYCLE_RATE_HZ);
				}
				PX4_INFO("stopped");
				free (gpio_led_data);
				gpio_led_data = NULL;
				exit(0);
			} else {
				PX4_WARN("not running");
				exit(1);
			}

		} else {
			print_usage("unrecognized command");
			exit(1);
		}
	}
out:
	free (gpio_led_data);
	gpio_led_data = NULL;
	exit(1);
}

void gpio_led_start(FAR void *arg)
{
	FAR struct gpio_led_s *priv = (FAR struct gpio_led_s *)arg;

	char *gpio_dev = PX4FMU_DEVICE_PATH;

	/* open GPIO device */
	priv->gpio_fd = open(gpio_dev, 0);

	if (priv->gpio_fd < 0) {
		PX4_ERR("gpio_led: GPIO device \"%s\" open fail\n", gpio_dev);
		gpio_led_state = Falied;
		return;
	}

	/* configure GPIO pin */

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
		PX4_ERR("gpio_led: failed to queue work: %d\n", ret);
		close(priv->gpio_fd);
		gpio_led_state = Falied;
		return;
	}
	gpio_led_state = Running;
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

	if (priv->vehicle_status.arming_state == VEHICLE_STATUS_ARMING_STATE_ARMED) {
		if (priv->battery_status.warning == BATTERY_STATUS_BATTERY_WARNING_NONE
		    && !priv->vehicle_status.failsafe) {
			pattern = 0x3f;	// ****** solid (armed)

		} else {
			pattern = 0x3e;	// *****_ slow blink (armed, battery low or failsafe)
		}

	} else if (priv->vehicle_status.arming_state == VEHICLE_STATUS_ARMING_STATE_STANDBY) {
		pattern = 0x38;	// ***___ slow blink (disarmed, ready)

	} else if (priv->vehicle_status.arming_state == VEHICLE_STATUS_ARMING_STATE_STANDBY_ERROR) {
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
	if (gpio_led_state == Running) {
		work_queue(LPWORK, &priv->work, gpio_led_cycle, priv, USEC2TICK(1000000/CYCLE_RATE_HZ));

	} else {
		/* switch off LED on stop */
		ioctl(priv->gpio_fd, GPIO_CLEAR, priv->pin);
		orb_unsubscribe(priv->vehicle_status_sub);
		orb_unsubscribe(priv->battery_status_sub);
		close(priv->gpio_fd);
		gpio_led_state = Stopped;
	}
}
