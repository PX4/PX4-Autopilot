/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
 *   Author: Mohammed Kabir <mhkabir98@gmail.com>
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
 * @file camera_trigger.c
 *
 * External camera-IMU synchronisation and triggering via FMU auxillary pins.
 *
 * @author Mohammed Kabir <mhkabir98@gmail.com>
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
#include <poll.h>
#include <drivers/drv_gpio.h>
#include <drivers/drv_hrt.h>

struct camera_trigger_s {
	struct work_s work;
	int gpio_fd;
	int pin;
	struct camera_trigger_s trigger;
	int camera_trigger_sub;
};

static struct camera_trigger_s camera_trigger_data;
static bool camera_trigger_started = false;

__EXPORT int camera_trigger_main(int argc, char *argv[]);

void camera_trigger_start(FAR void *arg);

void camera_trigger_cycle(FAR void *arg);

int camera_trigger_main(int argc, char *argv[])
{
	if (argc < 2) {
		errx(1, "usage: camera_trigger {start|stop} [-p <n>]\n"
		     "\t-p <n>\tUse specified AUX OUT pin number (default: 1)"
		    );


	} else {

		if (!strcmp(argv[1], "start")) {
			if (camera_trigger_started) {
				errx(1, "already running");
			}

			int pin = 1;

			memset(&camera_trigger_data, 0, sizeof(camera_trigger_data));
			camera_trigger_data.pin = pin;

			int ret = work_queue(LPWORK, &camera_trigger_data.work, camera_trigger_start, &camera_trigger_data, 0);

			if (ret != 0) {
				errx(1, "failed to queue work: %d", ret);

			} else {
				camera_trigger_started = true;
				warnx("starting trigger");
				exit(0);
			}
		} else if (!strcmp(argv[1], "stop")) {
			if (camera_trigger_started) {
				camera_trigger_started = false;
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

void camera_trigger_start(FAR void *arg)
{
	FAR struct camera_trigger_s *priv = (FAR struct camera_trigger_s *)arg;

	/* open GPIO device */
	priv->gpio_fd = open(PX4FMU_DEVICE_PATH, 0);

	if (priv->gpio_fd < 0) {
		
		warnx("camera_trigger: GPIO device open fail");
		camera_trigger_started = false;
		return;
	}
	else
	{
		warnx("camera_trigger: GPIO device opened");
	}

	memset(&priv->trigger, 0, sizeof(priv->trigger));

	priv->camera_trigger_sub = orb_subscribe(ORB_ID(camera_trigger));

	/* configure GPIO pin and pull it high */
	ioctl(priv->gpio_fd, GPIO_SET_OUTPUT, priv->pin);
	ioctl(priv->gpio_fd, GPIO_SET, priv->pin);
	
	/* add worker to queue */
	int ret = work_queue(LPWORK, &priv->work, camera_trigger_cycle, priv, 0);

	if (ret != 0) {
		warnx("camera_trigger: failed to queue work: %d\n", ret);
		camera_trigger_started = false;
		return;
	}
}

void camera_trigger_cycle(FAR void *arg)
{
	FAR struct camera_trigger_s *priv = (FAR struct camera_trigger_s *)arg;

	/* check for trigger updates*/
	bool updated;
	orb_check(priv->camera_trigger_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(camera_trigger), priv->camera_trigger_sub, &priv->trigger);
	}
	
	if(priv->trigger.trigger_on == true){
		/* pull pin low for 4us */
		ioctl(priv->gpio_fd, GPIO_CLEAR, priv->pin);
		usleep(4000);
		ioctl(priv->gpio_fd, GPIO_SET, priv->pin);
	}

	if (camera_trigger_started) {
		work_queue(LPWORK, &priv->work, camera_trigger_cycle, priv, USEC2TICK(1000));

	} else {
		warnx("camera_trigger: stop");
	}
}
