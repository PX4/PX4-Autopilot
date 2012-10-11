/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Lorenz Meier <lm@inf.ethz.ch>
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
 * @file led.c
 * Plain, stupid led outputs
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <math.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <termios.h>
#include <time.h>
#include <sys/prctl.h>
#include <arch/board/up_hrt.h>
#include <arch/board/drv_led.h>

#include <systemlib/err.h>
#include <systemlib/systemlib.h>

__EXPORT int led_main(int argc, char *argv[]);


static bool thread_should_exit = false;	/**< Deamon exit flag */
static bool thread_running = false;	/**< Deamon status flag */
static int led_task;			/**< Handle of deamon task / thread */
static int leds;

static int led_init(void)
{
	leds = open("/dev/led", O_RDONLY | O_NONBLOCK);

	if (leds < 0) {
		errx(1, "[led] LED: open fail\n");
	}

	if (ioctl(leds, LED_ON, LED_BLUE) || ioctl(leds, LED_ON, LED_AMBER)) {
		errx(1, "[led] LED: ioctl fail\n");
	}

	return 0;
}

static void led_deinit(void)
{
	close(leds);
}

static int led_toggle(int led)
{
	static int last_blue = LED_ON;
	static int last_amber = LED_ON;

	if (led == LED_BLUE) last_blue = (last_blue == LED_ON) ? LED_OFF : LED_ON;

	if (led == LED_AMBER) last_amber = (last_amber == LED_ON) ? LED_OFF : LED_ON;

	return ioctl(leds, ((led == LED_BLUE) ? last_blue : last_amber), led);
}

static int led_on(int led)
{
	return ioctl(leds, LED_ON, led);
}

static int led_off(int led)
{
	return ioctl(leds, LED_OFF, led);
}

/**
 * Mainloop of led.
 */
int led_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);
	fprintf(stderr, "usage: led {start|stop|status} [-d <UART>]\n\n");
	exit(1);
}

/**
 * The deamon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 * 
 * The actual stack size should be set in the call
 * to task_create().
 */
int led_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("led already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		led_task = task_spawn("led",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_MAX - 15,
				      4096,
				      led_thread_main,
				      (argv) ? (const char **)&argv[2] : (const char **)NULL);
		thread_running = true;
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			printf("\tled is running\n");
		} else {
			printf("\tled not started\n");
		}
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

int led_thread_main(int argc, char *argv[])
{
	/* welcome user */
	printf("[led] Control started, taking over motors\n");

	/* open leds */
	led_init();

	unsigned int rate = 200;

	while (!thread_should_exit) {
		/* swell blue led */
		

		/* 200 Hz base loop */
		usleep(1000000 / rate);
	}

	/* close leds */
	led_deinit();

	printf("[led] ending now...\n\n");
	fflush(stdout);

	thread_running = false;

	return OK;
}

