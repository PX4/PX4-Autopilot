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
 * @file px4io.c
 * Top-level logic for the PX4IO module.
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <debug.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>

#include <nuttx/clock.h>

#include <drivers/drv_pwm_output.h>
#include <drivers/drv_hrt.h>

#include "px4io.h"

__EXPORT int user_start(int argc, char *argv[]);

extern void up_cxxinitialize(void);

struct sys_state_s 	system_state;

int user_start(int argc, char *argv[])
{
	/* run C++ ctors before we go any further */
	up_cxxinitialize();

	/* reset all to zero */
	memset(&system_state, 0, sizeof(system_state));
	/* default to 50 Hz PWM outputs */
	system_state.servo_rate = 50;

	/* configure the high-resolution time/callout interface */
	hrt_init();

	/* print some startup info */
	lowsyslog("\nPX4IO: starting\n");

	/* default all the LEDs to off while we start */
	LED_AMBER(false);
	LED_BLUE(false);
	LED_SAFETY(false);

	/* turn on servo power */
	POWER_SERVO(true);

	/* start the safety switch handler */
	safety_init();

	/* configure the first 8 PWM outputs (i.e. all of them) */
	up_pwm_servo_init(0xff);

	/* start the flight control signal handler */
	task_create("FCon",
		    SCHED_PRIORITY_DEFAULT,
		    1024,
		    (main_t)controls_main,
		    NULL);


	struct mallinfo minfo = mallinfo();
	lowsyslog("free %u largest %u\n", minfo.mxordblk, minfo.fordblks);

	/* we're done here, go run the communications loop */
	comms_main();
}