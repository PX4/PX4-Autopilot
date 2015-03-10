/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * @file hwtest.c
 *
 * Simple output test.
 * @ref Documentation https://pixhawk.org/dev/examples/write_output
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#include <stdio.h>
#include <string.h>

#include <drivers/drv_hrt.h>
#include <nuttx/config.h>
#include <systemlib/err.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_controls_0.h>
#include <uORB/topics/actuator_controls_1.h>
#include <uORB/topics/actuator_controls_2.h>
#include <uORB/topics/actuator_controls_3.h>
#include <uORB/uORB.h>

__EXPORT int ex_hwtest_main(int argc, char *argv[]);

int ex_hwtest_main(int argc, char *argv[])
{
	warnx("DO NOT FORGET TO STOP THE COMMANDER APP!");
	warnx("(run <commander stop> to do so)");
	warnx("usage: http://px4.io/dev/examples/write_output");

	struct actuator_controls_s actuators;
	memset(&actuators, 0, sizeof(actuators));
	orb_advert_t actuator_pub_fd = orb_advertise(ORB_ID(actuator_controls_0), &actuators);

	struct actuator_armed_s arm;
	memset(&arm, 0 , sizeof(arm));

	arm.timestamp = hrt_absolute_time();
	arm.ready_to_arm = true;
	arm.armed = true;
	orb_advert_t arm_pub_fd = orb_advertise(ORB_ID(actuator_armed), &arm);
	orb_publish(ORB_ID(actuator_armed), arm_pub_fd, &arm);

	/* read back values to validate */
	int arm_sub_fd = orb_subscribe(ORB_ID(actuator_armed));
	orb_copy(ORB_ID(actuator_armed), arm_sub_fd, &arm);

	if (arm.ready_to_arm && arm.armed) {
		warnx("Actuator armed");

	} else {
		errx(1, "Arming actuators failed");
	}

	hrt_abstime stime;

	int count = 0;

	while (count != 36) {
		stime = hrt_absolute_time();

		while (hrt_absolute_time() - stime < 1000000) {
			for (int i = 0; i != 2; i++) {
				if (count <= 5) {
					actuators.control[i] = -1.0f;

				} else if (count <= 10) {
					actuators.control[i] = -0.7f;

				} else if (count <= 15) {
					actuators.control[i] = -0.5f;

				} else if (count <= 20) {
					actuators.control[i] = -0.3f;

				} else if (count <= 25) {
					actuators.control[i] = 0.0f;

				} else if (count <= 30) {
					actuators.control[i] = 0.3f;

				} else {
					actuators.control[i] = 0.5f;
				}
			}

			actuators.timestamp = hrt_absolute_time();
			orb_publish(ORB_ID(actuator_controls_0), actuator_pub_fd, &actuators);
			usleep(10000);
		}

		warnx("count %i", count);
		count++;
	}

	return OK;
}
