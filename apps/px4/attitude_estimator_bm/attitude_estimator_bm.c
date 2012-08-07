/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: Tobias Naegeli <naegelit@student.ethz.ch>
 *           Laurens Mackay <mackayl@student.ethz.ch>
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
 * @file attitude_estimator_bm.c
 * Black Magic Attitude Estimator
 */



#include <nuttx/config.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>
#include <stdbool.h>
#include <fcntl.h>
#include <arch/board/up_hrt.h>
#include <string.h>
#include <poll.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>
#include <math.h>
#include <errno.h>

#include "attitude_bm.h"

static unsigned int loop_interval_alarm = 4500;	// loop interval in microseconds

__EXPORT int attitude_estimator_bm_main(int argc, char *argv[]);

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * user_start
 ****************************************************************************/
int attitude_estimator_bm_update(struct sensor_combined_s *raw, float_vect3 *euler, float_vect3 *rates, float_vect3 *x_n_b, float_vect3 *y_n_b, float_vect3 *z_n_b);

int attitude_estimator_bm_update(struct sensor_combined_s *raw, float_vect3 *euler, float_vect3 *rates, float_vect3 *x_n_b, float_vect3 *y_n_b, float_vect3 *z_n_b)
{
	float_vect3 gyro_values;
	gyro_values.x =  raw->gyro_rad_s[0];
	gyro_values.y =  raw->gyro_rad_s[1];
	gyro_values.z =  raw->gyro_rad_s[2];

	float_vect3 accel_values;
	accel_values.x = raw->accelerometer_m_s2[0];
	accel_values.y = raw->accelerometer_m_s2[1];
	accel_values.z = raw->accelerometer_m_s2[2];

	float_vect3 mag_values;
	mag_values.x = raw->magnetometer_ga[0];
	mag_values.y = raw->magnetometer_ga[1];
	mag_values.z = raw->magnetometer_ga[2];

	attitude_blackmagic(&accel_values, &mag_values, &gyro_values);

	/* read out values */
	attitude_blackmagic_get_all(euler, rates, x_n_b, y_n_b, z_n_b);

	return OK;
}


int attitude_estimator_bm_main(int argc, char *argv[])
{
	// print text
	printf("Black Magic Attitude Estimator initialized..\n\n");
	fflush(stdout);

	/* data structures to read euler angles and rotation matrix back */
	float_vect3 euler = {.x = 0, .y = 0, .z = 0};
	float_vect3 rates;
	float_vect3 x_n_b;
	float_vect3 y_n_b;
	float_vect3 z_n_b;

	int overloadcounter = 19;

	/* initialize */
	attitude_blackmagic_init();

	/* store start time to guard against too slow update rates */
	uint64_t last_run = hrt_absolute_time();

	struct sensor_combined_s sensor_combined_s_local = { .gyro_raw = {0}};
	struct vehicle_attitude_s att = {.roll = 0.0f, .pitch = 0.0f, .yaw = 0.0f,
					 .rollspeed = 0.0f, .pitchspeed = 0.0f, .yawspeed = 0.0f,
					 .R = {0}, .timestamp = 0};

	uint64_t last_data = 0;

	/* subscribe to raw data */
	int sub_raw = orb_subscribe(ORB_ID(sensor_combined));

	/* rate-limit raw data updates to 200Hz */
	//orb_set_interval(sub_raw, 5);

	bool hil_enabled = false;
	bool publishing = false;

	/* advertise attitude */
	int pub_att = orb_advertise(ORB_ID(vehicle_attitude), &att);
	publishing = true;

	struct pollfd fds[] = {
		{ .fd = sub_raw,   .events = POLLIN },
	};

	/* subscribe to system status */
	struct vehicle_status_s vstatus = {0};
	int vstatus_sub = orb_subscribe(ORB_ID(vehicle_status));

	unsigned int loopcounter = 0;

	uint64_t last_checkstate_stamp = 0;

	/* Main loop*/
	while (true) {


		/* wait for sensor update */
		int ret = poll(fds, 1, 1000);

		if (ret < 0) {
			/* XXX this is seriously bad - should be an emergency */
		} else if (ret == 0) {
			/* XXX this means no sensor data - should be critical or emergency */
			printf("[attitude estimator bm] WARNING: Not getting sensor data - sensor app running?\n");
		} else {
			orb_copy(ORB_ID(sensor_combined), sub_raw, &sensor_combined_s_local);

			uint64_t now = hrt_absolute_time();
			unsigned int time_elapsed = now - last_run;
			last_run = now;

			//#if 0

			if (time_elapsed > loop_interval_alarm) {
				//TODO: add warning, cpu overload here
				if (overloadcounter == 20) {
					printf("CPU OVERLOAD DETECTED IN ATTITUDE ESTIMATOR BLACK MAGIC (%lu > %lu)\n", time_elapsed, loop_interval_alarm);
					overloadcounter = 0;
				}

				overloadcounter++;
			}

			//#endif
			//			now = hrt_absolute_time();
			/* filter values */
			attitude_estimator_bm_update(&sensor_combined_s_local, &euler, &rates, &x_n_b, &y_n_b, &z_n_b);

			//			time_elapsed = hrt_absolute_time() - now;
			//			if (blubb == 20)
			//			{
			//				printf("Estimator: %lu\n", time_elapsed);
			//				blubb = 0;
			//			}
			//			blubb++;

			//		if (last_data > 0 && sensor_combined_s_local.timestamp - last_data > 8000) printf("sensor data missed! (%llu)\n", sensor_combined_s_local.timestamp - last_data);

			//			printf("%llu -> %llu = %llu\n", last_data, sensor_combined_s_local.timestamp, sensor_combined_s_local.timestamp - last_data);
			// last_data = sensor_combined_s_local.timestamp;

			/*correct yaw */
			//			euler.z = euler.z + M_PI;

			/* send out */

			att.timestamp = sensor_combined_s_local.timestamp;
			att.roll = euler.x;
			att.pitch = euler.y;
			att.yaw = euler.z + M_PI;

			if (att.yaw > 2.0f * ((float)M_PI)) {
				att.yaw -= 2.0f * ((float)M_PI);
			}

			att.rollspeed = rates.x;
			att.pitchspeed = rates.y;
			att.yawspeed = rates.z;

			att.R[0][0] = x_n_b.x;
			att.R[0][1] = x_n_b.y;
			att.R[0][2] = x_n_b.z;

			// Broadcast
			if (publishing) orb_publish(ORB_ID(vehicle_attitude), pub_att, &att);
		}


		// XXX add remaining entries

		if (hrt_absolute_time() - last_checkstate_stamp > 500000) {
			/* Check HIL state */
			orb_copy(ORB_ID(vehicle_status), vstatus_sub, &vstatus);
			/* switching from non-HIL to HIL mode */
			//printf("[attitude_estimator_bm] Vehicle mode: %i \t AND: %i, HIL: %i\n", vstatus.mode, vstatus.mode & VEHICLE_MODE_FLAG_HIL_ENABLED, hil_enabled);
			if ((vstatus.mode & VEHICLE_MODE_FLAG_HIL_ENABLED) && !hil_enabled) {
				hil_enabled = true;
				publishing = false;
				int ret = close(pub_att);
				printf("Closing attitude: %i \n", ret);

				/* switching from HIL to non-HIL mode */

			} else if (!publishing && !hil_enabled) {
				/* advertise the topic and make the initial publication */
				pub_att = orb_advertise(ORB_ID(vehicle_attitude), &att);
				hil_enabled = false;
				publishing = true;
			}
			last_checkstate_stamp = hrt_absolute_time();
		}

		loopcounter++;
	}

	/* Should never reach here */
	return 0;
}


