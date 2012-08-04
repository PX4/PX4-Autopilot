/****************************************************************************
 *
 *   Copyright (C) 2008-2012 PX4 Development Team. All rights reserved.
 *   Author: Tobias Naegeli <naegelit@student.ethz.ch>
 *           Laurens Mackay <mackayl@student.ethz.ch>
 *           Lorenz Meier <lm@inf.ethz.ch>
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

/*
 * @file Extended Kalman Filter for Attitude Estimation
 */


/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <poll.h>
#include <fcntl.h>
#include <v1.0/common/mavlink.h>
#include <float.h>
#include <nuttx/sched.h>
#include <sys/prctl.h>
#include <termios.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <arch/board/up_hrt.h>

#include "codegen/attitudeKalmanfilter_initialize.h"
#include "codegen/attitudeKalmanfilter.h"

__EXPORT int attitude_estimator_ekf_main(int argc, char *argv[]);


// #define N_STATES 6

// #define PROJECTION_INITIALIZE_COUNTER_LIMIT 5000
// #define REPROJECTION_COUNTER_LIMIT 125

static unsigned int loop_interval_alarm = 4500;	// loop interval in microseconds

static float dt = 1;
/* 0, 0, -9.81, 1, 1, 1, wo (gyro offset), w */
/* state vector x has the following entries [ax,ay,az||mx,my,mz||wox,woy,woz||wx,wy,wz]' */
static float z_k[9] = {0};					/**< Measurement vector */
static float x_aposteriori[12] = {0};		/**< */
static float P_aposteriori[144] = {100.f, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
				   0, 100.f,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
				   0,   0, 100.f,   0,   0,   0,   0,   0,   0,   0,   0,   0,
				   0,   0,   0, 100.f,   0,   0,   0,   0,   0,   0,   0,   0,
				   0,   0,   0,   0,  100.f,  0,   0,   0,   0,   0,   0,   0,
				   0,   0,   0,   0,   0, 100.f,   0,   0,   0,   0,   0,   0,
				   0,   0,   0,   0,   0,   0, 100.f,   0,   0,   0,   0,   0,
				   0,   0,   0,   0,   0,   0,   0, 100.f,   0,   0,   0,   0,
				   0,   0,   0,   0,   0,   0,   0,   0, 100.f,   0,   0,   0,
				   0,   0,   0,   0,   0,   0,   0,   0,   0, 100.f,   0,   0,
				   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 100.f,   0,
				   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 100.f
				  };	/**< init: diagonal matrix with big values */
static float knownConst[7] = {1, 1, 1, 1, 1, 1, 1};			/**< knownConst has the following entries [PrvaA,PrvarM,PrvarWO,PrvarW||MsvarA,MsvarM,MsvarW] */
static float Rot_matrix[9] = {1.f,  0,  0,
			      0,  1.f,  0,
			      0,  0,  1.f
			     };		/**< init: identity matrix */

// static float x_aposteriori_k[12] = {0};
// static float P_aposteriori_k[144] = {0};

/*
 * [Rot_matrix,x_aposteriori,P_aposteriori] = attitudeKalmanfilter(dt,z_k,x_aposteriori_k,P_aposteriori_k,knownConst)
 */

/*
 * EKF Attitude Estimator main function.
 *
 * Estimates the attitude recursively once started.
 *
 * @param argc number of commandline arguments (plus command name)
 * @param argv strings containing the arguments
 */
int attitude_estimator_ekf_main(int argc, char *argv[])
{
	// print text
	printf("Extended Kalman Filter Attitude Estimator initialized..\n\n");
	fflush(stdout);

	int overloadcounter = 19;

	/* Initialize filter */
	attitudeKalmanfilter_initialize();

	/* store start time to guard against too slow update rates */
	uint64_t last_run = hrt_absolute_time();

	struct sensor_combined_s raw = {0};
	struct vehicle_attitude_s att = {};

	uint64_t last_data = 0;
	uint64_t last_measurement = 0;

	/* subscribe to raw data */
	int sub_raw = orb_subscribe(ORB_ID(sensor_combined));
	/* advertise attitude */
	int pub_att = orb_advertise(ORB_ID(vehicle_attitude), &att);


	int loopcounter = 0;
	int printcounter = 0;

	/* Main loop*/
	while (true) {

		struct pollfd fds[1] = {
			{ .fd = sub_raw,   .events = POLLIN },
		};
		int ret = poll(fds, 1, 1000);

		/* check for a timeout */
		if (ret == 0) {
			/* */

			/* update successful, copy data on every 2nd run and execute filter */
		} else if (loopcounter & 0x01) {

			orb_copy(ORB_ID(sensor_combined), sub_raw, &raw);

			/* Calculate data time difference in seconds */
			dt = (raw.timestamp - last_measurement) / 1000000.0f;
			last_measurement = raw.timestamp;

			// XXX Read out accel range via SPI on init, assuming 4G range at 14 bit res here
			float range_g = 4.0f;
			float mag_offset[3] = {0};
			/* scale from 14 bit to m/s2 */
			z_k[3] = ((raw.accelerometer_raw[0] * range_g) / 8192.0f) / 9.81f;  // = accel * (1 / 32768.0f / 8.0f * 9.81f);
			z_k[4] = ((raw.accelerometer_raw[1] * range_g) / 8192.0f) / 9.81f;  // = accel * (1 / 32768.0f / 8.0f * 9.81f);
			z_k[5] = ((raw.accelerometer_raw[2] * range_g) / 8192.0f) / 9.81f;  // = accel * (1 / 32768.0f / 8.0f * 9.81f);

			// XXX Read out mag range via I2C on init, assuming 0.88 Ga and 12 bit res here
			z_k[0] = (raw.magnetometer_raw[0] - mag_offset[0]) * 0.01f;
			z_k[1] = (raw.magnetometer_raw[1] - mag_offset[1]) * 0.01f;
			z_k[2] = (raw.magnetometer_raw[2] - mag_offset[2]) * 0.01f;

			/* Fill in gyro measurements */
			z_k[6] =  raw.gyro_raw[0] * 0.00026631611f /* = gyro * (500.0f / 180.0f * pi / 32768.0f ) */;
			z_k[7] =  raw.gyro_raw[1] * 0.00026631611f /* = gyro * (500.0f / 180.0f * pi / 32768.0f ) */;
			z_k[8] =  raw.gyro_raw[2] * 0.00026631611f /* = gyro * (500.0f / 180.0f * pi / 32768.0f ) */;


			uint64_t now = hrt_absolute_time();
			unsigned int time_elapsed = now - last_run;
			last_run = now;

			if (time_elapsed > loop_interval_alarm) {
				//TODO: add warning, cpu overload here
				if (overloadcounter == 20) {
					printf("CPU OVERLOAD DETECTED IN ATTITUDE ESTIMATOR BLACK MAGIC (%lu > %lu)\n", time_elapsed, loop_interval_alarm);
					overloadcounter = 0;
				}

				overloadcounter++;
			}

//			now = hrt_absolute_time();
			/* filter values */
			/*
			 * function [Rot_matrix,x_aposteriori,P_aposteriori] = attitudeKalmanfilter(dt,z_k,x_aposteriori_k,P_aposteriori_k,knownConst)
			 */
			uint64_t timing_start = hrt_absolute_time();
			attitudeKalmanfilter(dt, z_k, x_aposteriori, P_aposteriori, knownConst, Rot_matrix, x_aposteriori, P_aposteriori);
			uint64_t timing_diff = hrt_absolute_time() - timing_start;

			/* print rotation matrix every 200th time */
			if (printcounter % 200 == 0) {
				printf("EKF attitude iteration: %d, runtime: %d us, dt: %d us (%d Hz)\n", loopcounter, (int)timing_diff, (int)(dt * 1000000.0f), (int)(1.0f / dt));
				printf("\n%d\t%d\t%d\n%d\t%d\t%d\n%d\t%d\t%d\n", (int)(Rot_matrix[0] * 100), (int)(Rot_matrix[1] * 100), (int)(Rot_matrix[2] * 100),
				       (int)(Rot_matrix[3] * 100), (int)(Rot_matrix[4] * 100), (int)(Rot_matrix[5] * 100),
				       (int)(Rot_matrix[6] * 100), (int)(Rot_matrix[7] * 100), (int)(Rot_matrix[8] * 100));
			}

			printcounter++;

//			time_elapsed = hrt_absolute_time() - now;
//			if (blubb == 20)
//			{
//				printf("Estimator: %lu\n", time_elapsed);
//				blubb = 0;
//			}
//			blubb++;

			if (last_data > 0 && raw.timestamp - last_data > 8000) printf("sensor data missed! (%llu)\n", raw.timestamp - last_data);

//			printf("%llu -> %llu = %llu\n", last_data, raw.timestamp, raw.timestamp - last_data);
			last_data = raw.timestamp;

			/* send out */
			att.timestamp = raw.timestamp;
			// att.roll = euler.x;
			// att.pitch = euler.y;
			// att.yaw = euler.z + F_M_PI;

			// if (att.yaw > 2 * F_M_PI) {
			// 	att.yaw -= 2 * F_M_PI;
			// }

			// att.rollspeed = rates.x;
			// att.pitchspeed = rates.y;
			// att.yawspeed = rates.z;

			// memcpy()R

			// Broadcast
			orb_publish(ORB_ID(vehicle_attitude), pub_att, &att);
		}

		loopcounter++;
	}

	/* Should never reach here */
	return 0;
}





