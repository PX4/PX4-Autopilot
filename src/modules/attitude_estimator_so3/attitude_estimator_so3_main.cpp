/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 * @file attitude_estimator_so3_main.cpp
 *
 * @author Hyon Lim <limhyon@gmail.com>
 * @author Anton Babushkin <anton.babushkin@me.com>
 *
 * Implementation of nonlinear complementary filters on the SO(3).
 * This code performs attitude estimation by using accelerometer, gyroscopes and magnetometer.
 * Result is provided as quaternion, 1-2-3 Euler angle and rotation matrix.
 *
 * Theory of nonlinear complementary filters on the SO(3) is based on [1].
 * Quaternion realization of [1] is based on [2].
 * Optmized quaternion update code is based on Sebastian Madgwick's implementation.
 *
 * References
 *  [1] Mahony, R.; Hamel, T.; Pflimlin, Jean-Michel, "Nonlinear Complementary Filters on the Special Orthogonal Group," Automatic Control, IEEE Transactions on , vol.53, no.5, pp.1203,1218, June 2008
 *  [2] Euston, M.; Coote, P.; Mahony, R.; Jonghyuk Kim; Hamel, T., "A complementary filter for attitude estimation of a fixed-wing UAV," Intelligent Robots and Systems, 2008. IROS 2008. IEEE/RSJ International Conference on , vol., no., pp.340,345, 22-26 Sept. 2008
 */

#include <nuttx/config.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <poll.h>
#include <fcntl.h>
#include <float.h>
#include <nuttx/sched.h>
#include <sys/prctl.h>
#include <termios.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <uORB/uORB.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/parameter_update.h>
#include <drivers/drv_hrt.h>

#include <systemlib/systemlib.h>
#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#ifdef __cplusplus
extern "C" {
#endif
#include "attitude_estimator_so3_params.h"
#ifdef __cplusplus
}
#endif

extern "C" __EXPORT int attitude_estimator_so3_main(int argc, char *argv[]);

static bool thread_should_exit = false;		/**< Deamon exit flag */
static bool thread_running = false;		/**< Deamon status flag */
static int attitude_estimator_so3_task;				/**< Handle of deamon task / thread */

//! Auxiliary variables to reduce number of repeated operations
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	/** quaternion of sensor frame relative to auxiliary frame */
static float dq0 = 0.0f, dq1 = 0.0f, dq2 = 0.0f, dq3 = 0.0f;	/** quaternion of sensor frame relative to auxiliary frame */
static float gyro_bias[3] = {0.0f, 0.0f, 0.0f}; /** bias estimation */
static float q0q0, q0q1, q0q2, q0q3;
static float q1q1, q1q2, q1q3;
static float q2q2, q2q3;
static float q3q3;
static bool bFilterInit = false;

/**
 * Mainloop of attitude_estimator_so3.
 */
int attitude_estimator_so3_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

/* Function prototypes */
float invSqrt(float number);
void NonlinearSO3AHRSinit(float ax, float ay, float az, float mx, float my, float mz);
void NonlinearSO3AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float twoKp, float twoKi, float dt);

static void
usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);

	fprintf(stderr, "usage: attitude_estimator_so3 {start|stop|status}\n");
	exit(1);
}

/**
 * The attitude_estimator_so3 app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_spawn_cmd().
 */
int attitude_estimator_so3_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		attitude_estimator_so3_task = task_spawn_cmd("attitude_estimator_so3",
					      SCHED_DEFAULT,
					      SCHED_PRIORITY_MAX - 5,
					      14000,
					      attitude_estimator_so3_thread_main,
					      (argv) ? (char * const *)&argv[2] : (char * const *)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;

		while (thread_running){
			usleep(200000);
		}
		
		warnx("stopped");
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("running");
			exit(0);

		} else {
			warnx("not started");
			exit(1);
		}

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float invSqrt(float number) 
{
    volatile long i;
    volatile float x, y;
    volatile const float f = 1.5F;

    x = number * 0.5F;
    y = number;
    i = * (( long * ) &y);
    i = 0x5f375a86 - ( i >> 1 );
    y = * (( float * ) &i);
    y = y * ( f - ( x * y * y ) );
    return y;
}

//! Using accelerometer, sense the gravity vector.
//! Using magnetometer, sense yaw.
void NonlinearSO3AHRSinit(float ax, float ay, float az, float mx, float my, float mz)
{
    float initialRoll, initialPitch;
    float cosRoll, sinRoll, cosPitch, sinPitch;
    float magX, magY;
    float initialHdg, cosHeading, sinHeading;

    initialRoll = atan2(-ay, -az);
    initialPitch = atan2(ax, -az);

    cosRoll = cosf(initialRoll);
    sinRoll = sinf(initialRoll);
    cosPitch = cosf(initialPitch);
    sinPitch = sinf(initialPitch);

    magX = mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch;

    magY = my * cosRoll - mz * sinRoll;

    initialHdg = atan2f(-magY, magX);

    cosRoll = cosf(initialRoll * 0.5f);
    sinRoll = sinf(initialRoll * 0.5f);

    cosPitch = cosf(initialPitch * 0.5f);
    sinPitch = sinf(initialPitch * 0.5f);

    cosHeading = cosf(initialHdg * 0.5f);
    sinHeading = sinf(initialHdg * 0.5f);

    q0 = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
    q1 = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
    q2 = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
    q3 = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;

    // auxillary variables to reduce number of repeated operations, for 1st pass
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;
}

void NonlinearSO3AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float twoKp, float twoKi, float dt) 
{
	float recipNorm;
	float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;

	// Make filter converge to initial solution faster
	// This function assumes you are in static position.
	// WARNING : in case air reboot, this can cause problem. But this is very unlikely happen.
	if(bFilterInit == false) {
		NonlinearSO3AHRSinit(ax,ay,az,mx,my,mz);
		bFilterInit = true;
	}
        	
	//! If magnetometer measurement is available, use it.
	if(!((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))) {
		float hx, hy, hz, bx, bz;
		float halfwx, halfwy, halfwz;
	
		// Normalise magnetometer measurement
		// Will sqrt work better? PX4 system is powerful enough?
    	recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    	mx *= recipNorm;
    	my *= recipNorm;
    	mz *= recipNorm;
    
    	// Reference direction of Earth's magnetic field
    	hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    	hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
		hz = 2.0f * mx * (q1q3 - q0q2) + 2.0f * my * (q2q3 + q0q1) + 2.0f * mz * (0.5f - q1q1 - q2q2);
    	bx = sqrt(hx * hx + hy * hy);
    	bz = hz;
    
    	// Estimated direction of magnetic field
    	halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
    	halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
    	halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);
    
    	// Error is sum of cross product between estimated direction and measured direction of field vectors
    	halfex += (my * halfwz - mz * halfwy);
    	halfey += (mz * halfwx - mx * halfwz);
    	halfez += (mx * halfwy - my * halfwx);
	}

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
		float halfvx, halfvy, halfvz;
	
		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
	
		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex += ay * halfvz - az * halfvy;
		halfey += az * halfvx - ax * halfvz;
		halfez += ax * halfvy - ay * halfvx;
	}

	// Apply feedback only when valid data has been gathered from the accelerometer or magnetometer
	if(halfex != 0.0f && halfey != 0.0f && halfez != 0.0f) {
		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			gyro_bias[0] += twoKi * halfex * dt;	// integral error scaled by Ki
			gyro_bias[1] += twoKi * halfey * dt;
			gyro_bias[2] += twoKi * halfez * dt;
			
			// apply integral feedback
			gx += gyro_bias[0];
			gy += gyro_bias[1];
			gz += gyro_bias[2];
		}
		else {
			gyro_bias[0] = 0.0f;	// prevent integral windup
			gyro_bias[1] = 0.0f;
			gyro_bias[2] = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	//! Integrate rate of change of quaternion
#if 0
	gx *= (0.5f * dt);		// pre-multiply common factors
	gy *= (0.5f * dt);
	gz *= (0.5f * dt);
#endif 

	// Time derivative of quaternion. q_dot = 0.5*q\otimes omega.
	//! q_k = q_{k-1} + dt*\dot{q}
	//! \dot{q} = 0.5*q \otimes P(\omega)
	dq0 = 0.5f*(-q1 * gx - q2 * gy - q3 * gz);
	dq1 = 0.5f*(q0 * gx + q2 * gz - q3 * gy);
	dq2 = 0.5f*(q0 * gy - q1 * gz + q3 * gx);
	dq3 = 0.5f*(q0 * gz + q1 * gy - q2 * gx); 

	q0 += dt*dq0;
	q1 += dt*dq1;
	q2 += dt*dq2;
	q3 += dt*dq3;
	
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	// Auxiliary variables to avoid repeated arithmetic
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
   	q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;   
}

/*
 * Nonliner complementary filter on SO(3), attitude estimator main function.
 *
 * Estimates the attitude once started.
 *
 * @param argc number of commandline arguments (plus command name)
 * @param argv strings containing the arguments
 */
int attitude_estimator_so3_thread_main(int argc, char *argv[])
{
	//! Time constant
	float dt = 0.005f;
	
	/* output euler angles */
	float euler[3] = {0.0f, 0.0f, 0.0f};
	
	/* Initialization */
	float Rot_matrix[9] = {1.f,  0.0f,  0.0f, 0.0f,  1.f,  0.0f, 0.0f,  0.0f,  1.f };		/**< init: identity matrix */
	float acc[3] = {0.0f, 0.0f, 0.0f};
	float gyro[3] = {0.0f, 0.0f, 0.0f};
	float mag[3] = {0.0f, 0.0f, 0.0f};

	warnx("main thread started");

	struct sensor_combined_s raw;
	memset(&raw, 0, sizeof(raw));

	//! Initialize attitude vehicle uORB message.
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));

	struct vehicle_control_mode_s control_mode;
	memset(&control_mode, 0, sizeof(control_mode));

	uint64_t last_data = 0;
	uint64_t last_measurement = 0;

	/* subscribe to raw data */
	int sub_raw = orb_subscribe(ORB_ID(sensor_combined));
	/* rate-limit raw data updates to 333 Hz (sensors app publishes at 200, so this is just paranoid) */
	orb_set_interval(sub_raw, 3);

	/* subscribe to param changes */
	int sub_params = orb_subscribe(ORB_ID(parameter_update));

	/* subscribe to control mode */
	int sub_control_mode = orb_subscribe(ORB_ID(vehicle_control_mode));

	/* advertise attitude */
	//orb_advert_t pub_att = orb_advertise(ORB_ID(vehicle_attitude), &att);
	//orb_advert_t att_pub = -1;
	orb_advert_t att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);

	int loopcounter = 0;

	thread_running = true;

	// XXX write this out to perf regs

	/* keep track of sensor updates */
	uint64_t sensor_last_timestamp[3] = {0, 0, 0};

	struct attitude_estimator_so3_params so3_comp_params;
	struct attitude_estimator_so3_param_handles so3_comp_param_handles;

	/* initialize parameter handles */
	parameters_init(&so3_comp_param_handles);
	parameters_update(&so3_comp_param_handles, &so3_comp_params);

	uint64_t start_time = hrt_absolute_time();
	bool initialized = false;
	bool state_initialized = false;

	float gyro_offsets[3] = { 0.0f, 0.0f, 0.0f };
	unsigned offset_count = 0;

	/* register the perf counter */
	perf_counter_t so3_comp_loop_perf = perf_alloc(PC_ELAPSED, "attitude_estimator_so3");

	/* Main loop*/
	while (!thread_should_exit) {

		struct pollfd fds[2];
		fds[0].fd = sub_raw;
		fds[0].events = POLLIN;
		fds[1].fd = sub_params;
		fds[1].events = POLLIN;
		int ret = poll(fds, 2, 1000);

		if (ret < 0) {
			/* XXX this is seriously bad - should be an emergency */
		} else if (ret == 0) {
			/* check if we're in HIL - not getting sensor data is fine then */
			orb_copy(ORB_ID(vehicle_control_mode), sub_control_mode, &control_mode);

			if (!control_mode.flag_system_hil_enabled) {
				warnx("WARNING: Not getting sensors - sensor app running?");
			}
		} else {
			/* only update parameters if they changed */
			if (fds[1].revents & POLLIN) {
				/* read from param to clear updated flag */
				struct parameter_update_s update;
				orb_copy(ORB_ID(parameter_update), sub_params, &update);

				/* update parameters */
				parameters_update(&so3_comp_param_handles, &so3_comp_params);
			}

			/* only run filter if sensor values changed */
			if (fds[0].revents & POLLIN) {

				/* get latest measurements */
				orb_copy(ORB_ID(sensor_combined), sub_raw, &raw);

				if (!initialized) {

					gyro_offsets[0] += raw.gyro_rad_s[0];
					gyro_offsets[1] += raw.gyro_rad_s[1];
					gyro_offsets[2] += raw.gyro_rad_s[2];
					offset_count++;

					if (hrt_absolute_time() > start_time + 3000000l) {
						initialized = true;
						gyro_offsets[0] /= offset_count;
						gyro_offsets[1] /= offset_count;
						gyro_offsets[2] /= offset_count;
						warnx("gyro initialized, offsets: %.5f %.5f %.5f", (double)gyro_offsets[0], (double)gyro_offsets[1], (double)gyro_offsets[2]);
					}

				} else {

					perf_begin(so3_comp_loop_perf);

					/* Calculate data time difference in seconds */
					dt = (raw.timestamp - last_measurement) / 1000000.0f;
					last_measurement = raw.timestamp;

					/* Fill in gyro measurements */
					if (sensor_last_timestamp[0] != raw.timestamp) {
						sensor_last_timestamp[0] = raw.timestamp;
					}

					gyro[0] = raw.gyro_rad_s[0] - gyro_offsets[0];
					gyro[1] = raw.gyro_rad_s[1] - gyro_offsets[1];
					gyro[2] = raw.gyro_rad_s[2] - gyro_offsets[2];

					/* update accelerometer measurements */
					if (sensor_last_timestamp[1] != raw.accelerometer_timestamp) {
						sensor_last_timestamp[1] = raw.accelerometer_timestamp;
					}

					acc[0] = raw.accelerometer_m_s2[0];
					acc[1] = raw.accelerometer_m_s2[1];
					acc[2] = raw.accelerometer_m_s2[2];

					/* update magnetometer measurements */
					if (sensor_last_timestamp[2] != raw.magnetometer_timestamp) {
						sensor_last_timestamp[2] = raw.magnetometer_timestamp;
					}

					mag[0] = raw.magnetometer_ga[0];
					mag[1] = raw.magnetometer_ga[1];
					mag[2] = raw.magnetometer_ga[2];

					/* initialize with good values once we have a reasonable dt estimate */
					if (!state_initialized && dt < 0.05f && dt > 0.001f) {
						state_initialized = true;
						warnx("state initialized");
					}

					/* do not execute the filter if not initialized */
					if (!state_initialized) {
						continue;
					}

					// NOTE : Accelerometer is reversed.
					// Because proper mount of PX4 will give you a reversed accelerometer readings.
					NonlinearSO3AHRSupdate(gyro[0], gyro[1], gyro[2],
										-acc[0], -acc[1], -acc[2],
										mag[0], mag[1], mag[2],
										so3_comp_params.Kp,
										so3_comp_params.Ki, 
										dt);

					// Convert q->R, This R converts inertial frame to body frame.
					Rot_matrix[0] = q0q0 + q1q1 - q2q2 - q3q3;// 11
					Rot_matrix[1] = 2.f * (q1*q2 + q0*q3);	// 12
					Rot_matrix[2] = 2.f * (q1*q3 - q0*q2);	// 13
					Rot_matrix[3] = 2.f * (q1*q2 - q0*q3);	// 21
					Rot_matrix[4] = q0q0 - q1q1 + q2q2 - q3q3;// 22
					Rot_matrix[5] = 2.f * (q2*q3 + q0*q1);	// 23
					Rot_matrix[6] = 2.f * (q1*q3 + q0*q2);	// 31
					Rot_matrix[7] = 2.f * (q2*q3 - q0*q1);	// 32
					Rot_matrix[8] = q0q0 - q1q1 - q2q2 + q3q3;// 33

					//1-2-3 Representation.
					//Equation (290) 
					//Representing Attitude: Euler Angles, Unit Quaternions, and Rotation Vectors, James Diebel.
					// Existing PX4 EKF code was generated by MATLAB which uses coloum major order matrix.
					euler[0] = atan2f(Rot_matrix[5], Rot_matrix[8]);	//! Roll
					euler[1] = -asinf(Rot_matrix[2]);	//! Pitch
					euler[2] = atan2f(Rot_matrix[1], Rot_matrix[0]);		//! Yaw
					
					/* swap values for next iteration, check for fatal inputs */
					if (isfinite(euler[0]) && isfinite(euler[1]) && isfinite(euler[2])) {
						// Publish only finite euler angles
						att.roll = euler[0] - so3_comp_params.roll_off;
						att.pitch = euler[1] - so3_comp_params.pitch_off;
						att.yaw = euler[2] - so3_comp_params.yaw_off;
					} else {
						/* due to inputs or numerical failure the output is invalid, skip it */
						// Due to inputs or numerical failure the output is invalid
						warnx("infinite euler angles, rotation matrix:");
						warnx("%.3f %.3f %.3f", (double)Rot_matrix[0], (double)Rot_matrix[1], (double)Rot_matrix[2]);
						warnx("%.3f %.3f %.3f", (double)Rot_matrix[3], (double)Rot_matrix[4], (double)Rot_matrix[5]);
						warnx("%.3f %.3f %.3f", (double)Rot_matrix[6], (double)Rot_matrix[7], (double)Rot_matrix[8]);
						// Don't publish anything
						continue;
					}

					if (last_data > 0 && raw.timestamp > last_data + 12000) {
						warnx("sensor data missed");
					}

					last_data = raw.timestamp;

					/* send out */
					att.timestamp = raw.timestamp;
					
					// Quaternion
					att.q[0] = q0;
					att.q[1] = q1;
					att.q[2] = q2;
					att.q[3] = q3;
					att.q_valid = true;

					// Euler angle rate. But it needs to be investigated again.
					/*
					att.rollspeed = 2.0f*(-q1*dq0 + q0*dq1 - q3*dq2 + q2*dq3);
					att.pitchspeed = 2.0f*(-q2*dq0 + q3*dq1 + q0*dq2 - q1*dq3);
					att.yawspeed = 2.0f*(-q3*dq0 -q2*dq1 + q1*dq2 + q0*dq3);
					*/
					att.rollspeed = gyro[0];
					att.pitchspeed = gyro[1];
					att.yawspeed = gyro[2];

					att.rollacc = 0;
					att.pitchacc = 0;
					att.yawacc = 0;

					/* TODO: Bias estimation required */
					memcpy(&att.rate_offsets, &(gyro_bias), sizeof(att.rate_offsets));

					/* copy rotation matrix */
					memcpy(&att.R, Rot_matrix, sizeof(float)*9);
					att.R_valid = true;
					
					// Publish
					if (att_pub > 0) {
						orb_publish(ORB_ID(vehicle_attitude), att_pub, &att);
					} else {
						warnx("NaN in roll/pitch/yaw estimate!");
						 orb_advertise(ORB_ID(vehicle_attitude), &att);
					}

					perf_end(so3_comp_loop_perf);
				}
			}
		}

		loopcounter++;
	}

	thread_running = false;

	return 0;
}
