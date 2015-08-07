/****************************************************************************
 *
 *   Copyright (c) 2012 PX4 Development Team. All rights reserved.
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
 * @file calibration_routines.cpp
 * Calibration routines implementations.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#include <px4_defines.h>
#include <px4_posix.h>
#include <px4_time.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <float.h>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <mavlink/mavlink_log.h>
#include <geo/geo.h>
#include <string.h>

#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/sensor_combined.h>

#include "calibration_routines.h"
#include "calibration_messages.h"
#include "commander_helper.h"

int sphere_fit_least_squares(const float x[], const float y[], const float z[],
			     unsigned int size, unsigned int max_iterations, float delta, float *sphere_x, float *sphere_y, float *sphere_z,
			     float *sphere_radius)
{

	float x_sumplain = 0.0f;
	float x_sumsq = 0.0f;
	float x_sumcube = 0.0f;

	float y_sumplain = 0.0f;
	float y_sumsq = 0.0f;
	float y_sumcube = 0.0f;

	float z_sumplain = 0.0f;
	float z_sumsq = 0.0f;
	float z_sumcube = 0.0f;

	float xy_sum = 0.0f;
	float xz_sum = 0.0f;
	float yz_sum = 0.0f;

	float x2y_sum = 0.0f;
	float x2z_sum = 0.0f;
	float y2x_sum = 0.0f;
	float y2z_sum = 0.0f;
	float z2x_sum = 0.0f;
	float z2y_sum = 0.0f;

	for (unsigned int i = 0; i < size; i++) {

		float x2 = x[i] * x[i];
		float y2 = y[i] * y[i];
		float z2 = z[i] * z[i];

		x_sumplain += x[i];
		x_sumsq += x2;
		x_sumcube += x2 * x[i];

		y_sumplain += y[i];
		y_sumsq += y2;
		y_sumcube += y2 * y[i];

		z_sumplain += z[i];
		z_sumsq += z2;
		z_sumcube += z2 * z[i];

		xy_sum += x[i] * y[i];
		xz_sum += x[i] * z[i];
		yz_sum += y[i] * z[i];

		x2y_sum += x2 * y[i];
		x2z_sum += x2 * z[i];

		y2x_sum += y2 * x[i];
		y2z_sum += y2 * z[i];

		z2x_sum += z2 * x[i];
		z2y_sum += z2 * y[i];
	}

	//
	//Least Squares Fit a sphere A,B,C with radius squared Rsq to 3D data
	//
	//    P is a structure that has been computed with the data earlier.
	//    P.npoints is the number of elements; the length of X,Y,Z are identical.
	//    P's members are logically named.
	//
	//    X[n] is the x component of point n
	//    Y[n] is the y component of point n
	//    Z[n] is the z component of point n
	//
	//    A is the x coordiante of the sphere
	//    B is the y coordiante of the sphere
	//    C is the z coordiante of the sphere
	//    Rsq is the radius squared of the sphere.
	//
	//This method should converge; maybe 5-100 iterations or more.
	//
	float x_sum = x_sumplain / size;        //sum( X[n] )
	float x_sum2 = x_sumsq / size;    //sum( X[n]^2 )
	float x_sum3 = x_sumcube / size;    //sum( X[n]^3 )
	float y_sum = y_sumplain / size;        //sum( Y[n] )
	float y_sum2 = y_sumsq / size;    //sum( Y[n]^2 )
	float y_sum3 = y_sumcube / size;    //sum( Y[n]^3 )
	float z_sum = z_sumplain / size;        //sum( Z[n] )
	float z_sum2 = z_sumsq / size;    //sum( Z[n]^2 )
	float z_sum3 = z_sumcube / size;    //sum( Z[n]^3 )

	float XY = xy_sum / size;        //sum( X[n] * Y[n] )
	float XZ = xz_sum / size;        //sum( X[n] * Z[n] )
	float YZ = yz_sum / size;        //sum( Y[n] * Z[n] )
	float X2Y = x2y_sum / size;    //sum( X[n]^2 * Y[n] )
	float X2Z = x2z_sum / size;    //sum( X[n]^2 * Z[n] )
	float Y2X = y2x_sum / size;    //sum( Y[n]^2 * X[n] )
	float Y2Z = y2z_sum / size;    //sum( Y[n]^2 * Z[n] )
	float Z2X = z2x_sum / size;    //sum( Z[n]^2 * X[n] )
	float Z2Y = z2y_sum / size;    //sum( Z[n]^2 * Y[n] )

	//Reduction of multiplications
	float F0 = x_sum2 + y_sum2 + z_sum2;
	float F1 =  0.5f * F0;
	float F2 = -8.0f * (x_sum3 + Y2X + Z2X);
	float F3 = -8.0f * (X2Y + y_sum3 + Z2Y);
	float F4 = -8.0f * (X2Z + Y2Z + z_sum3);

	//Set initial conditions:
	float A = x_sum;
	float B = y_sum;
	float C = z_sum;

	//First iteration computation:
	float A2 = A * A;
	float B2 = B * B;
	float C2 = C * C;
	float QS = A2 + B2 + C2;
	float QB = -2.0f * (A * x_sum + B * y_sum + C * z_sum);

	//Set initial conditions:
	float Rsq = F0 + QB + QS;

	//First iteration computation:
	float Q0 = 0.5f * (QS - Rsq);
	float Q1 = F1 + Q0;
	float Q2 = 8.0f * (QS - Rsq + QB + F0);
	float aA, aB, aC, nA, nB, nC, dA, dB, dC;

	//Iterate N times, ignore stop condition.
	unsigned int n = 0;

	while (n < max_iterations) {
		n++;

		//Compute denominator:
		aA = Q2 + 16.0f * (A2 - 2.0f * A * x_sum + x_sum2);
		aB = Q2 + 16.0f * (B2 - 2.0f * B * y_sum + y_sum2);
		aC = Q2 + 16.0f * (C2 - 2.0f * C * z_sum + z_sum2);
		aA = (fabsf(aA) < FLT_EPSILON) ? 1.0f : aA;
		aB = (fabsf(aB) < FLT_EPSILON) ? 1.0f : aB;
		aC = (fabsf(aC) < FLT_EPSILON) ? 1.0f : aC;

		//Compute next iteration
		nA = A - ((F2 + 16.0f * (B * XY + C * XZ + x_sum * (-A2 - Q0) + A * (x_sum2 + Q1 - C * z_sum - B * y_sum))) / aA);
		nB = B - ((F3 + 16.0f * (A * XY + C * YZ + y_sum * (-B2 - Q0) + B * (y_sum2 + Q1 - A * x_sum - C * z_sum))) / aB);
		nC = C - ((F4 + 16.0f * (A * XZ + B * YZ + z_sum * (-C2 - Q0) + C * (z_sum2 + Q1 - A * x_sum - B * y_sum))) / aC);

		//Check for stop condition
		dA = (nA - A);
		dB = (nB - B);
		dC = (nC - C);

		if ((dA * dA + dB * dB + dC * dC) <= delta) { break; }

		//Compute next iteration's values
		A = nA;
		B = nB;
		C = nC;
		A2 = A * A;
		B2 = B * B;
		C2 = C * C;
		QS = A2 + B2 + C2;
		QB = -2.0f * (A * x_sum + B * y_sum + C * z_sum);
		Rsq = F0 + QB + QS;
		Q0 = 0.5f * (QS - Rsq);
		Q1 = F1 + Q0;
		Q2 = 8.0f * (QS - Rsq + QB + F0);
	}

	*sphere_x = A;
	*sphere_y = B;
	*sphere_z = C;
	*sphere_radius = sqrtf(Rsq);

	return 0;
}

enum detect_orientation_return detect_orientation(int mavlink_fd, int cancel_sub, int accel_sub, bool lenient_still_position)
{
	const unsigned ndim = 3;
	
	struct sensor_combined_s sensor;
	float		accel_ema[ndim] = { 0.0f };		// exponential moving average of accel
	float		accel_disp[3] = { 0.0f, 0.0f, 0.0f };	// max-hold dispersion of accel
	float		ema_len = 0.5f;				// EMA time constant in seconds
	const float	normal_still_thr = 0.25;		// normal still threshold
	float		still_thr2 = powf(lenient_still_position ? (normal_still_thr * 3) : normal_still_thr, 2);
	float		accel_err_thr = 5.0f;			// set accel error threshold to 5m/s^2
	hrt_abstime	still_time = lenient_still_position ? 500000 : 1300000;	// still time required in us
    
	px4_pollfd_struct_t fds[1];
	fds[0].fd = accel_sub;
	fds[0].events = POLLIN;
	
	hrt_abstime t_start = hrt_absolute_time();
	/* set timeout to 30s */
	hrt_abstime timeout = 30000000;
	hrt_abstime t_timeout = t_start + timeout;
	hrt_abstime t = t_start;
	hrt_abstime t_prev = t_start;
	hrt_abstime t_still = 0;
	
	unsigned poll_errcount = 0;
	
	while (true) {
		/* wait blocking for new data */
		int poll_ret = px4_poll(fds, 1, 1000);
		
		if (poll_ret) {
			orb_copy(ORB_ID(sensor_combined), accel_sub, &sensor);
			t = hrt_absolute_time();
			float dt = (t - t_prev) / 1000000.0f;
			t_prev = t;
			float w = dt / ema_len;
			
			for (unsigned i = 0; i < ndim; i++) {
				
				float di = 0.0f;
				switch (i) {
					case 0:
						di = sensor.accelerometer_m_s2[0];
						break;
					case 1:
						di = sensor.accelerometer_m_s2[1];
						break;
					case 2:
						di = sensor.accelerometer_m_s2[2];
						break;
				}
				
				float d = di - accel_ema[i];
				accel_ema[i] += d * w;
				d = d * d;
				accel_disp[i] = accel_disp[i] * (1.0f - w);
				
				if (d > still_thr2 * 8.0f) {
					d = still_thr2 * 8.0f;
				}
				
				if (d > accel_disp[i]) {
					accel_disp[i] = d;
				}
			}
			
			/* still detector with hysteresis */
			if (accel_disp[0] < still_thr2 &&
			    accel_disp[1] < still_thr2 &&
			    accel_disp[2] < still_thr2) {
				/* is still now */
				if (t_still == 0) {
					/* first time */
					mavlink_and_console_log_info(mavlink_fd, "[cal] detected rest position, hold still...");
					t_still = t;
					t_timeout = t + timeout;
					
				} else {
					/* still since t_still */
					if (t > t_still + still_time) {
						/* vehicle is still, exit from the loop to detection of its orientation */
						break;
					}
				}
				
			} else if (accel_disp[0] > still_thr2 * 4.0f ||
				   accel_disp[1] > still_thr2 * 4.0f ||
				   accel_disp[2] > still_thr2 * 4.0f) {
				/* not still, reset still start time */
				if (t_still != 0) {
					mavlink_and_console_log_info(mavlink_fd, "[cal] detected motion, hold still...");
					usleep(200000);
					t_still = 0;
				}
			}
			
		} else if (poll_ret == 0) {
			poll_errcount++;
		}
		
		if (t > t_timeout) {
			poll_errcount++;
		}
		
		if (poll_errcount > 1000) {
			mavlink_and_console_log_critical(mavlink_fd, CAL_ERROR_SENSOR_MSG);
			return DETECT_ORIENTATION_ERROR;
		}
	}

	if (fabsf(accel_ema[0] - CONSTANTS_ONE_G) < accel_err_thr &&
	    fabsf(accel_ema[1]) < accel_err_thr &&
	    fabsf(accel_ema[2]) < accel_err_thr) {
		return DETECT_ORIENTATION_TAIL_DOWN;        // [ g, 0, 0 ]
	}
	
	if (fabsf(accel_ema[0] + CONSTANTS_ONE_G) < accel_err_thr &&
	    fabsf(accel_ema[1]) < accel_err_thr &&
	    fabsf(accel_ema[2]) < accel_err_thr) {
		return DETECT_ORIENTATION_NOSE_DOWN;        // [ -g, 0, 0 ]
	}
	
	if (fabsf(accel_ema[0]) < accel_err_thr &&
	    fabsf(accel_ema[1] - CONSTANTS_ONE_G) < accel_err_thr &&
	    fabsf(accel_ema[2]) < accel_err_thr) {
		return DETECT_ORIENTATION_LEFT;        // [ 0, g, 0 ]
	}
	
	if (fabsf(accel_ema[0]) < accel_err_thr &&
	    fabsf(accel_ema[1] + CONSTANTS_ONE_G) < accel_err_thr &&
	    fabsf(accel_ema[2]) < accel_err_thr) {
		return DETECT_ORIENTATION_RIGHT;        // [ 0, -g, 0 ]
	}
	
	if (fabsf(accel_ema[0]) < accel_err_thr &&
	    fabsf(accel_ema[1]) < accel_err_thr &&
	    fabsf(accel_ema[2] - CONSTANTS_ONE_G) < accel_err_thr) {
		return DETECT_ORIENTATION_UPSIDE_DOWN;        // [ 0, 0, g ]
	}
	
	if (fabsf(accel_ema[0]) < accel_err_thr &&
	    fabsf(accel_ema[1]) < accel_err_thr &&
	    fabsf(accel_ema[2] + CONSTANTS_ONE_G) < accel_err_thr) {
		return DETECT_ORIENTATION_RIGHTSIDE_UP;        // [ 0, 0, -g ]
	}
	
	mavlink_and_console_log_critical(mavlink_fd, "[cal] ERROR: invalid orientation");
	
	return DETECT_ORIENTATION_ERROR;	// Can't detect orientation
}

const char* detect_orientation_str(enum detect_orientation_return orientation)
{
	static const char* rgOrientationStrs[] = {
		"back",		// tail down
		"front",	// nose down
		"left",
		"right",
		"up",		// upside-down
		"down",		// right-side up
		"error"
	};
	
	return rgOrientationStrs[orientation];
}

calibrate_return calibrate_from_orientation(int		mavlink_fd,
					    int		cancel_sub,
					    bool	side_data_collected[detect_orientation_side_count],
					    calibration_from_orientation_worker_t calibration_worker,
					    void*	worker_data,
					    bool	lenient_still_position)
{
	calibrate_return result = calibrate_return_ok;
	
	// Setup subscriptions to onboard accel sensor
	
	int sub_accel = orb_subscribe(ORB_ID(sensor_combined));
	if (sub_accel < 0) {
		mavlink_and_console_log_critical(mavlink_fd, CAL_QGC_FAILED_MSG, "No onboard accel");
		return calibrate_return_error;
	}
	
	unsigned orientation_failures = 0;
	
	// Rotate through all requested orientation
	while (true) {
		if (calibrate_cancel_check(mavlink_fd, cancel_sub)) {
			result = calibrate_return_cancelled;
			break;
		}
		
		if (orientation_failures > 4) {
			result = calibrate_return_error;
			mavlink_and_console_log_critical(mavlink_fd, CAL_QGC_FAILED_MSG, "timeout: no motion");
			break;
		}
		
		unsigned int side_complete_count = 0;
		
		// Update the number of completed sides
		for (unsigned i = 0; i < detect_orientation_side_count; i++) {
			if (side_data_collected[i]) {
				side_complete_count++;
			}
		}
		
		if (side_complete_count == detect_orientation_side_count) {
			// We have completed all sides, move on
			break;
		}
		
		/* inform user which orientations are still needed */
		char pendingStr[256];
		pendingStr[0] = 0;
		
		for (unsigned int cur_orientation=0; cur_orientation<detect_orientation_side_count; cur_orientation++) {
			if (!side_data_collected[cur_orientation]) {
				strcat(pendingStr, " ");
				strcat(pendingStr, detect_orientation_str((enum detect_orientation_return)cur_orientation));
			}
		}
		mavlink_and_console_log_info(mavlink_fd, "[cal] pending:%s", pendingStr);
		
		mavlink_and_console_log_info(mavlink_fd, "[cal] hold vehicle still on a pending side");
		enum detect_orientation_return orient = detect_orientation(mavlink_fd, cancel_sub, sub_accel, lenient_still_position);
		
		if (orient == DETECT_ORIENTATION_ERROR) {
			orientation_failures++;
			mavlink_and_console_log_info(mavlink_fd, "[cal] detected motion, hold still...");
			continue;
		}
		
		/* inform user about already handled side */
		if (side_data_collected[orient]) {
			orientation_failures++;
			mavlink_and_console_log_critical(mavlink_fd, "%s side already completed", detect_orientation_str(orient));
			mavlink_and_console_log_critical(mavlink_fd, "rotate to a pending side");
			continue;
		}
		
		mavlink_and_console_log_info(mavlink_fd, CAL_QGC_ORIENTATION_DETECTED_MSG, detect_orientation_str(orient));
		orientation_failures = 0;
		
		// Call worker routine
		result = calibration_worker(orient, cancel_sub, worker_data);
		if (result != calibrate_return_ok ) {
			break;
		}
		
		mavlink_and_console_log_info(mavlink_fd, CAL_QGC_SIDE_DONE_MSG, detect_orientation_str(orient));
		
		// Note that this side is complete
		side_data_collected[orient] = true;
		tune_neutral(true);
		usleep(200000);
	}
	
	if (sub_accel >= 0) {
		px4_close(sub_accel);
	}
	
	return result;
}

int calibrate_cancel_subscribe(void)
{
	return orb_subscribe(ORB_ID(vehicle_command));
}

void calibrate_cancel_unsubscribe(int cmd_sub)
{
	orb_unsubscribe(cmd_sub);
}

static void calibrate_answer_command(int mavlink_fd, struct vehicle_command_s &cmd, unsigned result)
{
	switch (result) {
		case vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED:
			tune_positive(true);
			break;
			
		case vehicle_command_s::VEHICLE_CMD_RESULT_DENIED:
			mavlink_log_critical(mavlink_fd, "command denied during calibration: %u", cmd.command);
			tune_negative(true);
			break;
			
		default:
			break;
	}
}

bool calibrate_cancel_check(int mavlink_fd, int cancel_sub)
{
	px4_pollfd_struct_t fds[1];
	fds[0].fd = cancel_sub;
	fds[0].events = POLLIN;

	if (px4_poll(&fds[0], 1, 0) > 0) {
		struct vehicle_command_s cmd;
		memset(&cmd, 0, sizeof(cmd));
		
		orb_copy(ORB_ID(vehicle_command), cancel_sub, &cmd);
		
		if (cmd.command == vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION &&
		    (int)cmd.param1 == 0 &&
		    (int)cmd.param2 == 0 &&
		    (int)cmd.param3 == 0 &&
		    (int)cmd.param4 == 0 &&
		    (int)cmd.param5 == 0 &&
		    (int)cmd.param6 == 0) {
			calibrate_answer_command(mavlink_fd, cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);
			mavlink_log_critical(mavlink_fd, CAL_QGC_CANCELLED_MSG);
			return true;
		} else {
			calibrate_answer_command(mavlink_fd, cmd, vehicle_command_s::VEHICLE_CMD_RESULT_DENIED);
		}
	}
	
	return false;
}
