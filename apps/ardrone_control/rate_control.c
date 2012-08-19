/****************************************************************************
 *
 *   Copyright (C) 2008-2012 PX4 Development Team. All rights reserved.
 *   Author: @author Lorenz Meier <lm@inf.ethz.ch>
 *           @author Tobias Naegeli <nagelit@student.ethz.ch>
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
 * @file rate_control.c
 * Implementation of attitude rate control
 */

#include "rate_control.h"
#include "ardrone_motor_control.h"
#include <arch/board/up_hrt.h>

extern int ardrone_write;
extern int gpios;

typedef struct  {
	uint16_t motor_front_nw; ///< Front motor in + configuration, front left motor in x configuration
	uint16_t motor_right_ne; ///< Right motor in + configuration, front right motor in x configuration
	uint16_t motor_back_se; ///< Back motor in + configuration, back right motor in x configuration
	uint16_t motor_left_sw; ///< Left motor in + configuration, back left motor in x configuration
	uint8_t target_system; ///< System ID of the system that should set these motor commands
} quad_motors_setpoint_t;


void control_rates(int ardrone_write, struct sensor_combined_s *raw, struct ardrone_motors_setpoint_s *setpoints)
{
	static quad_motors_setpoint_t actuators_desired;
	//static quad_motors_setpoint_t quad_motors_setpoint_desired;

	static int16_t outputBand = 0;

//	static uint16_t control_counter;
	static hrt_abstime now_time;
	static hrt_abstime last_time;

	static float setpointXrate;
	static float setpointYrate;
	static float setpointZrate;

	static float setpointRateCast[3];
	static float Kp;
//	static float Ki;
	static float setpointThrustCast;
	static float startpointFullControll;
	static float maxThrustSetpoints;

	static float gyro_filtered[3];
	static float gyro_filtered_offset[3];
	static float gyro_alpha;
	static float gyro_alpha_offset;
//	static float errXrate;
	static float attRatesScaled[3];

	static uint16_t offsetCnt;
//	static float antiwindup;
	static int motor_skip_counter;

	static int read_ret;

	static bool initialized;

	if (initialized == false) {
		initialized = true;

		/* Read sensors for initial values */

		gyro_filtered_offset[0] = 0.00026631611f * (float)raw->gyro_raw[0];
		gyro_filtered_offset[1] = 0.00026631611f * (float)raw->gyro_raw[1];
		gyro_filtered_offset[2] = 0.00026631611f * (float)raw->gyro_raw[2];

		gyro_filtered[0] = 0.00026631611f * (float)raw->gyro_raw[0];
		gyro_filtered[1] = 0.00026631611f * (float)raw->gyro_raw[1];
		gyro_filtered[2] = 0.00026631611f * (float)raw->gyro_raw[2];

		outputBand = 0;
		startpointFullControll = 150.0f;
		maxThrustSetpoints = 511.0f;
		//Kp=60;
		//Kp=40.0f;
		//Kp=45;
		Kp = 30.0f;
//		Ki=0.0f;
//		antiwindup=50.0f;
	}

	/* Get setpoint */


	//Rate Controller
	setpointRateCast[0] = -((float)setpoints->motor_right_ne - 9999.0f) * 0.01f / 180.0f * 3.141f;
	setpointRateCast[1] = -((float)setpoints->motor_front_nw - 9999.0f) * 0.01f / 180.0f * 3.141f;
	setpointRateCast[2] = 0; //-((float)setpoints->motor_back_se-9999.0f)*0.01f;
	//Ki=actuatorDesired.motorRight_NE*0.001f;
	setpointThrustCast = setpoints->motor_left_sw;

	attRatesScaled[0] = 0.000317603994f * (float)raw->gyro_raw[0];
	attRatesScaled[1] = 0.000317603994f * (float)raw->gyro_raw[1];
	attRatesScaled[2] = 0.000317603994f * (float)raw->gyro_raw[2];

	//filtering of the gyroscope values

	//compute filter coefficient alpha

	//gyro_alpha=0.005/(2.0f*3.1415f*200.0f+0.005f);
	//gyro_alpha=0.009;
	gyro_alpha = 0.09f;
	gyro_alpha_offset = 0.001f;
	//gyro_alpha=0.001;
	//offset estimation and filtering
	offsetCnt++;
	uint8_t i;

	for (i = 0; i < 3; i++) {
		if (offsetCnt < 5000) {
			gyro_filtered_offset[i] = attRatesScaled[i] * gyro_alpha_offset +  gyro_filtered_offset[i] * (1 - gyro_alpha_offset);
		}

		gyro_filtered[i] = 1.0f * ((attRatesScaled[i] - gyro_filtered_offset[i]) * gyro_alpha + gyro_filtered[i] * (1 - gyro_alpha)) - 0 * setpointRateCast[i];
	}

	// //START DEBUG
	// /* write filtered values to global_data_attitude */
	// global_data_attitude->rollspeed = gyro_filtered[0];
	// global_data_attitude->pitchspeed = gyro_filtered[1];
	// global_data_attitude->yawspeed = gyro_filtered[2];
	// //END DEBUG

	//rate controller

	//X-axis
	setpointXrate = -Kp * (setpointRateCast[0] - gyro_filtered[0]);
	//Y-axis
	setpointYrate = -Kp * (setpointRateCast[1] - gyro_filtered[1]);
	//Z-axis
	setpointZrate = -Kp * (setpointRateCast[2] - gyro_filtered[2]);




	//Mixing
	if (setpointThrustCast <= 0) {
		setpointThrustCast = 0;
		outputBand = 0;
	}

	if ((setpointThrustCast < startpointFullControll) && (setpointThrustCast > 0)) {
		outputBand = 0.75f * setpointThrustCast;
	}

	if ((setpointThrustCast >= startpointFullControll) && (setpointThrustCast < maxThrustSetpoints - 0.75f * startpointFullControll)) {
		outputBand = 0.75f * startpointFullControll;
	}

	if (setpointThrustCast >= maxThrustSetpoints - 0.75f * startpointFullControll) {
		setpointThrustCast = 0.75f * startpointFullControll;
		outputBand = 0.75f * startpointFullControll;
	}

	actuators_desired.motor_front_nw = setpointThrustCast + (setpointXrate + setpointYrate + setpointZrate);
	actuators_desired.motor_right_ne = setpointThrustCast + (-setpointXrate + setpointYrate - setpointZrate);
	actuators_desired.motor_back_se = setpointThrustCast + (-setpointXrate - setpointYrate + setpointZrate);
	actuators_desired.motor_left_sw = setpointThrustCast + (setpointXrate - setpointYrate - setpointZrate);


	if ((setpointThrustCast + setpointXrate + setpointYrate + setpointZrate) > (setpointThrustCast + outputBand)) {
		actuators_desired.motor_front_nw = setpointThrustCast + outputBand;
	}

	if ((setpointThrustCast + setpointXrate + setpointYrate + setpointZrate) < (setpointThrustCast - outputBand)) {
		actuators_desired.motor_front_nw = setpointThrustCast - outputBand;
	}

	if ((setpointThrustCast + (-setpointXrate) + setpointYrate - setpointZrate) > (setpointThrustCast + outputBand)) {
		actuators_desired.motor_right_ne = setpointThrustCast + outputBand;
	}

	if ((setpointThrustCast + (-setpointXrate) + setpointYrate - setpointZrate) < (setpointThrustCast - outputBand)) {
		actuators_desired.motor_right_ne = setpointThrustCast - outputBand;
	}

	if ((setpointThrustCast + (-setpointXrate) + (-setpointYrate) + setpointZrate) > (setpointThrustCast + outputBand)) {
		actuators_desired.motor_back_se = setpointThrustCast + outputBand;
	}

	if ((setpointThrustCast + (-setpointXrate) + (-setpointYrate) + setpointZrate) < (setpointThrustCast - outputBand)) {
		actuators_desired.motor_back_se = setpointThrustCast - outputBand;
	}

	if ((setpointThrustCast + setpointXrate + (-setpointYrate) - setpointZrate) > (setpointThrustCast + outputBand)) {
		actuators_desired.motor_left_sw = setpointThrustCast + outputBand;
	}

	if ((setpointThrustCast + setpointXrate + (-setpointYrate) - setpointZrate) < (setpointThrustCast - outputBand)) {
		actuators_desired.motor_left_sw = setpointThrustCast - outputBand;
	}

	//printf("%lu,%lu,%lu,%lu\n",actuators_desired.motor_front_nw, actuators_desired.motor_right_ne, actuators_desired.motor_back_se, actuators_desired.motor_left_sw);

	if (motor_skip_counter % 5 == 0) {
		uint8_t motorSpeedBuf[5];
		ar_get_motor_packet(motorSpeedBuf, actuators_desired.motor_front_nw, actuators_desired.motor_right_ne, actuators_desired.motor_back_se, actuators_desired.motor_left_sw);
//		uint8_t* motorSpeedBuf = ar_get_motor_packet(1, 1, 1, 1);
//			if(motor_skip_counter %50 == 0)
//			{
//				if(0==actuators_desired.motor_front_nw || 0 == actuators_desired.motor_right_ne || 0 == actuators_desired.motor_back_se || 0 == actuators_desired.motor_left_sw)
//					printf("Motors set: %u, %u, %u, %u\n", actuators_desired.motor_front_nw, actuators_desired.motor_right_ne, actuators_desired.motor_back_se, actuators_desired.motor_left_sw);
//			printf("input: %u\n", setpoints->motor_front_nw);
//			printf("Roll casted desired: %f, Pitch casted desired: %f, Yaw casted desired: %f\n", setpointRateCast[0], setpointRateCast[1], setpointRateCast[2]);
//			}
		write(ardrone_write, motorSpeedBuf, 5);
//			motor_skip_counter = 0;
	}

	motor_skip_counter++;

	//START DEBUG
//	global_data_lock(&global_data_ardrone_control->access_conf);
//	global_data_ardrone_control->timestamp = hrt_absolute_time();
//	global_data_ardrone_control->gyro_scaled[0] = attRatesScaled[0];
//	global_data_ardrone_control->gyro_scaled[1] = attRatesScaled[1];
//	global_data_ardrone_control->gyro_scaled[2] = attRatesScaled[2];
//	global_data_ardrone_control->gyro_filtered[0] = gyro_filtered[0];
//	global_data_ardrone_control->gyro_filtered[1] = gyro_filtered[1];
//	global_data_ardrone_control->gyro_filtered[2] = gyro_filtered[2];
//	global_data_ardrone_control->gyro_filtered_offset[0] = gyro_filtered_offset[0];
//	global_data_ardrone_control->gyro_filtered_offset[1] = gyro_filtered_offset[1];
//	global_data_ardrone_control->gyro_filtered_offset[2] = gyro_filtered_offset[2];
//	global_data_ardrone_control->setpoint_rate_cast[0] = setpointRateCast[0];
//	global_data_ardrone_control->setpoint_rate_cast[1] = setpointRateCast[1];
//	global_data_ardrone_control->setpoint_rate_cast[2] = setpointRateCast[2];
//	global_data_ardrone_control->setpoint_thrust_cast = setpointThrustCast;
//	global_data_ardrone_control->setpoint_rate[0] = setpointXrate;
//	global_data_ardrone_control->setpoint_rate[1] = setpointYrate;
//	global_data_ardrone_control->setpoint_rate[2] = setpointZrate;
//	global_data_ardrone_control->motor_front_nw = actuators_desired.motor_front_nw;
//	global_data_ardrone_control->motor_right_ne = actuators_desired.motor_right_ne;
//	global_data_ardrone_control->motor_back_se = actuators_desired.motor_back_se;
//	global_data_ardrone_control->motor_left_sw = actuators_desired.motor_left_sw;
//	global_data_unlock(&global_data_ardrone_control->access_conf);
//	global_data_broadcast(&global_data_ardrone_control->access_conf);
	//END DEBUG



//	gettimeofday(&tv, NULL);
//	now = ((uint32_t)tv.tv_sec) * 1000 + tv.tv_usec/1000;
//	time_elapsed = now - last_run;
//	if (time_elapsed*1000 > CONTROL_LOOP_USLEEP)
//	{
//		sleep_time = (int32_t)CONTROL_LOOP_USLEEP - ((int32_t)time_elapsed*1000 - (int32_t)CONTROL_LOOP_USLEEP);
//
//		if(motor_skip_counter %500 == 0)
//		{
//			printf("Desired: %u, New usleep: %i, Time elapsed: %u, Now: %u, Last run: %u\n",(uint32_t)CONTROL_LOOP_USLEEP, sleep_time, time_elapsed*1000, now, last_run);
//		}
//	}
//
//	if (sleep_time <= 0)
//	{
//		printf("WARNING: CPU Overload!\n");
//		printf("Desired: %u, New usleep: %i, Time elapsed: %u, Now: %u, Last run: %u\n",(uint32_t)CONTROL_LOOP_USLEEP, sleep_time, time_elapsed*1000, now, last_run);
//		usleep(CONTROL_LOOP_USLEEP);
//	}
//	else
//	{
//		usleep(sleep_time);
//	}
//	last_run = now;
//
//	now_time = hrt_absolute_time();
//	if(control_counter % 500 == 0)
//	{
//		printf("Now: %lu\n",(unsigned long)now_time);
//		printf("Last: %lu\n",(unsigned long)last_time);
//		printf("Difference: %lu\n", (unsigned long)(now_time - last_time));
//		printf("now seconds: %lu\n", (unsigned long)(now_time / 1000000));
//	}
//	last_time = now_time;
//
//	now_time = hrt_absolute_time() / 1000000;
//	if(now_time - last_time > 0)
//	{
//		printf("Counter: %ld\n",control_counter);
//		last_time = now_time;
//		control_counter = 0;
//	}
//	control_counter++;
}
