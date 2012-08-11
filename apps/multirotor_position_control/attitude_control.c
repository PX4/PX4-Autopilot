/****************************************************************************
 *
 *   Copyright (C) 2008-2012 PX4 Development Team. All rights reserved.
 *   Author: Thomas Gubler <thomasgubler@student.ethz.ch>
 *           Julian Oes <joes@student.ethz.ch>
 *           Laurens Mackay <mackayl@student.ethz.ch>
 *           Tobias Naegeli <naegelit@student.ethz.ch>
 *           Martin Rutschmann <rutmarti@student.ethz.ch>
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
 * @file Implementation of attitude controller
 */

#include "attitude_control.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <px4/attitude_estimator_bm/matrix.h> //TODO: move matrix.h to somewhere else?
#include "ardrone_motor_control.h"
#include <float.h>
#include <math.h>
#include "pid.h"
#include <arch/board/up_hrt.h>

extern int ardrone_write;
extern int gpios;

#define CONTROL_PID_ATTITUDE_INTERVAL	5e-3f

void turn_xy_plane(const float_vect3 *vector, float yaw,
		   float_vect3 *result);
void navi2body_xy_plane(const float_vect3 *vector, const float yaw,
			float_vect3 *result);

void turn_xy_plane(const float_vect3 *vector, float yaw,
		   float_vect3 *result)
{
	//turn clockwise
	static uint16_t counter;

	result->x = (cosf(yaw) * vector->x + sinf(yaw) * vector->y);
	result->y = (-sinf(yaw) * vector->x + cosf(yaw) * vector->y);
	result->z = vector->z; //leave direction normal to xy-plane untouched

	counter++;
}

void navi2body_xy_plane(const float_vect3 *vector, const float yaw,
			float_vect3 *result)
{
	turn_xy_plane(vector, yaw, result);
//	result->x = vector->x;
//	result->y = vector->y;
//	result->z = vector->z;
	//	result->x = cos(yaw) * vector->x + sin(yaw) * vector->y;
	//	result->y = -sin(yaw) * vector->x + cos(yaw) * vector->y;
	//	result->z = vector->z; //leave direction normal to xy-plane untouched
}

void control_attitude(float roll, float pitch, float yaw, float thrust, const struct vehicle_attitude_s *att, const struct vehicle_status_s *status, int ardrone_pub, struct ardrone_control_s *ar_control)
{
	static int motor_skip_counter = 0;

	static PID_t yaw_pos_controller;
	static PID_t yaw_speed_controller;
	static PID_t nick_controller;
	static PID_t roll_controller;

	const float min_thrust = 0.02f;			/**< 2% minimum thrust */
	const float max_thrust = 1.0f;			/**< 100% max thrust */
	const float scaling = 512.0f;			/**< 100% thrust equals a value of 512 */

	const float min_gas = min_thrust * scaling;
	const float max_gas = max_thrust * scaling;

	static uint16_t motor_pwm[4] = {0, 0, 0, 0};
	static float motor_calc[4] = {0.0f, 0.0f, 0.0f, 0.0f};
//	static float remote_control_weight_z = 1;
//	static float position_control_weight_z = 0;

	static float pid_yawpos_lim;
	static float pid_yawspeed_lim;
	static float pid_att_lim;

	static bool initialized;

	static float_vect3 attitude_setpoint_navigationframe_from_positioncontroller;
	static commander_state_machine_t current_state;

	/* initialize the pid controllers when the function is called for the first time */
	if (initialized == false) {

		pid_init(&yaw_pos_controller,
			 global_data_parameter_storage->pm.param_values[PARAM_PID_YAWPOS_P],
			 global_data_parameter_storage->pm.param_values[PARAM_PID_YAWPOS_I],
			 global_data_parameter_storage->pm.param_values[PARAM_PID_YAWPOS_D],
			 global_data_parameter_storage->pm.param_values[PARAM_PID_YAWPOS_AWU],
			 PID_MODE_DERIVATIV_CALC, 154);

		pid_init(&yaw_speed_controller,
			 global_data_parameter_storage->pm.param_values[PARAM_PID_YAWSPEED_P],
			 global_data_parameter_storage->pm.param_values[PARAM_PID_YAWSPEED_I],
			 global_data_parameter_storage->pm.param_values[PARAM_PID_YAWSPEED_D],
			 global_data_parameter_storage->pm.param_values[PARAM_PID_YAWSPEED_AWU],
			 PID_MODE_DERIVATIV_CALC, 155);

		pid_init(&nick_controller,
			 global_data_parameter_storage->pm.param_values[PARAM_PID_ATT_P],
			 global_data_parameter_storage->pm.param_values[PARAM_PID_ATT_I],
			 global_data_parameter_storage->pm.param_values[PARAM_PID_ATT_D],
			 global_data_parameter_storage->pm.param_values[PARAM_PID_ATT_AWU],
			 PID_MODE_DERIVATIV_SET, 156);

		pid_init(&roll_controller,
			 global_data_parameter_storage->pm.param_values[PARAM_PID_ATT_P],
			 global_data_parameter_storage->pm.param_values[PARAM_PID_ATT_I],
			 global_data_parameter_storage->pm.param_values[PARAM_PID_ATT_D],
			 global_data_parameter_storage->pm.param_values[PARAM_PID_ATT_AWU],
			 PID_MODE_DERIVATIV_SET, 157);

		pid_yawpos_lim = 	global_data_parameter_storage->pm.param_values[PARAM_PID_YAWPOS_LIM];
		pid_yawspeed_lim =	(max_gas - min_gas) * global_data_parameter_storage->pm.param_values[PARAM_PID_YAWSPEED_LIM];
		pid_att_lim =	(max_gas - min_gas) * global_data_parameter_storage->pm.param_values[PARAM_PID_ATT_LIM];

		//TODO: true initialization? get gps while on ground?
		attitude_setpoint_navigationframe_from_positioncontroller.x = 0.0f;
		attitude_setpoint_navigationframe_from_positioncontroller.y = 0.0f;
		attitude_setpoint_navigationframe_from_positioncontroller.z = 0.0f;

		initialized = true;
	}

	/* load new parameters with lower rate */
	if (motor_skip_counter % 50 == 0) {
		pid_set_parameters(&yaw_pos_controller,
				   global_data_parameter_storage->pm.param_values[PARAM_PID_YAWPOS_P],
				   global_data_parameter_storage->pm.param_values[PARAM_PID_YAWPOS_I],
				   global_data_parameter_storage->pm.param_values[PARAM_PID_YAWPOS_D],
				   global_data_parameter_storage->pm.param_values[PARAM_PID_YAWPOS_AWU]);

		pid_set_parameters(&yaw_speed_controller,
				   global_data_parameter_storage->pm.param_values[PARAM_PID_YAWSPEED_P],
				   global_data_parameter_storage->pm.param_values[PARAM_PID_YAWSPEED_I],
				   global_data_parameter_storage->pm.param_values[PARAM_PID_YAWSPEED_D],
				   global_data_parameter_storage->pm.param_values[PARAM_PID_YAWSPEED_AWU]);

		pid_set_parameters(&nick_controller,
				   global_data_parameter_storage->pm.param_values[PARAM_PID_ATT_P],
				   global_data_parameter_storage->pm.param_values[PARAM_PID_ATT_I],
				   global_data_parameter_storage->pm.param_values[PARAM_PID_ATT_D],
				   global_data_parameter_storage->pm.param_values[PARAM_PID_ATT_AWU]);

		pid_set_parameters(&roll_controller,
				   global_data_parameter_storage->pm.param_values[PARAM_PID_ATT_P],
				   global_data_parameter_storage->pm.param_values[PARAM_PID_ATT_I],
				   global_data_parameter_storage->pm.param_values[PARAM_PID_ATT_D],
				   global_data_parameter_storage->pm.param_values[PARAM_PID_ATT_AWU]);

		pid_yawpos_lim = global_data_parameter_storage->pm.param_values[PARAM_PID_YAWPOS_LIM];
		pid_yawspeed_lim = (max_gas - min_gas) * global_data_parameter_storage->pm.param_values[PARAM_PID_YAWSPEED_LIM];
		pid_att_lim = (max_gas - min_gas) * global_data_parameter_storage->pm.param_values[PARAM_PID_ATT_LIM];
	}

	current_state = status->state_machine;
	float_vect3 attitude_setpoint_bodyframe = {.x = 0.0f, .y = 0.0f, .z = 0.0f}; //this is the setpoint in the bodyframe "mixed" together from the setpoint from the remote and the setpoint from the position controller

	if (current_state == SYSTEM_STATE_AUTO) {

		attitude_setpoint_navigationframe_from_positioncontroller.x = ar_control->attitude_setpoint_navigationframe_from_positioncontroller[0];
		attitude_setpoint_navigationframe_from_positioncontroller.y = ar_control->attitude_setpoint_navigationframe_from_positioncontroller[1];
		attitude_setpoint_navigationframe_from_positioncontroller.z = ar_control->attitude_setpoint_navigationframe_from_positioncontroller[2];

		float yaw_e = att->yaw - attitude_setpoint_navigationframe_from_positioncontroller.z;

		// don't turn around the wrong side (only works if yaw angle is between +- 180 degree)
		if (yaw_e > M_PI_F) {
			yaw_e -= 2.0f * M_PI_F;
		}

		if (yaw_e < -M_PI_F) {
			yaw_e += 2.0f * M_PI_F;
		}

		attitude_setpoint_navigationframe_from_positioncontroller.z = pid_calculate(&yaw_pos_controller, 0.0f, yaw_e, 0.0f, CONTROL_PID_ATTITUDE_INTERVAL);


		/* limit control output */
		if (attitude_setpoint_navigationframe_from_positioncontroller.z > pid_yawpos_lim) {
			attitude_setpoint_navigationframe_from_positioncontroller.z = pid_yawpos_lim;
			yaw_pos_controller.saturated = 1;
		}

		if (attitude_setpoint_navigationframe_from_positioncontroller.z < -pid_yawpos_lim) {
			attitude_setpoint_navigationframe_from_positioncontroller.z = -pid_yawpos_lim;
			yaw_pos_controller.saturated = 1;
		}

		//transform attitude setpoint from position controller from navi to body frame on xy_plane
		float_vect3 attitude_setpoint_bodyframe_from_positioncontroller;
		navi2body_xy_plane(&attitude_setpoint_navigationframe_from_positioncontroller, att->yaw , &attitude_setpoint_bodyframe_from_positioncontroller); //yaw angle= att->yaw
		//now everything is in body frame


		//TODO: here we decide which input (position controller or ppm) we use. For now we have only the ppm, this should be decided dpending on the state machione (manula or auto) ppm should always overwrite auto (?)
		attitude_setpoint_bodyframe.x = attitude_setpoint_bodyframe_from_positioncontroller.x;
		attitude_setpoint_bodyframe.y = attitude_setpoint_bodyframe_from_positioncontroller.y;
		attitude_setpoint_bodyframe.z = attitude_setpoint_bodyframe_from_positioncontroller.z;

	} else if (current_state == SYSTEM_STATE_MANUAL) {
		attitude_setpoint_bodyframe.x = -roll * M_PI_F / 8.0f;
		attitude_setpoint_bodyframe.y = -pitch * M_PI_F / 8.0f;
		attitude_setpoint_bodyframe.z = -yaw * M_PI_F;
	}

	/* add an attitude offset which needs to be estimated somewhere */
	attitude_setpoint_bodyframe.x += global_data_parameter_storage->pm.param_values[PARAM_ATT_XOFFSET];
	attitude_setpoint_bodyframe.y += global_data_parameter_storage->pm.param_values[PARAM_ATT_YOFFSET];

	/*Calculate Controllers*/
	//control Nick
	float nick_control = pid_calculate(&nick_controller, attitude_setpoint_bodyframe.y, att->pitch, att->pitchspeed, CONTROL_PID_ATTITUDE_INTERVAL);
	//control Roll
	float roll_control = pid_calculate(&roll_controller, attitude_setpoint_bodyframe.x, att->roll, att->rollspeed, CONTROL_PID_ATTITUDE_INTERVAL);
	//control Yaw Speed
	float yaw_rate_control = pid_calculate(&yaw_speed_controller, attitude_setpoint_bodyframe.z, att->yawspeed, 0.0f, CONTROL_PID_ATTITUDE_INTERVAL); 	//attitude_setpoint_bodyframe.z is yaw speed!

	/*
	 * compensate the vertical loss of thrust
	 * when thrust plane has an angle.
	 * start with a factor of 1.0 (no change)
	 */
	float zcompensation = 1.0f;

	if (fabsf(att->roll) > 1.0f) {
		zcompensation *= 1.85081571768f;

	} else {
		zcompensation *= 1.0f / cosf(att->roll);
	}

	if (fabsf(att->pitch) > 1.0f) {
		zcompensation *= 1.85081571768f;

	} else {
		zcompensation *= 1.0f / cosf(att->pitch);
	}

	// use global_data.position_control_output.z and mix parameter global_data.param[PARAM_MIX_POSITION_Z_WEIGHT]
	// to compute thrust for Z position control
	//
	//	float motor_thrust = min_gas +
	//			( ( 1 - global_data.param[PARAM_MIX_POSITION_Z_WEIGHT] ) * ( max_gas - min_gas ) * global_data.gas_remote * zcompensation )
	//		   + ( global_data.param[PARAM_MIX_POSITION_Z_WEIGHT] * ( max_gas - min_gas ) * controlled_thrust * zcompensation );
	//calculate the basic thrust



	float motor_thrust = 0;

	// FLYING MODES
	if (current_state == SYSTEM_STATE_MANUAL) {
		motor_thrust = thrust;

	} else if (current_state == SYSTEM_STATE_GROUND_READY ||
		current_state == SYSTEM_STATE_STABILIZED ||
		current_state == SYSTEM_STATE_AUTO ||
		current_state == SYSTEM_STATE_MISSION_ABORT) {
		motor_thrust = thrust;	//TODO

	} else if (current_state == SYSTEM_STATE_EMCY_LANDING) {
		motor_thrust = thrust;	//TODO

	} else if (current_state == SYSTEM_STATE_EMCY_CUTOFF) {
		/* immediately cut off motors */
		motor_thrust = 0;

	} else {
		/* limit motor throttle to zero for an unknown mode */
		motor_thrust = 0;
	}

	printf("t1:%1.3f ");

	/* compensate thrust vector for roll / pitch contributions */
	motor_thrust *= zcompensation;

	printf("t2:%1.3f\n");

	/* limit yaw rate output */
	if (yaw_rate_control > pid_yawspeed_lim) {
		yaw_rate_control = pid_yawspeed_lim;
		yaw_speed_controller.saturated = 1;
	}

	if (yaw_rate_control < -pid_yawspeed_lim) {
		yaw_rate_control = -pid_yawspeed_lim;
		yaw_speed_controller.saturated = 1;
	}

	if (nick_control > pid_att_lim) {
		nick_control = pid_att_lim;
		nick_controller.saturated = 1;
	}

	if (nick_control < -pid_att_lim) {
		nick_control = -pid_att_lim;
		nick_controller.saturated = 1;
	}


	if (roll_control > pid_att_lim) {
		roll_control = pid_att_lim;
		roll_controller.saturated = 1;
	}

	if (roll_control < -pid_att_lim) {
		roll_control = -pid_att_lim;
		roll_controller.saturated = 1;
	}

	/* Emit controller values */
	ar_control->setpoint_thrust_cast = motor_thrust;
	ar_control->setpoint_attitude[0] = attitude_setpoint_bodyframe.x;
	ar_control->setpoint_attitude[1] = attitude_setpoint_bodyframe.y;
	ar_control->setpoint_attitude[2] = attitude_setpoint_bodyframe.z;
	ar_control->attitude_control_output[0] = roll_control;
	ar_control->attitude_control_output[1] = nick_control;
	ar_control->attitude_control_output[2] = yaw_rate_control;
	ar_control->zcompensation = zcompensation;
	orb_publish(ORB_ID(ardrone_control), ardrone_pub, ar_control);

	float output_band = 0.f;
	float band_factor = 0.75f;
	const float startpoint_full_control = 0.25f;	/**< start full control at 25% thrust */
	float yaw_factor = 1.0f;

	if (motor_thrust <= min_thrust) {
		motor_thrust = min_thrust;
		output_band = 0.f;

	} else if (motor_thrust < startpoint_full_control && motor_thrust > min_thrust) {
		output_band = band_factor * (motor_thrust - min_thrust);

	} else if (motor_thrust >= startpoint_full_control && motor_thrust < max_thrust - band_factor * startpoint_full_control) {
		output_band = band_factor * startpoint_full_control;

	} else if (motor_thrust >= max_thrust - band_factor * startpoint_full_control) {
		output_band = band_factor * (max_thrust - motor_thrust);
	}

	//add the yaw, nick and roll components to the basic thrust //TODO:this should be done by the mixer

	// FRONT (MOTOR 1)
	motor_calc[0] = motor_thrust + (roll_control / 2 + nick_control / 2 - yaw_rate_control);

	// RIGHT (MOTOR 2)
	motor_calc[1] = motor_thrust + (-roll_control / 2 + nick_control / 2 + yaw_rate_control);

	// BACK (MOTOR 3)
	motor_calc[2] = motor_thrust + (-roll_control / 2 - nick_control / 2 - yaw_rate_control);

	// LEFT (MOTOR 4)
	motor_calc[3] = motor_thrust + (roll_control / 2 - nick_control / 2 + yaw_rate_control);

	// if we are not in the output band
	if (!(motor_calc[0] < motor_thrust + output_band && motor_calc[0] > motor_thrust - output_band
	      && motor_calc[1] < motor_thrust + output_band && motor_calc[1] > motor_thrust - output_band
	      && motor_calc[2] < motor_thrust + output_band && motor_calc[2] > motor_thrust - output_band
	      && motor_calc[3] < motor_thrust + output_band && motor_calc[3] > motor_thrust - output_band)) {

		yaw_factor = 0.5f;
		// FRONT (MOTOR 1)
		motor_calc[0] = motor_thrust + (roll_control / 2 + nick_control / 2 - yaw_rate_control * yaw_factor);

		// RIGHT (MOTOR 2)
		motor_calc[1] = motor_thrust + (-roll_control / 2 + nick_control / 2 + yaw_rate_control * yaw_factor);

		// BACK (MOTOR 3)
		motor_calc[2] = motor_thrust + (-roll_control / 2 - nick_control / 2 - yaw_rate_control * yaw_factor);

		// LEFT (MOTOR 4)
		motor_calc[3] = motor_thrust + (roll_control / 2 - nick_control / 2 + yaw_rate_control * yaw_factor);
	}

	for (int i = 0; i < 4; i++) {
		//check for limits
		if (motor_calc[i] < motor_thrust - output_band) {
			motor_calc[i] = motor_thrust - output_band;
		}

		if (motor_calc[i] > motor_thrust + output_band) {
			motor_calc[i] = motor_thrust + output_band;
		}
	}

	/* set the motor values */

	/* scale up from 0..1 to 10..512) */
	motor_pwm[0] = (uint16_t) motor_calc[0] * ((float)max_gas - min_gas) + min_gas;
	motor_pwm[1] = (uint16_t) motor_calc[1] * ((float)max_gas - min_gas) + min_gas;
	motor_pwm[2] = (uint16_t) motor_calc[2] * ((float)max_gas - min_gas) + min_gas;
	motor_pwm[3] = (uint16_t) motor_calc[3] * ((float)max_gas - min_gas) + min_gas;

	/* Keep motors spinning while armed and prevent overflows */

	/* Failsafe logic - should never be necessary */
	motor_pwm[0] = (motor_pwm[0] > 0) ? motor_pwm[0] : 10;
	motor_pwm[1] = (motor_pwm[1] > 0) ? motor_pwm[1] : 10;
	motor_pwm[2] = (motor_pwm[2] > 0) ? motor_pwm[2] : 10;
	motor_pwm[3] = (motor_pwm[3] > 0) ? motor_pwm[3] : 10;

	/* Failsafe logic - should never be necessary */
	motor_pwm[0] = (motor_pwm[0] <= 512) ? motor_pwm[0] : 512;
	motor_pwm[1] = (motor_pwm[1] <= 512) ? motor_pwm[1] : 512;
	motor_pwm[2] = (motor_pwm[2] <= 512) ? motor_pwm[2] : 512;
	motor_pwm[3] = (motor_pwm[3] <= 512) ? motor_pwm[3] : 512;

	/* send motors via UART */
	if (motor_skip_counter % 5 == 0) {
		uint8_t motorSpeedBuf[5] = {1, 2, 3, 4, 5};
		ar_get_motor_packet(motorSpeedBuf, motor_pwm[0], motor_pwm[1], motor_pwm[2], motor_pwm[3]);
		write(ardrone_write, motorSpeedBuf, 5);
	}

	motor_skip_counter++;

}
