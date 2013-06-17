/****************************************************************************
 *
 *   Copyright (C) 2008-2013 PX4 Development Team. All rights reserved.
 *   Author: Samuel Zihlmann <samuezih@ee.ethz.ch>
 *   		 Lorenz Meier <lm@inf.ethz.ch>
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
 * @file flow_position_control_params.h
 * 
 * Parameters for position controller
 */

#include <systemlib/param/param.h>

struct flow_position_control_params {
	float pos_p;
	float pos_d;
	float height_p;
	float height_i;
	float height_d;
	float height_rate;
	float height_min;
	float height_max;
	float thrust_feedforward;
	float limit_speed_x;
	float limit_speed_y;
	float limit_height_error;
	float limit_thrust_int;
	float limit_thrust_upper;
	float limit_thrust_lower;
	float limit_yaw_step;
	float manual_threshold;
	float rc_scale_pitch;
	float rc_scale_roll;
	float rc_scale_yaw;
};

struct flow_position_control_param_handles {
	param_t pos_p;
	param_t pos_d;
	param_t height_p;
	param_t height_i;
	param_t height_d;
	param_t height_rate;
	param_t height_min;
	param_t height_max;
	param_t thrust_feedforward;
	param_t limit_speed_x;
	param_t limit_speed_y;
	param_t limit_height_error;
	param_t limit_thrust_int;
	param_t limit_thrust_upper;
	param_t limit_thrust_lower;
	param_t limit_yaw_step;
	param_t manual_threshold;
	param_t rc_scale_pitch;
	param_t rc_scale_roll;
	param_t rc_scale_yaw;
};

/**
 * Initialize all parameter handles and values
 *
 */
int parameters_init(struct flow_position_control_param_handles *h);

/**
 * Update all parameters
 *
 */
int parameters_update(const struct flow_position_control_param_handles *h, struct flow_position_control_params *p);
