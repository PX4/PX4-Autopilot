/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Anton Babushkin <anton.babushkin@me.com>
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
 * @file multirotor_pos_control_params.h
 *
 * Parameters for multirotor_pos_control
 */

#include <systemlib/param/param.h>

struct multirotor_position_control_params {
	float takeoff_alt;
	float takeoff_gap;
	float thr_min;
	float thr_max;
	float z_p;
	float z_d;
	float z_vel_p;
	float z_vel_i;
	float z_vel_d;
	float z_vel_max;
	float xy_p;
	float xy_d;
	float xy_vel_p;
	float xy_vel_i;
	float xy_vel_d;
	float xy_vel_max;
	float tilt_max;

	float rc_scale_pitch;
	float rc_scale_roll;
	float rc_scale_yaw;
};

struct multirotor_position_control_param_handles {
	param_t takeoff_alt;
	param_t takeoff_gap;
	param_t thr_min;
	param_t thr_max;
	param_t z_p;
	param_t z_d;
	param_t z_vel_p;
	param_t z_vel_i;
	param_t z_vel_d;
	param_t z_vel_max;
	param_t xy_p;
	param_t xy_d;
	param_t xy_vel_p;
	param_t xy_vel_i;
	param_t xy_vel_d;
	param_t xy_vel_max;
	param_t tilt_max;

	param_t rc_scale_pitch;
	param_t rc_scale_roll;
	param_t rc_scale_yaw;
};

/**
 * Initialize all parameter handles and values
 *
 */
int parameters_init(struct multirotor_position_control_param_handles *h);

/**
 * Update all parameters
 *
 */
int parameters_update(const struct multirotor_position_control_param_handles *h, struct multirotor_position_control_params *p);
