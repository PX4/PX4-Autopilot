/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Samuel Zihlmann <samuezih@ee.ethz.ch>
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
 * @file radar_flow_params.h
 * 
 * Parameters for radar
 */

#include <systemlib/param/param.h>

struct radar_flow_params {
	float pos_sp_x;
	float pos_sp_y;
	int beep_bottom_sonar;
	int beep_front_sonar;
	float kalman_k1;
	float kalman_k2;
	float front_lp_alpha;
	int with_sonar;
	int with_pos_update;
	float s0;
	float s1;
	float s2;
	float s3;
	float s4;
	float s5;
	float s6;
	float s7;
	int debug;
};

struct radar_flow_param_handles {
	param_t pos_sp_x;
	param_t pos_sp_y;
	param_t beep_bottom_sonar;
	param_t beep_front_sonar;
	param_t kalman_k1;
	param_t kalman_k2;
	param_t front_lp_alpha;
	param_t with_sonar;
	param_t with_pos_update;
	param_t s0;
	param_t s1;
	param_t s2;
	param_t s3;
	param_t s4;
	param_t s5;
	param_t s6;
	param_t s7;
	param_t debug;
};

/**
 * Initialize all parameter handles and values
 *
 */
int parameters_init(struct radar_flow_param_handles *h);

/**
 * Update all parameters
 *
 */
int parameters_update(const struct radar_flow_param_handles *h, struct radar_flow_params *p);
