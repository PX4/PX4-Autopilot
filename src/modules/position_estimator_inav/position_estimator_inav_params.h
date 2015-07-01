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

/*
 * @file position_estimator_inav_params.c
 *
 * @author Anton Babushkin <rk3dov@gmail.com>
 *
 * Parameters definition for position_estimator_inav
 */

#include <systemlib/param/param.h>

struct position_estimator_inav_params {
	float w_z_baro;
	float w_z_gps_p;
	float w_z_gps_v;
	float w_z_vision_p;
	float w_z_sonar;
	float w_xy_gps_p;
	float w_xy_gps_v;
	float w_xy_vision_p;
	float w_xy_vision_v;
	float w_mocap_p;
	float w_xy_flow;
	float w_xy_res_v;
	float w_gps_flow;
	float w_acc_bias;
	float flow_k;
	float flow_q_min;
	float sonar_filt;
	float sonar_err;
	float land_t;
	float land_disp;
	float land_thr;
	int32_t no_vision;
	float delay_gps;
};

struct position_estimator_inav_param_handles {
	param_t w_z_baro;
	param_t w_z_gps_p;
	param_t w_z_gps_v;
	param_t w_z_vision_p;
	param_t w_z_sonar;
	param_t w_xy_gps_p;
	param_t w_xy_gps_v;
	param_t w_xy_vision_p;
	param_t w_xy_vision_v;
	param_t w_mocap_p;
	param_t w_xy_flow;
	param_t w_xy_res_v;
	param_t w_gps_flow;
	param_t w_acc_bias;
	param_t flow_k;
	param_t flow_q_min;
	param_t sonar_filt;
	param_t sonar_err;
	param_t land_t;
	param_t land_disp;
	param_t land_thr;
	param_t no_vision;
	param_t delay_gps;
};

#define CBRK_NO_VISION_KEY	328754

/**
 * Initialize all parameter handles and values
 *
 */
int inav_parameters_init(struct position_estimator_inav_param_handles *h);

/**
 * Update all parameters
 *
 */
int inav_parameters_update(const struct position_estimator_inav_param_handles *h, struct position_estimator_inav_params *p);
