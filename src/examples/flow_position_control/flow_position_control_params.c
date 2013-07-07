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
 * @file flow_position_control_params.c
 */

#include "flow_position_control_params.h"

/* controller parameters */

// Position control P gain
PARAM_DEFINE_FLOAT(FPC_POS_P, 3.0f);
// Position control D / damping gain
PARAM_DEFINE_FLOAT(FPC_POS_D, 0.0f);
// Altitude control P gain
PARAM_DEFINE_FLOAT(FPC_H_P, 0.15f);
// Altitude control I (integrator) gain
PARAM_DEFINE_FLOAT(FPC_H_I, 0.00001f);
// Altitude control D gain
PARAM_DEFINE_FLOAT(FPC_H_D, 0.8f);
// Altitude control rate limiter
PARAM_DEFINE_FLOAT(FPC_H_RATE, 0.1f);
// Altitude control minimum altitude
PARAM_DEFINE_FLOAT(FPC_H_MIN, 0.5f);
// Altitude control maximum altitude (higher than 1.5m is untested)
PARAM_DEFINE_FLOAT(FPC_H_MAX, 1.5f);
// Altitude control feed forward throttle - adjust to the
// throttle position (0..1) where the copter hovers in manual flight
PARAM_DEFINE_FLOAT(FPC_T_FFWD, 0.7f); // adjust this before flight
PARAM_DEFINE_FLOAT(FPC_L_S_X, 1.2f);
PARAM_DEFINE_FLOAT(FPC_L_S_Y, 1.2f);
PARAM_DEFINE_FLOAT(FPC_L_H_ERR, 0.1f);
PARAM_DEFINE_FLOAT(FPC_L_TH_I, 0.05f);
PARAM_DEFINE_FLOAT(FPC_L_TH_U, 0.8f);
PARAM_DEFINE_FLOAT(FPC_L_TH_L, 0.6f);
PARAM_DEFINE_FLOAT(FPC_L_YAW_STEP, 0.03f);
PARAM_DEFINE_FLOAT(FPC_MAN_THR, 0.1f);


int parameters_init(struct flow_position_control_param_handles *h)
{
	/* PID parameters */
	h->pos_p	 			=	param_find("FPC_POS_P");
	h->pos_d 				=	param_find("FPC_POS_D");
	h->height_p 			=	param_find("FPC_H_P");
	h->height_i 			=	param_find("FPC_H_I");
	h->height_d 			=	param_find("FPC_H_D");
	h->height_rate 			=	param_find("FPC_H_RATE");
	h->height_min			=	param_find("FPC_H_MIN");
	h->height_max			=	param_find("FPC_H_MAX");
	h->thrust_feedforward 	=	param_find("FPC_T_FFWD");
	h->limit_speed_x 		=	param_find("FPC_L_S_X");
	h->limit_speed_y 		=	param_find("FPC_L_S_Y");
	h->limit_height_error	=	param_find("FPC_L_H_ERR");
	h->limit_thrust_int 	=	param_find("FPC_L_TH_I");
	h->limit_thrust_upper 	=	param_find("FPC_L_TH_U");
	h->limit_thrust_lower 	=	param_find("FPC_L_TH_L");
	h->limit_yaw_step		=	param_find("FPC_L_YAW_STEP");
	h->manual_threshold 	=	param_find("FPC_MAN_THR");
	h->rc_scale_pitch		=   param_find("RC_SCALE_PITCH");
	h->rc_scale_roll		=   param_find("RC_SCALE_ROLL");
	h->rc_scale_yaw			=   param_find("RC_SCALE_YAW");

	return OK;
}

int parameters_update(const struct flow_position_control_param_handles *h, struct flow_position_control_params *p)
{
	param_get(h->pos_p, &(p->pos_p));
	param_get(h->pos_d, &(p->pos_d));
	param_get(h->height_p, &(p->height_p));
	param_get(h->height_i, &(p->height_i));
	param_get(h->height_d, &(p->height_d));
	param_get(h->height_rate, &(p->height_rate));
	param_get(h->height_min, &(p->height_min));
	param_get(h->height_max, &(p->height_max));
	param_get(h->thrust_feedforward, &(p->thrust_feedforward));
	param_get(h->limit_speed_x, &(p->limit_speed_x));
	param_get(h->limit_speed_y, &(p->limit_speed_y));
	param_get(h->limit_height_error, &(p->limit_height_error));
	param_get(h->limit_thrust_int, &(p->limit_thrust_int));
	param_get(h->limit_thrust_upper, &(p->limit_thrust_upper));
	param_get(h->limit_thrust_lower, &(p->limit_thrust_lower));
	param_get(h->limit_yaw_step, &(p->limit_yaw_step));
	param_get(h->manual_threshold, &(p->manual_threshold));
	param_get(h->rc_scale_pitch, &(p->rc_scale_pitch));
	param_get(h->rc_scale_roll, &(p->rc_scale_roll));
	param_get(h->rc_scale_yaw, &(p->rc_scale_yaw));

	return OK;
}
