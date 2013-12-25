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
 * @file multirotor_pos_control_params.c
 *
 * Parameters for multirotor_pos_control
 */

#include "multirotor_pos_control_params.h"

/* controller parameters */
PARAM_DEFINE_FLOAT(MPC_THR_MIN, 0.2f);
PARAM_DEFINE_FLOAT(MPC_THR_MAX, 0.8f);
PARAM_DEFINE_FLOAT(MPC_Z_P, 1.0f);
PARAM_DEFINE_FLOAT(MPC_Z_D, 0.0f);
PARAM_DEFINE_FLOAT(MPC_Z_VEL_P, 0.1f);
PARAM_DEFINE_FLOAT(MPC_Z_VEL_I, 0.0f);
PARAM_DEFINE_FLOAT(MPC_Z_VEL_D, 0.0f);
PARAM_DEFINE_FLOAT(MPC_Z_VEL_MAX, 3.0f);
PARAM_DEFINE_FLOAT(MPC_XY_P, 0.5f);
PARAM_DEFINE_FLOAT(MPC_XY_D, 0.0f);
PARAM_DEFINE_FLOAT(MPC_XY_VEL_P, 0.2f);
PARAM_DEFINE_FLOAT(MPC_XY_VEL_I, 0.0f);
PARAM_DEFINE_FLOAT(MPC_XY_VEL_D, 0.0f);
PARAM_DEFINE_FLOAT(MPC_XY_VEL_MAX, 5.0f);
PARAM_DEFINE_FLOAT(MPC_TILT_MAX, 0.5f);

int parameters_init(struct multirotor_position_control_param_handles *h)
{
	h->takeoff_alt = param_find("NAV_TAKEOFF_ALT");
	h->takeoff_gap = param_find("NAV_TAKEOFF_GAP");
	h->thr_min 	=	param_find("MPC_THR_MIN");
	h->thr_max 	=	param_find("MPC_THR_MAX");
	h->z_p 	=	param_find("MPC_Z_P");
	h->z_d 	=	param_find("MPC_Z_D");
	h->z_vel_p 	=	param_find("MPC_Z_VEL_P");
	h->z_vel_i 	=	param_find("MPC_Z_VEL_I");
	h->z_vel_d 	=	param_find("MPC_Z_VEL_D");
	h->z_vel_max 	=	param_find("MPC_Z_VEL_MAX");
	h->xy_p 	=	param_find("MPC_XY_P");
	h->xy_d 	=	param_find("MPC_XY_D");
	h->xy_vel_p 	=	param_find("MPC_XY_VEL_P");
	h->xy_vel_i 	=	param_find("MPC_XY_VEL_I");
	h->xy_vel_d 	=	param_find("MPC_XY_VEL_D");
	h->xy_vel_max 	=	param_find("MPC_XY_VEL_MAX");
	h->tilt_max 	=	param_find("MPC_TILT_MAX");

	h->rc_scale_pitch    =   param_find("RC_SCALE_PITCH");
	h->rc_scale_roll    =   param_find("RC_SCALE_ROLL");
	h->rc_scale_yaw      =   param_find("RC_SCALE_YAW");

	return OK;
}

int parameters_update(const struct multirotor_position_control_param_handles *h, struct multirotor_position_control_params *p)
{
	param_get(h->takeoff_alt, &(p->takeoff_alt));
	param_get(h->takeoff_gap, &(p->takeoff_gap));
	param_get(h->thr_min, &(p->thr_min));
	param_get(h->thr_max, &(p->thr_max));
	param_get(h->z_p, &(p->z_p));
	param_get(h->z_d, &(p->z_d));
	param_get(h->z_vel_p, &(p->z_vel_p));
	param_get(h->z_vel_i, &(p->z_vel_i));
	param_get(h->z_vel_d, &(p->z_vel_d));
	param_get(h->z_vel_max, &(p->z_vel_max));
	param_get(h->xy_p, &(p->xy_p));
	param_get(h->xy_d, &(p->xy_d));
	param_get(h->xy_vel_p, &(p->xy_vel_p));
	param_get(h->xy_vel_i, &(p->xy_vel_i));
	param_get(h->xy_vel_d, &(p->xy_vel_d));
	param_get(h->xy_vel_max, &(p->xy_vel_max));
	param_get(h->tilt_max, &(p->tilt_max));

	param_get(h->rc_scale_pitch, &(p->rc_scale_pitch));
	param_get(h->rc_scale_roll, &(p->rc_scale_roll));
	param_get(h->rc_scale_yaw, &(p->rc_scale_yaw));

	return OK;
}
