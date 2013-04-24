/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: Tobias Naegeli <naegelit@student.ethz.ch>
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
 * @file mission_commander_flow_params.c
 * 
 * Parameters for mission commander
 */

#include "mission_commander_flow_params.h"

/* commander parameters */
PARAM_DEFINE_FLOAT(MCF_MISS_XOFF, 3.0f);
PARAM_DEFINE_FLOAT(MCF_MISS_YOFF, 0.0f);
PARAM_DEFINE_FLOAT(MCF_MISS_STEP_X, 0.003f);
PARAM_DEFINE_FLOAT(MCF_MISS_STEP_Y, 0.001f);
PARAM_DEFINE_FLOAT(MCF_MISS_S_YAW, 0.004f);
PARAM_DEFINE_FLOAT(MCF_MISS_YAW_TH, 0.1f);
PARAM_DEFINE_FLOAT(MCF_MISS_WP_R, 0.4f);
PARAM_DEFINE_INT32(MCF_MIN_FDIST, 1000);
PARAM_DEFINE_INT32(MCF_MIN_FSDIST, 1500);
PARAM_DEFINE_INT32(MCF_MIN_SDIST, 1000);
PARAM_DEFINE_INT32(MCF_REAC_FDIST, 2000);
PARAM_DEFINE_INT32(MCF_REAC_FSDIST, 2000);
PARAM_DEFINE_INT32(MCF_REAC_SDIST, 1000);
PARAM_DEFINE_FLOAT(MCF_REAC_ANG, 0.26f);
PARAM_DEFINE_FLOAT(MCF_REAC_O_ANG, 0.09f);
PARAM_DEFINE_FLOAT(MCF_REAC_PASS, 1.0f);
PARAM_DEFINE_FLOAT(MCF_REAC_FREE, 0.5f);
PARAM_DEFINE_INT32(MCF_DEBUG, 0);


int parameters_init(struct mission_commander_flow_param_handles *h)
{
	/* PID parameters */
	h->mission_x_offset					=	param_find("MCF_MISS_XOFF");
	h->mission_y_offset					=	param_find("MCF_MISS_YOFF");
	h->mission_update_step_x			=	param_find("MCF_MISS_STEP_X");
	h->mission_update_step_y			=	param_find("MCF_MISS_STEP_Y");
	h->mission_update_step_yaw			=	param_find("MCF_MISS_S_YAW");
	h->mission_yaw_thld					=	param_find("MCF_MISS_YAW_TH");
	h->mission_wp_radius				=	param_find("MCF_MISS_WP_R");
	h->mission_min_front_dist			=	param_find("MCF_MIN_FDIST");
	h->mission_min_front_side_dist		=	param_find("MCF_MIN_FSDIST");
	h->mission_min_side_dist			=	param_find("MCF_MIN_SDIST");
	h->mission_react_front_dist			=	param_find("MCF_REAC_FDIST");
	h->mission_react_front_side_dist	=	param_find("MCF_REAC_FSDIST");
	h->mission_react_side_dist			=	param_find("MCF_REAC_SDIST");
	h->reaction_min_react_angle			=	param_find("MCF_REAC_ANG");
	h->reaction_min_overreact_angle 	=	param_find("MCF_REAC_O_ANG");
	h->reaction_min_pass_distance		=	param_find("MCF_REAC_PASS");
	h->reaction_min_free_distance		=	param_find("MCF_REAC_FREE");
	h->debug							=	param_find("MCF_DEBUG");

	return OK;
}

int parameters_update(const struct mission_commander_flow_param_handles *h, struct mission_commander_flow_params *p)
{
	param_get(h->mission_x_offset, &(p->mission_x_offset));
	param_get(h->mission_y_offset, &(p->mission_y_offset));
	param_get(h->mission_update_step_x, &(p->mission_update_step_x));
	param_get(h->mission_update_step_y, &(p->mission_update_step_y));
	param_get(h->mission_update_step_yaw, &(p->mission_update_step_yaw));
	param_get(h->mission_yaw_thld, &(p->mission_yaw_thld));
	param_get(h->mission_wp_radius, &(p->mission_wp_radius));
	param_get(h->mission_min_front_dist, &(p->mission_min_front_dist));
	param_get(h->mission_min_front_side_dist, &(p->mission_min_front_side_dist));
	param_get(h->mission_min_side_dist, &(p->mission_min_side_dist));
	param_get(h->mission_react_front_dist, &(p->mission_react_front_dist));
	param_get(h->mission_react_front_side_dist, &(p->mission_react_front_side_dist));
	param_get(h->mission_react_side_dist, &(p->mission_react_side_dist));
	param_get(h->reaction_min_react_angle, &(p->reaction_min_react_angle));
	param_get(h->reaction_min_overreact_angle, &(p->reaction_min_overreact_angle));
	param_get(h->reaction_min_pass_distance, &(p->reaction_min_pass_distance));
	param_get(h->reaction_min_free_distance, &(p->reaction_min_free_distance));
	param_get(h->debug, &(p->debug));

	/* calc counters from other parameters */
	p->counter_react_angle = (int)(p->reaction_min_react_angle / p->mission_update_step_yaw);
	p->counter_overreact_angle = (int)(p->reaction_min_overreact_angle / p->mission_update_step_yaw);
	p->counter_pass_distance = (int)(p->reaction_min_pass_distance / p->mission_update_step_x);
	p->counter_free_distance = (int)(p->reaction_min_free_distance / p->mission_update_step_x);

	/* fill radar control settings */
	p->radarControlSettings[0] = p->mission_update_step_x; // max x-step
	p->radarControlSettings[1] = p->mission_update_step_y; // TODO make a seperate y step
	p->radarControlSettings[2] = p->mission_update_step_yaw; // max yaw-step
	p->radarControlSettings[3] = (float) p->mission_react_side_dist; // react side distance
	p->radarControlSettings[4] = (float) p->mission_react_front_side_dist; // react front side distance
	p->radarControlSettings[5] = (float) p->mission_react_front_dist; // react front side distance
	p->radarControlSettings[6] = (float) p->mission_min_side_dist; // min side distance
	p->radarControlSettings[7] = (float) p->mission_min_front_side_dist; // min front side distance
	p->radarControlSettings[8] = (float) p->mission_min_front_dist; // min front distance

	return OK;
}
