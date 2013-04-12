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
PARAM_DEFINE_FLOAT(MCF_MISS_STEP, 0.005f);
PARAM_DEFINE_FLOAT(MCF_MISS_S_YAW, 0.025f);
PARAM_DEFINE_FLOAT(MCF_MISS_YAW_TH, 0.1f);
PARAM_DEFINE_FLOAT(MCF_MISS_WP_R, 0.4f);
PARAM_DEFINE_INT32(MCF_MIN_FDIST, 1000);
PARAM_DEFINE_INT32(MCF_MIN_SDIST, 500);
PARAM_DEFINE_INT32(MCF_REAC_FDIST, 2000);
PARAM_DEFINE_INT32(MCF_REAC_SDIST, 1000);
PARAM_DEFINE_INT32(MCF_MIN_REAC, 20);
PARAM_DEFINE_INT32(MCF_MIN_FREE, 10);
PARAM_DEFINE_INT32(MCF_DEBUG, 0);


int parameters_init(struct mission_commander_flow_param_handles *h)
{
	/* PID parameters */
	h->mission_x_offset				=	param_find("MCF_MISS_XOFF");
	h->mission_y_offset				=	param_find("MCF_MISS_YOFF");
	h->mission_update_step			=	param_find("MCF_MISS_STEP");
	h->mission_update_step_yaw		=	param_find("MCF_MISS_S_YAW");
	h->mission_yaw_thld				=	param_find("MCF_MISS_YAW_TH");
	h->mission_wp_radius			=	param_find("MCF_MISS_WP_R");
	h->mission_min_front_dist		=	param_find("MCF_MIN_FDIST");
	h->mission_min_side_dist		=	param_find("MCF_MIN_SDIST");
	h->mission_react_front_dist		=	param_find("MCF_REAC_FDIST");
	h->mission_react_side_dist		=	param_find("MCF_REAC_SDIST");
	h->mission_min_reaction_steps	=	param_find("MCF_MIN_REAC");
	h->mission_min_free_steps		=	param_find("MCF_MIN_FREE");
	h->debug						=	param_find("MCF_DEBUG");

	return OK;
}

int parameters_update(const struct mission_commander_flow_param_handles *h, struct mission_commander_flow_params *p)
{
	param_get(h->mission_x_offset, &(p->mission_x_offset));
	param_get(h->mission_y_offset, &(p->mission_y_offset));
	param_get(h->mission_update_step, &(p->mission_update_step));
	param_get(h->mission_update_step_yaw, &(p->mission_update_step_yaw));
	param_get(h->mission_yaw_thld, &(p->mission_yaw_thld));
	param_get(h->mission_wp_radius, &(p->mission_wp_radius));
	param_get(h->mission_min_front_dist, &(p->mission_min_front_dist));
	param_get(h->mission_min_side_dist, &(p->mission_min_side_dist));
	param_get(h->mission_react_front_dist, &(p->mission_react_front_dist));
	param_get(h->mission_react_side_dist, &(p->mission_react_side_dist));
	param_get(h->mission_min_reaction_steps, &(p->mission_min_reaction_steps));
	param_get(h->mission_min_free_steps, &(p->mission_min_free_steps));
	param_get(h->debug, &(p->debug));

	return OK;
}
