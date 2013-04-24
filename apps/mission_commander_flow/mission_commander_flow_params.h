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
 * @file mission_commander_flow_params.h
 * 
 * Parameters for mission commander
 */

#ifndef MISSION_COMMANDER_FLOW_PARAMS_H_
#define MISSION_COMMANDER_FLOW_PARAMS_H_

#include <systemlib/param/param.h>

struct mission_commander_flow_params {
	float mission_x_offset; // in m
	float mission_y_offset; // in m
	float mission_update_step_x; // in m
	float mission_update_step_y; // in m
	float mission_update_step_yaw; // in rad
	float mission_yaw_thld; // in rad
	float mission_wp_radius; // in m
	int mission_min_front_dist; // in mm
	int mission_min_front_side_dist; // in mm
	int mission_min_side_dist; // in mm
	int mission_react_front_dist; // in mm
	int mission_react_front_side_dist; // in mm
	int mission_react_side_dist; // in mm
	float reaction_min_react_angle;
	float reaction_min_overreact_angle;
	float reaction_min_pass_distance;
	float reaction_min_free_distance;
	int counter_react_angle;
	int counter_overreact_angle;
	int counter_pass_distance;
	int counter_free_distance;
	float radarControlSettings[9];
	int debug; // boolean if mission planning manually
};

struct mission_commander_flow_param_handles {
	param_t mission_x_offset;
	param_t mission_y_offset;
	param_t mission_update_step_x;
	param_t mission_update_step_y;
	param_t mission_update_step_yaw;
	param_t mission_yaw_thld;
	param_t mission_wp_radius;
	param_t mission_min_front_dist;
	param_t mission_min_front_side_dist;
	param_t mission_min_side_dist;
	param_t mission_react_front_dist;
	param_t mission_react_front_side_dist;
	param_t mission_react_side_dist;
	param_t reaction_min_react_angle;
	param_t reaction_min_overreact_angle;
	param_t reaction_min_pass_distance;
	param_t reaction_min_free_distance;
	param_t debug;
};

/**
 * Initialize all parameter handles and values
 *
 */
int parameters_init(struct mission_commander_flow_param_handles *h);

/**
 * Update all parameters
 *
 */
int parameters_update(const struct mission_commander_flow_param_handles *h, struct mission_commander_flow_params *p);

#endif /* MISSION_COMMANDER_FLOW_PARAMS_H_ */
