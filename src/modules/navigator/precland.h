/***************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
/**
 * @file precland.h
 *
 * Helper class to do precision landing with a landing target
 *
 * @author Nicolas de Palezieux (Sunflower Labs) <ndepal@gmail.com>
 */

#ifndef NAVIGATOR_PRECLAND_H
#define NAVIGATOR_PRECLAND_H

#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>
#include <uORB/topics/landing_target_pose.h>
#include <geo/geo.h>

#include "navigator_mode.h"
#include "mission_block.h"

enum class PrecLandState {
	Start, // Starting state
	HorizontalApproach, // Positioning over landing target while maintaining altitude
	DescendAboveTarget, // Stay over landing target while descending
	FinalApproach, // Final landing approach, even without landing target
	Search, // Search for landing target
	Fallback, // Fallback landing method
	Done // Done landing
};

enum class PrecLandMode {
	Opportunistic = 1, // only do precision landing if landing target visible at the beginning
	Required = 2 // try to find landing target if not visible at the beginning
};

class PrecLand : public MissionBlock
{
public:
	PrecLand(Navigator *navigator, const char *name);

	~PrecLand();

	virtual void on_inactive();

	virtual void on_activation();

	virtual void on_active();

	void set_mode(PrecLandMode mode) { _mode = mode; };

	PrecLandMode get_mode() { return _mode; };

private:
	// run the control loop for each state
	void run_state_start();
	void run_state_horizontal_approach();
	void run_state_descend_above_target();
	void run_state_final_approach();
	void run_state_search();
	void run_state_fallback();

	// attempt to switch to a different state. Returns true if state change was successful, false otherwise
	bool switch_to_state_start();
	bool switch_to_state_horizontal_approach();
	bool switch_to_state_descend_above_target();
	bool switch_to_state_final_approach();
	bool switch_to_state_search();
	bool switch_to_state_fallback();
	bool switch_to_state_done();

	// check if a given state could be changed into. Return true if possible to transition to state, false otherwise
	bool check_state_conditions(PrecLandState state);
	void slewrate(float &sp_x, float &sp_y);

	landing_target_pose_s _target_pose{}; /**< precision landing target position */
	int _targetPoseSub;
	bool _target_pose_valid; /**< wether we have received a landing target position message */
	struct map_projection_reference_s _map_ref {}; /**< reference for local/global projections */
	uint64_t _state_start_time; /**< time when we entered current state */
	uint64_t _last_slewrate_time; /**< time when we last limited setpoint changes */
	uint64_t _target_acquired_time; /**< time when we first saw the landing target during search */
	uint64_t _point_reached_time; /**< time when we reached a setpoint */
	int _search_cnt; /**< counter of how many times we had to search for the landing target */
	float _approach_alt; /**< altitude at which to stay during horizontal approach */

	matrix::Vector2f _sp_pev;
	matrix::Vector2f _sp_pev_prev;

	PrecLandState _state;

	PrecLandMode _mode;

	control::BlockParamFloat _param_timeout;
	control::BlockParamFloat _param_hacc_rad;
	control::BlockParamFloat _param_final_approach_alt;
	control::BlockParamFloat _param_search_alt;
	control::BlockParamFloat _param_search_timeout;
	control::BlockParamInt   _param_max_searches;
	control::BlockParamFloat _param_acceleration_hor;
	control::BlockParamFloat _param_xy_vel_cruise;

};

#endif
