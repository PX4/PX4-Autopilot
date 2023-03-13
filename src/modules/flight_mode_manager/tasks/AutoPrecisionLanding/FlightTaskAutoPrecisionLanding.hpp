/****************************************************************************
 *
 *   Copyright (c) 2017-2023 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskAutoPrecisionLanding.hpp
 *
 * Flight task for better precision landing
 *
 * @author Nicolas de Palezieux (Sunflower Labs) <ndepal@gmail.com>
 */

#pragma once

#include "FlightTaskAuto.hpp"

#include <matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/landing_target_pose.h>
#include <uORB/topics/precision_landing_status.h>

#define SEC2USEC 1000000.0f		// TODO: Get the correct define from some header
#define STATE_TIMEOUT 10000000 		// [us] Maximum time to spend in any state
#define ACCEPTANCE_RADIUS 0.20f		// Horizontal acceptance radius for the navigation to the landing target
// TODO: Get ACCEPTANCE_RADIUS from NAV_ACC_RAD

enum class PrecLandState {
	AutoRTL, // Starting state
	Search, // Search for landing target
	MoveAboveTarget, // Positioning over landing target while maintaining altitude
	DescendAboveTarget, // Stay over landing target while descending
	TouchingDown, // Final landing approach, even without landing target
	Fallback // Fallback landing method
};

enum class PrecLandMode {
	Opportunistic = 1, // only do precision landing if landing target visible at the beginning
	Required = 2 // try to find landing target if not visible at the beginning
};

class FlightTaskAutoPrecisionLanding : public FlightTaskAuto
{
public:
	FlightTaskAutoPrecisionLanding() = default;
	virtual ~FlightTaskAutoPrecisionLanding() = default;

	bool activate(const trajectory_setpoint_s &last_setpoint) override;

	bool update() override;

private:
	// run the control loop for each state
	void run_state_auto_rtl();
	void run_state_search();
	void run_state_move_above_target();
	void run_state_descend_above_target();
	void run_state_touching_down();
	void run_state_fallback();

	// attempt to switch to a different state. Returns true if state change was successful, false otherwise
	bool try_switch_to_state_auto_rtl();
	void try_switch_to_state_search();
	bool try_switch_to_state_move_above_target();
	bool try_switch_to_state_descend_above_target();
	bool try_switch_to_state_touching_down();
	void try_switch_to_state_fallback();

	void print_state_switch_message(const char *state_name);

	// check if a given state could be changed into. Return true if possible to transition to state, false otherwise
	bool check_state_conditions(PrecLandState state);
	void slewrate(float &sp_x, float &sp_y);

	bool hor_acc_radius_check();

	landing_target_pose_s _landing_target_pose{}; /**< precision landing target position */

	uORB::Subscription _landing_target_pose_sub{ORB_ID(landing_target_pose)};
	uORB::PublicationMulti<precision_landing_status_s> _precision_landing_status_pub{ORB_ID(precision_landing_status)};

	bool _landing_target_pose_valid{false}; /**< whether we have received a landing target position message */

	uint64_t _state_start_time{0}; /**< time when we entered current state */
	uint64_t _last_slewrate_time{0}; /**< time when we last limited setpoint changes */
	uint64_t _target_acquired_time{0}; /**< time when we first saw the landing target during search */
	uint64_t _point_reached_time{0}; /**< time when we reached a setpoint */

	int _search_cnt{0}; /**< counter of how many times we had to search for the landing target */

	matrix::Vector2f _sp_pev;
	matrix::Vector2f _sp_pev_prev;

	PrecLandState _state{PrecLandState::AutoRTL};

	DEFINE_PARAMETERS_CUSTOM_PARENT(FlightTask,
					(ParamFloat<px4::params::MPC_LAND_SPEED>) _param_mpc_land_speed, ///< velocity for controlled descend
					(ParamFloat<px4::params::MPC_ACC_HOR>) _param_acceleration_hor,
					(ParamFloat<px4::params::MPC_XY_CRUISE>) _param_xy_vel_cruise,
					(ParamFloat<px4::params::PLD_BTOUT>) _param_pld_btout,
					(ParamFloat<px4::params::PLD_HACC_RAD>) _param_pld_hacc_rad,
					(ParamFloat<px4::params::PLD_FAPPR_ALT>) _param_pld_fappr_alt,
					(ParamFloat<px4::params::PLD_SRCH_ALT>) _param_pld_srch_alt,
					(ParamFloat<px4::params::PLD_SRCH_TOUT>) _param_pld_srch_tout,
					(ParamInt<px4::params::PLD_MAX_SRCH>) _param_pld_max_srch,
					(ParamInt<px4::params::RTL_PLD_MD>) _param_rtl_pld_md
				       )

};
