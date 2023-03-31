/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 */

#pragma once

#include "FlightTaskAuto.hpp"
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/landing_target_pose.h>
#include <uORB/topics/precision_landing_status.h>
// #include <uORB/topics/vehicle_local_position_setpoint.h>
#include <systemlib/mavlink_log.h>
#include <uORB/topics/follow_target_estimator.h>
// #include <lib/ecl/geo/geo.h>
#include <mathlib/mathlib.h>

// TODO: Get the correct define from some header
#define SEC2USEC 1000000.0f
#define HOVER_SPEED_THRESHOLD 0.20f	// If drone has speed under this value it is considered to be hovering in place

class FlightTaskAutoPrecisionLanding : public FlightTask
{
public:
	FlightTaskAutoPrecisionLanding() = default;
	virtual ~FlightTaskAutoPrecisionLanding() = default;

	bool activate(const trajectory_setpoint_s &last_setpoint) override;

	bool update() override;

private:
	// TODO: Use ModuleParams
	DEFINE_PARAMETERS_CUSTOM_PARENT(FlightTask,
					(ParamFloat<px4::params::MPC_LAND_SPEED>) _param_mpc_land_speed, ///< velocity for controlled descend
					(ParamFloat<px4::params::PLD_SRCH_ALT>) _param_pld_srch_alt,
					(ParamFloat<px4::params::PLD_SRCH_TOUT>) _param_pld_srch_tout,
					(ParamInt<px4::params::PLD_MAX_SRCH>) _param_pld_max_srch,
					(ParamFloat<px4::params::PLD_HACC_RAD>) _param_pld_hacc_rad,
					(ParamFloat<px4::params::PLD_BTOUT>) _param_pld_btout,
					(ParamFloat<px4::params::RTL_RETURN_ALT>) _param_rtl_return_alt,
					(ParamFloat<px4::params::NAV_MC_ALT_RAD>) _param_nav_mc_alt_rad
				       )

	enum PRECLAND_STATE {
		AUTORTL_CLIMB,
		AUTORTL_APPROACH,
		MOVE_TO_SEARCH_ALTITUDE,
		SEARCHING_TARGET,
		MOVING_ABOVE_TARGET,
		LANDING
	};

	void do_state_transition(PRECLAND_STATE new_state);

	bool inside_acceptance_radius();

	bool precision_target_available();

	PRECLAND_STATE _precland_state;

	uORB::Subscription _landing_target_pose_sub{ORB_ID(landing_target_pose)};
	landing_target_pose_s _landing_target_pose{}; /**< precision landing target position */

	uORB::PublicationMulti<precision_landing_status_s> _precision_landing_status_pub{ORB_ID(precision_landing_status)};

	uint64_t _state_start_time{0}; /**< time when entering search state */
	int _search_count = 0;
	float _initial_yaw;
	float _target_yaw;
	Vector3f _initial_position;

	orb_advert_t	_mavlink_log_pub{nullptr};
	matrix::Vector3f _precision_target_ned = {NAN, NAN, NAN};

};
