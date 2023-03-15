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

enum class PrecLandState {
	Idle,
	Start,
	HorizontalApproach,
	DescendAboveTarget,
	Search,
	NormalLand,
	Finished,
};

enum class PrecLandMode {
	RegularPrecisionLand,
	RegularLandOpportunistic,
	ReturnToLaunchOpportunistic,
	ReturnToLaunchRequired,
	MissionPrecisionLand
};

class FlightTaskAutoPrecisionLanding : public FlightTask
{
public:
	FlightTaskAutoPrecisionLanding() = default;
	virtual ~FlightTaskAutoPrecisionLanding() = default;

	bool activate(const trajectory_setpoint_s &last_setpoint) override;

	bool update() override;

	bool updateInitialize() override;

private:

	// Jake:
	void run_state_idle();
	void run_state_start();
	void run_state_horizontal_approach();
	void run_state_descend_above_target();
	void run_state_search();
	void run_state_normal_land();
	void run_state_finished();

	void switch_state(PrecLandState state);
	void state_on_enter(PrecLandState state);
	void state_on_exit(PrecLandState state);

	void slewrate(float &sp_x, float &sp_y);
	bool hor_acc_radius_check();

	uORB::SubscriptionData<home_position_s>			_sub_home_position {ORB_ID(home_position)};
	uORB::SubscriptionData<vehicle_status_s>		_sub_vehicle_status {ORB_ID(vehicle_status)};
	uORB::SubscriptionData<landing_target_pose_s>	_landing_target_pose_sub {ORB_ID(landing_target_pose)};

	uORB::PublicationMulti<precision_landing_status_s> _precision_landing_status_pub {ORB_ID(precision_landing_status)};

	landing_target_pose_s _landing_target_pose {};

	float _horizontal_approach_alt {};

	// FIX THIS
	// INFO  [FlightTaskAutoPrecisionLanding] Switching to Start
	// INFO  [FlightTaskAutoPrecisionLanding] FlightTaskAutoPrecisionLanding::activate
	// INFO  [FlightTaskAutoPrecisionLanding] Switching to Start
	// INFO  [FlightTaskAutoPrecisionLanding] FlightTaskAutoPrecisionLanding::activate
	// INFO  [FlightTaskAutoPrecisionLanding] Switching to Start
	// INFO  [FlightTaskAutoPrecisionLanding] FlightTaskAutoPrecisionLanding::activate
	// INFO  [FlightTaskAutoPrecisionLanding] Switching to Start
	bool _fix_this_activate_update_loop {true};

	uint8_t _nav_state {0};

	bool _landing_target_valid {false}; /**< whether we have received a landing target position message */

	hrt_abstime _search_start_time {0};

	uint64_t _last_slewrate_time {0}; /**< time when we last limited setpoint changes */
	int _search_count {0};

	matrix::Vector2f _sp_pev;
	matrix::Vector2f _sp_pev_prev;

	PrecLandMode  _mode  {PrecLandMode::RegularPrecisionLand};
	PrecLandState _state {PrecLandState::Idle};

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
