/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskVtolTakeoffLoiterLand.hpp
 *
 * Flight task for VTOL takeoff, loiter, and land sequence.
 *
 */

#pragma once

#include "FlightTask.hpp"
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <lib/geo/geo.h>

enum class VtolTakeoffLoiterLandState {
	TAKEOFF = 0,
	TRANSITION_TO_FW = 1,
	LOITER_FW = 2,
	TRANSITION_TO_MC = 3,
	LOITER_MC = 4,
	LAND = 5,
	COMPLETE = 6
};

class FlightTaskVtolTakeoffLoiterLand : public FlightTask
{
public:
	FlightTaskVtolTakeoffLoiterLand() = default;
	virtual ~FlightTaskVtolTakeoffLoiterLand() = default;

	bool activate(const trajectory_setpoint_s &last_setpoint) override;
	bool updateInitialize() override;
	bool update() override;
	void reActivate() override;

protected:
	void _updateSetpoints();
	void _updateStateMachine();
	void _setTakeoffSetpoints();
	void _setLoiterSetpoints();
	void _setLandSetpoints();
	void _checkStateTransitions();

	DEFINE_PARAMETERS_CUSTOM_PARENT(FlightTask,
					(ParamFloat<px4::params::VT_FW_MIN_ALT>) _param_vt_fw_min_alt,
					(ParamFloat<px4::params::VT_TRANS_MIN_TM>) _param_vt_trans_min_tm,
					(ParamFloat<px4::params::VTO_LOITER_ALT>) _param_vto_loiter_alt,
					(ParamFloat<px4::params::VT_LOITER_RAD>) _param_vt_loiter_rad,
					(ParamFloat<px4::params::VT_LOITER_TIME>) _param_vt_loiter_time,
					(ParamFloat<px4::params::VT_LAND_ALT>) _param_vt_land_alt
				       )

private:
	VtolTakeoffLoiterLandState _state{VtolTakeoffLoiterLandState::TAKEOFF};
	hrt_abstime _state_start_time{0};
	hrt_abstime _loiter_start_time{0};

	matrix::Vector3f _takeoff_position{};
	matrix::Vector3f _loiter_center{};
	float _loiter_radius{50.0f};
	float _loiter_duration{60.0f}; // seconds

	uORB::SubscriptionData<vehicle_status_s> _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::SubscriptionData<position_setpoint_triplet_s> _position_setpoint_triplet_sub{ORB_ID(position_setpoint_triplet)};

	bool _is_vtol{false};
	bool _in_transition_mode{false};
};
