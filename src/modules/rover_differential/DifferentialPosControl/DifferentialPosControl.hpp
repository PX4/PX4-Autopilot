/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#pragma once

// PX4 includes
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/events.h>

// Libraries
#include <matrix/matrix/math.hpp>
#include <lib/pure_pursuit/PurePursuit.hpp>
#include <math.h>

// uORB includes
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/rover_speed_setpoint.h>
#include <uORB/topics/rover_attitude_setpoint.h>
#include <uORB/topics/pure_pursuit_status.h>
#include <uORB/topics/rover_position_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>

using namespace matrix;

/**
 * @brief Enum class for the different states of driving.
 */
enum class DrivingState {
	SPOT_TURNING, // The vehicle is currently turning on the spot.
	DRIVING      // The vehicle is currently driving.
};

/**
 * @brief Class for differential position control.
 */
class DifferentialPosControl : public ModuleParams
{
public:
	/**
	 * @brief Constructor for DifferentialPosControl.
	 * @param parent The parent ModuleParams object.
	 */
	DifferentialPosControl(ModuleParams *parent);
	~DifferentialPosControl() = default;

	/**
	 * @brief Generate and publish roverSpeedSetpoint and roverAttitudeSetpoint from roverPositionSetpoint.
	 */
	void updatePosControl();

	/**
	 * @brief Check if the necessary parameters are set.
	 * @return True if all checks pass.
	 */
	bool runSanityChecks();

	/**
	 * @brief Reset position controller.
	 */
	void reset() {_start_ned = Vector2f{NAN, NAN}; _target_waypoint_ned = Vector2f{NAN, NAN}; _arrival_speed = 0.f; _cruising_speed = _param_ro_speed_limit.get(); _stopped = false;};

protected:
	/**
	 * @brief Update the parameters of the module.
	 */
	void updateParams() override;

private:
	/**
	 * @brief Update uORB subscriptions used in position controller.
	 */
	void updateSubscriptions();

	// uORB subscriptions
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _rover_position_setpoint_sub{ORB_ID(rover_position_setpoint)};

	// uORB publications
	uORB::Publication<rover_speed_setpoint_s> _rover_speed_setpoint_pub{ORB_ID(rover_speed_setpoint)};
	uORB::Publication<pure_pursuit_status_s>     _pure_pursuit_status_pub{ORB_ID(pure_pursuit_status)};
	uORB::Publication<rover_attitude_setpoint_s> _rover_attitude_setpoint_pub{ORB_ID(rover_attitude_setpoint)};

	// Variables
	Quatf _vehicle_attitude_quaternion{};
	Vector2f _curr_pos_ned{};
	Vector2f _start_ned{};
	Vector2f _target_waypoint_ned{};
	float _arrival_speed{0.f};
	float _vehicle_yaw{0.f};
	float _vehicle_speed{0.f};
	float _cruising_speed{NAN};
	bool _stopped{false};
	uint8_t _reset_counter{0}; /**< counter for estimator resets in xy-direction */
	uint8_t _updated_reset_counter{0}; /**< counter for estimator resets in xy-direction */
	DrivingState _current_state{DrivingState::DRIVING};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::RD_TRANS_TRN_DRV>) _param_rd_trans_trn_drv,
		(ParamFloat<px4::params::RD_TRANS_DRV_TRN>) _param_rd_trans_drv_trn,
		(ParamFloat<px4::params::RO_MAX_THR_SPEED>) _param_ro_max_thr_speed,
		(ParamFloat<px4::params::RO_DECEL_LIM>)     _param_ro_decel_limit,
		(ParamFloat<px4::params::RO_JERK_LIM>)      _param_ro_jerk_limit,
		(ParamFloat<px4::params::RO_SPEED_LIM>)     _param_ro_speed_limit,
		(ParamFloat<px4::params::PP_LOOKAHD_GAIN>)  _param_pp_lookahd_gain,
		(ParamFloat<px4::params::PP_LOOKAHD_MAX>)   _param_pp_lookahd_max,
		(ParamFloat<px4::params::PP_LOOKAHD_MIN>)   _param_pp_lookahd_min,
		(ParamFloat<px4::params::NAV_ACC_RAD>)      _param_nav_acc_rad,
		(ParamFloat<px4::params::RO_SPEED_RED>)     _param_ro_speed_red,
		(ParamFloat<px4::params::RO_SPEED_TH>)      _param_ro_speed_th
	)
};
