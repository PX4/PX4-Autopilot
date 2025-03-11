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
 * @file FlightTaskManualAccelerationSlow.hpp
 *
 * Flight task for manual position mode with knobs for slower velocity and acceleration.
 *
 */

#pragma once

#include "FlightTaskManualAcceleration.hpp"
#include "StickAccelerationXY.hpp"
#include "Gimbal.hpp"

#include <lib/weather_vane/WeatherVane.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/velocity_limits.h>
#include <systemlib/mavlink_log.h>

class FlightTaskManualAccelerationSlow : public FlightTaskManualAcceleration
{
public:
	FlightTaskManualAccelerationSlow() = default;
	~FlightTaskManualAccelerationSlow() = default;
	bool update() override;

private:

	/**
	 * Get the input from a sanitized parameter aux index
	 * @param parameter_value value of the parameter that specifies the AUX channel index to use
	 * @return input from that AUX channel [-1,1]
	 */
	float getInputFromSanitizedAuxParameterIndex(int parameter_value);

	/**
	 * Input shaping of unit length raw stick input for gimbal setpoint
	 * @param stick_input raw calibrated stick value [-1, 1]
	 * @param maximum_rate rate [rad/s] with maximum stick deflection
	 * @return gimbal rate setpoint [rad/s] positive clockwise
	 */
	float shapePitchStickToGimbalRate(float stick_input, float maximum_rate);
	float shapeYawStickToGimbalRate(float stick_input, float maximum_rate);

	float RC_Expo_Direct(int32_t c, float expo, int16_t window);

	bool _velocity_limits_received_before{false};

	// Gimbal control
	Gimbal _gimbal{this};

	uORB::Subscription _velocity_limits_sub{ORB_ID(velocity_limits)};
	velocity_limits_s _velocity_limits{};

	uORB::Subscription _takeoff_status_sub{ORB_ID(takeoff_status)};
	bool haveTakenOff();

	DEFINE_PARAMETERS_CUSTOM_PARENT(FlightTaskManualAcceleration,
					(ParamInt<px4::params::MC_SLOW_MAP_HVEL>) _param_mc_slow_map_hvel,
					(ParamInt<px4::params::MC_SLOW_MAP_VVEL>) _param_mc_slow_map_vvel,
					(ParamInt<px4::params::MC_SLOW_MAP_YAWR>) _param_mc_slow_map_yawr,
					(ParamFloat<px4::params::MC_SLOW_MIN_HVEL>) _param_mc_slow_min_hvel,
					(ParamFloat<px4::params::MC_SLOW_MIN_VVEL>) _param_mc_slow_min_vvel,
					(ParamFloat<px4::params::MC_SLOW_MIN_YAWR>) _param_mc_slow_min_yawr,
					(ParamFloat<px4::params::MC_SLOW_DEF_HVEL>) _param_mc_slow_def_hvel,
					(ParamFloat<px4::params::MC_SLOW_DEF_VVEL>) _param_mc_slow_def_vvel,
					(ParamFloat<px4::params::MC_SLOW_DEF_YAWR>) _param_mc_slow_def_yawr,
					(ParamFloat<px4::params::MPC_VEL_MANUAL>) _param_mpc_vel_manual,
					(ParamFloat<px4::params::MPC_Z_VEL_MAX_UP>) _param_mpc_z_vel_max_up,
					(ParamFloat<px4::params::MPC_Z_VEL_MAX_DN>) _param_mpc_z_vel_max_dn,
					(ParamFloat<px4::params::MPC_MAN_Y_MAX>) _param_mpc_man_y_max,
					(ParamFloat<px4::params::MC_SLOW_EXP_PIT>) _param_mc_slow_expo_pitch,
					(ParamFloat<px4::params::MC_SLOW_EXPO_YAW>) _param_mc_slow_expo_yaw,
					(ParamInt<px4::params::MC_SLOW_WIN_PIT>) _param_mc_slow_window_pitch,
					(ParamInt<px4::params::MC_SLOW_WIN_YAW>) _param_mc_slow_window_yaw
				       )

	/**
	 * Check for parameter changes and update them if needed.
	 */
	void parameters_update();
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	float _prev_expo_pitch = 0.0f;
	int16_t _prev_window_pitch = 0;
	float _prev_expo_yaw = 0.0f;
	int16_t _prev_window_yaw = 0;
};
