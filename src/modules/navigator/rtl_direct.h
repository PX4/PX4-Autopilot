/***************************************************************************
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
 * @file rtl_direct.h
 *
 * Helper class for RTL
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <matrix/Vector2.hpp>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/home_position.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rtl_time_estimate.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/wind.h>

#include "mission_block.h"
#include "navigation.h"
#include "safe_point_land.hpp"

using namespace time_literals;

class Navigator;

class RtlDirect : public MissionBlock, public ModuleParams
{
public:
	RtlDirect(Navigator *navigator);

	~RtlDirect() = default;

	/**
	 * @brief on inactivation
	 *
	 */
	void on_inactivation() override;

	/**
	 * @brief on activation.
	 * Initialize the return to launch calculations.
	 *
	 */
	void on_activation() override;

	/**
	 * @brief on active
	 * Update the return to launch calculation and set new setpoints for controller if necessary.
	 *
	 */
	void on_active() override;

	/**
	 * @brief Calculate the estimated time needed to return to launch.
	 *
	 * @return estimated time to return to launch.
	 */
	rtl_time_estimate_s calc_rtl_time_estimate();

	void setReturnAltMin(bool min) { _enforce_rtl_alt = min; }
	void setRtlAlt(float alt) {_rtl_alt = alt;};

	void setRtlPosition(PositionYawSetpoint position, loiter_point_s loiter_pos);

private:
	/**
	 * @brief Return to launch state machine.
	 *
	 */
	enum class RTLState {
		CLIMBING,
		MOVE_TO_LOITER,
		LOITER_DOWN,
		LOITER_HOLD,
		MOVE_TO_LAND,
		TRANSITION_TO_MC,
		MOVE_TO_LAND_HOVER,
		LAND,
		IDLE
	} _rtl_state{RTLState::IDLE}; /*< Current state in the state machine.*/

private:
	/**
	 * @brief Get the horizontal wind velocity
	 *
	 * @return horizontal wind velocity.
	 */
	matrix::Vector2f get_wind();

	/**
	 * @brief Set the return to launch control setpoint.
	 *
	 */
	void set_rtl_item();

	/**
	 * @brief Get the Cruise Ground Speed
	 *
	 * @return Ground speed in cruise mode [m/s].
	 */
	float getCruiseGroundSpeed();

	/**
	 * @brief Get the climb rate
	 *
	 * @return Climb rate [m/s]
	 */
	float getClimbRate();

	/**
	 * @brief Get the descend rate
	 *
	 * @return descend rate [m/s]
	 */
	float getDescendRate();

	/**
	 * @brief Get the cruise speed
	 *
	 * @return cruise speed [m/s]
	 */
	float getCruiseSpeed();

	/**
	 * @brief Get the Hover Land Speed
	 *
	 * @return Hover land speed [m/s]
	 */
	float getHoverLandSpeed();

	/**
	 * Check for parameter changes and update them if needed.
	 */
	void parameters_update();

	RTLState getActivationLandState();

	void setLoiterPosition();

	bool _enforce_rtl_alt{false};
	bool _force_heading{false};

	PositionYawSetpoint _destination; ///< the RTL position to fly to
	loiter_point_s _land_approach;

	float _rtl_alt{0.0f};	///< AMSL altitude at which the vehicle should return to the home position

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::RTL_DESCEND_ALT>) _param_rtl_descend_alt,
		(ParamFloat<px4::params::RTL_LAND_DELAY>)  _param_rtl_land_delay,
		(ParamFloat<px4::params::RTL_MIN_DIST>)    _param_rtl_min_dist,
		(ParamInt<px4::params::RTL_PLD_MD>)        _param_rtl_pld_md,
		(ParamFloat<px4::params::RTL_LOITER_RAD>)  _param_rtl_loiter_rad,
		(ParamFloat<px4::params::RTL_TIME_FACTOR>) _param_rtl_time_factor,
		(ParamInt<px4::params::RTL_TIME_MARGIN>)   _param_rtl_time_margin,

		// external params
		(ParamBool<px4::params::WV_EN>) _param_wv_en
	)

	param_t		_param_mpc_z_v_auto_up{PARAM_INVALID};
	param_t		_param_mpc_z_v_auto_dn{PARAM_INVALID};
	param_t		_param_mpc_land_speed{PARAM_INVALID};
	param_t		_param_fw_climb_rate{PARAM_INVALID};
	param_t		_param_fw_sink_rate{PARAM_INVALID};

	param_t 	_param_fw_airspeed_trim{PARAM_INVALID};
	param_t 	_param_mpc_xy_cruise{PARAM_INVALID};
	param_t 	_param_rover_cruise_speed{PARAM_INVALID};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::SubscriptionData<vehicle_global_position_s> _global_pos_sub{ORB_ID(vehicle_global_position)};	/**< global position subscription */
	uORB::SubscriptionData<home_position_s> _home_pos_sub{ORB_ID(home_position)};		/**< home position subscription */
	uORB::SubscriptionData<vehicle_land_detected_s> _land_detected_sub{ORB_ID(vehicle_land_detected)};	/**< vehicle land detected subscription */
	uORB::SubscriptionData<vehicle_status_s> _vehicle_status_sub{ORB_ID(vehicle_status)};	/**< vehicle status subscription */
	uORB::SubscriptionData<wind_s>		_wind_sub{ORB_ID(wind)};
};
