/***************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
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
 * @file rtl.h
 *
 * Helper class for RTL
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#pragma once

#include <px4_platform_common/module_params.h>

#include "navigator_mode.h"
#include "mission_block.h"

#include <uORB/Subscription.hpp>
#include <uORB/topics/home_position.h>
#include <uORB/topics/rtl_time_estimate.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/wind.h>
#include <matrix/math.hpp>
#include <lib/geo/geo.h>
#include <dataman_client/DatamanClient.hpp>

class Navigator;

class RTL : public MissionBlock, public ModuleParams
{
public:
	RTL(Navigator *navigator);

	~RTL() = default;

	enum RTLType {
		RTL_TYPE_HOME_OR_RALLY = 0,
		RTL_TYPE_MISSION_LANDING,
		RTL_TYPE_MISSION_LANDING_REVERSED,
		RTL_TYPE_CLOSEST,
	};

	enum RTLDestinationType {
		RTL_DESTINATION_HOME = 0,
		RTL_DESTINATION_MISSION_LANDING,
		RTL_DESTINATION_SAFE_POINT,
	};

	enum RTLHeadingMode {
		RTL_NAVIGATION_HEADING = 0,
		RTL_DESTINATION_HEADING,
		RTL_CURRENT_HEADING,
	};

	enum RTLState {
		RTL_STATE_NONE = 0,
		RTL_STATE_CLIMB,
		RTL_STATE_RETURN,
		RTL_STATE_DESCEND,
		RTL_STATE_LOITER,
		RTL_STATE_TRANSITION_TO_MC,
		RTL_MOVE_TO_LAND_HOVER_VTOL,
		RTL_STATE_LAND,
		RTL_STATE_LANDED,
		RTL_STATE_HEAD_TO_CENTER,
	};

	/**
	 * @brief function to call regularly to do background work
	 */
	void run();

	void on_inactivation() override;
	void on_inactive() override;
	void on_activation() override;
	void on_active() override;

	void find_RTL_destination();

	void set_return_alt_min(bool min) { _rtl_alt_min = min; }

	int get_rtl_type() const { return _param_rtl_type.get(); }

	void get_rtl_xy_z_speed(float &xy, float &z);

	matrix::Vector2f get_wind();

	RTLState getRTLState() { return _rtl_state; }

	bool getRTLDestinationTypeMission() { return _destination.type == RTLDestinationType::RTL_DESTINATION_MISSION_LANDING; }

	void resetRtlState() { _rtl_state = RTL_STATE_NONE; }

	void updateSafePoints() { _initiate_safe_points_updated = true; }

private:

	void set_rtl_item();

	void advance_rtl();

	float calculate_return_alt_from_cone_half_angle(float cone_half_angle_deg);
	void calcRtlTimeEstimate(const RTLState rtl_state, rtl_time_estimate_s &rtl_time_estimate);

	float getCruiseGroundSpeed();

	float getClimbRate();

	float getDescendRate();

	float getCruiseSpeed();

	float getHoverLandSpeed();

	RTLState _rtl_state{RTL_STATE_NONE};

	enum class DatamanState {
		UpdateRequestWait,
		Read,
		ReadWait,
		Load,
		Error
	};

	struct RTLPosition {
		double lat;
		double lon;
		float alt;
		float yaw;
		uint8_t safe_point_index; ///< 0 = home position, 1 = mission landing, >1 = safe landing points (rally points)
		RTLDestinationType type{RTL_DESTINATION_HOME};

		void set(const home_position_s &home_position)
		{
			lat = home_position.lat;
			lon = home_position.lon;
			alt = home_position.alt;
			yaw = home_position.yaw;
			safe_point_index = 0;
			type = RTL_DESTINATION_HOME;
		}
	};

	DatamanState _dataman_state{DatamanState::UpdateRequestWait};
	DatamanState _error_state{DatamanState::UpdateRequestWait};
	uint16_t _update_counter{0}; ///< dataman update counter: if it does not match, safe points data was updated
	bool _safe_points_updated{false}; ///< flag indicating if safe points are updated to dataman cache
	DatamanCache _dataman_cache{"rtl_dm_cache_miss", 4};
	DatamanClient	&_dataman_client = _dataman_cache.client();
	bool _initiate_safe_points_updated{true}; ///< flag indicating if safe points update is needed

	mission_stats_entry_s _stats;

	RTLPosition _destination{}; ///< the RTL position to fly to (typically the home position or a safe point)

	hrt_abstime _destination_check_time{0};

	float _rtl_alt{0.0f};	// AMSL altitude at which the vehicle should return to the home position

	bool _rtl_alt_min{false};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::RTL_RETURN_ALT>)  _param_rtl_return_alt,
		(ParamFloat<px4::params::RTL_DESCEND_ALT>) _param_rtl_descend_alt,
		(ParamFloat<px4::params::RTL_LAND_DELAY>)  _param_rtl_land_delay,
		(ParamFloat<px4::params::RTL_MIN_DIST>)    _param_rtl_min_dist,
		(ParamInt<px4::params::RTL_TYPE>)          _param_rtl_type,
		(ParamInt<px4::params::RTL_CONE_ANG>)      _param_rtl_cone_half_angle_deg,
		(ParamInt<px4::params::RTL_PLD_MD>)        _param_rtl_pld_md,
		(ParamFloat<px4::params::RTL_LOITER_RAD>)  _param_rtl_loiter_rad,
		(ParamInt<px4::params::RTL_HDG_MD>)        _param_rtl_hdg_md,
		(ParamFloat<px4::params::RTL_TIME_FACTOR>) _param_rtl_time_factor,
		(ParamInt<px4::params::RTL_TIME_MARGIN>)   _param_rtl_time_margin
	)

	param_t		_param_mpc_z_v_auto_up{PARAM_INVALID};
	param_t		_param_mpc_z_v_auto_dn{PARAM_INVALID};
	param_t		_param_mpc_land_speed{PARAM_INVALID};
	param_t		_param_fw_climb_rate{PARAM_INVALID};
	param_t		_param_fw_sink_rate{PARAM_INVALID};

	param_t 	_param_fw_airspeed_trim{PARAM_INVALID};
	param_t 	_param_mpc_xy_cruise{PARAM_INVALID};
	param_t 	_param_rover_cruise_speed{PARAM_INVALID};

	uORB::SubscriptionData<wind_s>		_wind_sub{ORB_ID(wind)};
	uORB::Publication<rtl_time_estimate_s> _rtl_time_estimate_pub{ORB_ID(rtl_time_estimate)};
};

float time_to_home(const matrix::Vector3f &to_home_vec,
		   const matrix::Vector2f &wind_velocity, float vehicle_speed_m_s,
		   float vehicle_descent_speed_m_s);
