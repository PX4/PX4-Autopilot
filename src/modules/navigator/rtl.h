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
#include <dataman_client/DatamanClient.hpp>
#include "rtl_direct.h"
#include "rtl_direct_mission_land.h"
#include "rtl_mission_fast.h"
#include "rtl_mission_fast_reverse.h"

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/home_position.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rtl_time_estimate.h>

class Navigator;

class RTL : public NavigatorMode, public ModuleParams
{
public:
	RTL(Navigator *navigator);

	~RTL() = default;

	enum class RtlType {
		RTL_DIRECT,
		RTL_DIRECT_MISSION_LAND,
		RTL_MISSION_FAST,
		RTL_MISSION_FAST_REVERSE,
	};

	void on_inactivation() override;
	void on_inactive() override;
	void on_activation() override;
	void on_active() override;

	void initialize() override {};

	void set_return_alt_min(bool min) { _enforce_rtl_alt = min; }

	void updateSafePoints() { _initiate_safe_points_updated = true; }

private:
	bool hasMissionLandStart();

	/**
	 * @brief function to call regularly to do background work
	 */
	void updateDatamanCache();

	void setRtlTypeAndDestination();

	/**
	 * @brief Find RTL destination.
	 *
	 */
	void findRtlDestination(bool &isMissionLanding, RtlDirect::RtlPosition &rtl_position, float &rtl_alt);

	/**
	 * @brief Set the position of the land start marker in the planned mission as destination.
	 *
	 */
	void setLandPosAsDestination(RtlDirect::RtlPosition &rtl_position, mission_item_s &land_mission_item);

	/**
	 * @brief Set the safepoint as destination.
	 *
	 * @param mission_safe_point is the mission safe point/rally point to set as destination.
	 */
	void setSafepointAsDestination(RtlDirect::RtlPosition &rtl_position, const mission_safe_point_s &mission_safe_point);

	/**
	 * @brief
	 *
	 * @param cone_half_angle_deg
	 * @return float
	 */
	float calculate_return_alt_from_cone_half_angle(const RtlDirect::RtlPosition &rtl_position, float cone_half_angle_deg);

	void parameters_update();

	enum class DatamanState {
		UpdateRequestWait,
		Read,
		ReadWait,
		Load,
		Error
	};

	hrt_abstime _destination_check_time{0};

	RtlType _rtl_type{RtlType::RTL_DIRECT};

	DatamanState _dataman_state{DatamanState::UpdateRequestWait};
	DatamanState _error_state{DatamanState::UpdateRequestWait};
	uint16_t _update_counter{0}; ///< dataman update counter: if it does not match, safe points data was updated
	bool _safe_points_updated{false}; ///< flag indicating if safe points are updated to dataman cache
	DatamanCache _dataman_cache_geofence{"rtl_dm_cache_miss_geo", 4};
	DatamanClient	&_dataman_client_geofence = _dataman_cache_geofence.client();
	bool _initiate_safe_points_updated{true}; ///< flag indicating if safe points update is needed
	DatamanCache _dataman_cache_landItem{"rtl_dm_cache_miss_land", 2};
	int16_t _mission_counter = -1;

	mission_stats_entry_s _stats;

	RtlDirect _rtl_direct;

	RtlDirectMissionLand _rtl_direct_mission_land;

	RtlMissionFast _rtl_mission;

	RtlMissionFastReverse _rtl_mission_reverse;

	bool _enforce_rtl_alt{false};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::RTL_TYPE>)          _param_rtl_type,
		(ParamInt<px4::params::RTL_CONE_ANG>)      _param_rtl_cone_half_angle_deg,
		(ParamFloat<px4::params::RTL_RETURN_ALT>)  _param_rtl_return_alt,
		(ParamFloat<px4::params::RTL_MIN_DIST>)    _param_rtl_min_dist,
		(ParamFloat<px4::params::NAV_ACC_RAD>)      _param_nav_acc_rad
	)

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::SubscriptionData<vehicle_global_position_s> _global_pos_sub{ORB_ID(vehicle_global_position)};	/**< global position subscription */
	uORB::SubscriptionData<vehicle_status_s> _vehicle_status_sub{ORB_ID(vehicle_status)};	/**< vehicle status subscription */
	uORB::SubscriptionData<mission_s> _mission_sub{ORB_ID(mission)};
	uORB::SubscriptionData<home_position_s> _home_pos_sub{ORB_ID(home_position)};

	uORB::Publication<rtl_time_estimate_s> _rtl_time_estimate_pub{ORB_ID(rtl_time_estimate)};
};
