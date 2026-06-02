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
#include "navigation.h"
#include "rtl_base.h"
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
#include <uORB/topics/rtl_status.h>
#include <uORB/topics/rtl_time_estimate.h>

class Navigator;
class MissionRouteCache;
class RTLTestPeer;

class RTL : public NavigatorMode, public ModuleParams
{
public:
	RTL(Navigator *navigator);

	~RTL() = default;

	enum class RtlType {
		NONE = rtl_status_s::RTL_STATUS_TYPE_NONE,
		RTL_DIRECT = rtl_status_s::RTL_STATUS_TYPE_DIRECT_SAFE_POINT,
		RTL_DIRECT_MISSION_LAND = rtl_status_s::RTL_STATUS_TYPE_DIRECT_MISSION_LAND,
		RTL_MISSION_FAST = rtl_status_s::RTL_STATUS_TYPE_FOLLOW_MISSION,
		RTL_MISSION_FAST_REVERSE = rtl_status_s::RTL_STATUS_TYPE_FOLLOW_MISSION_REVERSE,
	};

	void on_inactive() override;
	void on_activation() override;
	void on_active() override;

	void initialize() override {};

	void set_return_alt_min(bool min) { _enforce_rtl_alt = min; }

	bool isLanding();

private:
	friend class RTLTestPeer;

	enum class DestinationType {
		DESTINATION_TYPE_HOME,
		DESTINATION_TYPE_MISSION_LAND,
		DESTINATION_TYPE_SAFE_POINT
	};

	/**
	 * @brief Check mission landing validity
	 * @return true if mission has a land start, a land and is valid
	 */
	bool hasMissionLandStart() const;

	/**
	 * @brief Check whether there are more waypoints between current waypoint
	 *        and the takeoff location than the end/land location.
	 * @return true if the reverse is more items away.
	 */
	bool reverseIsFurther() const;

	/**
	 * @brief Refresh the mission and safe-point cache used by RTL destination selection.
	 */
	void updateDatamanCache();

	void setRtlTypeAndDestination();

	/**
	 * @brief Publish the remaining time estimate to go to the RTL landing point.
	 *
	 */
	void publishRemainingTimeEstimate();

	/**
	 * @brief Find RTL destination.
	 *
	 */
	void findRtlDestination(DestinationType &destination_type, PositionYawSetpoint &destination, uint8_t &safe_point_index);

	/**
	 * @brief Find RTL destination if only safe points are considered
	 *
	 */
	PositionYawSetpoint findClosestSafePoint(float min_dist, uint8_t &safe_point_index);

	/**
	 * @brief Set the position of the land start marker in the planned mission as destination.
	 *
	 */
	void setLandPosAsDestination(PositionYawSetpoint &rtl_position, mission_item_s &land_mission_item) const;

	/**
	 * @brief calculate return altitude from return altitude parameter, current altitude and cone angle
	 *
	 * @param[in] rtl_position landing position of the rtl
	 *
	 * @return return altitude
	 */
	float computeReturnAltitude(const PositionYawSetpoint &rtl_position) const;

	/**
	 * @brief initialize RTL mission type
	 *
	 */
	void initRtlMissionType(RtlType new_rtl_type, float rtl_alt);

	/**
	 * @brief Update parameters
	 *
	 */
	void parameters_update();

	/**
	 * @brief Choose the most wind-aligned approach in a landing-approach block.
	 *
	 * Bearings are evaluated from the block's land location.
	 */
	loiter_point_s chooseBestLandingApproach(const land_approaches_s &vtol_land_approaches) const;

	/**
	 * @brief Return the wind-selected VTOL approach for destination, or an invalid loiter if none exists.
	 */
	loiter_point_s selectLandingApproach(const PositionYawSetpoint &destination) const;

	hrt_abstime _destination_check_time{0};

	RtlBase *_rtl_mission_type_handle{nullptr};
	RtlType _rtl_type{RtlType::RTL_DIRECT};

	bool _home_has_land_approach{false};           ///< Flag if the home position has a land approach defined
	bool _one_rally_point_has_land_approach{false}; ///< Flag if a rally point has a land approach defined

	RtlDirect _rtl_direct;

	bool _enforce_rtl_alt{false};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::RTL_TYPE>)          _param_rtl_type,
		(ParamInt<px4::params::RTL_CONE_ANG>)      _param_rtl_cone_ang,
		(ParamFloat<px4::params::RTL_RETURN_ALT>)  _param_rtl_return_alt,
		(ParamFloat<px4::params::RTL_MIN_DIST>)    _param_rtl_min_dist,
		(ParamFloat<px4::params::NAV_ACC_RAD>)     _param_nav_acc_rad,
		(ParamInt<px4::params::RTL_APPR_FORCE>)    _param_rtl_appr_force
	)

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::SubscriptionData<vehicle_global_position_s> _global_pos_sub{ORB_ID(vehicle_global_position)};	/**< global position subscription */
	uORB::SubscriptionData<vehicle_status_s> _vehicle_status_sub{ORB_ID(vehicle_status)};	/**< vehicle status subscription */
	uORB::SubscriptionData<mission_s> _mission_sub{ORB_ID(mission)};
	uORB::SubscriptionData<home_position_s> _home_pos_sub{ORB_ID(home_position)};
	uORB::SubscriptionData<wind_s>		_wind_sub{ORB_ID(wind)};

	uORB::Publication<rtl_time_estimate_s> _rtl_time_estimate_pub{ORB_ID(rtl_time_estimate)};
	uORB::Publication<rtl_status_s> _rtl_status_pub{ORB_ID(rtl_status)};
};
