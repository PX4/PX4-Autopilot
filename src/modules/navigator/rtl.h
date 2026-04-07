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
#include <dataman_client/DatamanClient.hpp>
#include "rtl_base.h"
#include "rtl_direct.h"
#include "rtl_direct_mission_land.h"
#include "rtl_mission_fast.h"
#include "rtl_mission_fast_reverse.h"
#include "rtl_mission_safe_point_follow.h"
#include "mission_route_planner.h"

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

	~RTL() override { delete _rtl_mission_type_handle; }

	enum class RtlType {
		NONE = rtl_status_s::RTL_STATUS_TYPE_NONE,
		RTL_DIRECT = rtl_status_s::RTL_STATUS_TYPE_DIRECT_SAFE_POINT,
		RTL_DIRECT_MISSION_LAND = rtl_status_s::RTL_STATUS_TYPE_DIRECT_MISSION_LAND,
		RTL_MISSION_FAST = rtl_status_s::RTL_STATUS_TYPE_FOLLOW_MISSION,
		RTL_MISSION_FAST_REVERSE = rtl_status_s::RTL_STATUS_TYPE_FOLLOW_MISSION_REVERSE,
		RTL_MISSION_SAFE_POINT_FOLLOW = rtl_status_s::RTL_STATUS_TYPE_FOLLOW_MISSION_SAFE_POINT,
	};

	void on_inactive() override;
	void on_inactivation() override;
	void on_activation() override;
	void on_active() override;

	void initialize() override {};

	void set_return_alt_min(bool min) { _enforce_rtl_alt = min; }

	bool isLanding();

	friend class RTLTestPeer;

private:
	enum class DestinationType {
		DESTINATION_TYPE_HOME,
		DESTINATION_TYPE_MISSION_LAND,
		DESTINATION_TYPE_SAFE_POINT,
		DESTINATION_TYPE_MISSION_TAKEOFF
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
	 * @brief Drop the route-safe-point continuity hints that depend on the active mission or safe-point set.
	 */
	void resetRouteSafePointState();

	/**
	 * @brief Refresh the mission and safe-point caches used by route-safe-point RTL.
	 */
	void updateDatamanCache();

	/** @brief Build the route-safe-point planner input for the current vehicle and mission state. */
	MissionRoutePlanner::Config buildRouteSafePointConfig(bool last_route_direction_reversed) const;

	/** @brief Evaluate a fresh route-safe-point plan. */
	bool evaluateRouteSafePointPlan(const MissionRouteCache &mission_route_cache,
					MissionRoutePlanner::Plan &new_plan);

	/** @brief Apply a valid route-safe-point plan to the RTL destination/output state. */
	void applyRouteSafePointPlan(const MissionRoutePlanner::Plan &plan,
				     bool current_route_direction_reversed,
				     RtlType &new_rtl_type,
				     DestinationType &destination_type,
				     PositionYawSetpoint &destination,
				     uint8_t &safe_point_index);

	/** @brief Convert a route-safe-point planner goal into the outer RTL destination category. */
	static DestinationType routePlanDestinationType(MissionRoutePlanner::GoalType goal_type);

	/** @brief Fall back from route-safe-point RTL to the existing direct or mission-land logic. */
	void applyRouteSafePointFallback(RtlType &new_rtl_type,
					 DestinationType &destination_type,
					 PositionYawSetpoint &destination,
					 uint8_t &safe_point_index);

	/** @brief Select the active RTL executor, destination, and route-safe-point plan if applicable. */
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
	/** @brief Resolve the RTL destination for a specific RTL_TYPE policy. */

	void findRtlDestinationForType(int rtl_type, DestinationType &destination_type,
				       PositionYawSetpoint &destination, uint8_t &safe_point_index);

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
	 * @brief Set the safepoint as destination.
	 *
	 * @param mission_safe_point is the mission safe point/rally point to set as destination.
	 */
	void setSafepointAsDestination(PositionYawSetpoint &rtl_position, const mission_item_s &mission_safe_point) const;

	/**
	 * @brief calculate return altitude from return altitude parameter, current altitude and cone angle
	 *
	 * @param[in] rtl_position landing position of the rtl
	 *
	 * @return return altitude
	 */
	float computeReturnAltitude(const PositionYawSetpoint &rtl_position) const;

	/**
	 * @brief Construct and initialize the selected mission-based RTL executor.
	 *
	 * @return true if the executor was created and initialized successfully
	 */
	bool initRtlMissionType(RtlType new_rtl_type, float rtl_alt);

	/**
	 * @brief Update parameters
	 *
	 */
	void parameters_update();

	/**
	 * @brief Choose best landing approach
	 *
	 * Choose best landing approach for home considering wind
	 *
	 * @return loiter_point_s best landing approach
	 */
	loiter_point_s chooseBestLandingApproach(const land_approaches_s &vtol_land_approaches) const;

	/** @brief Return the wind-selected VTOL landing approach for a destination, or an invalid loiter if none exists. */
	loiter_point_s selectLandingApproach(const PositionYawSetpoint &destination) const;


	hrt_abstime _destination_check_time{0};

	RtlBase *_rtl_mission_type_handle{nullptr};
	RtlType _rtl_type{RtlType::RTL_DIRECT};

	bool _home_has_land_approach{false};           ///< Flag if the home position has a land approach defined
	bool _any_safe_point_has_land_approach{false}; ///< Flag if a rally point has a land approach defined

	uint32_t _last_route_safe_point_warning_mission_id{0};
	uint32_t _route_plan_mission_id{0};
	uint16_t _route_plan_mission_count{0};
	uint8_t _route_plan_mission_dataman_id{0};
	uint32_t _route_plan_safe_points_id{0};
	uint8_t _route_plan_safe_points_dataman_id{0};

	RtlDirect _rtl_direct;

	bool _enforce_rtl_alt{false};
	// The planner cannot infer direction from mission_index alone when the vehicle is near a shared waypoint:
	// the same target index can mean "arriving from ahead" or "arriving from behind".
	// Keep the last chosen route direction as a continuity hint for the next type-6 planning pass.
	bool _route_safe_point_direction_reversed{false};
	MissionRoutePlanner::Segment _last_route_safe_point_loop_segment{};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::RTL_TYPE>)          _param_rtl_type,
		(ParamInt<px4::params::RTL_CONE_ANG>)      _param_rtl_cone_ang,
		(ParamFloat<px4::params::RTL_RETURN_ALT>)  _param_rtl_return_alt,
		(ParamFloat<px4::params::RTL_MIN_DIST>)    _param_rtl_min_dist,
		(ParamFloat<px4::params::NAV_ACC_RAD>)     _param_nav_acc_rad,
		(ParamInt<px4::params::RTL_APPR_FORCE>)    _param_rtl_appr_force,
		(ParamFloat<px4::params::MIS_MC_SEG_DIST>) _param_mis_mc_seg_dist,
		(ParamFloat<px4::params::MIS_FW_SEG_DIST>) _param_mis_fw_seg_dist,
		(ParamFloat<px4::params::RTL_RP_SEG_DIST>) _param_rtl_rp_seg_dist,
		(ParamFloat<px4::params::RTL_FW_UTURN_PEN>) _param_rtl_fw_uturn_pen
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
