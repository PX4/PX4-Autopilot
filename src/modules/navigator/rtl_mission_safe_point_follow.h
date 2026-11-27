/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file rtl_mission_safe_point_follow.h
 *
 * Return along the uploaded mission route (RTL_TYPE 7, multicopters). It joins the route, follows it to where it
 * branches off to a rally point, or to the mission landing or takeoff point, and lands there.
 * Built only with the mission route cache.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#pragma once

#include "mission_route_types.h"
#include "rtl_base.h"

#include <lib/rtl/rtl_time_estimator.h>

class RtlMissionSafePointFollow : public RtlBase
{
public:
	explicit RtlMissionSafePointFollow(Navigator *navigator);

	void on_activation() override;

	/** From the move to the landing point on. */
	bool isLanding() override;

	/** Idle on the ground: Return started there or touched down. */
	bool isIdle() const { return _mission_item.nav_cmd == NAV_CMD_IDLE; }

	rtl_time_estimate_s calc_rtl_time_estimate() override;

	/**
	 * Plan the route on this mission from the vehicle position. Without a route the Return holds.
	 * Only a mission that Mission checked against its fence, with its items and rally points in the route cache, gets one.
	 */
	void planRoute(const mission_s &mission);

	const mission_route::RtlRoutePlan &plan() const { return _plan; }

	/** The route was planned on this mission, these rally points and this fence. */
	bool plannedOn(const mission_s &mission) const;

private:
	friend class RtlMissionSafePointFollowTestPeer;

	bool setNextMissionItem() override;
	void setActiveMissionItems() override;

	/** RTL hands over the mission when it plans: an upload does not change the route being flown. */
	void updateMavlinkMission() override {}

	/** Read the planned mission from the route cache. Once it is replaced, its items no longer load. */
	bool loadMissionItemFromCache(int32_t index, mission_item_s &mission_item) override;

	/** The route item where the goal steps start: the branch-off item, the landing or takeoff item, or any item of a direct plan. */
	bool isGoal(int32_t index, const mission_item_s &mission_item) const;

	/** The next position item in the direction of the plan, DO_JUMP items ignored. */
	bool findAdjacentRouteIndex(int32_t from_index, int32_t &adjacent_index);

	/** The route item after the current one, or the branch-off point in place of its item. */
	bool loadNextRouteItem(mission_item_s &mission_item);

	/** The branch-off point in place of the route item at this index, if Return leaves the route there for a rally point. */
	void useBranchOffPoint(int32_t index, mission_item_s &mission_item) const;

	/** A climb in place to the branch-in altitude before the join, as Mission climbs before a higher item. */
	void handleJoinClimb(WorkItemType &new_work_item_type, mission_item_s next_mission_items[], size_t &num_found_items);

	/** The steps at the goal: the branch-off point, the move above the goal, RTL_LAND_DELAY and the landing. */
	void handleGoal(WorkItemType &new_work_item_type, mission_item_s next_mission_items[], size_t &num_found_items);

	/** The landing at a rally point or at the takeoff point. */
	void setGoalLandItem(mission_item_s &mission_item) const;

	void addRemainingLegsToTimeEstimate();

	mission_route::RtlRoutePlan _plan{};
	mission_route::FailureReason _plan_failure{mission_route::FailureReason::kNone}; ///< kNone without a plan: an input was not ready
	RtlTimeEstimator _rtl_time_estimator;

	DEFINE_PARAMETERS_CUSTOM_PARENT(
		RtlBase,
		(ParamFloat<px4::params::MIS_MC_SEG_DIST>) _param_mis_mc_seg_dist,
		(ParamFloat<px4::params::RTL_RP_SEG_DIST>) _param_rtl_rp_seg_dist,
		(ParamFloat<px4::params::RTL_DESCEND_ALT>) _param_rtl_descend_alt,
		(ParamFloat<px4::params::RTL_LAND_DELAY>) _param_rtl_land_delay,
		(ParamInt<px4::params::RTL_PLD_MD>) _param_rtl_pld_md
	)
};
