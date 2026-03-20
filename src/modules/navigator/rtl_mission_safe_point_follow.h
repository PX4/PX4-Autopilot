/***************************************************************************
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
 * RTL executor that follows the uploaded mission route toward a safe-point
 * branch-off, then leaves the route to land at the chosen safe point.
 * Falls back to the closest mission endpoint when no safe point is available.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#pragma once

#include "rtl_base.h"

class RtlMissionSafePointFollow : public RtlBase
{
public:
	/** @brief Execute the staged Route Safe Point Return plan built by RTL type 6. */
	RtlMissionSafePointFollow(Navigator *navigator, mission_s mission);
	~RtlMissionSafePointFollow() = default;

	void on_inactivation() override;
	void on_activation() override;

	bool isLanding() override { return _stage == Stage::LandAtGoal; }
	bool shouldGoStraightToGoal() const override { return _should_go_straight_to_goal; }
	RtlRoutePlanner::Segment lastFlownLoopSegment() const override { return _last_flown_loop_segment; }
	rtl_time_estimate_s calc_rtl_time_estimate() override;
	void setRoutePlan(const RtlRoutePlanner::Plan &plan) override;

private:
	enum class Stage {
		Idle = 0,             /**< No active SRP plan. */
		JoinRoute,            /**< Fly the virtual join waypoint at the vehicle projection. */
		TransitionAfterJoin,  /**< Apply a required VTOL back-transition before following the route. */
		FollowRoute,          /**< Follow the mission geometry in nominal or reverse direction. */
		BranchOff,            /**< Fly the virtual branch-off waypoint before leaving the route. */
		LandAtGoal            /**< Execute the final landing at the safe point or fallback endpoint. */
	};

	/** @brief Advance the SRP stage machine without replaying the full mission control flow. */
	bool setNextMissionItem() override;
	/** @brief Publish the current join, follow, branch-off, or landing setpoints for the active SRP stage. */
	void setActiveMissionItems() override;

	/** @brief Build a virtual waypoint used for joins, branch-offs, and synthetic move-to-point legs. */
	void setWaypointMissionItem(mission_item_s &mission_item, const RtlRoutePlanner::Position &position,
				    bool autocontinue, bool vtol_back_transition_required = false) const;
	/** @brief Build the synthetic SRP landing item for safe-point landings and reverse takeoff fallback. */
	void setLandMissionItem(mission_item_s &mission_item) const;
	/** @brief Convert a position-bearing mission item into a pure geometric route waypoint. */
	void normalizeRouteMissionItem(mission_item_s &mission_item) const;
	/** @brief Load the adjacent route position item in the currently selected traversal direction. */
	bool loadAdjacentRouteItem(mission_item_s &mission_item, int32_t *adjacent_index = nullptr);
	/** @brief Load the previous position item while ignoring DO_JUMP control flow. */
	bool loadPreviousRoutePositionItemNoJump(int32_t start_index, int32_t &previous_index);
	/** @brief Read one mission item from the active mission dataman stream. */
	bool loadMissionItemAtIndex(int32_t index, mission_item_s &mission_item);
	/** @brief Find the nearest position item attached to or preceding the supplied mission index. */
	bool findAttachedRoutePositionIndex(int32_t start_index, int32_t &attached_index);
	/** @brief Find the next position-bearing mission item at or after the supplied mission index. */
	bool findNextRoutePositionIndex(int32_t start_index, int32_t &next_index);
	/** @brief Return whether the current route target coincides with the selected branch-off anchor. */
	bool currentTargetIsBranchOff() const;
	/** @brief Return whether the join projection is already close enough to skip route following. */
	bool joinProjectionNearBranchOff() const;
	/** @brief Seed the loop-anchor memory from the active plan's vehicle projection. */
	void updateLastFlownLoopSegmentFromPlan();
	/** @brief Track the most recently flown loop edge so replans stay anchored on the active jump segment. */
	void updateLastFlownLoopSegmentForNominalAdvance();
	/** @brief Detect whether entering the segment at the given target requires a VTOL back-transition. */
	bool requiresBacktransitionForTarget(int32_t target_index, bool reversed);
	/** @brief Query the shared planner's segment-anchor logic for route-follow transition replay. */
	RtlRoutePlanner::TransitionAction transitionActionForTargetIndex(int32_t target_index,
			bool direction_reversed);
	/** @brief Publish a non-landing SRP setpoint pair and reset the work item back to route following. */
	void publishRouteItems(position_setpoint_triplet_s *pos_sp_triplet,
			       const position_setpoint_s &current_setpoint_copy,
			       const mission_item_s &current_mission_item,
			       const mission_item_s *next_mission_item);
	/** @brief Publish SRP landing setpoints through MissionBase::handleLanding() to preserve legacy landing semantics. */
	void publishLandingItems(position_setpoint_triplet_s *pos_sp_triplet,
				 const position_setpoint_s &current_setpoint_copy,
				 const mission_item_s &landing_mission_item);

	RtlRoutePlanner::Plan _plan{};
	Stage _stage{Stage::Idle};
	int32_t _branch_off_index{-1};
	bool _should_go_straight_to_goal{false};
	RtlRoutePlanner::Segment _last_flown_loop_segment{};
};
