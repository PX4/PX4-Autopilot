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
#include <lib/perf/perf_counter.h>
#include <lib/rtl/rtl_time_estimator.h>

class RtlMissionSafePointFollow : public RtlBase
{
public:
	/** @brief Execute the staged Route Safe Point Return plan built by RTL type 6. */
	RtlMissionSafePointFollow(Navigator *navigator, mission_s mission);
	~RtlMissionSafePointFollow() override;

	void on_inactivation() override;
	void on_activation() override;

	bool isLanding() override { return _state.stage == Stage::ApproachAtGoal || _state.stage == Stage::LandAtGoal; }
	MissionRoutePlanner::Segment lastFlownLoopSegment() const override { return _last_flown_loop_segment; }
	rtl_time_estimate_s calc_rtl_time_estimate() override;
	void setRtlAlt(float alt) override { _rtl_alt = alt; }
	void configureRouteSafePoint(const RouteSafePointConfig &config) override;

private:
	friend class RtlMissionSafePointFollowTestPeer;

	enum class Stage {
		Idle = 0,                /**< No active plan. */
		FollowRoute,             /**< Follow the mission geometry in nominal or reverse direction. */
		TransitionDuringRoute,   /**< Apply a VTOL transition during route following (prevents re-issuing). */
		BranchOff,               /**< Fly the virtual branch-off waypoint before leaving the route. */
		ApproachAtGoal,          /**< Fly the selected landing approach loiter before handing over to landing. */
		LandAtGoal               /**< Execute the final landing at the safe point or fallback endpoint. */
	};

	struct PlanState {
		Stage stage{Stage::Idle};
		int32_t transition_target_index{-1};
		VtolTransitionAction transition_action{VtolTransitionAction::None};
		bool transition_command_sent{false};
		bool advance_route_after_transition{false};

		void clearRouteTransition()
		{
			transition_target_index = -1;
			transition_action = VtolTransitionAction::None;
			transition_command_sent = false;
			advance_route_after_transition = false;
		}

		void resetProgress()
		{
			stage = Stage::Idle;
			clearRouteTransition();
		}
	};

	/** @brief Advance the RTL stage machine without replaying the full mission control flow. */
	bool setNextMissionItem() override;
	/** @brief Publish the current join, follow, branch-off, or landing setpoints for the active RTL stage. */
	void setActiveMissionItems() override;

	/** @brief Build a virtual waypoint used for joins, branch-offs, and synthetic move-to-point legs. */
	void setWaypointMissionItem(mission_item_s &mission_item, const MissionRoutePlanner::Position &position,
				    bool autocontinue, bool vtol_back_transition_required = false) const;
	/** @brief Build the synthetic RTL landing item for safe-point landings and reverse takeoff fallback. */
	void setLandMissionItem(mission_item_s &mission_item) const;
	/** @brief Build the synthetic goal-approach loiter item for VTOL safe-point landings. */
	void setGoalApproachMissionItem(mission_item_s &mission_item) const;

	/** @brief Convert a position-bearing mission item into a pure geometric route waypoint.
	 *
	 * During route following, the mission is treated as geometry only. Waypoints and loiter-style
	 * position items such as NAV_CMD_LOITER_UNLIMITED, NAV_CMD_LOITER_TIME_LIMIT, and
	 * NAV_CMD_LOITER_TO_ALT are converted to plain waypoints with autocontinue enabled and zero
	 * hold time so the vehicle keeps moving. Commands with distinct execution semantics, such as
	 * takeoff or landing, are left untouched.
	 * NAV_CMD_DELAY items are non-position and are skipped by geometry-only position traversal,
	 * but if one were encountered it would also be clamped here.
	 *
	*/
	void normalizeRouteMissionItem(mission_item_s &mission_item) const;
	/** @brief Load the adjacent route position item in the currently selected traversal direction. */
	bool loadAdjacentRouteItem(mission_item_s &mission_item, int32_t &adjacent_index);
	/** @brief Arm the synthetic route transition that should be issued on the next publication pass. */
	void armRouteTransition(VtolTransitionAction action, bool advance_route_after_transition);
	/** @brief Clear any staged route-transition bookkeeping. */
	void clearRouteTransitionState();
	/** @brief Publish and issue the staged route transition, then wait for completion. */
	void handleRouteTransitionStage(position_setpoint_triplet_s *pos_sp_triplet,
					const position_setpoint_s &current_setpoint_copy);
	/** @brief Publish the active route-following setpoints, endpoint handoff, and any pending transition. */
	void handleFollowRouteStage(position_setpoint_triplet_s *pos_sp_triplet,
				    const position_setpoint_s &current_setpoint_copy);
	/** @brief Return whether the current route target coincides with the selected branch-off anchor. */
	bool currentTargetIsBranchOff() const;
	/** @brief Return true when the selected safe-point goal has a concrete VTOL landing approach to fly. */
	bool useGoalLandApproach() const;
	/** @brief Return the stage that should execute after leaving the route or skipping directly to goal. */
	Stage finalGoalStage() const;
	/** @brief Return whether endpoint fallback targets the mission landing item. */
	bool goalIsMissionLanding() const;
	/** @brief Return whether endpoint fallback targets the mission takeoff item. */
	bool goalIsMissionTakeoff() const;
	/** @brief Return whether a mission item matches the currently selected endpoint fallback. */
	bool missionItemMatchesSelectedEndpoint(const mission_item_s &mission_item) const;
	/** @brief Return whether a mission item is a landing command. */
	static bool isLandingCommand(const mission_item_s &mission_item) { return MissionRoutePlanner::isLandingCmd(mission_item.nav_cmd); }
	/** @brief Return whether a mission item is a takeoff command. */
	static bool isTakeoffCommand(const mission_item_s &mission_item) { return MissionRoutePlanner::isTakeoffCmd(mission_item.nav_cmd); }
	/** @brief Return whether a mission index lies within the active mission bounds. */
	bool missionIndexInBounds(int32_t index) const;
	/** @brief Reset transient executor progress so inactive-state queries do not observe stale stages. */
	void resetExecutorProgress();
	/** @brief Seed the cached loop anchor from the active plan, clearing it when the projection is not on a DO_JUMP edge. */
	void updateLastFlownLoopSegmentFromPlan();
	/**
	 * @brief Refresh the cached loop anchor before a forward route advance.
	 *
	 * Skips DO_JUMP commands as executable control flow, but forward replans still need to
	 * remember whether the next nominal advance was about to traverse an active jump edge. The
	 * shared MissionBase helper inspects exactly that immediate pre-next-position window.
	 */
	void updateLastFlownLoopSegmentForNominalAdvance();
	/** @brief Publish a non-landing setpoint pair and clear any transient MissionBase work item. */
	void publishRouteItems(position_setpoint_triplet_s *pos_sp_triplet,
			       const position_setpoint_s &current_setpoint_copy,
			       const mission_item_s &current_mission_item,
			       const mission_item_s *next_mission_item,
			       bool sync_active_mission_item = true);
	/** @brief Publish landing setpoints through MissionBase::handleLanding() to preserve legacy landing semantics. */
	void publishLandingItems(position_setpoint_triplet_s *pos_sp_triplet,
				 const position_setpoint_s &current_setpoint_copy,
				 const mission_item_s &landing_mission_item);
	/** @brief Advance to the next route target, returning true on success or landing at goal on failure. */
	bool advanceRouteTarget();

	/** @brief Load a mission item through MissionRouteCache instead of MissionBase's local cache. */
	bool loadMissionItemFromCache(int32_t index, mission_item_s &mission_item) override;

	MissionRoutePlanner::Plan _plan{};
	PlanState _state{};
	MissionRoutePlanner::Segment _last_flown_loop_segment{};
	loiter_point_s _goal_land_approach{};
	float _rtl_alt{NAN};
	RtlTimeEstimator _rtl_time_estimator; /**< Time estimator consistent with other RTL modes. */
	perf_counter_t _calc_rtl_time_estimate_perf{perf_alloc(PC_ELAPSED, "rtl_route_calc_time_est")};

	DEFINE_PARAMETERS_CUSTOM_PARENT(
		RtlBase,
		(ParamInt<px4::params::RTL_PLD_MD>) _param_rtl_pld_md
	)
};
