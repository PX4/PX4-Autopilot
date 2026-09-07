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

#include "mission_route_types.h"
#include "rtl_base.h"
#include <lib/perf/perf_counter.h>
#include <lib/rtl/rtl_time_estimator.h>

#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
class RtlMissionSafePointFollow : public RtlBase
{
public:
	/** @brief Execute the staged Route Safe Point Return plan built by RTL type 7. */
	RtlMissionSafePointFollow(Navigator *navigator, mission_s mission);
	~RtlMissionSafePointFollow() override;

	void on_inactivation() override;
	void on_activation() override;

	bool isLanding() override { return _state.stage == Stage::ApproachAtGoal || _state.stage == Stage::LandAtGoal; }
	mission_route::ActiveJumpAnchor activeJumpAnchor() const override { return _active_jump_anchor; }
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
		mission_route::VtolTransitionAction transition_action{mission_route::VtolTransitionAction::kNone};
		bool transition_command_sent{false};
		bool advance_route_after_transition{false};

		void clearRouteTransition()
		{
			transition_target_index = -1;
			transition_action = mission_route::VtolTransitionAction::kNone;
			transition_command_sent = false;
			advance_route_after_transition = false;
		}
	};

	/** @brief Advance the RTL stage machine without replaying the full mission control flow. */
	bool setNextMissionItem() override;
	/** @brief Publish the current join, follow, branch-off, or landing setpoints for the active RTL stage. */
	void setActiveMissionItems() override;

	/** @brief Build the virtual branch-off waypoint. */
	void setWaypointMissionItem(mission_item_s &mission_item, const mission_route::Position &position) const;
	/** @brief Build the landing item: the mission LAND item for that goal, otherwise a synthetic landing at the goal. */
	void setLandMissionItem(mission_item_s &mission_item) const;
	/** @brief Build the synthetic goal-approach loiter item for VTOL safe-point landings. */
	void setGoalApproachMissionItem(mission_item_s &mission_item) const;
	/** @brief Build the item flown after the branch-off: the approach loiter when available, otherwise the landing. */
	void setGoalMissionItem(mission_item_s &mission_item) const;

	/** @brief Flatten waypoint and loiter holds while preserving takeoff and landing commands.
	 *
	 * Route waypoints and loiters use autocontinue with zero hold time. Non-position delays
	 * are also cleared if encountered, although route traversal normally skips them.
	 */
	void normalizeRouteMissionItem(mission_item_s &mission_item) const;
	/** @brief Find the next route position index after @p from_index in the planned direction, skipping DO_JUMP. */
	bool findAdjacentRouteIndex(int32_t from_index, int32_t &adjacent_index);
	/** @brief Load the next route item, substituting the branch-off or normalizing holds while preserving endpoints. */
	bool loadNextRouteItem(mission_item_s &next_route_item, int32_t &next_index);
	/** @brief Arm the synthetic route transition that should be issued on the next publication pass. */
	void armRouteTransition(mission_route::VtolTransitionAction action, bool advance_route_after_transition);
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
	/** @brief Return whether the plan targets a safe point rather than a mission endpoint. */
	bool goalIsSafePoint() const;
	/** @brief Return whether a mission index is the route item replaced by the branch-off waypoint. */
	bool isBranchOffIndex(int32_t index) const;
	/** @brief Return whether a mission item matches the currently selected endpoint fallback. */
	bool missionItemMatchesSelectedEndpoint(const mission_item_s &mission_item) const;
	/** @brief Return whether a mission item is a landing command. */
	static bool isLandingCommand(const mission_item_s &mission_item) { return mission_route::isLandingCmd(mission_item.nav_cmd); }
	/** @brief Return whether a mission item is a takeoff command. */
	static bool isTakeoffCommand(const mission_item_s &mission_item) { return mission_route::isTakeoffCmd(mission_item.nav_cmd); }
	/** @brief Return whether a mission index lies within the active mission bounds. */
	bool missionIndexInBounds(int32_t index) const;
	/** @brief Reset transient executor progress so inactive-state queries do not observe stale stages. */
	void resetExecutorProgress();
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
	/** @brief Publish the virtual branch-off waypoint and the subsequent goal item. */
	void publishBranchOffItems(position_setpoint_triplet_s *pos_sp_triplet,
				   const position_setpoint_s &current_setpoint_copy);
	/** @brief Advance to the next route target, or hand over to the goal stage when the route is exhausted. */
	void advanceRouteTarget();
	/** @brief Add the remaining legs of the active stage, from the vehicle to the goal, to the time estimator. */
	void addRemainingLegsToTimeEstimate(const vehicle_global_position_s &global_pos);

	/** @brief Load a mission item through MissionRouteCache instead of MissionBase's local cache. */
	bool loadMissionItemFromCache(int32_t index, mission_item_s &mission_item) override;
	bool isMissionValid() const override;
	uint8_t missionStartVtolState() const override { return _vtol_state_on_mission_upload; }
	bool shouldAcceptMissionUpdates() override { return false; }
	bool shouldReplayMissionActionItems() const override { return false; }

	mission_route::RtlRoutePlan _plan{};
	mission_item_s _goal_mission_land_item{};
	bool _goal_mission_land_item_valid{false};
	uint8_t _vtol_state_on_mission_upload{vtol_vehicle_status_s::VEHICLE_VTOL_STATE_UNDEFINED};
	PlanState _state{};
	mission_route::ActiveJumpAnchor _active_jump_anchor{};
	loiter_point_s _goal_land_approach{};
	float _rtl_alt{NAN};
	RtlTimeEstimator _rtl_time_estimator; /**< Time estimator consistent with other RTL modes. */
	perf_counter_t _calc_rtl_time_estimate_perf{perf_alloc(PC_ELAPSED, "rtl_route_calc_time_est")};

	DEFINE_PARAMETERS_CUSTOM_PARENT(
		RtlBase,
		(ParamInt<px4::params::RTL_PLD_MD>) _param_rtl_pld_md
	)
};

#endif // CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE
