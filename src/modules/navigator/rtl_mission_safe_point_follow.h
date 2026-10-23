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
 * Without a usable safe point it returns to the mission landing or takeoff,
 * whichever is cheaper along the route.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#pragma once

#include "mission_route_cache.h"
#include "mission_route_types.h"
#include "rtl_base.h"
#include "rtl_arrival.h"
#include <lib/rtl/rtl_time_estimator.h>
#include <uORB/topics/vtol_vehicle_status.h>

#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
class RtlMissionSafePointFollow : public RtlBase
{
public:
	RtlMissionSafePointFollow(Navigator *navigator, mission_s mission);

	void on_inactivation() override;
	void on_activation() override;
	void on_active() override;

	bool isLanding() override { return isExecutingGoalStage(); }
	mission_route::ActiveJumpAnchor activeJumpAnchor() const { return _active_jump_anchor; }
	rtl_time_estimate_s calc_rtl_time_estimate() override;
	void configureRoute(const mission_route::RtlRoutePlan &plan, const loiter_point_s &goal_land_approach);

private:
	friend class RtlMissionSafePointFollowTestPeer;
	friend class RTLTestPeer;

	enum class Stage {
		Idle = 0,                /**< Not executing a plan. */
		FollowRoute,             /**< Follow the mission geometry in nominal or reverse direction. */
		TransitionDuringRoute,   /**< Wait for the VTOL transition before resuming the route. */
		BranchOff,               /**< Fly the virtual branch-off waypoint before leaving the route. */
		Arrival,                 /**< Move, descend and hold at the destination (RtlArrival). */
		LandAtGoal               /**< Land at the safe point or mission endpoint. */
	};

	struct Progress {
		Stage stage{Stage::Idle};
		bool advance_route_after_transition{false};
	};

	bool setNextMissionItem() override;
	bool shouldReportMissionItemReached() const override;
	bool shouldLandFrontTransitionAsFixedWing() const override { return true; }
	/** @brief Publish the current join, follow, branch-off, or landing setpoints for the active RTL stage. */
	void setActiveMissionItems() override;

	/** @brief Build the landing item: the mission LAND item for that goal, otherwise a synthetic landing at the goal. */
	void setGoalLandMissionItem(mission_item_s &mission_item) const;
	/** @brief Build the arrival item for the phase of @p arrival. */
	void setGoalArrivalMissionItem(mission_item_s &mission_item, const RtlArrival &arrival) const;
	/** @brief Build the first destination item flown after the branch-off or a route shortcut. */
	void setGoalMissionItem(mission_item_s &mission_item) const;
	/** @brief Resolve the destination loiter, capped at the arrival altitude. */
	loiter_point_s goalLandApproach(float arrival_altitude, bool use_approach) const;
	float goalArrivalAltitude() const;
	/** @brief Freeze the arrival altitude and enter the destination sequence. */
	void enterGoalStage();
	bool isExecutingGoalStage() const;

	/** @brief Fly route waypoints and loiters through without holding; keep takeoff and landing items. */
	void normalizeRouteMissionItem(mission_item_s &mission_item) const;
	/** @brief Find the next route position index after @p from_index in the planned direction, skipping DO_JUMP. */
	bool findAdjacentRouteIndex(int32_t from_index, int32_t &adjacent_index);
	/** @brief Resolve a loaded item to its flown target; return true at the branch-off or selected endpoint. */
	bool resolveRouteTarget(int32_t index, mission_item_s &mission_item) const;
	/** @brief Load the next route item, substituting the branch-off or normalizing holds while preserving endpoints. */
	bool loadNextRouteItem(mission_item_s &next_route_item);
	/** @brief Publish the resolved route target and start its VTOL transition. */
	void armRouteTransition(mission_route::VtolTransitionAction action, bool advance_route_after_transition);
	/** @brief Prepare join and route transition items; a route transition aligns where the vehicle is. */
	void prepareVtolTransitionItem(bool aligning) override;
	bool routeReturnTransitionActive() const override;
	bool frontTransitionInhibited() const override;
	mission_route::VtolTransitionAction allowedRouteTransition(mission_route::VtolTransitionAction action) const;
	/** Resolve the VTOL state at a mission position anchor, using lookahead only without a preceding mode. */
	uint8_t getVtolStateAtMissionIndex(int32_t anchor_index);
	/** getVtolStateAtMissionIndex() for all _mission.count indices in two passes; false if they do not fit or an item cannot be read. */
	bool getVtolStatesForMission(uint8_t vtol_states[], int32_t capacity);
	/** @param vtol_states optional getVtolStatesForMission() result, replaces the scan per target */
	mission_route::VtolTransitionAction vtolTransitionActionForTarget(int32_t target_index, bool direction_reversed,
			const uint8_t *vtol_states = nullptr);
	mission_route::VtolTransitionAction vtolTransitionActionAfterReachingReverseTarget(int32_t reached_target_index);
	/** @brief Publish the route target, or start its VTOL transition or the branch-off. */
	void handleFollowRouteStage(const position_setpoint_s &current_setpoint_copy);
	/** @brief Return true when the selected safe-point goal has a concrete VTOL landing approach to fly. */
	bool useGoalLandApproach() const;
	/** @brief First stage of the destination sequence: arrival, or the landing for a mission LAND goal. */
	Stage goalEntryStage() const;
	bool goalIsMissionLanding() const;
	bool goalIsMissionTakeoff() const;
	bool goalIsSafePoint() const;
	/** @brief Return whether a mission index is the route item replaced by the branch-off waypoint. */
	bool isBranchOffIndex(int32_t index) const;
	bool missionItemMatchesSelectedEndpoint(const mission_item_s &mission_item) const;
	bool missionIndexInBounds(int32_t index) const;
	/** @brief Reset transient executor progress so inactive-state queries do not observe stale stages. */
	void resetExecutorProgress();
	/** @brief Publish landing setpoints through MissionBase::handleLanding(), as Mission does. */
	void publishLandingItems(position_setpoint_triplet_s *pos_sp_triplet,
				 const position_setpoint_s &current_setpoint_copy,
				 const mission_item_s &landing_mission_item);
	/** @brief Publish the virtual branch-off waypoint and the subsequent goal item. */
	void publishBranchOffItems(const position_setpoint_s &current_setpoint_copy);
	/** @brief Advance to the next route target, or hand over to the goal stage when the route is exhausted. */
	void advanceRouteTarget();
	/** @brief Estimate from the latest inactive plan or the active progress, without advancing execution. */
	void addRemainingLegsToTimeEstimate(const vehicle_global_position_s &global_pos);

	/** @brief Read MissionRouteCache; in the goal stages the current index returns the synthetic goal item. */
	bool loadMissionItemFromCache(int32_t index, mission_item_s &mission_item) override;
	bool isMissionValid() const override;
	bool shouldAcceptMissionUpdates() override { return false; }
	bool shouldReplayMissionActionItems() const override { return false; }
	/** Only an aborted mission landing returns to its landing start; a rally point or takeoff goal is not a mission item. */
	bool landingAbortMovesMissionIndex() const override { return goalIsMissionLanding(); }

	mission_route::RtlRoutePlan _plan{};
	uORB::SubscriptionData<vtol_vehicle_status_s> _vtol_status_sub{ORB_ID(vtol_vehicle_status)};
	mission_item_s _goal_mission_land_item{};
	bool _goal_mission_land_item_valid{false};
	Progress _state{};
	RtlArrival _arrival{};
	loiter_point_s _goal_land_approach{};
	float _goal_arrival_alt{NAN};
	RtlTimeEstimator _rtl_time_estimator;
	uint8_t _vtol_states[MissionRouteCache::kMaxFullMissionCacheSize] {}; ///< Scratch table of the time estimate.

	DEFINE_PARAMETERS_CUSTOM_PARENT(
		RtlBase,
		(ParamFloat<px4::params::RTL_DESCEND_ALT>) _param_rtl_descend_alt,
		(ParamFloat<px4::params::RTL_LAND_DELAY>) _param_rtl_land_delay,
		(ParamFloat<px4::params::RTL_LOITER_RAD>) _param_rtl_loiter_rad,
		(ParamInt<px4::params::RTL_PLD_MD>) _param_rtl_pld_md,
		(ParamInt<px4::params::NAV_FORCE_VT>) _param_nav_force_vt
	)
};

#endif // CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE
