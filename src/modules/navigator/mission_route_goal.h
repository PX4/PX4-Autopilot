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
 * @file mission_route_goal.h
 *
 * Mission-route goal and path selector. This unit owns route path solving,
 * safe-point scoring, endpoint fallback selection, and join/skip policy.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#pragma once

#include "mission_route_projection.h"
#include "mission_route_types.h"

#include <stdint.h>

#include <matrix/math.hpp>

namespace mission_route
{

enum class PathDirectionMode : uint8_t {
	kAuto = 0,
	kForceNominal,
	kForceReverse
};

enum class RouteGoalSegmentType : uint8_t {
	kOutsideActiveLoopJump = 0,
	kOnActiveLoopJump
};

struct GoalSelectionResult {
	bool success{false};
	FailureReason failure_reason{FailureReason::kUnknown};
	GoalSelection selection{};
};

class MissionRouteGoalSelector
{
public:
	MissionRouteGoalSelector(const Provider &provider, const MissionRouteProjection &projection) :
		_provider(provider), _projection(projection) {}

	RoutePath findShortestPathAlongRoute(float goal_route_along,
					     const ProjectionContext &projection_context,
					     const PlannerConfig &config,
					     PathDirectionMode direction_mode = PathDirectionMode::kAuto,
					     RouteGoalSegmentType goal_seg_type = RouteGoalSegmentType::kOutsideActiveLoopJump) const;

	GoalSelectionResult selectSafePoint(const ProjectionContext &projection_context,
					    const PlannerConfig &config,
					    ProjectionReferenceBatch &batch) const;

	GoalSelectionResult selectBestGoal(const ProjectionContext &projection_context,
					   const PlannerConfig &config,
					   ProjectionReferenceBatch &batch) const;

	JoinContext buildJoinContext(const Position &vehicle_position,
				     const ProjectionContext &projection_context,
				     const RoutePath &path) const;

	/**
	 * @brief Return true when a selected goal can bypass route following.
	 *
	 * Mission endpoint fallbacks reuse the join-altitude shortcut. Safe-point goals only skip
	 * when the vehicle is already close to the selected safe point or branch-off leg.
	 */
	bool canSkipRouteFollowToSelectedGoal(const Position &vehicle_position,
					      const GoalSelection &selection,
					      const PlannerConfig &config) const;

	/** @brief Check whether the vehicle is still close enough to the selected branch-off leg, in crosstrack and altitude. */
	bool closeToBranchOffSegment(const Position &position,
				     const GoalSelection &selection,
				     float acceptance_radius,
				     float altitude_acceptance_radius) const;

	/**
	 * @brief Skip planned route altitude when the first route target is the mission
	 * land item in nominal direction, or the takeoff item in reverse direction.
	 */
	bool canUseCurrentAltitudeForJoinTarget(const RoutePath &path) const;

private:
	const Provider &_provider;
	const MissionRouteProjection &_projection;

	bool mustFlyReverse(float goal_route_along,
			    float projection_route_along,
			    PathDirectionMode direction_mode) const;

	/** @brief Compute the desired course vector used for fixed-wing U-turn detection. */
	matrix::Vector2f computeDesiredCourseVector(const ProjectionContext &projection_context,
			float acceptance_radius,
			bool will_fly_reverse) const;

	bool uTurnRequired(const ProjectionContext &projection_context,
			   const PlannerConfig &config,
			   bool will_fly_reverse) const;

	RoutePath solveShortestRoutePath(float goal_route_along,
					 const ProjectionContext &projection_context,
					 const PlannerConfig &config,
					 PathDirectionMode direction_mode) const;

	RoutePath solveShortestRoutePathFromActiveLoop(float goal_route_along,
			const ProjectionContext &projection_context,
			const PlannerConfig &config,
			PathDirectionMode direction_mode) const;

	bool closeToSafePointDirect(const Position &vehicle_position,
				    const Position &safe_point_position,
				    const PlannerConfig &config) const;

	/** @brief Score one branch-off candidate. The returned path is invalid when the candidate is unusable. */
	RoutePath scoreBranchOffCandidate(const ProjectionContext &projection_context,
					  const PlannerConfig &config,
					  const RouteProjectionCandidate &branch_off) const;

	GoalSelection selectLowestCostSafePoint(const ProjectionContext &projection_context,
						const PlannerConfig &config,
						const ProjectionReferenceBatch &batch) const;

	GoalSelection selectMissionEndpointFallback(const ProjectionContext &projection_context,
			const PlannerConfig &config) const;
};

} // namespace mission_route
