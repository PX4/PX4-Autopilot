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
 * @file mission_route_planner.h
 *
 * Route planner for safe-point RTL (RTL_TYPE = 6). Projects the vehicle
 * and every safe point onto the uploaded mission geometry, then selects
 * the goal with the lowest total return cost.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#pragma once

#include "navigation.h"
#include "mission_block.h"
#include "safe_point_land.hpp"

#include <float.h>
#include <math.h>
#include <stdint.h>

#include <matrix/math.hpp>
#include <px4_platform_common/defines.h>

class MissionRoutePlannerCandidateBufferTestPeer;

class MissionRoutePlanner
{
public:
	static constexpr float kRoundingToleranceM{0.1f};
	static constexpr double kNullIslandThresholdDeg{1e-7};
	static constexpr double kCornerLatLonTolDeg{1e-5};
	static constexpr float kLandApproachAssociationDistanceM{10.f};
	static constexpr uint8_t MAX_SEGMENT_CANDIDATES{3};
	/**
	 * Maximum eligible rally points evaluated per planning pass.
	 * The planner uses a fixed-size batch buffer, so extra eligible safe points are skipped after this limit.
	 * RtlStatus.msg also uses uint8_t for safe_point_index, so this must stay <= 255.
	 */
	static constexpr uint8_t MAX_SAFE_POINT_BATCH{64};

	enum class GoalType : uint8_t {
		None = 0,
		SafePoint,
		MissionLand,
		MissionTakeoff
	};

	enum class VtolTransitionAction : uint8_t {
		None = 0,
		FrontTransition = 1,
		BackTransition = 2
	};

	struct Position {
		double lat{NAN};
		double lon{NAN};
		float alt{NAN};

		/** @brief Validate a global route-planning position. */
		bool valid() const
		{
			return PX4_ISFINITE(lat) && PX4_ISFINITE(lon) && PX4_ISFINITE(alt)
			       && !((fabs(lat) < kNullIslandThresholdDeg) && (fabs(lon) < kNullIslandThresholdDeg))
			       && (fabs(lat) < 90.0) && (fabs(lon) < 180.0);
		}
	};

	struct PointDistance {
		float xtrack{NAN};
		float along{NAN};
		float segment_length{NAN};
		float on_segment{NAN};

		/** @brief Validate the along-track and cross-track distances of a projection. */
		bool valid() const
		{
			return PX4_ISFINITE(xtrack) && xtrack >= 0.f
			       && PX4_ISFINITE(along) && along >= 0.f
			       && PX4_ISFINITE(segment_length) && segment_length >= 0.f
			       && PX4_ISFINITE(on_segment) && on_segment >= 0.f
			       && on_segment < (segment_length + kRoundingToleranceM);
		}
	};

	struct SegmentEndpoint {
		int32_t idx{-1};
		uint16_t nav_cmd{NAV_CMD_INVALID};

		/** @brief Validate a mission segment endpoint descriptor. */
		bool valid() const
		{
			return idx >= 0 && nav_cmd != NAV_CMD_INVALID;
		}
	};

	struct Segment {
		SegmentEndpoint start{};
		SegmentEndpoint end{};
		bool is_loop{false}; /**< True when this is the synthetic DO_JUMP edge from the attached position to the jump target. */
		uint8_t loops_remaining{0}; /**< Remaining repeats on the DO_JUMP that created this segment; zero for nominal segments. */

		/** @brief Validate a mission segment descriptor. */
		bool valid() const
		{
			return start.valid() && end.valid() && start.idx != end.idx
			       && (is_loop || start.idx < end.idx);
		}

		/** @brief Validate that this descriptor contains a populated DO_JUMP loop segment. */
		bool validLoop() const
		{
			return is_loop && valid();
		}
	};

	struct SegmentDistanceAlong {
		float start{NAN};
		float end{NAN};

		/** @brief Validate the along-track interval of a segment. */
		bool valid() const
		{
			return PX4_ISFINITE(start) && start > -FLT_EPSILON
			       && PX4_ISFINITE(end) && end > -FLT_EPSILON;
		}
	};

	struct SegmentPositions {
		Position start{};
		Position end{};

		/** @brief Validate the start and end positions of a segment. */
		bool valid() const
		{
			return start.valid() && end.valid();
		}
	};

	struct SegmentCandidate {
		Segment segment{};
		SegmentPositions segment_positions{};
		Position projection{};
		PointDistance dist{};

		/** @brief Validate a concrete segment projection candidate. */
		bool valid() const
		{
			return segment.valid() && segment_positions.valid() && projection.valid() && dist.valid();
		}
	};

	struct CandidateBuffer {
		SegmentCandidate candidates[MAX_SEGMENT_CANDIDATES] {};
		uint8_t count{0};
	};

	struct LoopContext {
		Segment segment{};
		SegmentPositions segment_positions{};
		SegmentDistanceAlong along{};

		/** @brief Validate loop-specific context captured from a jump segment projection. */
		bool valid() const
		{
			return segment.validLoop() && along.valid() && segment_positions.valid();
		}
	};

	struct ProjectionContext {
		Position vehicle_pos{};
		int32_t mission_index{-1};
		SegmentCandidate seg_candidate{};
		bool is_flying_reverse{false};
		matrix::Vector2f vehicle_vel_ne{NAN, NAN};
		bool velocity_valid{false};
		float dist_along_to_route_end{0.f};
		uint8_t mission_loops_remaining{0};
		LoopContext loop_ctx{};

		/** @brief Validate the vehicle projection context used for route planning. */
		bool valid() const
		{
			return vehicle_pos.valid() && seg_candidate.valid();
		}
	};

	struct Path {
		bool direction_reversed{false};
		bool u_turn_required{false};
		bool in_first_item_acc_rad{false};
		int32_t first_item_index{-1};
		uint16_t first_item_cmd{NAV_CMD_INVALID};
		float dist{FLT_MAX};

		/** @brief Validate a mission path decision. */
		bool valid() const
		{
			return first_item_index >= 0 && first_item_cmd != NAV_CMD_INVALID && PX4_ISFINITE(dist);
		}
	};

	struct Selection {
		Path path{};
		bool found{false};
		bool safe_point_found{false};
		bool direct_to_safe_point{false};
		int32_t safe_point_index{-1};
		GoalType goal_type{GoalType::None};
		Segment branch_off_segment{};
		Position branch_off_projection{};
		Position safe_point_position{};
		Position goal_position{};

		bool valid() const
		{
			if (!found || !path.valid() || goal_type == GoalType::None || !goal_position.valid()) {
				return false;
			}

			if (!safe_point_found) {
				return goal_type == GoalType::MissionLand || goal_type == GoalType::MissionTakeoff;
			}

			return goal_type == GoalType::SafePoint
			       && safe_point_index >= 0
			       && branch_off_segment.valid()
			       && branch_off_projection.valid()
			       && safe_point_position.valid();
		}

		/** @brief Return the mission index where the vehicle branches away from the mission path. */
		int32_t branchOffIndex() const
		{
			if (branch_off_segment.valid()) {
				return path.direction_reversed ? branch_off_segment.start.idx : branch_off_segment.end.idx;
			}

			return path.first_item_index;
		}
	};

	struct JoinContext {
		Position projection{};
		bool direction_reversed{false};
		bool skip_altitude_requirement{false}; /**< Execution-side override filled by MissionBase when JOIN_ROUTE is armed. */
		VtolTransitionAction transition_action{VtolTransitionAction::None}; /**< Execution-side VTOL transition filled by MissionBase when JOIN_ROUTE is armed. */

		bool valid() const { return projection.valid(); }
	};

	struct JoinPlan {
		ProjectionContext projection_context{};
		Path path{};
		JoinContext join_context{};

		bool valid() const { return projection_context.valid() && path.valid() && join_context.valid(); }
	};

	struct Plan {
		ProjectionContext projection_context{};
		JoinContext join_context{};
		Selection selection{};

		bool valid() const { return projection_context.valid() && join_context.valid() && selection.valid(); }
	};

	struct PlannerParameters {
		float vehicle_projection_search_dist{0.f};
		float safe_point_projection_search_dist{0.f};
		float acceptance_radius{0.f};
		float direct_acceptance_radius{0.f};
		float home_altitude_amsl{NAN};
		float u_turn_penalty_m{4000.f};
	};

	struct VehicleStateContext {
		bool is_multicopter{false};
		bool is_flying_reverse{false};
		matrix::Vector2f velocity_ne{NAN, NAN};
		bool velocity_valid{false};
		bool is_fixed_wing{false};
		bool in_transition_to_fw{false};
		bool require_vtol_approach{false};
	};

	struct ExecutionContext {
		Segment last_flown_loop_segment{}; /**< Optional cached DO_JUMP edge anchor; invalid when no active loop is being preserved. */
	};

	struct Config {
		PlannerParameters parameters{};
		VehicleStateContext state{};
		ExecutionContext execution{};
	};

	enum class FailureReason : uint8_t {
		None = 0,
		NoValidGlobalPos,
		NoValidWaypoints,
		NoValidPath,
		NoSegmentsFound,
		InternalError,
		NoLocalMinFound,
		PositionItemInvalid,
		NoValidCandidateFound,
		Unknown
	};

	/**
	 * @brief Abstraction for mission and safe-point data access.
	 *
	 * Production code uses MissionRouteCache as the provider and dataman abstraction
	 * for mission geometry, safe points, and mission-land items. Unit tests supply a
	 * VectorProvider backed by std::vector, which keeps the planner testable without
	 * any dataman or uORB dependencies.
	 */
	class Provider
	{
	public:
		virtual ~Provider() = default;

		virtual int missionCount() const = 0;
		virtual bool loadMissionItem(int index, mission_item_s &mission_item) const = 0;
		virtual int safePointCount() const = 0;
		virtual bool loadSafePointItem(int index, mission_item_s &safe_point_item) const = 0;
		virtual land_approaches_s readVtolLandApproaches(const PositionYawSetpoint &rtl_position,
				float home_altitude_amsl) const;
		virtual bool hasVtolLandApproach(const PositionYawSetpoint &rtl_position,
						 float home_altitude_amsl) const;
		virtual bool hasVtolLandApproach(int safe_point_index, float home_altitude_amsl) const;
		virtual bool anySafePointHasVtolLandApproach(float home_altitude_amsl) const;

		/** @brief Find the mission land item. Default scans backward for NAV_CMD_LAND / NAV_CMD_VTOL_LAND. */
		virtual bool getMissionLandItem(int32_t &index, mission_item_s &land_item) const
		{
			for (int i = missionCount() - 1; i >= 0; --i) {
				mission_item_s item{};

				if (loadMissionItem(i, item)
				    && (item.nav_cmd == NAV_CMD_LAND || item.nav_cmd == NAV_CMD_VTOL_LAND)) {
					index = i;
					land_item = item;
					return true;
				}
			}

			return false;
		}

		/** @brief Find the mission takeoff item. Default checks index 0 for NAV_CMD_TAKEOFF / NAV_CMD_VTOL_TAKEOFF. */
		virtual bool getMissionTakeoffItem(int32_t &index, mission_item_s &takeoff_item) const
		{
			// Assume that the mission starts with a takeoff item, only check index 0
			mission_item_s item{};

			if (missionCount() > 0 && loadMissionItem(0, item)
			    && (item.nav_cmd == NAV_CMD_TAKEOFF || item.nav_cmd == NAV_CMD_VTOL_TAKEOFF)) {
				index = 0;
				takeoff_item = item;
				return true;
			}

			return false;
		}
	};

	explicit MissionRoutePlanner(const Provider &provider) : _provider(provider) {}

	/** @brief Project the vehicle onto the mission route and choose the continuity-preserving branch-in candidate. */
	bool collectVehicleProjection(const Position &vehicle_position, int32_t mission_index,
				      const Config &config, ProjectionContext &projection_context,
				      FailureReason &failure_reason) const;
	/** @brief Build a nominal-direction path from the current projection to a mission goal. */
	Path findNominalPathToGoal(uint16_t goal_segment_end_idx, float goal_dist_along,
				   const ProjectionContext &projection_context,
				   const Config &config) const;
	/** @brief Build a reverse-direction path from the current projection to a mission goal. */
	Path findReversePathToGoal(uint16_t goal_segment_end_idx, float goal_dist_along,
				   const ProjectionContext &projection_context,
				   const Config &config) const;

	/** @brief Evaluate all valid safe points and choose the best route-follow return target. */
	Selection selectSafePoint(const ProjectionContext &projection_context, const Config &config) const;
	/** @brief Choose the RTL goal, preferring a safe point and falling back to a mission endpoint when needed. */
	Selection selectBestGoal(const ProjectionContext &projection_context, const Config &config) const;
	/** @brief Build the Mission-mode smart-rejoin plan back to the mission end using the nominal route direction. */
	bool planMissionResumeJoin(const Position &vehicle_position, int32_t mission_index,
				   const Config &config, JoinPlan &plan, FailureReason &failure_reason) const;
	/** @brief Build the full RTL plan: vehicle projection, join context, and selected goal. */
	bool planRouteToGoal(const Position &vehicle_position, int32_t mission_index,
			     const Config &config, Plan &plan, FailureReason &failure_reason) const;
	/** @brief Check whether the vehicle is still close enough to the cached branch-off leg to keep flying straight to the goal. */
	bool closeToBranchOffSegment(const Position &position, const Selection &selection,
				     float acceptance_radius) const;

	static float getAbsoluteAltitudeForMissionItem(const mission_item_s &mission_item, float home_altitude_amsl);
	static bool extractMissionPosition(const mission_item_s &mission_item, float home_altitude_amsl,
					   Position &position);
	static bool extractSafePointPosition(const mission_item_s &safe_point_item, float home_altitude_amsl,
					     Position &position);
	/** @brief Return a readable string for a planning failure reason. */
	static const char *failureReasonString(FailureReason failure_reason);
	/** @brief Return a readable string for a selected goal type. */
	static const char *goalTypeString(GoalType goal_type);

	struct SafePointBatchItem {
		Position position{};
		int32_t source_index{-1};
		CandidateBuffer candidate_buffer{};
	};

	struct SafePointBatch {
		uint8_t count{0};
		SafePointBatchItem items[MAX_SAFE_POINT_BATCH] {};
	};

private:
	friend class MissionRoutePlannerCandidateBufferTestPeer;

	struct ProjectionBatchOutputs {
		SegmentDistanceAlong dist_along_to_last_flown_segment{};
		float dist_along_to_route_end{0.f};
		uint8_t loops_remaining{0};
	};

	struct CandidateSearchState {
		bool prev_proj_on_end{true};
		bool proj_on_end_for_segment{false};
		float min_xtrack{FLT_MAX};
		float xtrack_limit{FLT_MAX};
	};

	struct BatchSearchState {
		CandidateSearchState candidate_states[MAX_SAFE_POINT_BATCH] {};
		uint32_t segments_processed{0};
		uint32_t local_min_found{0};
		uint32_t valid_candidate_found{0};
	};

	enum class PathDirectionMode : uint8_t {
		Auto = 0,
		ForceNominal,
		ForceReverse
	};

	/** @brief Convert one LOITER_TO_ALT safe-point item into a concrete landing-approach point. */
	static loiter_point_s makeVtolLandApproachPoint(const mission_item_s &mission_item, float home_altitude_amsl);
	/**
	 * @brief Scan the approach block attached to one rally point.
	 *
	 * A rally point owns all following NAV_CMD_LOITER_TO_ALT items until the next rally point.
	 * When @p vtol_land_approaches is null, the function returns as soon as one valid approach is found.
	 */
	static bool scanApproachesFollowingRallyPoint(const Provider &provider, int rally_point_index,
			float home_altitude_amsl, land_approaches_s *vtol_land_approaches = nullptr);

	/** @brief Read a mission item and return its attached valid position if it has one. */
	bool readValidMissionPosition(int index, float home_altitude_amsl, Position &position,
				      uint16_t *nav_cmd = nullptr) const;
	/** @brief Find the next mission index carrying a valid position item. */
	bool findNextValidPositionIndex(uint16_t start_index, float home_altitude_amsl,
					uint16_t &next_position_index) const;
	/** @brief Find the position item attached to or preceding the given mission index. */
	bool findAttachedValidPositionIndex(uint16_t start_index, float home_altitude_amsl,
					    uint16_t &attached_position_index) const;
	/** @brief Load the valid safe points that fit in the single planner pass. */
	void loadSafePointBatch(const Config &config, SafePointBatch &batch) const;
	/** @brief Clear all per-safe-point candidate buffers before running a new batch scan. */
	void resetSafePointBatchResults(SafePointBatch &batch) const;
	/** @brief Advance mission scanning state to the next position-bearing segment end. */
	bool prepareNextSegment(uint16_t index, Segment &segment, SegmentPositions &segment_positions,
				float home_altitude_amsl, FailureReason &failure_reason) const;
	/**
	 * @brief Evaluate one mission segment for one reference point and keep only locally minimal projections.
	 *
	 * This is the core corner-handling routine used by both vehicle and safe-point projection. It clamps the
	 * orthogonal projection to the segment, classifies near-corner hits, applies the legacy local-minimum rules,
	 * and maintains the shrinking cross-track candidate window.
	 */
	void processCandidateForSegment(const Position &reference_position, const Segment &segment,
					const SegmentPositions &segment_positions, float segment_length,
					const matrix::Vector2f &segment_vector, float total_dist,
					float extra_xtrack_dist, bool last_segment,
					bool segment_has_no_length, CandidateSearchState &state,
					uint32_t &local_min_found, uint32_t &valid_candidate_found,
					CandidateBuffer &candidate_buffer) const;
	/**
	 * @brief Scan the mission once and evaluate a batch of reference points against every position segment.
	 *
	 * The batch contains either a single vehicle position or up to MAX_SAFE_POINT_BATCH safe points. During the
	 * scan the function builds each geometric segment, applies the local-minimum projection filter for every batch
	 * item, keeps the closest projection candidates inside the caller's cross-track window, and reports route-level
	 * metadata such as total route length and the along-track bounds of the caller's current segment in outputs.
	 */
	bool findProjectionCandidatesBatch(int32_t mission_index, float home_altitude_amsl,
					   bool is_flying_reverse, float extra_xtrack_dist,
					   const Segment &last_flown_loop_segment,
					   SafePointBatch &batch, ProjectionBatchOutputs &outputs,
					   FailureReason &failure_reason) const;
	/** @brief Insert a segment candidate into the xtrack-sorted candidate buffer. */
	void insertCandidateSorted(CandidateBuffer &candidate_buffer, const SegmentCandidate &candidate) const;
	/** @brief Trim the projection candidate buffer once a tighter xtrack window is known. */
	void pruneProjectionCandidates(CandidateBuffer &candidate_buffer, float xtrack_limit) const;
	/** @brief Fill the along-track bounds of the last flown segment once they are reached during scanning. */
	bool fillDistAlongToLastFlownIfNecessary(int32_t mission_index, const Segment &segment_to_consider,
			bool is_flying_reverse, float total_dist, float segment_length,
			const Segment &last_flown_loop_segment,
			SegmentDistanceAlong &dist_along_to_last_flown_segment) const;
	/**
	 * @brief Return true when the segment projection is a local minimum.
	 *
	 * Three cases:
	 *
	 * 1. Interior projection (not on a corner) are always a local minimum.
	 *
	 * 2. Corner projection is a local minimum when both the previous and the current segment
	 *    project onto the same shared corner. This happens when two consecutive segments form
	 *    a V-shape and the reference point is closest to the apex.
	 *
	 *    The terminal route endpoint (last segment, proj_on_end) is also accepted because
	 *    there is no following segment to compare against.
	 *
	 * 3. DO_JUMP loop-edge corner projections are rejected because those corners
	 *    already belong to their nominal route segments, so only
	 *    interior projections on the loop jump segment are kept:
	 *
	 */
	bool localMinimumOnSegment(bool proj_on_start, bool proj_on_end, bool prev_proj_on_end,
				   bool jumping, bool last_segment) const;
	/** @brief Sanity-check a projection candidate before inserting it into the buffer. */
	bool validateCandidate(const SegmentCandidate &candidate) const;
	/** @brief Check whether a mission index lies inside a projected segment for the current travel direction. */
	bool isIndexInProjectionSegment(const Segment &projection_segment, int32_t mission_index,
					bool is_flying_reverse) const;
	/** @brief Accumulate the 2D mission distance between two position-bearing mission indices. */
	float accumulateRouteDistance(uint16_t from_index, uint16_t to_index, float home_altitude_amsl) const;
	/** @brief Build the loop-jump context used when the vehicle is projected onto a DO_JUMP segment. */
	LoopContext buildLoopContext(const SegmentCandidate &vehicle_projection, float home_altitude_amsl) const;
	/** @brief Compute the shorted path along the mission route, excluding any off-route branch-off leg. */
	Path findShortestPathAlongRoute(uint16_t goal_segment_end_idx, float goal_dist_along,
					const ProjectionContext &projection_context, const Config &config,
					PathDirectionMode direction_mode = PathDirectionMode::Auto) const;
	/** @brief Allow multicopters already close to a safe point to skip the route join and fly straight to it. */
	bool directToSafePoint(const Position &safe_point_position, const Position &vehicle_position,
			       const Config &config) const;
	/** @brief Force or infer the direction used to reach a goal from the projected vehicle location. */
	bool mustFlyReverse(float goal_dist_along, float projection_dist_along,
			    PathDirectionMode direction_mode) const;
	/** @brief Compute the desired course vector used for fixed-wing U-turn detection. */
	matrix::Vector2f computeDesiredCourseVector(const ProjectionContext &projection_context,
			bool will_fly_reverse) const;
	/** @brief Check whether a fixed-wing route change implies an immediate U-turn. */
	bool uTurnRequired(const ProjectionContext &projection_context, const Config &config,
			   bool will_fly_reverse) const;
	/** @brief Fall back to the closer mission endpoint when no safe point can be used. */
	Selection selectMissionEndpointFallback(const ProjectionContext &projection_context,
						const Config &config) const;
	/** @brief Clamp a mission index into the valid mission range before projection. */
	bool clampMissionIndex(int32_t &mission_index) const;

	const Provider &_provider;
};
