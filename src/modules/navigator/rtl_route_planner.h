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
 * @file rtl_route_planner.h
 *
 * Route planner for safe-point RTL (RTL_TYPE = 6).  Projects the vehicle
 * and every safe point onto the uploaded mission geometry, then selects
 * the goal with the shortest along-route cost.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#pragma once

#include "navigation.h"

#include <float.h>
#include <math.h>
#include <stdint.h>

#include <matrix/math.hpp>
#include <px4_platform_common/defines.h>

class RtlRoutePlanner
{
public:
	static constexpr float kRoundingToleranceM{0.1f};
	static constexpr double kNullIslandThresholdDeg{1e-7};
	static constexpr double kCornerLatLonTolDeg{1e-5};
	static constexpr uint8_t MAX_SEGMENT_CANDIDATES{3};
	/**
	 * Maximum safe points evaluated per planning cycle.
	 * NOTE: RtlStatus.msg uses uint8_t for safe_point_index, so this must stay <= 255.
	 * If raised above 255, the constrain() in rtl.cpp::setRtlTypeAndDestination will silently truncate.
	 */
	static constexpr uint8_t MAX_SAFE_POINT_BATCH{64};

	enum class GoalType : uint8_t {
		None = 0,
		SafePoint,
		MissionLand,
		MissionTakeoff
	};

	enum class TransitionAction : uint8_t {
		None = 0,
		FrontTransition,
		BackTransition
	};

	struct Position {
		double lat{NAN};
		double lon{NAN};
		float alt{NAN};

		bool valid() const;
	};

	struct PointDistance {
		float xtrack{NAN};
		float along{NAN};
		float segment_length{NAN};
		float on_segment{NAN};

		bool valid() const;
	};

	struct SegmentEndpoint {
		int32_t idx{-1};
		uint16_t nav_cmd{NAV_CMD_INVALID};

		bool valid() const;
	};

	struct Segment {
		SegmentEndpoint start{};
		SegmentEndpoint end{};
		bool is_loop{false};
		uint8_t loops_remaining{0};

		bool valid() const;
	};

	struct SegmentDistanceAlong {
		float start{NAN};
		float end{NAN};

		bool valid() const;
	};

	struct SegmentPositions {
		Position start{};
		Position end{};

		bool valid() const;
	};

	struct SegmentCandidate {
		Segment segment{};
		SegmentPositions segment_positions{};
		Position projection{};
		PointDistance dist{};

		bool valid() const;
	};

	struct CandidateBuffer {
		SegmentCandidate candidates[MAX_SEGMENT_CANDIDATES] {};
		uint8_t count{0};
	};

	struct LoopContext {
		Segment segment{};
		SegmentPositions segment_positions{};
		SegmentDistanceAlong along{};

		bool valid() const;
	};

	struct ProjectionContext {
		Position vehicle_pos{};
		int32_t mission_index{-1};
		SegmentCandidate projection{};
		bool is_flying_reverse{false};
		float vehicle_velocity_north{NAN};
		float vehicle_velocity_east{NAN};
		bool vehicle_velocity_valid{false};
		float dist_along_to_route_end{0.f};
		uint8_t mission_loops_remaining{0};
		LoopContext loop_ctx{};

		bool valid() const;
	};

	struct Path {
		bool direction_reversed{false};
		bool u_turn_required{false};
		bool in_first_item_acc_rad{false};
		int32_t first_item_index{-1};
		uint16_t first_item_cmd{NAV_CMD_INVALID};
		float dist{FLT_MAX};

		bool valid() const;
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

		bool valid() const;
		int32_t branchOffIndex() const;
	};

	struct JoinContext {
		Position projection{};
		bool vtol_back_transition_required{false};
		bool skip_altitude_requirement{false};

		bool valid() const { return projection.valid(); }
	};

	struct Plan {
		ProjectionContext projection_context{};
		JoinContext join_context{};
		Selection selection{};

		bool valid() const { return projection_context.valid() && join_context.valid() && selection.valid(); }
	};

	struct Config {
		float vehicle_projection_search_dist{0.f};
		float safe_point_projection_search_dist{0.f};
		float acceptance_radius{0.f};
		float direct_acceptance_radius{0.f};
		float home_altitude_amsl{NAN};
		bool is_multicopter{false};
		bool is_flying_reverse{false};
		float vehicle_velocity_north{NAN};
		float vehicle_velocity_east{NAN};
		bool vehicle_velocity_valid{false};
		bool vehicle_is_vtol{false};
		bool vehicle_is_fixed_wing{false};
		bool vehicle_in_transition_to_fw{false};
		float u_turn_penalty_m{4000.f};
		Segment last_flown_loop_segment{};
	};

	enum class FailureReason : uint8_t {
		None = 0,
		NoValidGlobalPos,
		NoValidWaypoints,
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
	 * Production code uses the RtlRoutePlannerProvider in rtl.cpp (reads from dataman).
	 * Unit tests supply a VectorProvider backed by std::vector, which keeps the planner
	 * testable without any dataman or uORB dependencies.
	 */
	class Provider
	{
	public:
		virtual ~Provider() = default;

		virtual int missionCount() const = 0;
		virtual bool loadMissionItem(int index, mission_item_s &mission_item) const = 0;
		virtual int safePointCount() const = 0;
		virtual bool loadSafePointItem(int index, mission_item_s &safe_point_item) const = 0;
	};

	explicit RtlRoutePlanner(const Provider &provider) : _provider(provider) {}

	/**
	 * @brief Scan the mission geometry for locally minimal projections of one reference point.
	 *
	 * The search keeps up to MAX_SEGMENT_CANDIDATES projections inside a shrinking cross-track window and
	 * can also return along-route bookkeeping for the last flown segment and route end.
	 */
	bool findProjectionCandidates(const Position &reference_position, int32_t mission_index,
				      float home_altitude_amsl,
				      bool is_flying_reverse, float extra_xtrack_dist,
				      CandidateBuffer &candidate_buffer,
				      SegmentDistanceAlong *dist_along_to_last_flown_segment,
				      float *dist_along_to_route_end,
				      uint8_t *loops_remaining,
				      FailureReason *failure_reason) const;

	/** @brief Project the vehicle onto the mission route and choose the continuity-preserving branch-in candidate. */
	bool collectVehicleProjection(const Position &vehicle_position, int32_t mission_index,
				      const Config &config, ProjectionContext &projection_context,
				      FailureReason *failure_reason) const;

	/** @brief Evaluate all valid safe points and choose the best route-follow return target. */
	Selection selectSafePoint(const ProjectionContext &projection_context, const Config &config) const;
	/** @brief Choose the SRP goal, preferring a safe point and falling back to a mission endpoint when needed. */
	Selection selectBestGoal(const ProjectionContext &projection_context, const Config &config) const;
	/** @brief Build the full SRP plan: vehicle projection, join context, and selected goal. */
	bool planRouteToGoal(const Position &vehicle_position, int32_t mission_index,
			     const Config &config, Plan &plan, FailureReason *failure_reason) const;
	/** @brief Check whether the vehicle is still close enough to the cached branch-off leg to keep flying straight to the goal. */
	bool closeToBranchOffSegment(const Position &position, const Selection &selection,
				     float acceptance_radius) const;
	/** @brief Determine whether entering the segment containing @p target_index requires a VTOL front- or back-transition. */
	TransitionAction transitionActionForTargetIndex(int32_t target_index, bool direction_reversed,
			const Config &config) const;

	static bool itemContainsPosition(const mission_item_s &mission_item);
	static float getAbsoluteAltitudeForMissionItem(const mission_item_s &mission_item, float home_altitude_amsl);
	static bool extractMissionPosition(const mission_item_s &mission_item, float home_altitude_amsl,
					   Position &position);
	static bool extractSafePointPosition(const mission_item_s &safe_point_item, float home_altitude_amsl,
					     Position &position);
	static const char *failureReasonString(FailureReason failure_reason);
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

	bool readValidMissionPosition(int index, float home_altitude_amsl, Position &position,
				      uint16_t *nav_cmd = nullptr) const;
	bool findNextValidPositionIndex(uint16_t start_index, float home_altitude_amsl,
					uint16_t &next_position_index) const;
	bool findAttachedValidPositionIndex(uint16_t start_index, float home_altitude_amsl,
					    uint16_t &attached_position_index) const;
	bool loadSafePointBatch(float home_altitude_amsl, SafePointBatch &batch) const;
	void resetSafePointBatchResults(SafePointBatch &batch) const;
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
	/** @brief Project a full batch of safe points in one mission scan while preserving loop and along-route bookkeeping. */
	bool findProjectionCandidatesBatch(int32_t mission_index, float home_altitude_amsl,
					   bool is_flying_reverse, float extra_xtrack_dist,
					   const Segment &last_flown_loop_segment,
					   SafePointBatch &batch,
					   SegmentDistanceAlong *dist_along_to_last_flown_segment,
					   float *dist_along_to_route_end,
					   uint8_t *loops_remaining,
					   FailureReason *failure_reason) const;
	void insertCandidateSorted(CandidateBuffer &candidate_buffer, const SegmentCandidate &candidate) const;
	void pruneProjectionCandidates(CandidateBuffer &candidate_buffer, float xtrack_limit) const;
	bool fillDistAlongToLastFlownIfNecessary(int32_t mission_index, const Segment &segment_to_consider,
			bool is_flying_reverse, float total_dist, float segment_length,
			const Segment &last_flown_loop_segment,
			SegmentDistanceAlong &dist_along_to_last_flown_segment) const;
	bool localMinimumOnSegment(bool proj_on_start, bool proj_on_end, bool prev_proj_on_end,
				   bool jumping, bool last_segment) const;
	bool validateCandidate(const SegmentCandidate &candidate) const;
	bool isIndexInProjectionSegment(const Segment &projection_segment, int32_t mission_index,
					bool is_flying_reverse) const;
	float accumulateRouteDistance(uint16_t from_index, uint16_t to_index, float home_altitude_amsl) const;
	LoopContext buildLoopContext(const SegmentCandidate &vehicle_projection, float home_altitude_amsl) const;
	Path findShortestPath(uint16_t goal_segment_end_idx, float goal_dist_along,
			      const ProjectionContext &projection_context, const Config &config,
			      PathDirectionMode direction_mode = PathDirectionMode::Auto) const;
	bool directToSafePoint(const Position &safe_point_position, const Position &vehicle_position,
			       const Config &config) const;
	bool mustFlyReverse(float goal_dist_along, float projection_dist_along,
			    PathDirectionMode direction_mode) const;
	void computeDesiredCourseVector(const ProjectionContext &projection_context, bool will_fly_reverse,
					float &desired_course_north, float &desired_course_east) const;
	bool uTurnRequired(const ProjectionContext &projection_context, const Config &config,
			   bool will_fly_reverse) const;
	Selection selectMissionEndpointFallback(const ProjectionContext &projection_context,
						const Config &config) const;
	bool clampMissionIndex(int32_t &mission_index) const;
	uint8_t getVtolStateAtAnchor(uint16_t anchor_index) const;
	bool findSegmentAnchorForTargetIndex(int32_t target_index, bool direction_reversed,
					     uint16_t &anchor_index) const;
	bool joinRequiresBackTransition(int32_t target_index, bool direction_reversed,
					const Config &config) const;

	const Provider &_provider;
};
