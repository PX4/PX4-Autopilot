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
 * @file mission_route_geofence.cpp
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include "mission_route_geofence.h"

#include "geofence.h"
#include "mission_route_projection.h"

namespace mission_route
{
namespace
{

constexpr size_t kPathBatchSize = Geofence::MAX_PATH_CHECKS;
static_assert(kMaxSegmentCandidates <= 8, "Candidate validity must fit in a byte");

struct PathBatch {
	Geofence::PathCheck paths[kPathBatchSize];
	bool results[kPathBatchSize];
	uint8_t reference_indices[kPathBatchSize];
	uint8_t candidate_indices[kPathBatchSize];
	uint8_t valid_masks[kMaxSafePointBatch];
	size_t count{0};
	bool emergency_only{false}; ///< The paths of the batch are checked against the emergency geofence only.
};

bool checkBatch(Geofence &geofence, PathBatch &batch)
{
	if (batch.count == 0) {
		return true;
	}

	if (!geofence.checkPathBatch(batch.paths, batch.count, batch.results, batch.emergency_only)) {
		return false;
	}

	for (size_t i = 0; i < batch.count; ++i) {
		if (batch.results[i]) {
			batch.valid_masks[batch.reference_indices[i]] |= 1U << batch.candidate_indices[i];
		}
	}

	batch.count = 0;
	return true;
}

} // namespace

// The vehicle breaches a polygon or circle of the emergency geofence, for example 20 m outside an inclusion polygon.
// Every way back in crosses that boundary, so segments from the vehicle are accepted without a fence check.
// Too high, too far from Home or outside the contingency geofence is no recovery: those segments are checked.
bool GeofencePathChecker::recovering(const Position &vehicle) const
{
	return !_geofence.isInsideEmerGeofenceAreas(vehicle.lat, vehicle.lon);
}

// A vehicle that is already outside the contingency geofence cannot keep it: it is closer to the emergency geofence
// than GF_CONT_H_MARGIN, or outside a contingency geofence area. Its segments are checked against the emergency
// geofence only.
// Example: the Return that GF_CONT_ACTION starts there may fly back in, but not across the emergency geofence.
bool GeofencePathChecker::vehicleOutsideContGeofence(const Position &vehicle) const
{
	return !_geofence.isInsideContGeofence(vehicle.lat, vehicle.lon);
}

bool GeofencePathChecker::pruneCandidates(ProjectionReferenceBatch &batch) const
{
	if (!_geofence.isReadyForPathChecks()) {
		return false;
	}

	// One buffer for every call, off the task stack. All-zero defaults keep it in .bss.
	static PathBatch shared_paths{};
	shared_paths.count = 0;
	shared_paths.emergency_only = false;

	for (uint8_t i = 0; i < batch.count; ++i) {
		shared_paths.valid_masks[i] = 0;
	}

	for (uint8_t i = 0; i < batch.count; ++i) {
		const ProjectionReference &reference = batch.items[i];
		const Position &anchor = reference.position;

		if (reference.candidate_buffer.count == 0 || !anchor.valid()) {
			continue;
		}

		// The reference is the vehicle (no source index) or a safe point.
		const bool is_vehicle = reference.source_index < 0;

		if (is_vehicle) {
			// A vehicle in recovery keeps all its candidates unchecked: set bits 0 to count - 1.
			if (recovering(anchor)) {
				shared_paths.valid_masks[i] = static_cast<uint8_t>((1U << reference.candidate_buffer.count) - 1U);
				continue;
			}

		} else if (!_geofence.checkPointAgainstAllGeofences(anchor.lat, anchor.lon, anchor.alt)
			   || !_geofence.isCloserThanContMaxDistToHome(anchor.lat, anchor.lon)
			   || !_geofence.isInsideContGeofenceAreas(anchor.lat, anchor.lon)) {
			// A safe point outside the allowed area keeps no candidates: a segment entirely inside an exclusion
			// area would cross no boundary. Neither does one closer to GF_MAX_HOR_DIST than the margin.
			// The paths are checked against both levels, so the point is too.
			continue;
		}

		// Segments keep the contingency geofence, like the legs of a mission: GF_CONT_H_MARGIN from the polygons and
		// circles, or the contingency geofence areas that replace the margin.
		const bool emergency_only = is_vehicle && vehicleOutsideContGeofence(anchor);
		const float margin = emergency_only ? 0.f : _geofence.getContGeofencePlanMargin();

		// A batch is checked one way: check the paths collected so far first if this reference needs the other way.
		if (shared_paths.emergency_only != emergency_only) {
			if (!checkBatch(_geofence, shared_paths)) {
				return false;
			}

			shared_paths.emergency_only = emergency_only;
		}

		for (uint8_t j = 0; j < reference.candidate_buffer.count; ++j) {
			const Position &projection = reference.candidate_buffer.candidates[j].projection;

			if (!projection.valid()) {
				continue;
			}

			shared_paths.paths[shared_paths.count] = {{anchor.lat, anchor.lon}, {projection.lat, projection.lon}, 0.f, margin};
			shared_paths.reference_indices[shared_paths.count] = i;
			shared_paths.candidate_indices[shared_paths.count] = j;
			++shared_paths.count;

			if (shared_paths.count == kPathBatchSize && !checkBatch(_geofence, shared_paths)) {
				return false;
			}
		}
	}

	if (!checkBatch(_geofence, shared_paths)) {
		return false;
	}

	// Compact only after every chunk succeeds, preserving the cross-track order
	// and the mapping for references whose candidates span two path batches.
	for (uint8_t i = 0; i < batch.count; ++i) {
		ProjectionCandidateBuffer &candidates = batch.items[i].candidate_buffer;
		uint8_t retained = 0;

		for (uint8_t j = 0; j < candidates.count; ++j) {
			if (shared_paths.valid_masks[i] & (1U << j)) {
				candidates.candidates[retained++] = candidates.candidates[j];
			}
		}

		candidates.count = retained;
	}

	return true;
}

bool GeofencePathChecker::checkPath(const Position &vehicle, const Position &end, bool &clear) const
{
	clear = false;

	if (!_geofence.isReadyForPathChecks()) {
		return false;
	}

	if (!vehicle.valid() || !end.valid()) {
		return true;
	}

	// The same rules as for one vehicle candidate in pruneCandidates().
	if (recovering(vehicle)) {
		clear = true;
		return true;
	}

	const bool emergency_only = vehicleOutsideContGeofence(vehicle);
	const float margin = emergency_only ? 0.f : _geofence.getContGeofencePlanMargin();
	const Geofence::PathCheck path{{vehicle.lat, vehicle.lon}, {end.lat, end.lon}, 0.f, margin};
	return _geofence.checkPathBatch(&path, 1, &clear, emergency_only);
}

} // namespace mission_route
