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

constexpr size_t kCandidateCapacity = kMaxSafePointBatch * kMaxSegmentCandidates;
constexpr size_t kPathBatchSize = kCandidateCapacity < Geofence::MAX_PATH_CHECKS
				  ? kCandidateCapacity : Geofence::MAX_PATH_CHECKS;
static_assert(kMaxSegmentCandidates <= 8, "Candidate validity must fit in a byte");

struct PathBatch {
	Geofence::PathCheck paths[kPathBatchSize];
	bool results[kPathBatchSize];
	uint8_t reference_indices[kPathBatchSize];
	uint8_t candidate_indices[kPathBatchSize];
	uint8_t valid_masks[kMaxSafePointBatch];
	size_t count{0};
};

void clearCandidates(ProjectionReferenceBatch &batch)
{
	for (uint8_t i = 0; i < batch.count && i < kMaxSafePointBatch; ++i) {
		batch.items[i].candidate_buffer.count = 0;
	}
}

bool checkBatch(Geofence &geofence, PathBatch &batch)
{
	if (batch.count == 0) {
		return true;
	}

	if (!geofence.checkPathBatch(batch.paths, batch.count, batch.results)) {
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

GeofencePathChecker::GeofencePathChecker(Geofence &geofence, const Position &vehicle_position) :
	_geofence(geofence),
	_recovering(geofence.isReadyForPathChecks() && vehicle_position.valid()
		    && !geofence.checkPointAgainstAllGeofences(vehicle_position.lat, vehicle_position.lon, vehicle_position.alt))
{
}

bool GeofencePathChecker::withinScalarLimits(const Position &position) const
{
	return position.valid()
	       && _geofence.isCloserThanMaxDistToHome(position.lat, position.lon, position.alt)
	       && _geofence.isBelowMaxAltitude(position.alt)
	       && _geofence.isWithinAltitudeBand(position.alt);
}

bool GeofencePathChecker::pruneCandidates(ProjectionReferenceBatch &batch) const
{
	if (!_geofence.isReadyForPathChecks() || batch.count > kMaxSafePointBatch) {
		clearCandidates(batch);
		return false;
	}

	for (uint8_t i = 0; i < batch.count; ++i) {
		if (batch.items[i].candidate_buffer.count > kMaxSegmentCandidates) {
			clearCandidates(batch);
			return false;
		}
	}

	// One buffer for every call, off the task stack. All-zero defaults keep it in .bss.
	static PathBatch shared_paths{};
	shared_paths.count = 0;

	for (uint8_t i = 0; i < batch.count; ++i) {
		shared_paths.valid_masks[i] = 0;
	}

	const bool horizontal_fence_empty = _geofence.isEmpty();

	for (uint8_t i = 0; i < batch.count; ++i) {
		const ProjectionReference &reference = batch.items[i];
		const Position &anchor = reference.position;

		// Recovery only exempts the vehicle reference; safe point segments are still checked.
		if (_recovering && reference.source_index < 0) {
			shared_paths.valid_masks[i] = static_cast<uint8_t>((1U << reference.candidate_buffer.count) - 1U);
			continue;
		}

		// First check that this vehicle or safe point is in an allowed area.
		// A segment entirely inside an exclusion area would cross no boundary.
		if (reference.candidate_buffer.count == 0 || !anchor.valid()
		    || !_geofence.checkPointAgainstAllGeofences(anchor.lat, anchor.lon, anchor.alt)) {
			continue;
		}

		for (uint8_t j = 0; j < reference.candidate_buffer.count; ++j) {
			const Position &projection = reference.candidate_buffer.candidates[j].projection;

			// Segment checks handle horizontal fences. Check distance from Home and
			// altitude limits separately at the projected point.
			if (!withinScalarLimits(projection)) {
				continue;
			}

			if (horizontal_fence_empty) {
				shared_paths.valid_masks[i] |= 1U << j;
				continue;
			}

			shared_paths.paths[shared_paths.count] = {{anchor.lat, anchor.lon}, {projection.lat, projection.lon}};
			shared_paths.reference_indices[shared_paths.count] = i;
			shared_paths.candidate_indices[shared_paths.count] = j;
			++shared_paths.count;

			if (shared_paths.count == kPathBatchSize && !checkBatch(_geofence, shared_paths)) {
				clearCandidates(batch);
				return false;
			}
		}
	}

	if (!checkBatch(_geofence, shared_paths)) {
		clearCandidates(batch);
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

bool GeofencePathChecker::checkPath(const Position &start, const Position &end, bool &clear) const
{
	clear = false;

	if (!_geofence.isReadyForPathChecks()) {
		return false;
	}

	if (!start.valid() || !end.valid()) {
		return true;
	}

	if (_recovering) {
		clear = true;
		return true;
	}

	if (!_geofence.checkPointAgainstAllGeofences(start.lat, start.lon, start.alt) || !withinScalarLimits(end)) {
		return true;
	}

	if (_geofence.isEmpty()) {
		clear = true;
		return true;
	}

	const Geofence::PathCheck path{{start.lat, start.lon}, {end.lat, end.lon}};
	return _geofence.checkPathBatch(&path, 1, &clear);
}

} // namespace mission_route
