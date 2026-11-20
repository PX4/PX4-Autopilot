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

#pragma once

#include "mission_route_path_checker.h"

class Geofence;

namespace mission_route
{

/**
 * Checks route segments against the polygons and circles of the loaded Navigator geofence.
 *
 * A segment that starts at the vehicle is checked by where the vehicle is:
 * - Inside the contingency geofence: the segment keeps it, like a mission leg. It stays further from the fence
 *   than GF_CONT_H_MARGIN, or inside the drawn contingency areas that replace the margin.
 * - Outside the contingency geofence but inside the emergency geofence: the segment is checked against the
 *   emergency geofence only, over its whole length. Example: the Return that GF_CONT_ACTION starts there may fly
 *   back through the margin, or back into a drawn contingency area, but not across the emergency geofence.
 * - Outside the emergency geofence (outside an inclusion area or inside an exclusion area): the segment is not
 *   checked, so that the vehicle can fly back in. This is the recovery. It applies to every checkPath() segment.
 * A segment that starts at a safe point always keeps the contingency geofence, also while the vehicle recovers.
 *
 * The altitude and Home limits are not checked on segments, and a vehicle that only breaches these is not in
 * recovery. Mission legs passed them in the mission check and the limits are convex, so points on the legs are
 * inside them: plan only on a checked mission. Safe points are checked against every limit.
 *
 * Calls must not overlap because they reuse the same scratch buffer; Navigator calls it from a single task.
 * Planning must still wait for the fence to finish loading.
 */
class GeofencePathChecker final : public PathChecker
{
public:
	explicit GeofencePathChecker(Geofence &geofence) : _geofence(geofence) {}

	bool pruneCandidates(ProjectionReferenceBatch &batch) const override;
	bool checkPath(const Position &vehicle, const Position &end, bool &clear) const override;

private:
	bool recovering(const Position &vehicle) const;
	bool vehicleOutsideContGeofence(const Position &vehicle) const;

	Geofence &_geofence;
};

} // namespace mission_route
