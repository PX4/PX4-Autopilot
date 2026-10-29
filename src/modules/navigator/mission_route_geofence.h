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
 * Checks route segments against the loaded Navigator geofence.
 *
 * Create one checker per planning call. Calls must not overlap because they reuse
 * the same scratch buffer; Navigator calls it from a single task.
 * When the vehicle breaches a polygon or circle, segments from the vehicle, including every
 * checkPath() segment, are not checked so it can recover. Safe point segments still are.
 * Outside only the altitude or Home limits, the vehicle may start segments, which are checked.
 * Planning must still wait for the fence to finish loading.
 */
class GeofencePathChecker final : public PathChecker
{
public:
	GeofencePathChecker(Geofence &geofence, const Position &vehicle_position);

	bool pruneCandidates(ProjectionReferenceBatch &batch) const override;
	bool checkReference(const ProjectionReference &reference, bool &clear) const override;
	bool checkCandidate(const ProjectionReference &reference, const RouteProjectionCandidate &candidate,
			    bool &clear) const override;
	bool checkPath(const Position &start, const Position &end, bool &clear) const override;

private:
	bool checkEndAndPath(const Position &start, const Position &end, bool &clear) const;
	bool exempt(const ProjectionReference &reference) const;
	bool referenceAllowed(const ProjectionReference &reference) const;
	bool vehicleAllowed(const Position &position) const;
	bool withinScalarLimits(const Position &position) const;

	Geofence &_geofence;
	bool _recovering{false};
};

} // namespace mission_route
