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
 * @file mission_route_internal_types.cpp
 *
 * Planner-internal type validation.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include "mission_route_internal_types.h"

namespace mission_route
{

bool ProjectionDistance::valid() const
{
	return PX4_ISFINITE(xtrack_m) && xtrack_m >= 0.f
	       && PX4_ISFINITE(along_route_m) && along_route_m >= 0.f
	       && PX4_ISFINITE(segment_length_m) && segment_length_m >= 0.f
	       && PX4_ISFINITE(along_segment_m) && along_segment_m >= 0.f
	       && along_segment_m < (segment_length_m + kRoundingToleranceM);
}

bool SegmentEndpoint::valid() const
{
	return idx >= 0 && nav_cmd != NAV_CMD_INVALID;
}

bool SegmentPositions::valid() const
{
	return start.valid() && end.valid();
}

bool Segment::valid() const
{
	const bool valid_jump = jump_item_index >= 0
				&& jump_item_index != start.idx
				&& jump_item_index != end.idx;
	const bool valid_nominal = jump_item_index == -1 && !has_remaining_repeats && start.idx < end.idx;

	return start.valid() && end.valid() && start.idx != end.idx && (valid_jump || valid_nominal);
}

bool Segment::validLoop() const
{
	return isLoop() && valid();
}

bool SegmentDistanceAlong::valid() const
{
	return PX4_ISFINITE(start_dist_along_route_m) && start_dist_along_route_m > -FLT_EPSILON
	       && PX4_ISFINITE(end_dist_along_route_m) && end_dist_along_route_m > -FLT_EPSILON;
}

bool RouteProjectionCandidate::valid() const
{
	return segment.valid() && segment_positions.valid() && projection.valid() && dist.valid();
}

bool RoutePath::valid() const
{
	return first_item.index >= 0 && first_item.nav_cmd != NAV_CMD_INVALID
	       && PX4_ISFINITE(total_cost_m) && total_cost_m >= 0.f;
}

} // namespace mission_route
