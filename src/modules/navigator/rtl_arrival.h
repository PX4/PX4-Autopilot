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

#include "navigation.h"
#include "safe_point_land.hpp"
#include <matrix/Vector2.hpp>
#include <uORB/topics/vehicle_status.h>

class MissionBlock;
class RtlTimeEstimator;

/** Destination arrival shared by RTL modes; the caller owns setpoints and final landing. */
class RtlArrival
{
public:
	enum class Phase {
		Move,
		Descend,
		Hold,
		Land
	};

	struct ItemConfig {
		loiter_point_s approach{};
		float transit_altitude{NAN};
		float land_delay{0.f};
		float yaw{NAN};
	};

	struct EstimateState {
		matrix::Vector2d position;
		float altitude;
	};

	struct EstimateContext {
		matrix::Vector2d direction_origin;
		float approach_altitude;
		float remaining_wait;
		uint8_t vehicle_type;
		bool include_descent_and_hold;
	};

	Phase phase() const { return _phase; }
	void reset() { _phase = Phase::Land; }
	void startDirectReturn();
	void startRouteArrival(bool use_selected_approach);

	/** Advance after the caller has accepted the current item. */
	void advance(const vehicle_status_s &vehicle, float land_delay);

	/** Update arrival fields without clearing caller-owned fields; Land leaves the item unchanged. */
	void buildMissionItem(const MissionBlock &builder, mission_item_s &item,
			      const ItemConfig &config, const vehicle_status_s &vehicle) const;

	/** Append unfinished arrival legs without changing execution; false means an indefinite hold prevents landing.
	 * The caller sets the estimator's vehicle type and adds its route and final landing legs.
	 */
	bool addTimeEstimate(RtlTimeEstimator &estimator, const ItemConfig &config,
			     const EstimateContext &context, EstimateState &state) const;

private:
	enum class Entry { Direct, Route };
	Entry _entry{Entry::Direct};
	Phase _phase{Phase::Land};
};
