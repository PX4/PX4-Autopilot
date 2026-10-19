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

#include "rtl_arrival.h"
#include "mission_block.h"
#include <lib/rtl/rtl_time_estimator.h>

void RtlArrival::startDirectReturn()
{
	_entry = Entry::Direct;
	_phase = Phase::Move;
}

void RtlArrival::startRouteArrival(bool use_selected_approach)
{
	_entry = Entry::Route;
	_phase = use_selected_approach ? Phase::Descend : Phase::Move;
}

void RtlArrival::advance(const vehicle_status_s &vehicle, float land_delay)
{
	const bool multicopter = vehicle.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	const bool land_after_move = multicopter && (_entry == Entry::Direct
				     ? !(fabsf(land_delay) > FLT_EPSILON)
				     : !vehicle.in_transition_mode && fabsf(land_delay) <= FLT_EPSILON);

	switch (_phase) {
	case Phase::Move:
		_phase = land_after_move ? Phase::Land : Phase::Descend;
		break;

	case Phase::Descend:
		_phase = Phase::Hold;
		break;

	case Phase::Hold:
		_phase = Phase::Land;
		break;

	case Phase::Land:
		break;
	}
}

void RtlArrival::buildMissionItem(const MissionBlock &builder, mission_item_s &item,
				  const ItemConfig &config, const vehicle_status_s &vehicle) const
{
	const bool fixed_wing = vehicle.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING
				|| (_entry == Entry::Route && vehicle.in_transition_to_fw);
	const PositionYawSetpoint position{
		.lat = config.approach.lat,
		.lon = config.approach.lon,
		.alt = _phase == Phase::Move ? config.transit_altitude :
		math::min(config.approach.height_m, config.transit_altitude),
		.yaw = _entry == Entry::Direct ? config.yaw : NAN
	};

	switch (_phase) {
	case Phase::Move:
		if (fixed_wing) {
			// Accept arrival at the orbit rather than requiring flight through its centre.
			builder.setLoiterHoldMissionItem(item, position, 0.f, config.approach.loiter_radius_m);

		} else {
			builder.setMoveToPositionMissionItem(item, position);
		}

		break;

	case Phase::Descend:
		builder.setLoiterToAltMissionItem(item, position, config.approach.loiter_radius_m);
		break;

	case Phase::Hold:
		builder.setLoiterHoldMissionItem(item, position, config.land_delay, config.approach.loiter_radius_m);
		break;

	case Phase::Land:
		break;
	}
}

bool RtlArrival::addTimeEstimate(RtlTimeEstimator &estimator, const ItemConfig &config,
				 const EstimateContext &context, EstimateState &state) const
{
	if (_phase == Phase::Land) {
		return true;
	}

	const matrix::Vector2d approach_position{config.approach.lat, config.approach.lon};
	const float approach_height = math::min(config.approach.height_m, config.transit_altitude);
	const bool move_remaining = _phase == Phase::Move || (_entry == Entry::Route && _phase == Phase::Descend);

	if (move_remaining) {
		matrix::Vector2f direction{};
		get_vector_to_next_waypoint(context.direction_origin(0), context.direction_origin(1),
					    approach_position(0), approach_position(1), &direction(0), &direction(1));
		float distance = get_distance_to_next_waypoint(state.position(0), state.position(1),
				 approach_position(0), approach_position(1));

		if (_entry == Entry::Direct && context.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
			// Preserve direct RTL's existing signed-radius accounting.
			distance = math::max(0.f, distance - config.approach.loiter_radius_m);
		}

		const float altitude_change = _entry == Entry::Route ? context.approach_altitude - state.altitude : 0.f;
		estimator.addDistance(distance, direction, altitude_change);
		state.position = approach_position;

		if (_entry == Entry::Route) {
			state.altitude = context.approach_altitude;
		}
	}

	if (context.include_descent_and_hold && (_phase == Phase::Move || _phase == Phase::Descend)) {
		const float initial_altitude = _entry == Entry::Direct && _phase == Phase::Move
					       ? config.transit_altitude : state.altitude;
		estimator.addVertDistance(approach_height - initial_altitude);
		state.altitude = approach_height;
	}

	if (_entry == Entry::Direct) {
		// Direct RTL forecasts landing from the configured approach even during an active hold.
		state.position = approach_position;
		state.altitude = approach_height;
	}

	if (context.include_descent_and_hold) {
		estimator.addWait(context.remaining_wait);

		if (config.land_delay < -FLT_EPSILON) {
			return false;
		}
	}

	return true;
}
