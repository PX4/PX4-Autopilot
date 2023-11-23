/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * @file FigureEight.cpp
 * Helper class for fixed wing position controller when flying a figure 8 loiter pattern.
 */

#include "FigureEight.hpp"

#include "lib/geo/geo.h"
#include <lib/matrix/matrix/math.hpp>

using namespace matrix;

static constexpr float NORMALIZED_MAJOR_RADIUS{1.0f};
static constexpr bool NORTH_CIRCLE_IS_COUNTER_CLOCKWISE{false};
static constexpr bool SOUTH_CIRCLE_IS_COUNTER_CLOCKWISE{true};
static constexpr float DEFAULT_MAJOR_TO_MINOR_AXIS_RATIO{2.5f};
static constexpr float MINIMAL_FEASIBLE_MAJOR_TO_MINOR_AXIS_RATIO{2.0f};

FigureEight::FigureEight(NPFG &npfg, matrix::Vector2f &wind_vel, float &eas2tas) :
	ModuleParams(nullptr),
	_npfg(npfg),
	_wind_vel(wind_vel),
	_eas2tas(eas2tas)
{

}

void FigureEight::resetPattern()
{
	// Set the current segment invalid
	_current_segment = FigureEightSegment::SEGMENT_UNDEFINED;
	_pos_passed_circle_center_along_major_axis = false;
}

void FigureEight::updateSetpoint(const matrix::Vector2f &curr_pos_local, const matrix::Vector2f &ground_speed,
				 const FigureEightPatternParameters &parameters, float target_airspeed)
{
	// Sanitize inputs
	FigureEightPatternParameters valid_parameters{sanitizeParameters(parameters)};

	// Calculate the figure eight pattern points.
	FigureEightPatternPoints pattern_points;
	calculateFigureEightPoints(pattern_points, valid_parameters);

	// Do the figure of eight initialization if needed.
	initializePattern(curr_pos_local, ground_speed, valid_parameters, pattern_points);

	// Check if we need to switch to next segment
	updateSegment(curr_pos_local, valid_parameters,  pattern_points);

	// Apply control logic based on segment
	applyControl(curr_pos_local, ground_speed, valid_parameters, target_airspeed, pattern_points);
}

FigureEight::FigureEightPatternParameters FigureEight::sanitizeParameters(const FigureEightPatternParameters
		&parameters)
{
	FigureEightPatternParameters valid_parameters{parameters};

	if (!PX4_ISFINITE(parameters.loiter_minor_radius)) {
		valid_parameters.loiter_minor_radius = fabsf(_param_nav_loiter_rad.get());
	}

	if (!PX4_ISFINITE(parameters.loiter_radius)) {
		valid_parameters.loiter_radius = DEFAULT_MAJOR_TO_MINOR_AXIS_RATIO * valid_parameters.loiter_minor_radius;
		valid_parameters.loiter_direction_counter_clockwise = _param_nav_loiter_rad.get() < 0;
	}

	valid_parameters.loiter_radius = math::max(valid_parameters.loiter_radius,
					 MINIMAL_FEASIBLE_MAJOR_TO_MINOR_AXIS_RATIO * valid_parameters.loiter_minor_radius);

	return valid_parameters;
}

void FigureEight::initializePattern(const matrix::Vector2f &curr_pos_local, const matrix::Vector2f &ground_speed,
				    const FigureEightPatternParameters &parameters, FigureEightPatternPoints pattern_points)
{
	// Initialize the currently active segment, if it hasn't been active yet, or the pattern has been changed.
	if ((_current_segment == FigureEightSegment::SEGMENT_UNDEFINED) || (_active_parameters != parameters)) {
		Vector2f center_to_pos_local;
		calculatePositionToCenterNormalizedRotated(center_to_pos_local, curr_pos_local, parameters);
		Vector2f ground_speed_rotated = Dcm2f(-calculateRotationAngle(parameters)) * ground_speed;

		Vector2f north_center_to_pos_local{center_to_pos_local - pattern_points.normalized_north_circle_offset};
		Vector2f south_center_to_pos_local{center_to_pos_local - pattern_points.normalized_south_circle_offset};
		const bool north_is_closer = north_center_to_pos_local.norm() < south_center_to_pos_local.norm();

		// Get the normalized switch distance.
		float switch_distance_normalized = _npfg.switchDistance(FLT_MAX) * NORMALIZED_MAJOR_RADIUS / parameters.loiter_radius;

		//Far away from current figure of eight. Fly towards closer circle

		if (center_to_pos_local.norm() > NORMALIZED_MAJOR_RADIUS + switch_distance_normalized) {
			if (north_is_closer) {
				_current_segment = FigureEightSegment::SEGMENT_CIRCLE_NORTH;

			} else {
				_current_segment = FigureEightSegment::SEGMENT_CIRCLE_SOUTH;
			}

			_pos_passed_circle_center_along_major_axis = true;

		} else {
			if (north_is_closer) {
				const bool is_circling_counter_clockwise{north_center_to_pos_local.cross(ground_speed_rotated) < 0.f};

				if ((ground_speed_rotated(0) > 0.f) && (is_circling_counter_clockwise == NORTH_CIRCLE_IS_COUNTER_CLOCKWISE)) {
					// Flying north and right rotation
					_current_segment = FigureEightSegment::SEGMENT_CIRCLE_NORTH;
					_pos_passed_circle_center_along_major_axis = true;

				} else {
					// Flying to the entry of the south circle
					_current_segment = FigureEightSegment::SEGMENT_POINT_SOUTHWEST;
					_pos_passed_circle_center_along_major_axis = false;
				}

			} else {
				const bool is_circling_counter_clockwise{south_center_to_pos_local.cross(ground_speed_rotated) < 0.f};

				if ((ground_speed_rotated(0) < 0.f) && (is_circling_counter_clockwise == SOUTH_CIRCLE_IS_COUNTER_CLOCKWISE)) {
					// Flying south and right rotation
					_current_segment = FigureEightSegment::SEGMENT_CIRCLE_SOUTH;
					_pos_passed_circle_center_along_major_axis = true;

				} else {
					// Flying to the entry of the north circle
					_current_segment = FigureEightSegment::SEGMENT_POINT_NORTHWEST;
					_pos_passed_circle_center_along_major_axis = false;
				}
			}
		}

		_active_parameters = parameters;
	}
}

void FigureEight::calculateFigureEightPoints(FigureEightPatternPoints &pattern_points,
		const FigureEightPatternParameters &parameters)
{
	const float normalized_minor_radius = (parameters.loiter_minor_radius / parameters.loiter_radius) *
					      NORMALIZED_MAJOR_RADIUS;
	const float cos_transition_angle = parameters.loiter_minor_radius / (parameters.loiter_radius -
					   parameters.loiter_minor_radius);
	const float sin_transition_angle = sqrtf(1.0f - cos_transition_angle * cos_transition_angle);
	pattern_points.normalized_north_circle_offset = Vector2f{NORMALIZED_MAJOR_RADIUS - normalized_minor_radius, 0.0f};
	pattern_points.normalized_north_entry_offset =  Vector2f{NORMALIZED_MAJOR_RADIUS - ((normalized_minor_radius) * (1.0f + cos_transition_angle)),
								 -normalized_minor_radius * sin_transition_angle};
	pattern_points.normalized_north_exit_offset = Vector2f{NORMALIZED_MAJOR_RADIUS - ((normalized_minor_radius) * (1.0f + cos_transition_angle)),
							       normalized_minor_radius * sin_transition_angle};
	pattern_points.normalized_south_circle_offset = Vector2f{-NORMALIZED_MAJOR_RADIUS + normalized_minor_radius, 0.0f};
	pattern_points.normalized_south_entry_offset = Vector2f{-NORMALIZED_MAJOR_RADIUS + ((normalized_minor_radius) * (1.0f + cos_transition_angle)),
								-normalized_minor_radius * sin_transition_angle};
	pattern_points.normalized_south_exit_offset = Vector2f{-NORMALIZED_MAJOR_RADIUS + ((normalized_minor_radius) * (1.0f + cos_transition_angle)),
							       normalized_minor_radius * sin_transition_angle};
}

void FigureEight::updateSegment(const matrix::Vector2f &curr_pos_local, const FigureEightPatternParameters &parameters,
				const FigureEightPatternPoints &pattern_points)
{
	Vector2f center_to_pos_local;
	calculatePositionToCenterNormalizedRotated(center_to_pos_local, curr_pos_local, parameters);

	// Get the normalized switch distance.
	float switch_distance_normalized = _npfg.switchDistance(FLT_MAX) * NORMALIZED_MAJOR_RADIUS / parameters.loiter_radius;

	// Update segment if segment exit condition has been reached
	switch (_current_segment) {
	case FigureEightSegment::SEGMENT_CIRCLE_NORTH: {
			if (center_to_pos_local(0) > pattern_points.normalized_north_circle_offset(0)) {
				_pos_passed_circle_center_along_major_axis = true;
			}

			Vector2f vector_to_exit_normalized = pattern_points.normalized_north_exit_offset - center_to_pos_local;

			/* Exit condition: Switch distance away from north-east point of north circle and at least once was above the circle center. Failsafe action, if poor tracking,
			-                       switch to next if the vehicle is on the east side and below the north exit point. */
			if (_pos_passed_circle_center_along_major_axis &&
			    ((vector_to_exit_normalized.norm() < switch_distance_normalized) ||
			     ((center_to_pos_local(0) < pattern_points.normalized_north_exit_offset(0)) &&
			      (center_to_pos_local(1) > FLT_EPSILON) &&
			      (center_to_pos_local.norm() < NORMALIZED_MAJOR_RADIUS)))) {
				_current_segment = FigureEightSegment::SEGMENT_NORTHEAST_SOUTHWEST;
			}
		}
		break;

	case FigureEightSegment::SEGMENT_POINT_SOUTHWEST: // fall through
	case FigureEightSegment::SEGMENT_NORTHEAST_SOUTHWEST: {
			_pos_passed_circle_center_along_major_axis = false;
			Vector2f vector_to_exit_normalized = pattern_points.normalized_south_entry_offset - center_to_pos_local;

			/* Exit condition: Switch distance away from south-west point of south circle. Failsafe action, if poor tracking,
			switch to next if the vehicle is on the west side and below entry point of the south circle or has left the radius. */
			if ((vector_to_exit_normalized.norm() < switch_distance_normalized) ||
			    ((center_to_pos_local(0) < pattern_points.normalized_south_entry_offset(0)) && (center_to_pos_local(1) < FLT_EPSILON))
			    ||
			    (center_to_pos_local(0) < -NORMALIZED_MAJOR_RADIUS)) {
				_current_segment = FigureEightSegment::SEGMENT_CIRCLE_SOUTH;
			}
		}
		break;

	case FigureEightSegment::SEGMENT_CIRCLE_SOUTH: {
			if (center_to_pos_local(0) < pattern_points.normalized_south_circle_offset(0)) {
				_pos_passed_circle_center_along_major_axis = true;
			}

			Vector2f vector_to_exit_normalized = pattern_points.normalized_south_exit_offset - center_to_pos_local;

			/* Exit condition: Switch distance away from south-east point of south circle and at least once was below the circle center. Failsafe action, if poor tracking,
			-                       switch to next if the vehicle is on the east side and above the south exit point. */
			if (_pos_passed_circle_center_along_major_axis &&
			    ((vector_to_exit_normalized.norm() < switch_distance_normalized) ||
			     ((center_to_pos_local(0) > pattern_points.normalized_south_exit_offset(0)) &&
			      (center_to_pos_local(1) > FLT_EPSILON) &&
			      (center_to_pos_local.norm() < NORMALIZED_MAJOR_RADIUS)))) {
				_current_segment = FigureEightSegment::SEGMENT_SOUTHEAST_NORTHWEST;
			}

		}
		break;

	case FigureEightSegment::SEGMENT_POINT_NORTHWEST: // Fall through
	case FigureEightSegment::SEGMENT_SOUTHEAST_NORTHWEST: {
			_pos_passed_circle_center_along_major_axis = false;
			Vector2f vector_to_exit_normalized = pattern_points.normalized_north_entry_offset - center_to_pos_local;

			/* Exit condition: Switch distance away from north-west point of north circle. Failsafe action, if poor tracking,
			switch to next if the vehicle is on the west side and above entry point of the north circle or has left the radius. */
			if ((vector_to_exit_normalized.norm() < switch_distance_normalized) ||
			    ((center_to_pos_local(0) > pattern_points.normalized_north_entry_offset(0)) && (center_to_pos_local(1) < FLT_EPSILON))
			    ||
			    (center_to_pos_local(0) > NORMALIZED_MAJOR_RADIUS)) {
				_current_segment = FigureEightSegment::SEGMENT_CIRCLE_NORTH;
			}
		}
		break;

	case FigureEightSegment::SEGMENT_UNDEFINED:
	default:
		break;
	}
}

void FigureEight::applyControl(const matrix::Vector2f &curr_pos_local, const matrix::Vector2f &ground_speed,
			       const FigureEightPatternParameters &parameters, float target_airspeed,
			       const FigureEightPatternPoints &pattern_points)
{
	Vector2f center_to_pos_local;
	calculatePositionToCenterNormalizedRotated(center_to_pos_local, curr_pos_local, parameters);

	switch (_current_segment) {
	case FigureEightSegment::SEGMENT_CIRCLE_NORTH: {
			applyCircle(NORTH_CIRCLE_IS_COUNTER_CLOCKWISE, pattern_points.normalized_north_circle_offset, curr_pos_local,
				    ground_speed, parameters, target_airspeed);
		}
		break;

	case FigureEightSegment::SEGMENT_NORTHEAST_SOUTHWEST: {
			// Follow path from north-east to south-west
			applyLine(pattern_points.normalized_north_exit_offset, pattern_points.normalized_south_entry_offset, curr_pos_local,
				  ground_speed, parameters, target_airspeed);
		}
		break;

	case FigureEightSegment::SEGMENT_CIRCLE_SOUTH: {
			applyCircle(SOUTH_CIRCLE_IS_COUNTER_CLOCKWISE, pattern_points.normalized_south_circle_offset, curr_pos_local,
				    ground_speed, parameters, target_airspeed);
		}
		break;

	case FigureEightSegment::SEGMENT_SOUTHEAST_NORTHWEST: {
			// follow path from south-east to north-west
			applyLine(pattern_points.normalized_south_exit_offset, pattern_points.normalized_north_entry_offset, curr_pos_local,
				  ground_speed, parameters, target_airspeed);
		}
		break;

	case FigureEightSegment::SEGMENT_POINT_SOUTHWEST: {
			// Follow path from current position to south-west
			applyLine(center_to_pos_local, pattern_points.normalized_south_entry_offset, curr_pos_local,
				  ground_speed, parameters, target_airspeed);
		}
		break;

	case FigureEightSegment::SEGMENT_POINT_NORTHWEST: {
			// Follow path from current position to north-west
			applyLine(center_to_pos_local, pattern_points.normalized_north_entry_offset, curr_pos_local,
				  ground_speed, parameters, target_airspeed);
		}
		break;

	case FigureEightSegment::SEGMENT_UNDEFINED:
	default:
		break;
	}
}

void FigureEight::calculatePositionToCenterNormalizedRotated(matrix::Vector2f &center_to_pos_local_normalized_rotated,
		const matrix::Vector2f &curr_pos_local, const FigureEightPatternParameters &parameters) const
{
	Vector2f center_to_pos_local = curr_pos_local - parameters.center_pos_local;

	// normalize position with respect to radius
	Vector2f center_to_pos_local_normalized;
	center_to_pos_local_normalized(0) = center_to_pos_local(0) * NORMALIZED_MAJOR_RADIUS / parameters.loiter_radius;
	center_to_pos_local_normalized(1) = center_to_pos_local(1) * NORMALIZED_MAJOR_RADIUS / parameters.loiter_radius;

	// rotate position with respect to figure eight orientation and direction.
	center_to_pos_local_normalized_rotated = Dcm2f(-calculateRotationAngle(parameters)) * center_to_pos_local_normalized;
}

float FigureEight::calculateRotationAngle(const FigureEightPatternParameters &parameters) const
{
	// rotate position with respect to figure eight orientation and direction.
	float yaw_rotation = parameters.loiter_orientation;

	// figure eight pattern is symmetric, changing the direction is the same as a rotation by 180° around center
	if (parameters.loiter_direction_counter_clockwise) {
		yaw_rotation += M_PI_F;
	}

	return yaw_rotation;
}

void FigureEight::applyCircle(bool loiter_direction_counter_clockwise, const matrix::Vector2f &normalized_circle_offset,
			      const matrix::Vector2f &curr_pos_local, const matrix::Vector2f &ground_speed,
			      const FigureEightPatternParameters &parameters, float target_airspeed)
{
	const float loiter_direction_multiplier = loiter_direction_counter_clockwise ? -1.f : 1.f;

	Vector2f circle_offset = normalized_circle_offset * (parameters.loiter_radius / NORMALIZED_MAJOR_RADIUS);
	Vector2f circle_offset_rotated = Dcm2f(calculateRotationAngle(parameters)) * circle_offset;
	Vector2f circle_center = parameters.center_pos_local + circle_offset_rotated;

	_npfg.setAirspeedNom(target_airspeed * _eas2tas);
	_npfg.setAirspeedMax(_param_fw_airspd_max.get() * _eas2tas);

	const Vector2f vector_center_to_vehicle = curr_pos_local - circle_center;
	const float dist_to_center = vector_center_to_vehicle.norm();

	Vector2f unit_vec_center_to_closest_pt = vector_center_to_vehicle.normalized();

	if (dist_to_center < 0.1f) {
		// the logic breaks down at the circle center, employ some mitigation strategies
		// until we exit this region
		if (ground_speed.norm() < 0.1f) {
			// arbitrarily set the point in the northern top of the circle
			unit_vec_center_to_closest_pt = Vector2f{1.0f, 0.0f};

		} else {
			// set the point in the direction we are moving
			unit_vec_center_to_closest_pt = ground_speed.normalized();
		}
	}

	const Vector2f unit_path_tangent = loiter_direction_multiplier * Vector2f{-unit_vec_center_to_closest_pt(1), unit_vec_center_to_closest_pt(0)};

	float path_curvature = loiter_direction_multiplier / parameters.loiter_minor_radius;
	_target_bearing = atan2f(unit_path_tangent(1), unit_path_tangent(0));
	_closest_point_on_path = unit_vec_center_to_closest_pt * parameters.loiter_minor_radius + circle_center;
	_npfg.guideToPath(curr_pos_local, ground_speed, _wind_vel, unit_path_tangent,
			  _closest_point_on_path, path_curvature);

	_roll_setpoint = _npfg.getRollSetpoint();
	_indicated_airspeed_setpoint = _npfg.getAirspeedRef() / _eas2tas;
}

void FigureEight::applyLine(const matrix::Vector2f &normalized_line_start_offset,
			    const matrix::Vector2f &normalized_line_end_offset, const matrix::Vector2f &curr_pos_local,
			    const matrix::Vector2f &ground_speed, const FigureEightPatternParameters &parameters, float target_airspeed)
{
	const Dcm2f rotation_matrix(calculateRotationAngle(parameters));

	// Calculate start offset depending on radius
	const Vector2f start_offset = normalized_line_start_offset * (parameters.loiter_radius / NORMALIZED_MAJOR_RADIUS);
	const Vector2f start_offset_rotated = rotation_matrix * start_offset;
	const Vector2f line_segment_start_position = parameters.center_pos_local + start_offset_rotated;

	const Vector2f end_offset = normalized_line_end_offset * (parameters.loiter_radius / NORMALIZED_MAJOR_RADIUS);
	const Vector2f end_offset_rotated = rotation_matrix * end_offset;
	const Vector2f line_segment_end_position = parameters.center_pos_local + end_offset_rotated;

	_npfg.setAirspeedNom(target_airspeed * _eas2tas);
	_npfg.setAirspeedMax(_param_fw_airspd_max.get() * _eas2tas);
	const Vector2f path_tangent = line_segment_end_position - line_segment_start_position;
	const Vector2f unit_path_tangent = path_tangent.normalized();
	_target_bearing = atan2f(unit_path_tangent(1), unit_path_tangent(0));
	const Vector2f vector_A_to_vehicle = curr_pos_local - line_segment_start_position;
	_closest_point_on_path = line_segment_start_position + vector_A_to_vehicle.dot(unit_path_tangent) * unit_path_tangent;
	_npfg.guideToPath(curr_pos_local, ground_speed, _wind_vel, path_tangent.normalized(), line_segment_start_position,
			  0.0f);
	_roll_setpoint = _npfg.getRollSetpoint();
	_indicated_airspeed_setpoint = _npfg.getAirspeedRef() / _eas2tas;
}
