/****************************************************************************
 *
 *   Copyright (c) 2023-2024 PX4 Development Team. All rights reserved.
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

#include "BoatGuidance.hpp"

#include <mathlib/math/Limits.hpp>

using namespace matrix;

BoatGuidance::BoatGuidance(ModuleParams *parent) : ModuleParams(parent)
{
	updateParams();

	_currentState = GuidanceState::kDriving;
}

void BoatGuidance::computeGuidance(float yaw, vehicle_local_position_s vehicle_local_position,
				   float dt)
{
	if (_position_setpoint_triplet_sub.updated()) {
		_position_setpoint_triplet_sub.copy(&_position_setpoint_triplet);
	}

	if (_vehicle_global_position_sub.updated()) {
		_vehicle_global_position_sub.copy(&_vehicle_global_position);
	}

	const matrix::Vector2d global_position(_vehicle_global_position.lat, _vehicle_global_position.lon);
	const matrix::Vector2d current_waypoint(_position_setpoint_triplet.current.lat, _position_setpoint_triplet.current.lon);
	const matrix::Vector2d next_waypoint(_position_setpoint_triplet.next.lat, _position_setpoint_triplet.next.lon);
	const matrix::Vector2d previous_waypoint(_position_setpoint_triplet.previous.lat,
			_position_setpoint_triplet.previous.lon);

	if (!_global_local_proj_ref.isInitialized()
	    || (_global_local_proj_ref.getProjectionReferenceTimestamp() != vehicle_local_position.timestamp)) {
		_global_local_proj_ref.initReference(vehicle_local_position.ref_lat, vehicle_local_position.ref_lon,
						     vehicle_local_position.timestamp);
	}

	const Vector2f current_waypoint_local_frame = _global_local_proj_ref.project(
				current_waypoint(0),
				current_waypoint(1));
	const Vector2f previous_waypoint_local_frame = _global_local_proj_ref.project(
				previous_waypoint(0),
				previous_waypoint(1));
	const Vector2f local_frame_position = Vector2f(vehicle_local_position.x, vehicle_local_position.y);

	const float distance_to_next_wp = get_distance_to_next_waypoint(
			global_position(0),
			global_position(1),
			current_waypoint(0),
			current_waypoint(1));

	float heading_error = 0.f;
	float speed_interpolation = 0.f;
	float heading_to_next_waypoint = 0.f;
	float heading_error_to_next_waypoint = 0.f;
	float desired_speed = _param_bt_spd_cruise.get();

	// Go back to driving, when the waypoint has been reached
	if (_current_waypoint != current_waypoint) {
		_currentState = GuidanceState::kDriving;
	}

	// Make boat stop when it arrives at the last waypoint
	if ((current_waypoint == next_waypoint) && distance_to_next_wp <= _param_nav_acc_rad.get()) {
		_currentState = GuidanceState::kGoalReached;
	}

	switch (_currentState) {
	case GuidanceState::kDriving: {
			if (PX4_ISFINITE(previous_waypoint(0)) && PX4_ISFINITE(previous_waypoint(1))) {

				float look_ahead_distance = getLookAheadDistance(
								    current_waypoint_local_frame,
								    previous_waypoint_local_frame,
								    local_frame_position
							    );

				float desired_heading = calcDesiredHeading(
								current_waypoint_local_frame,
								previous_waypoint_local_frame,
								local_frame_position,
								look_ahead_distance
							);

				heading_error = matrix::wrap_pi(desired_heading - yaw);
				_desired_angular_velocity = heading_error;

				heading_to_next_waypoint = get_bearing_to_next_waypoint(
								   previous_waypoint(0),
								   previous_waypoint(1),
								   current_waypoint(0),
								   current_waypoint(1)
							   );

			} else {
				_previous_local_position = local_frame_position;
				_previous_position = global_position;
				_currentState = GuidanceState::kDrivingToAPoint;
			}

			break;
		}

	// Control logic if there is no previous waypoint
	case GuidanceState::kDrivingToAPoint: {
			float look_ahead_distance = getLookAheadDistance(
							    current_waypoint_local_frame,
							    _previous_local_position,
							    local_frame_position
						    );

			float desired_heading = calcDesiredHeading(
							current_waypoint_local_frame,
							_previous_local_position,
							local_frame_position,
							look_ahead_distance
						);

			heading_error = matrix::wrap_pi(desired_heading - yaw);
			_desired_angular_velocity = heading_error;

			heading_to_next_waypoint = get_bearing_to_next_waypoint(
							   _previous_position(0),
							   _previous_position(1),
							   current_waypoint(0),
							   current_waypoint(1)
						   );
			break;
		}

	case GuidanceState::kGoalReached:
		desired_speed = 0.f;
		heading_error = 0.f;
		_desired_angular_velocity = 0.f;
		break;
	}

	heading_error_to_next_waypoint = matrix::wrap_pi(heading_to_next_waypoint - yaw);

	// Interpolate the speed based on the heading error
	if (PX4_ISFINITE(heading_error_to_next_waypoint) && desired_speed > 0.1f) {

		speed_interpolation = math::interpolate<float>(abs(heading_error_to_next_waypoint),
				      _param_bt_min_heading_error.get() * M_PI_F / 180.f,
				      _param_bt_max_heading_error.get() * M_PI_F / 180.f,
				      _param_bt_spd_cruise.get(),
				      _param_bt_spd_min.get());
		desired_speed = math::constrain(speed_interpolation, _param_bt_spd_min.get(), _max_speed);

	}

	boat_setpoint_s output{};
	output.speed = math::constrain(desired_speed, -_max_speed, _max_speed);
	output.yaw_rate = math::constrain(_desired_angular_velocity, -_max_angular_velocity, _max_angular_velocity);
	output.closed_loop_speed_control = true;
	output.closed_loop_yaw_rate_control = true;
	output.timestamp = hrt_absolute_time();

	_boat_setpoint_pub.publish(output);

	_current_waypoint = current_waypoint;
}

float BoatGuidance::getLookAheadDistance(const Vector2f &curr_wp_local, const Vector2f &prev_wp_local,
		const Vector2f &curr_pos_local)
{
	// Calculate crosstrack error
	const Vector2f prev_wp_to_curr_wp_local = curr_wp_local - prev_wp_local;

	if (prev_wp_to_curr_wp_local.norm() < FLT_EPSILON) { // Avoid division by 0 (this case should not happen)
		return 0.f;
	}

	const Vector2f prev_wp_to_curr_pos_local = curr_pos_local - prev_wp_local;
	const Vector2f distance_on_line_segment = ((prev_wp_to_curr_pos_local * prev_wp_to_curr_wp_local) /
			prev_wp_to_curr_wp_local.norm()) * prev_wp_to_curr_wp_local.normalized();
	const Vector2f crosstrack_error = (prev_wp_local + distance_on_line_segment) - curr_pos_local;

	if (crosstrack_error.length() < _param_look_ahead_distance.get()) {
		return _param_look_ahead_distance.get();

	} else {
		return crosstrack_error.length();
	}
}

float BoatGuidance::calcDesiredHeading(const Vector2f &curr_wp_local, const Vector2f &prev_wp_local,
				       Vector2f const &curr_pos_local, float const &lookahead_distance)
{
	// Setup variables
	const float line_segment_slope = (curr_wp_local(1) - prev_wp_local(1)) / (curr_wp_local(0) - prev_wp_local(0));
	const float line_segment_rover_offset = prev_wp_local(1) - curr_pos_local(1) + line_segment_slope * (curr_pos_local(
			0) - prev_wp_local(0));
	const float a = -line_segment_slope;
	const float c = -line_segment_rover_offset;
	const float r = lookahead_distance;
	const float x0 = -a * c / (a * a + 1.0f);
	const float y0 = -c / (a * a + 1.0f);

	// Calculate intersection points
	if (c * c > r * r * (a * a + 1.0f) + FLT_EPSILON) { // No intersection points exist
		return 0.f;

	} else if (abs(c * c - r * r * (a * a + 1.0f)) < FLT_EPSILON) { // One intersection point exists
		return atan2f(y0, x0);

	} else { // Two intersetion points exist
		const float d = r * r - c * c / (a * a + 1.0f);
		const float mult = sqrt(d / (a * a + 1.0f));
		const float ax = x0 + mult;
		const float bx = x0 - mult;
		const float ay = y0 - a * mult;
		const float by = y0 + a * mult;
		const Vector2f point1(ax, ay);
		const Vector2f point2(bx, by);
		const Vector2f distance1 = (curr_wp_local - curr_pos_local) - point1;
		const Vector2f distance2 = (curr_wp_local - curr_pos_local) - point2;

		// Return intersection point closer to current waypoint
		if (distance1.norm_squared() < distance2.norm_squared()) {
			return atan2f(ay, ax);

		} else {
			return atan2f(by, bx);
		}
	}
}

void BoatGuidance::updateParams()
{
	ModuleParams::updateParams();
}
