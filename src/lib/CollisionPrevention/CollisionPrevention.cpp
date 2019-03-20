/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file CollisionPrevention.cpp
 * CollisionPrevention controller.
 *
 */

#include <CollisionPrevention/CollisionPrevention.hpp>
using namespace matrix;
using namespace time_literals;


CollisionPrevention::CollisionPrevention(ModuleParams *parent) :
	ModuleParams(parent)
{

}

CollisionPrevention::~CollisionPrevention()
{
	//unadvertise publishers
	if (_constraints_pub != nullptr) {
		orb_unadvertise(_constraints_pub);
	}

	if (_mavlink_log_pub != nullptr) {
		orb_unadvertise(_mavlink_log_pub);
	}
}

bool CollisionPrevention::initializeSubscriptions(SubscriptionArray &subscription_array)
{
	if (!subscription_array.get(ORB_ID(obstacle_distance), _sub_obstacle_distance)) {
		return false;
	}

	return true;
}

void CollisionPrevention::reset_constraints()
{

	_move_constraints_x_normalized.zero();  //normalized constraint in x-direction
	_move_constraints_y_normalized.zero();  //normalized constraint in y-direction

	_move_constraints_x.zero();  //constraint in x-direction
	_move_constraints_y.zero();  //constraint in y-direction
}

void CollisionPrevention::publish_constraints(const Vector2f &original_setpoint, const Vector2f &adapted_setpoint)
{

	collision_constraints_s	constraints;	/**< collision constraints message */

	//fill in values
	constraints.timestamp = hrt_absolute_time();
	constraints.constraints_normalized_x[0] = _move_constraints_x_normalized(0);
	constraints.constraints_normalized_x[1] = _move_constraints_x_normalized(1);
	constraints.constraints_normalized_y[0] = _move_constraints_y_normalized(0);
	constraints.constraints_normalized_y[1] = _move_constraints_y_normalized(1);

	constraints.original_setpoint[0] = original_setpoint(0);
	constraints.original_setpoint[1] = original_setpoint(1);
	constraints.adapted_setpoint[0] = adapted_setpoint(0);
	constraints.adapted_setpoint[1] = adapted_setpoint(1);

	// publish constraints
	if (_constraints_pub != nullptr) {
		orb_publish(ORB_ID(collision_constraints), _constraints_pub, &constraints);

	} else {
		_constraints_pub = orb_advertise(ORB_ID(collision_constraints), &constraints);
	}
}

void CollisionPrevention::update_range_constraints()
{
	const obstacle_distance_s &obstacle_distance = _sub_obstacle_distance->get();

	if (hrt_elapsed_time(&obstacle_distance.timestamp) < RANGE_STREAM_TIMEOUT_US) {
		float max_detection_distance = obstacle_distance.max_distance / 100.0f; //convert to meters

		int distances_array_size = sizeof(obstacle_distance.distances) / sizeof(obstacle_distance.distances[0]);

		for (int i = 0; i < distances_array_size; i++) {
			//determine if distance bin is valid and contains a valid distance measurement
			if (obstacle_distance.distances[i] < obstacle_distance.max_distance &&
			    obstacle_distance.distances[i] > obstacle_distance.min_distance && i * obstacle_distance.increment < 360) {
				float distance = obstacle_distance.distances[i] / 100.0f; //convert to meters
				float angle = math::radians((float)i * obstacle_distance.increment);

				//calculate normalized velocity reductions
				float vel_lim_x = (max_detection_distance - distance) / (max_detection_distance - _param_mpc_col_prev_d.get()) * cos(
							  angle);
				float vel_lim_y = (max_detection_distance - distance) / (max_detection_distance - _param_mpc_col_prev_d.get()) * sin(
							  angle);

				if (vel_lim_x > 0 && vel_lim_x > _move_constraints_x_normalized(1)) { _move_constraints_x_normalized(1) = vel_lim_x; }

				if (vel_lim_y > 0 && vel_lim_y > _move_constraints_y_normalized(1)) { _move_constraints_y_normalized(1) = vel_lim_y; }

				if (vel_lim_x < 0 && -vel_lim_x > _move_constraints_x_normalized(0)) { _move_constraints_x_normalized(0) = -vel_lim_x; }

				if (vel_lim_y < 0 && -vel_lim_y > _move_constraints_y_normalized(0)) { _move_constraints_y_normalized(0) = -vel_lim_y; }
			}
		}

	} else if (_last_message + MESSAGE_THROTTLE_US < hrt_absolute_time()) {
		mavlink_log_critical(&_mavlink_log_pub, "No range data received");
		_last_message = hrt_absolute_time();
	}
}

void CollisionPrevention::modifySetpoint(Vector2f &original_setpoint, const float max_speed)
{
	reset_constraints();

	//calculate movement constraints based on range data
	update_range_constraints();
	_move_constraints_x = _move_constraints_x_normalized;
	_move_constraints_y = _move_constraints_y_normalized;

	// calculate the maximum velocity along x,y axis when moving in the demanded direction
	float vel_mag = original_setpoint.norm();
	float v_max_x, v_max_y;

	if (vel_mag > 0.0f) {
		v_max_x = abs(max_speed / vel_mag * original_setpoint(0));
		v_max_y = abs(max_speed / vel_mag * original_setpoint(1));

	} else {
		v_max_x = 0.0f;
		v_max_y = 0.0f;
	}

	//scale the velocity reductions with the maximum possible velocity along the respective axis
	_move_constraints_x *= v_max_x;
	_move_constraints_y *= v_max_y;

	//apply the velocity reductions to form velocity limits
	_move_constraints_x(0) = v_max_x - _move_constraints_x(0);
	_move_constraints_x(1) = v_max_x - _move_constraints_x(1);
	_move_constraints_y(0) = v_max_y - _move_constraints_y(0);
	_move_constraints_y(1) = v_max_y - _move_constraints_y(1);

	//constrain the velocity setpoint to respect the velocity limits
	Vector2f new_setpoint;
	new_setpoint(0) = math::constrain(original_setpoint(0), -_move_constraints_x(0), _move_constraints_x(1));
	new_setpoint(1) = math::constrain(original_setpoint(1), -_move_constraints_y(0), _move_constraints_y(1));

	//warn user if collision prevention starts to interfere
	bool currently_interfering = (new_setpoint(0) < 0.95f * original_setpoint(0)
				      || new_setpoint(0) > 1.05f * original_setpoint(0) || new_setpoint(1) < 0.95f * original_setpoint(1)
				      || new_setpoint(1) > 1.05f * original_setpoint(1));

	if (currently_interfering && (currently_interfering != _interfering)) {
		mavlink_log_critical(&_mavlink_log_pub, "Collision Warning");
	}

	_interfering = currently_interfering;

	publish_constraints(original_setpoint, new_setpoint);
	original_setpoint = new_setpoint;
}
