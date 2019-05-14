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

bool CollisionPrevention::isZero(const Vector2f &vec, const float limit) const
{
	return (vec(0) < limit && vec(0) > -limit) && (vec(1) < limit && vec(1) > -limit);
}

void CollisionPrevention::update_velocity_constraints(const float max_acc, Vector2f &setpoint)
{
	const obstacle_distance_s &obstacle_distance = _sub_obstacle_distance->get();
	Vector2f setpoint_sqrd = setpoint * setpoint.norm();
	float slide_max_rad = 1.57f;

	PX4_INFO_RAW("_______________________START with SP [%.3f, %.3f]\n", (double)setpoint(0), (double)setpoint(1));

	if (hrt_elapsed_time(&obstacle_distance.timestamp) < RANGE_STREAM_TIMEOUT_US && !isZero(setpoint, 1e-3)) {

		int distances_array_size = sizeof(obstacle_distance.distances) / sizeof(obstacle_distance.distances[0]);

		for (int i = 0; i < distances_array_size; i++) {
			//determine if distance bin is valid and contains a valid distance measurement
			if (obstacle_distance.distances[i] < obstacle_distance.max_distance &&
			    obstacle_distance.distances[i] > obstacle_distance.min_distance && i * obstacle_distance.increment < 360) {
				float distance = obstacle_distance.distances[i] / 100.0f; //convert to meters
				float angle = math::radians((float)i * obstacle_distance.increment);

				//max velocity squared in bin direction
				float vel_max_sqrd = max_acc * (distance - _param_mpc_col_prev_d.get());

				if (distance < _param_mpc_col_prev_d.get()) {
					vel_max_sqrd = 0.f;
				}

				//limit setpoint
				Vector2f bin_direction = {cos(angle), sin(angle)};
				Vector2f orth_direction = {sin(angle), cos(angle)};
				float sp_parallel = setpoint_sqrd.dot(bin_direction);
				float sp_orth = setpoint_sqrd.dot(orth_direction);

				if (sp_parallel > vel_max_sqrd) {
					Vector2f setpoint_temp = setpoint_sqrd - (sp_parallel - vel_max_sqrd) * bin_direction;


					//limit sliding angle
					float angle_demanded_temp = acos(setpoint_temp.dot(setpoint) / (setpoint_temp.norm() * setpoint.norm()));

					if (angle_demanded_temp > slide_max_rad) {
						float angle_diff_bin = acos(sp_parallel / setpoint_sqrd.norm());
						float orth_len = vel_max_sqrd * tan(angle_diff_bin + slide_max_rad);

						if (sp_orth > 0) {
							setpoint_temp = vel_max_sqrd * bin_direction + orth_len * orth_direction;

						} else {
							setpoint_temp = vel_max_sqrd * bin_direction - orth_len * orth_direction;
						}

						PX4_INFO_RAW("sliding angle limited from %.3f to %.3f\n", (double)angle_demanded_temp, (double)slide_max_rad);
					}

					setpoint_sqrd = setpoint_temp;
					PX4_INFO_RAW("vel_lim %.3f parallel component %.3f  SP sqrd [%.3f, %.3f]\n", (double)vel_max_sqrd, (double)sp_parallel,
						     (double)setpoint_sqrd(0), (double)setpoint_sqrd(1));
				}
			}
		}

		if (!isZero(setpoint_sqrd, 1e-3)) {
			setpoint = setpoint_sqrd.normalized() * std::sqrt(setpoint_sqrd.norm());

		} else {
			setpoint.zero();
		}

		PX4_INFO_RAW("Final SP [%.3f, %.3f] \n", (double)setpoint(0), (double)setpoint(1));

	} else if (_last_message + MESSAGE_THROTTLE_US < hrt_absolute_time()) {
		mavlink_log_critical(&_mavlink_log_pub, "No range data received");
		_last_message = hrt_absolute_time();
	}
}

void CollisionPrevention::modifySetpoint(Vector2f &original_setpoint, const float max_speed,
		const float max_acc)
{
	//calculate movement constraints based on range data
	Vector2f new_setpoint = original_setpoint;
	update_velocity_constraints(max_acc, new_setpoint);

	//warn user if collision prevention starts to interfere
	bool currently_interfering = (new_setpoint(0) < original_setpoint(0) - 0.05f * max_speed
				      || new_setpoint(0) > original_setpoint(0) + 0.05f * max_speed
				      || new_setpoint(1) < original_setpoint(1) - 0.05f * max_speed
				      || new_setpoint(1) > original_setpoint(1) + 0.05f * max_speed);

	if (currently_interfering && (currently_interfering != _interfering)) {
		mavlink_log_critical(&_mavlink_log_pub, "Collision Warning");
	}

	_interfering = currently_interfering;
	original_setpoint = new_setpoint;
}
