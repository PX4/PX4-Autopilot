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
namespace
{
static const int INTERNAL_MAP_INCREMENT_DEG = 10; //cannot be lower than 5 degrees, should divide 360 evenly
static const int INTERNAL_MAP_USED_BINS = 360 / INTERNAL_MAP_INCREMENT_DEG;

float wrap_360(float f)
{
	return wrap(f, 0.f, 360.f);
}

int wrap_bin(int i)
{
	i = i % INTERNAL_MAP_USED_BINS;

	while (i < 0) {
		i += INTERNAL_MAP_USED_BINS;
	}

	return i;
}
}

CollisionPrevention::CollisionPrevention(ModuleParams *parent) :
	ModuleParams(parent)
{
	static_assert(INTERNAL_MAP_INCREMENT_DEG >= 5, "INTERNAL_MAP_INCREMENT_DEG needs to be at least 5");
	static_assert(360 % INTERNAL_MAP_INCREMENT_DEG == 0, "INTERNAL_MAP_INCREMENT_DEG should divide 360 evenly");
	//initialize internal obstacle map
	_obstacle_map_body_frame.timestamp = getTime();
	_obstacle_map_body_frame.increment = INTERNAL_MAP_INCREMENT_DEG;
	_obstacle_map_body_frame.min_distance = UINT16_MAX;
	_obstacle_map_body_frame.max_distance = 0;
	_obstacle_map_body_frame.angle_offset = 0.f;
	uint32_t internal_bins = sizeof(_obstacle_map_body_frame.distances) / sizeof(_obstacle_map_body_frame.distances[0]);
	uint64_t current_time = getTime();

	for (uint32_t i = 0 ; i < internal_bins; i++) {
		_data_timestamps[i] = current_time;
		_obstacle_map_body_frame.distances[i] = UINT16_MAX;
	}
}

CollisionPrevention::~CollisionPrevention()
{
	//unadvertise publishers
	if (_mavlink_log_pub != nullptr) {
		orb_unadvertise(_mavlink_log_pub);
	}
}

hrt_abstime CollisionPrevention::getTime()
{
	return hrt_absolute_time();
}

hrt_abstime CollisionPrevention::getElapsedTime(const hrt_abstime *ptr)
{
	return hrt_absolute_time() - *ptr;
}

void CollisionPrevention::_addObstacleSensorData(const obstacle_distance_s &obstacle,
		const matrix::Quatf &vehicle_attitude)
{
	int msg_index = 0;
	float vehicle_orientation_deg = math::degrees(Eulerf(vehicle_attitude).psi());
	float increment_factor = 1.f / obstacle.increment;


	if (obstacle.frame == obstacle.MAV_FRAME_GLOBAL || obstacle.frame == obstacle.MAV_FRAME_LOCAL_NED) {
		//Obstacle message arrives in local_origin frame (north aligned)
		//corresponding data index (convert to world frame and shift by msg offset)
		for (int i = 0; i < INTERNAL_MAP_USED_BINS; i++) {
			float bin_angle_deg = (float)i * INTERNAL_MAP_INCREMENT_DEG + _obstacle_map_body_frame.angle_offset;
			msg_index = ceil(wrap_360(vehicle_orientation_deg + bin_angle_deg - obstacle.angle_offset) * increment_factor);

			//add all data points inside to FOV
			if (obstacle.distances[msg_index] != UINT16_MAX) {
				_obstacle_map_body_frame.distances[i] = obstacle.distances[msg_index];
				_data_timestamps[i] = _obstacle_map_body_frame.timestamp;
			}

		}

	} else if (obstacle.frame == obstacle.MAV_FRAME_BODY_FRD) {
		//Obstacle message arrives in body frame (front aligned)
		//corresponding data index (shift by msg offset)
		for (int i = 0; i < INTERNAL_MAP_USED_BINS; i++) {
			float bin_angle_deg = (float)i * INTERNAL_MAP_INCREMENT_DEG +
					      _obstacle_map_body_frame.angle_offset;
			msg_index = ceil(wrap_360(bin_angle_deg - obstacle.angle_offset) * increment_factor);

			//add all data points inside to FOV
			if (obstacle.distances[msg_index] != UINT16_MAX) {
				_obstacle_map_body_frame.distances[i] = obstacle.distances[msg_index];
				_data_timestamps[i] = _obstacle_map_body_frame.timestamp;
			}

		}

	} else {
		mavlink_log_critical(&_mavlink_log_pub, "Obstacle message received in unsupported frame %.0f\n",
				     (double)obstacle.frame);
	}
}

void CollisionPrevention::_updateObstacleMap()
{
	_sub_vehicle_attitude.update();

	// add distance sensor data
	for (unsigned i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {

		//if a new distance sensor message has arrived
		if (_sub_distance_sensor[i].updated()) {
			distance_sensor_s distance_sensor {};
			_sub_distance_sensor[i].copy(&distance_sensor);

			// consider only instances with valid data and orientations useful for collision prevention
			if ((getElapsedTime(&distance_sensor.timestamp) < RANGE_STREAM_TIMEOUT_US) &&
			    (distance_sensor.orientation != distance_sensor_s::ROTATION_DOWNWARD_FACING) &&
			    (distance_sensor.orientation != distance_sensor_s::ROTATION_UPWARD_FACING)) {

				//update message description
				_obstacle_map_body_frame.timestamp = math::max(_obstacle_map_body_frame.timestamp, distance_sensor.timestamp);
				_obstacle_map_body_frame.max_distance = math::max((int)_obstacle_map_body_frame.max_distance,
									(int)distance_sensor.max_distance * 100);
				_obstacle_map_body_frame.min_distance = math::min((int)_obstacle_map_body_frame.min_distance,
									(int)distance_sensor.min_distance * 100);

				_addDistanceSensorData(distance_sensor, Quatf(_sub_vehicle_attitude.get().q));
			}
		}
	}

	// add obstacle distance data
	if (_sub_obstacle_distance.update()) {
		const obstacle_distance_s &obstacle_distance = _sub_obstacle_distance.get();

		// Update map with obstacle data if the data is not stale
		if (getElapsedTime(&obstacle_distance.timestamp) < RANGE_STREAM_TIMEOUT_US && obstacle_distance.increment > 0.f) {
			//update message description
			_obstacle_map_body_frame.timestamp = math::max(_obstacle_map_body_frame.timestamp, obstacle_distance.timestamp);
			_obstacle_map_body_frame.max_distance = math::max((int)_obstacle_map_body_frame.max_distance,
								(int)obstacle_distance.max_distance);
			_obstacle_map_body_frame.min_distance = math::min((int)_obstacle_map_body_frame.min_distance,
								(int)obstacle_distance.min_distance);
			_addObstacleSensorData(obstacle_distance, Quatf(_sub_vehicle_attitude.get().q));
		}
	}

	// publish fused obtacle distance message with data from offboard obstacle_distance and distance sensor
	_obstacle_distance_pub.publish(_obstacle_map_body_frame);
}

void CollisionPrevention::_addDistanceSensorData(distance_sensor_s &distance_sensor,
		const matrix::Quatf &vehicle_attitude)
{
	//clamp at maximum sensor range
	float distance_reading = math::min(distance_sensor.current_distance, distance_sensor.max_distance);

	//discard values below min range
	if ((distance_reading > distance_sensor.min_distance)) {

		float sensor_yaw_body_rad = _sensorOrientationToYawOffset(distance_sensor, _obstacle_map_body_frame.angle_offset);
		float sensor_yaw_body_deg = math::degrees(wrap_2pi(sensor_yaw_body_rad));

		// calculate the field of view boundary bin indices
		int lower_bound = (int)floor((sensor_yaw_body_deg  - math::degrees(distance_sensor.h_fov / 2.0f)) /
					     INTERNAL_MAP_INCREMENT_DEG);
		int upper_bound = (int)floor((sensor_yaw_body_deg  + math::degrees(distance_sensor.h_fov / 2.0f)) /
					     INTERNAL_MAP_INCREMENT_DEG);

		//floor values above zero, ceil values below zero
		if (lower_bound < 0) { lower_bound++; }

		if (upper_bound < 0) { upper_bound++; }

		// rotate vehicle attitude into the sensor body frame
		matrix::Quatf attitude_sensor_frame = vehicle_attitude;
		attitude_sensor_frame.rotate(Vector3f(0.f, 0.f, sensor_yaw_body_rad));
		float sensor_dist_scale = cosf(Eulerf(attitude_sensor_frame).theta());

		for (int bin = lower_bound; bin <= upper_bound; ++bin) {
			int wrapped_bin = wrap_bin(bin);

			// compensate measurement for vehicle tilt and convert to cm
			_obstacle_map_body_frame.distances[wrapped_bin] = (int)(100 * distance_reading * sensor_dist_scale);
			_data_timestamps[wrapped_bin] = _obstacle_map_body_frame.timestamp;
		}
	}
}


void CollisionPrevention::_calculateConstrainedSetpoint(Vector2f &setpoint,
		const Vector2f &curr_pos, const Vector2f &curr_vel)
{
	_updateObstacleMap();

	//read parameters
	float col_prev_d = _param_mpc_col_prev_d.get();
	float col_prev_dly = _param_mpc_col_prev_dly.get();
	float col_prev_ang_rad = math::radians(_param_mpc_col_prev_ang.get());
	float xy_p = _param_mpc_xy_p.get();
	float max_jerk = _param_mpc_jerk_max.get();
	float max_accel = _param_mpc_acc_hor.get();
	matrix::Quatf attitude = Quatf(_sub_vehicle_attitude.get().q);
	float vehicle_yaw_angle_rad = Eulerf(attitude).psi();

	float setpoint_length = setpoint.norm();

	hrt_abstime constrain_time = getTime();

	if ((constrain_time - _obstacle_map_body_frame.timestamp) < RANGE_STREAM_TIMEOUT_US) {
		if (setpoint_length > 0.001f) {

			Vector2f setpoint_dir = setpoint / setpoint_length;
			float vel_max = setpoint_length;
			float min_dist_to_keep = math::max(_obstacle_map_body_frame.min_distance / 100.0f, col_prev_d);

			float sp_angle_body_frame = atan2(setpoint_dir(1), setpoint_dir(0)) - vehicle_yaw_angle_rad;
			float sp_angle_with_offset_deg = wrap_360(math::degrees(sp_angle_body_frame) - _obstacle_map_body_frame.angle_offset);
			int sp_index = floor(sp_angle_with_offset_deg / INTERNAL_MAP_INCREMENT_DEG);

			for (int i = 0; i < INTERNAL_MAP_USED_BINS; i++) { //disregard unused bins at the end of the message

				//delete stale values
				hrt_abstime data_age = constrain_time - _data_timestamps[i];

				if (data_age > RANGE_STREAM_TIMEOUT_US) {
					_obstacle_map_body_frame.distances[i] = UINT16_MAX;
				}

				float distance = _obstacle_map_body_frame.distances[i] * 0.01f; //convert to meters
				float angle = math::radians((float)i * INTERNAL_MAP_INCREMENT_DEG + _obstacle_map_body_frame.angle_offset);

				// convert from body to local frame in the range [0, 2*pi]
				angle  = wrap_2pi(vehicle_yaw_angle_rad + angle);

				//get direction of current bin
				Vector2f bin_direction = {cos(angle), sin(angle)};

				if (_obstacle_map_body_frame.distances[i] > _obstacle_map_body_frame.min_distance
				    && _obstacle_map_body_frame.distances[i] < UINT16_MAX) {

					if (setpoint_dir.dot(bin_direction) > 0
					    && setpoint_dir.dot(bin_direction) > cosf(col_prev_ang_rad)) {
						//calculate max allowed velocity with a P-controller (same gain as in the position controller)
						float curr_vel_parallel = math::max(0.f, curr_vel.dot(bin_direction));
						float delay_distance = curr_vel_parallel * (col_prev_dly + data_age * 1e-6f);
						float stop_distance =  math::max(0.f, distance - min_dist_to_keep - delay_distance);
						float vel_max_posctrl = xy_p * stop_distance;
						float vel_max_smooth = math::trajectory::computeMaxSpeedFromBrakingDistance(max_jerk, max_accel, stop_distance);
						Vector2f  vel_max_vec = bin_direction * math::min(vel_max_posctrl, vel_max_smooth);
						float vel_max_bin = vel_max_vec.dot(setpoint_dir);

						//constrain the velocity
						if (vel_max_bin >= 0) {
							vel_max = math::min(vel_max, vel_max_bin);
						}
					}

				} else if (_obstacle_map_body_frame.distances[i] == UINT16_MAX && i == sp_index) {
					vel_max = 0.f;
				}
			}

			setpoint = setpoint_dir * vel_max;
		}

	} else {
		// if distance data are stale, switch to Loiter
		_publishVehicleCmdDoLoiter();
		mavlink_log_critical(&_mavlink_log_pub, "No range data received, loitering.");
	}
}

void CollisionPrevention::modifySetpoint(Vector2f &original_setpoint, const float max_speed,
		const Vector2f &curr_pos, const Vector2f &curr_vel)
{
	//calculate movement constraints based on range data
	Vector2f new_setpoint = original_setpoint;
	_calculateConstrainedSetpoint(new_setpoint, curr_pos, curr_vel);

	//warn user if collision prevention starts to interfere
	bool currently_interfering = (new_setpoint(0) < original_setpoint(0) - 0.05f * max_speed
				      || new_setpoint(0) > original_setpoint(0) + 0.05f * max_speed
				      || new_setpoint(1) < original_setpoint(1) - 0.05f * max_speed
				      || new_setpoint(1) > original_setpoint(1) + 0.05f * max_speed);

	if (currently_interfering && (currently_interfering != _interfering)) {
		mavlink_log_critical(&_mavlink_log_pub, "Collision Warning");
	}

	_interfering = currently_interfering;

	// publish constraints
	collision_constraints_s	constraints{};
	constraints.timestamp = hrt_absolute_time();
	original_setpoint.copyTo(constraints.original_setpoint);
	new_setpoint.copyTo(constraints.adapted_setpoint);
	_constraints_pub.publish(constraints);

	original_setpoint = new_setpoint;
}

void CollisionPrevention::_publishVehicleCmdDoLoiter()
{
	vehicle_command_s command{};
	command.timestamp = hrt_absolute_time();
	command.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
	command.param1 = (float)1; // base mode
	command.param3 = (float)0; // sub mode
	command.target_system = 1;
	command.target_component = 1;
	command.source_system = 1;
	command.source_component = 1;
	command.confirmation = false;
	command.from_external = false;
	command.param2 = (float)PX4_CUSTOM_MAIN_MODE_AUTO;
	command.param3 = (float)PX4_CUSTOM_SUB_MODE_AUTO_LOITER;

	// publish the vehicle command
	_vehicle_command_pub.publish(command);
}
