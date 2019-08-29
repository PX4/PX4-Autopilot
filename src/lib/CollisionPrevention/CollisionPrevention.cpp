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

#include <FlightTasks/tasks/Utility/TrajMath.hpp>

using namespace matrix;
using namespace time_literals;


CollisionPrevention::CollisionPrevention(ModuleParams *parent) :
	ModuleParams(parent)
{

	//initialize internal obstacle map
	_obstacle_map_body_frame.timestamp = hrt_absolute_time();
	_obstacle_map_body_frame.increment = 10.f;		//cannot be lower than 5 degrees
	_obstacle_map_body_frame.min_distance = UINT16_MAX;
	_obstacle_map_body_frame.max_distance = 0;
	_obstacle_map_body_frame.angle_offset = 0.f;
	memset(&_obstacle_map_body_frame.distances[0], UINT16_MAX, sizeof(_obstacle_map_body_frame.distances));

	uint64_t current_time = hrt_absolute_time();

	for (uint i = 0 ; i < sizeof(_data_timestamps); i++) {
		_data_timestamps[i] = current_time;
	}
}

CollisionPrevention::~CollisionPrevention()
{
	//unadvertise publishers
	if (_mavlink_log_pub != nullptr) {
		orb_unadvertise(_mavlink_log_pub);
	}
}

void CollisionPrevention::_addObstacleSensorData(const obstacle_distance_s &obstacle,
		const matrix::Quatf &vehicle_attitude)
{
	//Obstacle message will arrive in local_origin frame, waiting for mavlink PR to add other frame options
	for (int i = 0; i < floor(360.f / _obstacle_map_body_frame.increment); i++) {
		float bin_angle_rad =  math::radians((float)i * _obstacle_map_body_frame.increment +
						     _obstacle_map_body_frame.angle_offset);

		//corresponding data index (convert to world frame and shift by msg offset)
		int msg_index = ceil(math::degrees(wrap_2pi(Eulerf(vehicle_attitude).psi() + bin_angle_rad - math::radians(
				obstacle.angle_offset))) / obstacle.increment);

		//add all data points inside to FOV
		if (obstacle.distances[msg_index] != UINT16_MAX) {
			_obstacle_map_body_frame.distances[i] = obstacle.distances[msg_index];
			_data_timestamps[i] = hrt_absolute_time();
		}
	}
}

void CollisionPrevention::_updateObstacleMap()
{
	_sub_vehicle_attitude.update();

	// add distance sensor data
	for (unsigned i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
		distance_sensor_s distance_sensor {};
		_sub_distance_sensor[i].copy(&distance_sensor);

		// consider only instances with updated, valid data and orientations useful for collision prevention
		if ((hrt_elapsed_time(&distance_sensor.timestamp) < RANGE_STREAM_TIMEOUT_US) &&
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

	// add obstacle distance data
	_sub_obstacle_distance.update();
	const obstacle_distance_s &obstacle_distance = _sub_obstacle_distance.get();

	// Update map with obstacle data if the data is not stale
	if (hrt_elapsed_time(&obstacle_distance.timestamp) < RANGE_STREAM_TIMEOUT_US) {
		//update message description
		_obstacle_map_body_frame.timestamp = math::max(_obstacle_map_body_frame.timestamp, obstacle_distance.timestamp);
		_obstacle_map_body_frame.max_distance = math::max((int)_obstacle_map_body_frame.max_distance,
							(int)obstacle_distance.max_distance);
		_obstacle_map_body_frame.min_distance = math::min((int)_obstacle_map_body_frame.min_distance,
							(int)obstacle_distance.min_distance);

		_addObstacleSensorData(obstacle_distance, Quatf(_sub_vehicle_attitude.get().q));
	}

	// publish fused obtacle distance message with data from offboard obstacle_distance and distance sensor
	_obstacle_distance_pub.publish(_obstacle_map_body_frame);
}

void CollisionPrevention::_addDistanceSensorData(distance_sensor_s &distance_sensor,
		const matrix::Quatf &vehicle_attitude)
{
	if ((distance_sensor.current_distance > distance_sensor.min_distance) &&
	    (distance_sensor.current_distance < distance_sensor.max_distance)) {

		float sensor_yaw_body_rad = _sensorOrientationToYawOffset(distance_sensor, _obstacle_map_body_frame.angle_offset);
		float sensor_yaw_body_deg = math::degrees(wrap_2pi(sensor_yaw_body_rad));

		// calculate the field of view boundary bin indices
		int lower_bound = (int)floor((sensor_yaw_body_deg  - math::degrees(distance_sensor.h_fov / 2.0f)) /
					     _obstacle_map_body_frame.increment);
		int upper_bound = (int)floor((sensor_yaw_body_deg  + math::degrees(distance_sensor.h_fov / 2.0f)) /
					     _obstacle_map_body_frame.increment);

		//floor values above zero, ceil values below zero
		if (lower_bound < 0) { lower_bound++; }

		if (upper_bound < 0) { upper_bound++; }

		// rotate vehicle attitude into the sensor body frame
		const int map_bins_used = 360.f / _obstacle_map_body_frame.increment;
		matrix::Quatf attitude_sensor_frame = vehicle_attitude;
		attitude_sensor_frame.rotate(Vector3f(0.f, 0.f, sensor_yaw_body_rad));
		float attitude_sensor_frame_pitch = cosf(Eulerf(attitude_sensor_frame).theta());

		for (int bin = lower_bound; bin <= upper_bound; ++bin) {
			int wrap_bin = bin;

			while (wrap_bin < 0) {
				// wrap bin index around the array
				wrap_bin += map_bins_used;
			}

			while (wrap_bin >= map_bins_used) {
				// wrap bin index around the array
				wrap_bin -= map_bins_used;
			}

			// compensate measurement for vehicle tilt and convert to cm
			_obstacle_map_body_frame.distances[wrap_bin] = (int)(100 * distance_sensor.current_distance *
					attitude_sensor_frame_pitch);
			_data_timestamps[wrap_bin] = hrt_absolute_time();
		}
	}
}


void CollisionPrevention::_calculateConstrainedSetpoint(Vector2f &setpoint,
		const Vector2f &curr_pos, const Vector2f &curr_vel)
{
	_updateObstacleMap();
	float setpoint_length = setpoint.norm();
	float col_prev_d = _param_mpc_col_prev_d.get();
	float col_prev_dly = _param_mpc_col_prev_dly.get();
	float col_prev_ang_rad = math::radians(_param_mpc_col_prev_ang.get());
	float xy_p = _param_mpc_xy_p.get();
	float max_jerk = _param_mpc_jerk_max.get();
	float max_accel = _param_mpc_acc_hor.get();
	matrix::Quatf attitude = Quatf(_sub_vehicle_attitude.get().q);
	float vehicle_yaw_angle_rad = Eulerf(attitude).psi();

	if (hrt_elapsed_time(&_obstacle_map_body_frame.timestamp) < RANGE_STREAM_TIMEOUT_US) {
		if (setpoint_length > 0.001f) {

			Vector2f setpoint_dir = setpoint / setpoint_length;
			float vel_max = setpoint_length;
			float min_dist_to_keep = math::max(_obstacle_map_body_frame.min_distance / 100.0f, col_prev_d);

			for (int i = 0; i < 360.f / _obstacle_map_body_frame.increment; i++) { //disregard unused bins at the end of the message

				//delete stale values
				if (hrt_elapsed_time(&_data_timestamps[i]) > RANGE_STREAM_TIMEOUT_US) {
					_obstacle_map_body_frame.distances[i] = UINT16_MAX;
				}


				float distance = _obstacle_map_body_frame.distances[i] / 100.0f; //convert to meters
				float angle = math::radians((float)i * _obstacle_map_body_frame.increment + _obstacle_map_body_frame.angle_offset);

				// convert from body to local frame in the range [0, 360]
				angle  = wrap_2pi(vehicle_yaw_angle_rad + angle);

				//get direction of current bin
				Vector2f bin_direction = {cos(angle), sin(angle)};

				if (_obstacle_map_body_frame.distances[i] < _obstacle_map_body_frame.max_distance &&
				    _obstacle_map_body_frame.distances[i] > _obstacle_map_body_frame.min_distance) {

					if (setpoint_dir.dot(bin_direction) > 0
					    && setpoint_dir.dot(bin_direction) > cosf(col_prev_ang_rad)) {
						//calculate max allowed velocity with a P-controller (same gain as in the position controller)
						float curr_vel_parallel = math::max(0.f, curr_vel.dot(bin_direction));
						float delay_distance = curr_vel_parallel * col_prev_dly;
						float stop_distance =  math::max(0.f, distance - min_dist_to_keep - delay_distance);
						float vel_max_posctrl = xy_p * stop_distance;
						float vel_max_smooth = trajmath::computeMaxSpeedFromBrakingDistance(max_jerk, max_accel, stop_distance);
						Vector2f  vel_max_vec = bin_direction * math::min(vel_max_posctrl, vel_max_smooth);
						float vel_max_bin = vel_max_vec.dot(setpoint_dir);

						//constrain the velocity
						if (vel_max_bin >= 0) {
							vel_max = math::min(vel_max, vel_max_bin);
						}
					}

				} else if (_obstacle_map_body_frame.distances[i] == UINT16_MAX) {
					float sp_bin = setpoint_dir.dot(bin_direction);
					float ang_half_bin = cosf(math::radians(_obstacle_map_body_frame.increment) /
								  1.95f); //half a bin plus some margin for floating point errors

					//if the setpoint lies outside the FOV set velocity to zero
					if (sp_bin >= ang_half_bin) {
						vel_max = 0.f;
					}

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
