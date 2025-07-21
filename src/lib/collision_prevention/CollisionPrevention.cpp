/****************************************************************************
 *
 *   Copyright (c) 2018-2024 PX4 Development Team. All rights reserved.
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

#include "CollisionPrevention.hpp"
#include "ObstacleMath.hpp"
#include <px4_platform_common/events.h>

using namespace matrix;

CollisionPrevention::CollisionPrevention(ModuleParams *parent) :
	ModuleParams(parent)
{
	static_assert(BIN_SIZE >= 5, "BIN_SIZE must be at least 5");
	static_assert(360 % BIN_SIZE == 0, "BIN_SIZE must divide 360 evenly");

	// initialize internal obstacle map
	_obstacle_map_body_frame.frame = obstacle_distance_s::MAV_FRAME_BODY_FRD;
	_obstacle_map_body_frame.increment = BIN_SIZE;
	_obstacle_map_body_frame.min_distance = UINT16_MAX;

	for (uint32_t i = 0 ; i < BIN_COUNT; i++) {
		_obstacle_map_body_frame.distances[i] = UINT16_MAX;
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

bool CollisionPrevention::is_active()
{
	bool activated = _param_cp_dist.get() > 0;

	if (activated && !_was_active) {
		_time_activated = getTime();
	}

	_was_active = activated;
	return activated;
}

void CollisionPrevention::modifySetpoint(Vector2f &setpoint_accel, const Vector2f &setpoint_vel)
{
	if (_vehicle_attitude_sub.updated()) {
		vehicle_attitude_s vehicle_attitude;

		if (_vehicle_attitude_sub.copy(&vehicle_attitude)) {
			_vehicle_attitude = Quatf(vehicle_attitude.q);
			_vehicle_yaw = Eulerf(_vehicle_attitude).psi();
		}
	}

	//calculate movement constraints based on range data
	const Vector2f original_setpoint = setpoint_accel;
	_updateObstacleMap();
	_updateObstacleData();
	_calculateConstrainedSetpoint(setpoint_accel, setpoint_vel);

	// publish constraints
	collision_constraints_s	constraints{};
	original_setpoint.copyTo(constraints.original_setpoint);
	setpoint_accel.copyTo(constraints.adapted_setpoint);
	constraints.timestamp = getTime();
	_constraints_pub.publish(constraints);
}

void CollisionPrevention::_updateObstacleMap()
{
	// add distance sensor data
	for (auto &dist_sens_sub : _distance_sensor_subs) {
		distance_sensor_s distance_sensor;

		if (dist_sens_sub.update(&distance_sensor)) {
			// consider only instances with valid data and orientations useful for collision prevention
			if ((getElapsedTime(&distance_sensor.timestamp) < RANGE_STREAM_TIMEOUT_US) &&
			    (distance_sensor.orientation != distance_sensor_s::ROTATION_DOWNWARD_FACING) &&
			    (distance_sensor.orientation != distance_sensor_s::ROTATION_UPWARD_FACING)) {

				// update message description
				_obstacle_map_body_frame.timestamp = math::max(_obstacle_map_body_frame.timestamp, distance_sensor.timestamp);
				_obstacle_map_body_frame.max_distance = math::max(_obstacle_map_body_frame.max_distance,
									(uint16_t)(distance_sensor.max_distance * 100.0f));
				_obstacle_map_body_frame.min_distance = math::min(_obstacle_map_body_frame.min_distance,
									(uint16_t)(distance_sensor.min_distance * 100.0f));

				_addDistanceSensorData(distance_sensor, _vehicle_attitude);
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
			_obstacle_map_body_frame.max_distance = math::max(_obstacle_map_body_frame.max_distance,
								obstacle_distance.max_distance);
			_obstacle_map_body_frame.min_distance = math::min(_obstacle_map_body_frame.min_distance,
								obstacle_distance.min_distance);
			_addObstacleSensorData(obstacle_distance, _vehicle_yaw);
		}
	}

	// publish fused obtacle distance message with data from offboard obstacle_distance and distance sensor
	_obstacle_distance_fused_pub.publish(_obstacle_map_body_frame);
}

void CollisionPrevention::_updateObstacleData()
{
	_obstacle_data_present = false;
	_closest_dist = UINT16_MAX;
	_closest_dist_dir.setZero();

	for (int i = 0; i < BIN_COUNT; i++) {
		// if the data is stale, reset the bin
		if (getTime() - _data_timestamps[i] > RANGE_STREAM_TIMEOUT_US) {
			_obstacle_map_body_frame.distances[i] = UINT16_MAX;
		}

		float angle = wrap_2pi(_vehicle_yaw + math::radians((float)i * BIN_SIZE +
				       _obstacle_map_body_frame.angle_offset));
		const Vector2f bin_direction = {cosf(angle), sinf(angle)};
		const uint16_t bin_distance = _obstacle_map_body_frame.distances[i];

		// check if there is avaliable data and the data of the map is not stale
		if (bin_distance < UINT16_MAX
		    && (getTime() - _obstacle_map_body_frame.timestamp) < RANGE_STREAM_TIMEOUT_US) {
			_obstacle_data_present = true;
		}

		if (bin_distance * 0.01f < _closest_dist) {
			_closest_dist = bin_distance * 0.01f;
			_closest_dist_dir = bin_direction;
		}
	}
}

void CollisionPrevention::_calculateConstrainedSetpoint(Vector2f &setpoint_accel, const Vector2f &setpoint_vel)
{
	const float setpoint_length = setpoint_accel.norm();
	_min_dist_to_keep = math::max(_obstacle_map_body_frame.min_distance / 100.0f, _param_cp_dist.get());

	const hrt_abstime now = getTime();

	const bool is_stick_deflected = setpoint_length > 0.001f;

	if (_obstacle_data_present && is_stick_deflected) {

		_transformSetpoint(setpoint_accel);

		float vel_comp_accel = INFINITY;
		Vector2f vel_comp_accel_dir{};

		_getVelocityCompensationAcceleration(_vehicle_yaw, setpoint_vel, now,
						     vel_comp_accel, vel_comp_accel_dir);

		Vector2f constr_accel_setpoint{};

		if (_checkSetpointDirectionFeasability()) {
			constr_accel_setpoint = _constrainAccelerationSetpoint(setpoint_length);
		}

		setpoint_accel = constr_accel_setpoint + vel_comp_accel * vel_comp_accel_dir;

	} else if (!_obstacle_data_present) {
		// allow no movement
		setpoint_accel.setZero();

		// if distance data is stale, switch to Loiter
		if (getElapsedTime(&_last_timeout_warning) > 1_s && getElapsedTime(&_time_activated) > 1_s) {
			if ((now - _obstacle_map_body_frame.timestamp) > TIMEOUT_HOLD_US &&
			    getElapsedTime(&_time_activated) > TIMEOUT_HOLD_US) {
				_publishVehicleCmdDoLoiter();
			}

			PX4_WARN("No obstacle data, not moving...");
			_last_timeout_warning = now;
		}
	}
}

// TODO this gives false output if the offset is not a multiple of the resolution. to be fixed...
void CollisionPrevention::_addObstacleSensorData(const obstacle_distance_s &obstacle, const float vehicle_yaw)
{

	float vehicle_orientation_deg = math::degrees(vehicle_yaw);


	if (obstacle.frame == obstacle.MAV_FRAME_GLOBAL || obstacle.frame == obstacle.MAV_FRAME_LOCAL_NED) {
		// Obstacle message arrives in local_origin frame (north aligned)
		// corresponding data index (convert to world frame and shift by msg offset)
		for (int i = 0; i < BIN_COUNT; i++) {
			for (int j = 0; (j < 360 / obstacle.increment) && (j < BIN_COUNT); j++) {
				float bin_lower_angle = ObstacleMath::get_lower_bound_angle(i, _obstacle_map_body_frame.increment,
							_obstacle_map_body_frame.angle_offset);
				float bin_upper_angle = ObstacleMath::get_lower_bound_angle(i + 1, _obstacle_map_body_frame.increment,
							_obstacle_map_body_frame.angle_offset);
				float msg_lower_angle = ObstacleMath::get_lower_bound_angle(j, obstacle.increment,
							obstacle.angle_offset - vehicle_orientation_deg);
				float msg_upper_angle = ObstacleMath::get_lower_bound_angle(j + 1, obstacle.increment,
							obstacle.angle_offset - vehicle_orientation_deg);

				// if a bin stretches over the 0/360 degree line, adjust the angles
				if (bin_lower_angle > bin_upper_angle) {
					bin_lower_angle -= 360;
				}

				if (msg_lower_angle > msg_upper_angle) {
					msg_lower_angle -= 360;
				}

				// Check for overlaps.
				if ((msg_lower_angle > bin_lower_angle && msg_lower_angle < bin_upper_angle) ||
				    (msg_upper_angle > bin_lower_angle && msg_upper_angle < bin_upper_angle) ||
				    (msg_lower_angle <= bin_lower_angle && msg_upper_angle >= bin_upper_angle) ||
				    (msg_lower_angle >= bin_lower_angle && msg_upper_angle <= bin_upper_angle)) {
					if (obstacle.distances[j] != UINT16_MAX) {
						if (_enterData(i, obstacle.max_distance * 0.01f, obstacle.distances[j] * 0.01f)) {
							_obstacle_map_body_frame.distances[i] = obstacle.distances[j];
							_data_timestamps[i] = _obstacle_map_body_frame.timestamp;
							_data_maxranges[i] = obstacle.max_distance;
							_data_fov[i] = 1;
						}
					}
				}

			}
		}

	} else if (obstacle.frame == obstacle.MAV_FRAME_BODY_FRD) {
		// Obstacle message arrives in body frame (front aligned)
		// corresponding data index (shift by msg offset)
		for (int i = 0; i < BIN_COUNT; i++) {
			for (int j = 0; j < 360 / obstacle.increment; j++) {
				float bin_lower_angle = ObstacleMath::get_lower_bound_angle(i, _obstacle_map_body_frame.increment,
							_obstacle_map_body_frame.angle_offset);
				float bin_upper_angle = ObstacleMath::get_lower_bound_angle(i + 1, _obstacle_map_body_frame.increment,
							_obstacle_map_body_frame.angle_offset);
				float msg_lower_angle = ObstacleMath::get_lower_bound_angle(j, obstacle.increment, obstacle.angle_offset);
				float msg_upper_angle = ObstacleMath::get_lower_bound_angle(j + 1, obstacle.increment, obstacle.angle_offset);

				// if a bin stretches over the 0/360 degree line, adjust the angles
				if (bin_lower_angle > bin_upper_angle) {
					bin_lower_angle -= 360;
				}

				if (msg_lower_angle > msg_upper_angle) {
					msg_lower_angle -= 360;
				}

				// Check for overlaps.
				if ((msg_lower_angle > bin_lower_angle && msg_lower_angle < bin_upper_angle) ||
				    (msg_upper_angle > bin_lower_angle && msg_upper_angle < bin_upper_angle) ||
				    (msg_lower_angle <= bin_lower_angle && msg_upper_angle >= bin_upper_angle) ||
				    (msg_lower_angle >= bin_lower_angle && msg_upper_angle <= bin_upper_angle)) {
					if (obstacle.distances[j] != UINT16_MAX) {

						if (_enterData(i, obstacle.max_distance * 0.01f, obstacle.distances[j] * 0.01f)) {
							_obstacle_map_body_frame.distances[i] = obstacle.distances[j];
							_data_timestamps[i] = _obstacle_map_body_frame.timestamp;
							_data_maxranges[i] = obstacle.max_distance;
							_data_fov[i] = 1;
						}
					}
				}

			}
		}

	} else {
		mavlink_log_critical(&_mavlink_log_pub, "Obstacle message received in unsupported frame %i\t",
				     obstacle.frame);
		events::send<uint8_t>(events::ID("col_prev_unsup_frame"), events::Log::Error,
				      "Obstacle message received in unsupported frame {1}", obstacle.frame);
	}
}

bool
CollisionPrevention::_enterData(int map_index, float sensor_range, float sensor_reading)
{
	//use data from this sensor if:
	//1. this sensor data is in range, the bin contains already valid data and this data is coming from the same or less range sensor
	//2. this sensor data is in range, and the last reading was out of range
	//3. this sensor data is out of range, the last reading was as well and this is the sensor with longest range
	//4. this sensor data is out of range, the last reading was valid and coming from the same sensor

	uint16_t sensor_range_cm = static_cast<uint16_t>(100.0f * sensor_range + 0.5f); //convert to cm

	if (sensor_reading < sensor_range) {
		if ((_obstacle_map_body_frame.distances[map_index] < _data_maxranges[map_index]
		     && sensor_range_cm <= _data_maxranges[map_index])
		    || _obstacle_map_body_frame.distances[map_index] >= _data_maxranges[map_index]) {

			return true;
		}

	} else {
		if ((_obstacle_map_body_frame.distances[map_index] >= _data_maxranges[map_index]
		     && sensor_range_cm >= _data_maxranges[map_index])
		    || (_obstacle_map_body_frame.distances[map_index] < _data_maxranges[map_index]
			&& sensor_range_cm == _data_maxranges[map_index])) {

			return true;
		}
	}

	return false;
}

bool
CollisionPrevention::_checkSetpointDirectionFeasability()
{
	bool setpoint_feasible = true;

	for (int i = 0; i < BIN_COUNT; i++) {
		// check if our setpoint is either pointing in a direction where data exists, or if not, wether we are allowed to go where there is no data
		if ((_obstacle_map_body_frame.distances[i] == UINT16_MAX && i == _setpoint_index) && (!_param_cp_go_no_data.get()
				|| (_param_cp_go_no_data.get() && _data_fov[i]))) {
			setpoint_feasible =  false;

		}
	}

	return setpoint_feasible;
}

void
CollisionPrevention::_transformSetpoint(const Vector2f &setpoint)
{
	const float sp_angle_body_frame = atan2f(setpoint(1), setpoint(0)) - _vehicle_yaw;
	const float sp_angle_with_offset_deg = ObstacleMath::wrap_360(math::degrees(sp_angle_body_frame) -
					       _obstacle_map_body_frame.angle_offset);
	_setpoint_index = floor(sp_angle_with_offset_deg / BIN_SIZE);
	// change setpoint direction slightly (max by _param_cp_guide_ang degrees) to help guide through narrow gaps
	_setpoint_dir = setpoint.unit_or_zero();
	_adaptSetpointDirection(_setpoint_dir, _setpoint_index, _vehicle_yaw);
}

void
CollisionPrevention::_addDistanceSensorData(distance_sensor_s &distance_sensor, const Quatf &vehicle_attitude)
{
	// clamp at maximum sensor range
	float distance_reading = math::min(distance_sensor.current_distance, distance_sensor.max_distance);

	// negative values indicate out of range but valid measurements.
	if (fabsf(distance_sensor.current_distance - -1.f) < FLT_EPSILON && distance_sensor.signal_quality == 0) {
		distance_reading = distance_sensor.max_distance;
	}

	// discard values below min range
	if (distance_reading > distance_sensor.min_distance) {
		float sensor_yaw_body_rad = ObstacleMath::sensor_orientation_to_yaw_offset(static_cast<ObstacleMath::SensorOrientation>
					    (distance_sensor.orientation), distance_sensor.q);
		float sensor_yaw_body_deg = math::degrees(wrap_2pi(sensor_yaw_body_rad));

		// calculate the field of view boundary bin indices
		int lower_bound = (int)round((sensor_yaw_body_deg  - math::degrees(distance_sensor.h_fov / 2.0f)) / BIN_SIZE);
		int upper_bound = (int)round((sensor_yaw_body_deg  + math::degrees(distance_sensor.h_fov / 2.0f)) / BIN_SIZE);

		if (distance_reading < distance_sensor.max_distance) {
			ObstacleMath::project_distance_on_horizontal_plane(distance_reading, sensor_yaw_body_rad, vehicle_attitude);
		}

		uint16_t sensor_range = static_cast<uint16_t>(100.0f * distance_sensor.max_distance + 0.5f); // convert to cm

		for (int bin = lower_bound; bin <= upper_bound; ++bin) {
			int wrapped_bin = ObstacleMath::wrap_bin(bin, BIN_COUNT);

			if (_enterData(wrapped_bin, distance_sensor.max_distance, distance_reading)) {
				_obstacle_map_body_frame.distances[wrapped_bin] = static_cast<uint16_t>(100.0f * distance_reading + 0.5f);
				_data_timestamps[wrapped_bin] = _obstacle_map_body_frame.timestamp;
				_data_maxranges[wrapped_bin] = sensor_range;
				_data_fov[wrapped_bin] = 1;
			}
		}
	}
}

void
CollisionPrevention::_adaptSetpointDirection(Vector2f &setpoint_dir, int &setpoint_index, float vehicle_yaw_angle_rad)
{
	const int guidance_bins = floor(_param_cp_guide_ang.get() / BIN_SIZE);
	const int sp_index_original = setpoint_index;
	float best_cost = 9999.f;
	int new_sp_index = setpoint_index;

	for (int i = sp_index_original - guidance_bins; i <= sp_index_original + guidance_bins; i++) {

		// apply moving average filter to the distance array to be able to center in larger gaps
		const int filter_size = 1;
		float mean_dist = 0;

		for (int j = i - filter_size; j <= i + filter_size; j++) {
			int bin = ObstacleMath::wrap_bin(j, BIN_COUNT);

			if (_obstacle_map_body_frame.distances[bin] == UINT16_MAX) {
				mean_dist += _param_cp_dist.get() * 100.f;

			} else {
				mean_dist += _obstacle_map_body_frame.distances[bin];
			}
		}

		const int bin = ObstacleMath::wrap_bin(i, BIN_COUNT);
		mean_dist = mean_dist / (2.f * filter_size + 1.f);
		const float deviation_cost = _param_cp_dist.get() * 50.f * abs(i - sp_index_original);
		const float bin_cost = deviation_cost - mean_dist - _obstacle_map_body_frame.distances[bin];

		if (bin_cost < best_cost && _obstacle_map_body_frame.distances[bin] != UINT16_MAX) {
			best_cost = bin_cost;
			new_sp_index = bin;
		}
	}

	//only change setpoint direction if it was moved to a different bin
	if (new_sp_index != setpoint_index) {
		float angle = math::radians((float)new_sp_index * BIN_SIZE + _obstacle_map_body_frame.angle_offset);
		angle = wrap_2pi(vehicle_yaw_angle_rad + angle);
		setpoint_dir = {cosf(angle), sinf(angle)};
		setpoint_index = new_sp_index;
	}
}

float CollisionPrevention::_getObstacleDistance(const Vector2f &direction)
{
	float obstacle_distance = 0.f;
	const float direction_norm = direction.norm();

	if (direction_norm > FLT_EPSILON) {
		Vector2f dir = direction / direction_norm;
		const float sp_angle_body_frame = atan2f(dir(1), dir(0)) - _vehicle_yaw;
		const float sp_angle_with_offset_deg =
			ObstacleMath::wrap_360(math::degrees(sp_angle_body_frame) - _obstacle_map_body_frame.angle_offset);

		const int dir_index = ObstacleMath::get_bin_at_angle(BIN_SIZE, sp_angle_with_offset_deg);
		obstacle_distance   = _obstacle_map_body_frame.distances[dir_index] * 0.01f;
	}

	return obstacle_distance;
}

Vector2f
CollisionPrevention::_constrainAccelerationSetpoint(const float &setpoint_length)
{
	Vector2f new_setpoint{};
	const Vector2f normal_component = _closest_dist_dir * (_setpoint_dir.dot(_closest_dist_dir));
	const Vector2f tangential_component = _setpoint_dir - normal_component;

	const float normal_scale = _getScale(_closest_dist);


	const float closest_dist_tangential = _getObstacleDistance(tangential_component);
	const float tangential_scale = _getScale(closest_dist_tangential);


	// only scale accelerations towards the obstacle
	if (_closest_dist_dir.dot(_setpoint_dir) > 0) {
		new_setpoint = (tangential_component * tangential_scale + normal_component * normal_scale) * setpoint_length;

	} else {
		new_setpoint = _setpoint_dir * setpoint_length;
	}

	return new_setpoint;
}

float
CollisionPrevention::_getScale(const float &reference_distance)
{
	float scale = (reference_distance - _min_dist_to_keep);
	const float scale_distance = math::max(_min_dist_to_keep, _param_mpc_vel_manual.get() / _param_mpc_xy_p.get());

	// if scale is positive, square it and scale it with the scale_distance
	scale = scale > 0 ? powf(scale / scale_distance, 2) : scale;
	scale = math::min(scale, 1.0f);
	return scale;
}

void CollisionPrevention::_getVelocityCompensationAcceleration(const float vehicle_yaw_angle_rad,
		const Vector2f &setpoint_vel,
		const hrt_abstime now, float &vel_comp_accel, Vector2f &vel_comp_accel_dir)
{
	for (int i = 0; i < BIN_COUNT; i++) {
		const float max_range = _data_maxranges[i] * 0.01f;

		// get the vector pointing into the direction of current bin
		float bin_angle = wrap_2pi(vehicle_yaw_angle_rad
					   + math::radians((float)i * BIN_SIZE + _obstacle_map_body_frame.angle_offset));

		const Vector2f bin_direction = { cosf(bin_angle), sinf(bin_angle) };
		float bin_distance = _obstacle_map_body_frame.distances[i];

		// only consider bins which are between min and max values
		if (bin_distance > _obstacle_map_body_frame.min_distance && bin_distance < UINT16_MAX) {
			const float distance = bin_distance * 0.01f;

			// Assume current velocity is sufficiently close to the setpoint velocity, this breaks down if flying high
			// acceleration maneuvers
			const float curr_vel_parallel = math::max(0.f, setpoint_vel.dot(bin_direction));
			float delay_distance = curr_vel_parallel * _param_cp_delay.get();

			const hrt_abstime data_age = now - _data_timestamps[i];

			if (distance < max_range) {
				delay_distance += curr_vel_parallel * (data_age * 1e-6f);
			}

			const float stop_distance = distance - _min_dist_to_keep - delay_distance;

			float curr_acc_vel_constraint;

			if (stop_distance >= 0.f) {
				const float max_vel = math::trajectory::computeMaxSpeedFromDistance(_param_mpc_jerk_max.get(),
						      _param_mpc_acc_hor.get(), stop_distance, 0.f);
				curr_acc_vel_constraint = _param_mpc_xy_vel_p_acc.get() * math::min(max_vel - curr_vel_parallel, 0.f);

			} else {
				curr_acc_vel_constraint = -1.f * _param_mpc_xy_vel_p_acc.get() * curr_vel_parallel;
			}

			if (curr_acc_vel_constraint < vel_comp_accel) {
				vel_comp_accel = curr_acc_vel_constraint;
				vel_comp_accel_dir = bin_direction;
			}
		}
	}
}

void CollisionPrevention::_publishVehicleCmdDoLoiter()
{
	vehicle_command_s command{};
	command.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
	command.param1 = 1.f; // base mode VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED
	command.param2 = (float)PX4_CUSTOM_MAIN_MODE_AUTO;
	command.param3 = (float)PX4_CUSTOM_SUB_MODE_AUTO_LOITER;
	command.target_system = 1;
	command.target_component = 1;
	command.source_system = 1;
	command.source_component = 1;
	command.confirmation = false;
	command.from_external = false;
	command.timestamp = getTime();
	_vehicle_command_pub.publish(command);
}
