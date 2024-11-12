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

#include "CollisionPrevention.hpp"
#include <px4_platform_common/events.h>

using namespace matrix;

namespace
{
static constexpr int INTERNAL_MAP_INCREMENT_DEG = 10; //cannot be lower than 5 degrees, should divide 360 evenly
static constexpr int INTERNAL_MAP_USED_BINS = 360 / INTERNAL_MAP_INCREMENT_DEG;

static float wrap_360(float f)
{
	return wrap(f, 0.f, 360.f);
}

static int wrap_bin(int i)
{
	i = i % INTERNAL_MAP_USED_BINS;

	while (i < 0) {
		i += INTERNAL_MAP_USED_BINS;
	}

	return i;
}

} // namespace

CollisionPrevention::CollisionPrevention(ModuleParams *parent) :
	ModuleParams(parent)
{
	static_assert(INTERNAL_MAP_INCREMENT_DEG >= 5, "INTERNAL_MAP_INCREMENT_DEG needs to be at least 5");
	static_assert(360 % INTERNAL_MAP_INCREMENT_DEG == 0, "INTERNAL_MAP_INCREMENT_DEG should divide 360 evenly");

	// initialize internal obstacle map
	_obstacle_map_body_frame.timestamp = getTime();
	_obstacle_map_body_frame.frame = obstacle_distance_s::MAV_FRAME_BODY_FRD;
	_obstacle_map_body_frame.increment = INTERNAL_MAP_INCREMENT_DEG;
	_obstacle_map_body_frame.min_distance = UINT16_MAX;
	_obstacle_map_body_frame.max_distance = 0;
	_obstacle_map_body_frame.angle_offset = 0.f;
	uint32_t internal_bins = sizeof(_obstacle_map_body_frame.distances) / sizeof(_obstacle_map_body_frame.distances[0]);
	uint64_t current_time = getTime();

	for (uint32_t i = 0 ; i < internal_bins; i++) {
		_data_timestamps[i] = current_time;
		_data_maxranges[i] = 0;
		_data_fov[i] = 0;
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

void
CollisionPrevention::_addObstacleSensorData(const obstacle_distance_s &obstacle, const Quatf &vehicle_attitude)
{
	int msg_index = 0;
	float vehicle_orientation_deg = math::degrees(Eulerf(vehicle_attitude).psi());
	float increment_factor = 1.f / obstacle.increment;

	if (obstacle.frame == obstacle.MAV_FRAME_GLOBAL || obstacle.frame == obstacle.MAV_FRAME_LOCAL_NED) {
		// Obstacle message arrives in local_origin frame (north aligned)
		// corresponding data index (convert to world frame and shift by msg offset)
		for (int i = 0; i < INTERNAL_MAP_USED_BINS; i++) {
			float bin_angle_deg = (float)i * INTERNAL_MAP_INCREMENT_DEG + _obstacle_map_body_frame.angle_offset;
			msg_index = ceil(wrap_360(vehicle_orientation_deg + bin_angle_deg - obstacle.angle_offset) * increment_factor);

			//add all data points inside to FOV
			if (obstacle.distances[msg_index] != UINT16_MAX) {
				if (_enterData(i, obstacle.max_distance * 0.01f, obstacle.distances[msg_index] * 0.01f)) {
					_obstacle_map_body_frame.distances[i] = obstacle.distances[msg_index];
					_data_timestamps[i] = _obstacle_map_body_frame.timestamp;
					_data_maxranges[i] = obstacle.max_distance;
					_data_fov[i] = 1;
				}
			}
		}

	} else if (obstacle.frame == obstacle.MAV_FRAME_BODY_FRD) {
		// Obstacle message arrives in body frame (front aligned)
		// corresponding data index (shift by msg offset)
		for (int i = 0; i < INTERNAL_MAP_USED_BINS; i++) {
			float bin_angle_deg = (float)i * INTERNAL_MAP_INCREMENT_DEG +
					      _obstacle_map_body_frame.angle_offset;
			msg_index = ceil(wrap_360(bin_angle_deg - obstacle.angle_offset) * increment_factor);

			//add all data points inside to FOV
			if (obstacle.distances[msg_index] != UINT16_MAX) {

				if (_enterData(i, obstacle.max_distance * 0.01f, obstacle.distances[msg_index] * 0.01f)) {
					_obstacle_map_body_frame.distances[i] = obstacle.distances[msg_index];
					_data_timestamps[i] = _obstacle_map_body_frame.timestamp;
					_data_maxranges[i] = obstacle.max_distance;
					_data_fov[i] = 1;
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

	for (int i = 0; i < INTERNAL_MAP_USED_BINS; i++) {
		// check if our setpoint is either pointing in a direction where data exists, or if not, wether we are allowed to go where there is no data
		if ((_obstacle_map_body_frame.distances[i] == UINT16_MAX && i == _setpoint_index) && (!_param_cp_go_nodata.get()
				|| (_param_cp_go_nodata.get() && _data_fov[i]))) {
			setpoint_feasible =  false;

		}
	}

	return setpoint_feasible;
}

void
CollisionPrevention::_transformSetpoint(const Vector2f &setpoint)
{
	const float vehicle_yaw_angle_rad = Eulerf(Quatf(_sub_vehicle_attitude.get().q)).psi();
	_setpoint_dir = setpoint / setpoint.norm();;
	const float sp_angle_body_frame = atan2f(_setpoint_dir(1), _setpoint_dir(0)) - vehicle_yaw_angle_rad;
	const float sp_angle_with_offset_deg = wrap_360(math::degrees(sp_angle_body_frame) -
					       _obstacle_map_body_frame.angle_offset);
	_setpoint_index = floor(sp_angle_with_offset_deg / INTERNAL_MAP_INCREMENT_DEG);
	// change setpoint direction slightly (max by _param_cp_guide_ang degrees) to help guide through narrow gaps
	_adaptSetpointDirection(_setpoint_dir, _setpoint_index, vehicle_yaw_angle_rad);
}

void
CollisionPrevention::_updateObstacleMap()
{
	_sub_vehicle_attitude.update();

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
			_obstacle_map_body_frame.max_distance = math::max(_obstacle_map_body_frame.max_distance,
								obstacle_distance.max_distance);
			_obstacle_map_body_frame.min_distance = math::min(_obstacle_map_body_frame.min_distance,
								obstacle_distance.min_distance);
			_addObstacleSensorData(obstacle_distance, Quatf(_sub_vehicle_attitude.get().q));
		}
	}

	// publish fused obtacle distance message with data from offboard obstacle_distance and distance sensor
	_obstacle_distance_pub.publish(_obstacle_map_body_frame);
}

void CollisionPrevention::_updateObstacleData()
{
	_obstacle_data_present = false;
	_closest_dist = UINT16_MAX;
	_closest_dist_dir.setZero();
	const float vehicle_yaw_angle_rad = Eulerf(Quatf(_sub_vehicle_attitude.get().q)).psi();

	for (int i = 0; i < INTERNAL_MAP_USED_BINS; i++) {

		// if the data is stale, reset the bin
		if (getTime() - _data_timestamps[i] > RANGE_STREAM_TIMEOUT_US) {
			_obstacle_map_body_frame.distances[i] = UINT16_MAX;
		}

		float angle = wrap_2pi(vehicle_yaw_angle_rad + math::radians((float)i * INTERNAL_MAP_INCREMENT_DEG +
				       _obstacle_map_body_frame.angle_offset));
		const Vector2f bin_direction = {cosf(angle), sinf(angle)};
		uint bin_distance = _obstacle_map_body_frame.distances[i];

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
	if ((distance_reading > distance_sensor.min_distance)) {

		float sensor_yaw_body_rad = _sensorOrientationToYawOffset(distance_sensor, _obstacle_map_body_frame.angle_offset);
		float sensor_yaw_body_deg = math::degrees(wrap_2pi(sensor_yaw_body_rad));

		// calculate the field of view boundary bin indices
		int lower_bound = (int)floor((sensor_yaw_body_deg  - math::degrees(distance_sensor.h_fov / 2.0f)) /
					     INTERNAL_MAP_INCREMENT_DEG);
		int upper_bound = (int)floor((sensor_yaw_body_deg  + math::degrees(distance_sensor.h_fov / 2.0f)) /
					     INTERNAL_MAP_INCREMENT_DEG);

		// floor values above zero, ceil values below zero
		if (lower_bound < 0) { lower_bound++; }

		if (upper_bound < 0) { upper_bound++; }

		// rotate vehicle attitude into the sensor body frame
		Quatf attitude_sensor_frame = vehicle_attitude;
		attitude_sensor_frame.rotate(Vector3f(0.f, 0.f, sensor_yaw_body_rad));
		float sensor_dist_scale = cosf(Eulerf(attitude_sensor_frame).theta());

		if (distance_reading < distance_sensor.max_distance) {
			distance_reading = distance_reading * sensor_dist_scale;
		}

		uint16_t sensor_range = static_cast<uint16_t>(100.0f * distance_sensor.max_distance + 0.5f); // convert to cm

		for (int bin = lower_bound; bin <= upper_bound; ++bin) {
			int wrapped_bin = wrap_bin(bin);

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
	const int guidance_bins = floor(_param_cp_guide_ang.get() / INTERNAL_MAP_INCREMENT_DEG);
	const int sp_index_original = setpoint_index;
	float best_cost = 9999.f;
	int new_sp_index = setpoint_index;

	for (int i = sp_index_original - guidance_bins; i <= sp_index_original + guidance_bins; i++) {

		// apply moving average filter to the distance array to be able to center in larger gaps
		const int filter_size = 1;
		float mean_dist = 0;

		for (int j = i - filter_size; j <= i + filter_size; j++) {
			int bin = wrap_bin(j);

			if (_obstacle_map_body_frame.distances[bin] == UINT16_MAX) {
				mean_dist += _param_cp_dist.get() * 100.f;

			} else {
				mean_dist += _obstacle_map_body_frame.distances[bin];
			}
		}

		const int bin = wrap_bin(i);
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
		float angle = math::radians((float)new_sp_index * INTERNAL_MAP_INCREMENT_DEG + _obstacle_map_body_frame.angle_offset);
		angle = wrap_2pi(vehicle_yaw_angle_rad + angle);
		setpoint_dir = {cosf(angle), sinf(angle)};
		setpoint_index = new_sp_index;
	}
}

float
CollisionPrevention::_sensorOrientationToYawOffset(const distance_sensor_s &distance_sensor, float angle_offset) const
{
	float offset = angle_offset > 0.0f ? math::radians(angle_offset) : 0.0f;

	switch (distance_sensor.orientation) {
	case distance_sensor_s::ROTATION_YAW_0:
		offset = 0.0f;
		break;

	case distance_sensor_s::ROTATION_YAW_45:
		offset = M_PI_F / 4.0f;
		break;

	case distance_sensor_s::ROTATION_YAW_90:
		offset = M_PI_F / 2.0f;
		break;

	case distance_sensor_s::ROTATION_YAW_135:
		offset = 3.0f * M_PI_F / 4.0f;
		break;

	case distance_sensor_s::ROTATION_YAW_180:
		offset = M_PI_F;
		break;

	case distance_sensor_s::ROTATION_YAW_225:
		offset = -3.0f * M_PI_F / 4.0f;
		break;

	case distance_sensor_s::ROTATION_YAW_270:
		offset = -M_PI_F / 2.0f;
		break;

	case distance_sensor_s::ROTATION_YAW_315:
		offset = -M_PI_F / 4.0f;
		break;

	case distance_sensor_s::ROTATION_CUSTOM:
		offset = Eulerf(Quatf(distance_sensor.q)).psi();
		break;
	}

	return offset;
}

void
CollisionPrevention::_calculateConstrainedSetpoint(Vector2f &setpoint_accel, const Vector2f &setpoint_vel)
{
	_updateObstacleMap();
	_updateObstacleData();

	const Quatf attitude = Quatf(_sub_vehicle_attitude.get().q);
	const float vehicle_yaw_angle_rad = Eulerf(attitude).psi();

	const float setpoint_length = setpoint_accel.norm();
	_min_dist_to_keep = math::max(_obstacle_map_body_frame.min_distance / 100.0f, _param_cp_dist.get());

	const hrt_abstime now = getTime();

	float vel_comp_accel = INFINITY;
	Vector2f vel_comp_accel_dir{};
	Vector2f constr_accel_setpoint{};

	const bool is_stick_deflected = setpoint_length > 0.001f;

	if (_obstacle_data_present && is_stick_deflected) {

		_transformSetpoint(setpoint_accel);

		_getVelocityCompensationAcceleration(vehicle_yaw_angle_rad, setpoint_vel, now,
						     vel_comp_accel, vel_comp_accel_dir);

		if (_checkSetpointDirectionFeasability()) {
			constr_accel_setpoint = _constrainAccelerationSetpoint(setpoint_length);
		}

		setpoint_accel = constr_accel_setpoint + vel_comp_accel * vel_comp_accel_dir;

	} else if (!_obstacle_data_present)

	{
		// allow no movement
		PX4_WARN("No obstacle data, not moving...");
		setpoint_accel.setZero();

		// if distance data is stale, switch to Loiter
		if (getElapsedTime(&_last_timeout_warning) > 1_s && getElapsedTime(&_time_activated) > 1_s) {
			if ((now - _obstacle_map_body_frame.timestamp) > TIMEOUT_HOLD_US &&
			    getElapsedTime(&_time_activated) > TIMEOUT_HOLD_US) {
				_publishVehicleCmdDoLoiter();
			}

			_last_timeout_warning = getTime();
		}
	}
}

float CollisionPrevention::_getObstacleDistance(const Vector2f &direction)
{
	const float direction_norm = direction.norm();

	if (direction_norm > FLT_EPSILON) {
		Vector2f dir = direction / direction_norm;
		const float vehicle_yaw_angle_rad = Eulerf(Quatf(_sub_vehicle_attitude.get().q)).psi();
		const float sp_angle_body_frame = atan2f(dir(1), dir(0)) - vehicle_yaw_angle_rad;
		const float sp_angle_with_offset_deg =
			wrap_360(math::degrees(sp_angle_body_frame) - _obstacle_map_body_frame.angle_offset);
		int dir_index = floor(sp_angle_with_offset_deg / INTERNAL_MAP_INCREMENT_DEG);
		dir_index = math::constrain(dir_index, 0, 71);
		return _obstacle_map_body_frame.distances[dir_index] * 0.01f;

	} else {
		return 0.f;
	}
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
	for (int i = 0; i < INTERNAL_MAP_USED_BINS; i++) {
		const float max_range = _data_maxranges[i] * 0.01f;

		// get the vector pointing into the direction of current bin
		float bin_angle = wrap_2pi(vehicle_yaw_angle_rad + math::radians((float)i * INTERNAL_MAP_INCREMENT_DEG +
					   _obstacle_map_body_frame.angle_offset));

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

			const float stop_distance = math::max(0.f, distance - _min_dist_to_keep - delay_distance);

			const float max_vel = math::trajectory::computeMaxSpeedFromDistance(_param_mpc_jerk_max.get(),
					      _param_mpc_acc_hor.get(), stop_distance, 0.f);
			// we dont take the minimum of the last term because of stop_distance is zero but current velocity is not, we want the acceleration to become negative and slow us down.
			const float curr_acc_vel_constraint = _param_mpc_vel_p_acc.get() * (max_vel - curr_vel_parallel);

			if (curr_acc_vel_constraint < vel_comp_accel) {
				vel_comp_accel = curr_acc_vel_constraint;
				vel_comp_accel_dir = bin_direction;
			}
		}
	}
}

void
CollisionPrevention::modifySetpoint(Vector2f &setpoint_accel, const Vector2f &setpoint_vel)
{
	//calculate movement constraints based on range data
	Vector2f original_setpoint = setpoint_accel;
	_calculateConstrainedSetpoint(setpoint_accel, setpoint_vel);

	// publish constraints
	collision_constraints_s	constraints{};
	original_setpoint.copyTo(constraints.original_setpoint);
	setpoint_accel.copyTo(constraints.adapted_setpoint);
	constraints.timestamp = getTime();
	_constraints_pub.publish(constraints);
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
