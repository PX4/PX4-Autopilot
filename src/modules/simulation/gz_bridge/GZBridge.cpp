/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#include "GZBridge.hpp"

#include <uORB/Subscription.hpp>

#include <lib/atmosphere/atmosphere.h>
#include <lib/mathlib/mathlib.h>

#include <px4_platform_common/getopt.h>

#include <iostream>
#include <string>

GZBridge::GZBridge(const std::string &world, const std::string &model_name) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_world_name(world),
	_model_name(model_name)
{
	updateParams();
}

GZBridge::~GZBridge()
{
	for (auto &sub_topic : _node.SubscribedTopics()) {
		_node.Unsubscribe(sub_topic);
	}
}

int GZBridge::init()
{
	// clock
	std::string clock_topic = "/world/" + _world_name + "/clock";

	if (!_node.Subscribe(clock_topic, &GZBridge::clockCallback, this)) {
		PX4_ERR("failed to subscribe to %s", clock_topic.c_str());
		return PX4_ERROR;
	}

	// pose: /world/$WORLD/pose/info
	std::string world_pose_topic = "/world/" + _world_name + "/pose/info";

	if (!_node.Subscribe(world_pose_topic, &GZBridge::poseInfoCallback, this)) {
		PX4_ERR("failed to subscribe to %s", world_pose_topic.c_str());
		return PX4_ERROR;
	}

	// IMU: /world/$WORLD/model/$MODEL/link/base_link/sensor/imu_sensor/imu
	std::string imu_topic = "/world/" + _world_name + "/model/" + _model_name + "/link/base_link/sensor/imu_sensor/imu";

	if (!_node.Subscribe(imu_topic, &GZBridge::imuCallback, this)) {
		PX4_ERR("failed to subscribe to %s", imu_topic.c_str());
		return PX4_ERROR;
	}

	// mag: /world/$WORLD/model/$MODEL/link/base_link/sensor/magnetometer_sensor/magnetometer
	std::string mag_topic = "/world/" + _world_name + "/model/" + _model_name +
				"/link/base_link/sensor/magnetometer_sensor/magnetometer";

	if (!_node.Subscribe(mag_topic, &GZBridge::magnetometerCallback, this)) {
		PX4_ERR("failed to subscribe to %s", mag_topic.c_str());
		return PX4_ERROR;
	}

	// odom: /world/$WORLD/model/$MODEL/link/base_link/odometry_with_covariance
	std::string odometry_topic = "/model/" + _model_name + "/odometry_with_covariance";

	if (!_node.Subscribe(odometry_topic, &GZBridge::odometryCallback, this)) {
		PX4_ERR("failed to subscribe to %s", odometry_topic.c_str());
		return PX4_ERROR;
	}

	// Laser Scan: optional
	std::string laser_scan_topic = "/world/" + _world_name + "/model/" + _model_name + "/link/link/sensor/lidar_2d_v2/scan";

	if (!_node.Subscribe(laser_scan_topic, &GZBridge::laserScanCallback, this)) {
		PX4_WARN("failed to subscribe to %s", laser_scan_topic.c_str());
	}

	// Distance Sensor(AFBRS50): optional
	std::string lidar_sensor = "/world/" + _world_name + "/model/" + _model_name +
				   "/link/lidar_sensor_link/sensor/lidar/scan";

	if (!_node.Subscribe(lidar_sensor, &GZBridge::laserScantoLidarSensorCallback, this)) {
		PX4_WARN("failed to subscribe to %s", lidar_sensor.c_str());
	}

	// Airspeed: /world/$WORLD/model/$MODEL/link/airspeed_link/sensor/air_speed/air_speed
	std::string airspeed_topic = "/world/" + _world_name + "/model/" + _model_name +
				     "/link/airspeed_link/sensor/air_speed/air_speed";

	if (!_node.Subscribe(airspeed_topic, &GZBridge::airspeedCallback, this)) {
		PX4_ERR("failed to subscribe to %s", airspeed_topic.c_str());
		return PX4_ERROR;
	}

	// Air pressure: /world/$WORLD/model/$MODEL/link/base_link/sensor/air_pressure_sensor/air_pressure
	std::string air_pressure_topic = "/world/" + _world_name + "/model/" + _model_name +
					 "/link/base_link/sensor/air_pressure_sensor/air_pressure";

	if (!_node.Subscribe(air_pressure_topic, &GZBridge::barometerCallback, this)) {
		PX4_ERR("failed to subscribe to %s", air_pressure_topic.c_str());
		return PX4_ERROR;
	}

	// GPS: /world/$WORLD/model/$MODEL/link/base_link/sensor/navsat_sensor/navsat
	std::string nav_sat_topic = "/world/" + _world_name + "/model/" + _model_name +
				    "/link/base_link/sensor/navsat_sensor/navsat";

	if (!_node.Subscribe(nav_sat_topic, &GZBridge::navSatCallback, this)) {
		PX4_ERR("failed to subscribe to %s", nav_sat_topic.c_str());
		return PX4_ERROR;
	}

	std::string flow_topic = "/world/" + _world_name + "/model/" + _model_name +
				 "/link/flow_link/sensor/optical_flow/optical_flow";

	if (!_node.Subscribe(flow_topic, &GZBridge::opticalFlowCallback, this)) {
		PX4_ERR("failed to subscribe to %s", flow_topic.c_str());
		return PX4_ERROR;
	}

	if (!_mixing_interface_esc.init(_model_name)) {
		PX4_ERR("failed to init ESC output");
		return PX4_ERROR;
	}

	if (!_mixing_interface_servo.init(_model_name)) {
		PX4_ERR("failed to init servo output");
		return PX4_ERROR;
	}

	if (!_mixing_interface_wheel.init(_model_name)) {
		PX4_ERR("failed to init motor output");
		return PX4_ERROR;
	}

	if (!_gimbal.init(_world_name, _model_name)) {
		PX4_ERR("failed to init gimbal");
		return PX4_ERROR;
	}

	ScheduleNow();
	return OK;
}

void GZBridge::clockCallback(const gz::msgs::Clock &msg)
{
	// NOTE: PX4-SITL time needs to stay in sync with gz, so this clock-sync will happen on every callback.
	struct timespec ts;
	ts.tv_sec = msg.sim().sec();
	ts.tv_nsec = msg.sim().nsec();

	if (!_realtime_clock_set) {
		// Set initial real time clock at startup
		px4_clock_settime(CLOCK_REALTIME, &ts);
		_realtime_clock_set = true;

	} else {
		// Keep monotonic clock synchronized
		px4_clock_settime(CLOCK_MONOTONIC, &ts);
	}
}

void GZBridge::opticalFlowCallback(const px4::msgs::OpticalFlow &msg)
{
	sensor_optical_flow_s report = {};

	report.timestamp = hrt_absolute_time();
	report.timestamp_sample = msg.time_usec();
	report.pixel_flow[0] = msg.integrated_x();
	report.pixel_flow[1] = msg.integrated_y();
	report.quality = msg.quality();
	report.integration_timespan_us = msg.integration_time_us();

	// Static data
	device::Device::DeviceId id;
	id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SIMULATION;
	id.devid_s.bus = 0;
	id.devid_s.address = 0;
	id.devid_s.devtype = DRV_FLOW_DEVTYPE_SIM;
	report.device_id = id.devid;

	// values taken from PAW3902
	report.mode = sensor_optical_flow_s::MODE_LOWLIGHT;
	report.max_flow_rate = 7.4f;
	report.min_ground_distance = 0.f;
	report.max_ground_distance = 30.f;
	report.error_count = 0;

	// No delta angle
	// No distance
	// This means that delta angle will come from vehicle gyro
	// Distance will come from vehicle distance sensor

	_optical_flow_pub.publish(report);
}

void GZBridge::magnetometerCallback(const gz::msgs::Magnetometer &msg)
{
	const uint64_t timestamp = hrt_absolute_time();

	device::Device::DeviceId id{};
	id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SIMULATION;
	id.devid_s.devtype = DRV_MAG_DEVTYPE_MAGSIM;
	id.devid_s.bus = 1;
	id.devid_s.address = 3; // TODO: any value other than 3 causes Commander to not use the mag.... wtf

	sensor_mag_s report{};
	report.timestamp = timestamp;
	report.timestamp_sample = timestamp;
	report.device_id = id.devid;
	report.temperature = this->_temperature;

	// FIMEX: once we're on jetty or later
	// The magnetometer plugin publishes in units of gauss and in a weird left handed coordinate system
	// https://github.com/gazebosim/gz-sim/pull/2460
	report.x = -msg.field_tesla().y();
	report.y = -msg.field_tesla().x();
	report.z = msg.field_tesla().z();

	_sensor_mag_pub.publish(report);
}

void GZBridge::barometerCallback(const gz::msgs::FluidPressure &msg)
{
	const uint64_t timestamp = hrt_absolute_time();

	device::Device::DeviceId id{};
	id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SIMULATION;
	id.devid_s.devtype = DRV_BARO_DEVTYPE_BAROSIM;
	id.devid_s.bus = 1;
	id.devid_s.address = 1;

	sensor_baro_s report{};
	report.timestamp = timestamp;
	report.timestamp_sample = timestamp;
	report.device_id = id.devid;
	report.pressure = msg.pressure();
	report.temperature = this->_temperature;
	_sensor_baro_pub.publish(report);
}


void GZBridge::airspeedCallback(const gz::msgs::AirSpeed &msg)
{
	const uint64_t timestamp = hrt_absolute_time();

	device::Device::DeviceId id{};
	id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SIMULATION;
	id.devid_s.devtype = DRV_DIFF_PRESS_DEVTYPE_SIM;
	id.devid_s.bus = 1;
	id.devid_s.address = 1;

	differential_pressure_s report{};
	report.timestamp = timestamp;
	report.timestamp_sample = timestamp;
	report.device_id = id.devid;
	report.differential_pressure_pa = msg.diff_pressure(); // hPa to Pa;
	report.temperature = static_cast<float>(msg.temperature()) + atmosphere::kAbsoluteNullCelsius; // K to C
	_differential_pressure_pub.publish(report);

	this->_temperature = report.temperature;
}

void GZBridge::imuCallback(const gz::msgs::IMU &msg)
{
	const uint64_t timestamp = hrt_absolute_time();

	// FLU -> FRD
	static const auto q_FLU_to_FRD = gz::math::Quaterniond(0, 1, 0, 0);

	gz::math::Vector3d accel_b = q_FLU_to_FRD.RotateVector(gz::math::Vector3d(
					     msg.linear_acceleration().x(),
					     msg.linear_acceleration().y(),
					     msg.linear_acceleration().z()));

	device::Device::DeviceId id{};
	id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SIMULATION;
	id.devid_s.devtype = DRV_IMU_DEVTYPE_SIM;
	id.devid_s.bus = 1;
	id.devid_s.address = 1;

	// publish accel
	sensor_accel_s accel{};

	accel.timestamp_sample = timestamp;
	accel.timestamp = timestamp;
	accel.device_id = id.devid;

	accel.x = accel_b.X();
	accel.y = accel_b.Y();
	accel.z = accel_b.Z();
	accel.temperature = NAN;
	accel.samples = 1;
	_sensor_accel_pub.publish(accel);

	gz::math::Vector3d gyro_b = q_FLU_to_FRD.RotateVector(gz::math::Vector3d(
					    msg.angular_velocity().x(),
					    msg.angular_velocity().y(),
					    msg.angular_velocity().z()));

	// publish gyro
	sensor_gyro_s gyro{};
	gyro.timestamp_sample = timestamp;
	gyro.timestamp = timestamp;
	gyro.device_id = id.devid;
	gyro.x = gyro_b.X();
	gyro.y = gyro_b.Y();
	gyro.z = gyro_b.Z();
	gyro.temperature = NAN;
	gyro.samples = 1;
	_sensor_gyro_pub.publish(gyro);
}

void GZBridge::poseInfoCallback(const gz::msgs::Pose_V &msg)
{
	const uint64_t timestamp = hrt_absolute_time();

	for (int p = 0; p < msg.pose_size(); p++) {
		if (msg.pose(p).name() == _model_name) {

			const double dt = math::constrain((timestamp - _timestamp_prev) * 1e-6, 0.001, 0.1);
			_timestamp_prev = timestamp;

			gz::msgs::Vector3d pose_position = msg.pose(p).position();
			gz::msgs::Quaternion pose_orientation = msg.pose(p).orientation();

			// ground truth
			gz::math::Quaterniond q_gr = gz::math::Quaterniond(
							     pose_orientation.w(),
							     pose_orientation.x(),
							     pose_orientation.y(),
							     pose_orientation.z());

			gz::math::Quaterniond q_nb;
			GZBridge::rotateQuaternion(q_nb, q_gr);

			// publish attitude groundtruth
			vehicle_attitude_s vehicle_attitude_groundtruth{};
			vehicle_attitude_groundtruth.timestamp_sample = timestamp;
			vehicle_attitude_groundtruth.q[0] = q_nb.W();
			vehicle_attitude_groundtruth.q[1] = q_nb.X();
			vehicle_attitude_groundtruth.q[2] = q_nb.Y();
			vehicle_attitude_groundtruth.q[3] = q_nb.Z();
			vehicle_attitude_groundtruth.timestamp = timestamp;
			_attitude_ground_truth_pub.publish(vehicle_attitude_groundtruth);

			// publish angular velocity groundtruth
			const matrix::Eulerf euler{matrix::Quatf(vehicle_attitude_groundtruth.q)};
			vehicle_angular_velocity_s vehicle_angular_velocity_groundtruth{};
			vehicle_angular_velocity_groundtruth.timestamp_sample = timestamp;
			const matrix::Vector3f angular_velocity = (euler - _euler_prev) / dt;
			_euler_prev = euler;
			angular_velocity.copyTo(vehicle_angular_velocity_groundtruth.xyz);

			vehicle_angular_velocity_groundtruth.timestamp = timestamp;
			_angular_velocity_ground_truth_pub.publish(vehicle_angular_velocity_groundtruth);

			vehicle_local_position_s local_position_groundtruth{};
			local_position_groundtruth.timestamp_sample = timestamp;
			// position ENU -> NED
			const matrix::Vector3d position{pose_position.y(), pose_position.x(), -pose_position.z()};
			const matrix::Vector3d velocity{(position - _position_prev) / dt};
			const matrix::Vector3d acceleration{(velocity - _velocity_prev) / dt};

			_position_prev = position;
			_velocity_prev = velocity;

			local_position_groundtruth.ax = acceleration(0);
			local_position_groundtruth.ay = acceleration(1);
			local_position_groundtruth.az = acceleration(2);
			local_position_groundtruth.vx = velocity(0);
			local_position_groundtruth.vy = velocity(1);
			local_position_groundtruth.vz = velocity(2);
			local_position_groundtruth.x = position(0);
			local_position_groundtruth.y = position(1);
			local_position_groundtruth.z = position(2);

			local_position_groundtruth.heading = euler.psi();

			if (_pos_ref.isInitialized()) {

				local_position_groundtruth.ref_lat = _pos_ref.getProjectionReferenceLat(); // Reference point latitude in degrees
				local_position_groundtruth.ref_lon = _pos_ref.getProjectionReferenceLon(); // Reference point longitude in degrees
				local_position_groundtruth.ref_alt = _alt_ref;
				local_position_groundtruth.ref_timestamp = _pos_ref.getProjectionReferenceTimestamp();
				local_position_groundtruth.xy_global = true;
				local_position_groundtruth.z_global = true;

			} else {
				local_position_groundtruth.ref_lat = static_cast<double>(NAN);
				local_position_groundtruth.ref_lon = static_cast<double>(NAN);
				local_position_groundtruth.ref_alt = NAN;
				local_position_groundtruth.ref_timestamp = 0;
				local_position_groundtruth.xy_global = false;
				local_position_groundtruth.z_global = false;
			}

			local_position_groundtruth.timestamp = timestamp;
			_lpos_ground_truth_pub.publish(local_position_groundtruth);
			return;
		}
	}
}

void GZBridge::odometryCallback(const gz::msgs::OdometryWithCovariance &msg)
{
	const uint64_t timestamp = hrt_absolute_time();

	vehicle_odometry_s report{};
	report.timestamp_sample = timestamp;
	report.timestamp = timestamp;

	// gz odometry position is in ENU frame and needs to be converted to NED
	report.pose_frame = vehicle_odometry_s::POSE_FRAME_NED;
	report.position[0] = msg.pose_with_covariance().pose().position().y();
	report.position[1] = msg.pose_with_covariance().pose().position().x();
	report.position[2] = -msg.pose_with_covariance().pose().position().z();

	// gz odometry orientation is "body FLU->ENU" and needs to be converted in "body FRD->NED"
	gz::msgs::Quaternion pose_orientation = msg.pose_with_covariance().pose().orientation();
	gz::math::Quaterniond q_gr = gz::math::Quaterniond(
					     pose_orientation.w(),
					     pose_orientation.x(),
					     pose_orientation.y(),
					     pose_orientation.z());
	gz::math::Quaterniond q_nb;
	GZBridge::rotateQuaternion(q_nb, q_gr);
	report.q[0] = q_nb.W();
	report.q[1] = q_nb.X();
	report.q[2] = q_nb.Y();
	report.q[3] = q_nb.Z();

	// gz odometry linear velocity is in body FLU and needs to be converted in body FRD
	report.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_BODY_FRD;
	report.velocity[0] = msg.twist_with_covariance().twist().linear().x();
	report.velocity[1] = -msg.twist_with_covariance().twist().linear().y();
	report.velocity[2] = -msg.twist_with_covariance().twist().linear().z();

	// gz odometry angular velocity is in body FLU and need to be converted in body FRD
	report.angular_velocity[0] = msg.twist_with_covariance().twist().angular().x();
	report.angular_velocity[1] = -msg.twist_with_covariance().twist().angular().y();
	report.angular_velocity[2] = -msg.twist_with_covariance().twist().angular().z();

	// VISION_POSITION_ESTIMATE covariance
	//  pose 6x6 cross-covariance matrix
	//  (states: x, y, z, roll, pitch, yaw).
	//  If unknown, assign NaN value to first element in the array.
	report.position_variance[0] = msg.pose_with_covariance().covariance().data(7);  // Y  row 1, col 1
	report.position_variance[1] = msg.pose_with_covariance().covariance().data(0);  // X  row 0, col 0
	report.position_variance[2] = msg.pose_with_covariance().covariance().data(14); // Z  row 2, col 2

	report.orientation_variance[0] = msg.pose_with_covariance().covariance().data(21); // R  row 3, col 3
	report.orientation_variance[1] = msg.pose_with_covariance().covariance().data(28); // P  row 4, col 4
	report.orientation_variance[2] = msg.pose_with_covariance().covariance().data(35); // Y  row 5, col 5

	report.velocity_variance[0] = msg.twist_with_covariance().covariance().data(7);  // Y  row 1, col 1
	report.velocity_variance[1] = msg.twist_with_covariance().covariance().data(0);  // X  row 0, col 0
	report.velocity_variance[2] = msg.twist_with_covariance().covariance().data(14); // Z  row 2, col 2

	// report.reset_counter = vpe.reset_counter;
	_visual_odometry_pub.publish(report);
}

static float generate_wgn()
{
	// generate white Gaussian noise sample with std=1

	// algorithm 1:
	// float temp=((float)(rand()+1))/(((float)RAND_MAX+1.0f));
	// return sqrtf(-2.0f*logf(temp))*cosf(2.0f*M_PI_F*rand()/RAND_MAX);
	// algorithm 2: from BlockRandGauss.hpp
	static float V1, V2, S;
	static bool phase = true;
	float X;

	if (phase) {
		do {
			float U1 = (float)rand() / (float)RAND_MAX;
			float U2 = (float)rand() / (float)RAND_MAX;
			V1 = 2.0f * U1 - 1.0f;
			V2 = 2.0f * U2 - 1.0f;
			S = V1 * V1 + V2 * V2;
		} while (S >= 1.0f || fabsf(S) < 1e-8f);

		X = V1 * float(sqrtf(-2.0f * float(logf(S)) / S));

	} else {
		X = V2 * float(sqrtf(-2.0f * float(logf(S)) / S));
	}

	phase = !phase;
	return X;
}

void GZBridge::addGpsNoise(double &latitude, double &longitude, double &altitude,
			   float &vel_north, float &vel_east, float &vel_down)
{
	_gps_pos_noise_n = _pos_markov_time * _gps_pos_noise_n +
			   _pos_random_walk * generate_wgn() * _pos_noise_amplitude -
			   0.02f * _gps_pos_noise_n;

	_gps_pos_noise_e = _pos_markov_time * _gps_pos_noise_e +
			   _pos_random_walk * generate_wgn() * _pos_noise_amplitude -
			   0.02f * _gps_pos_noise_e;

	_gps_pos_noise_d = _pos_markov_time * _gps_pos_noise_d +
			   _pos_random_walk * generate_wgn() * _pos_noise_amplitude * 1.5f -
			   0.02f * _gps_pos_noise_d;

	latitude += math::degrees((double)_gps_pos_noise_n / CONSTANTS_RADIUS_OF_EARTH);
	longitude += math::degrees((double)_gps_pos_noise_e / CONSTANTS_RADIUS_OF_EARTH);
	altitude += (double)_gps_pos_noise_d;

	_gps_vel_noise_n = _vel_markov_time * _gps_vel_noise_n +
			   _vel_noise_density * generate_wgn() * _vel_noise_amplitude;

	_gps_vel_noise_e = _vel_markov_time * _gps_vel_noise_e +
			   _vel_noise_density * generate_wgn() * _vel_noise_amplitude;

	_gps_vel_noise_d = _vel_markov_time * _gps_vel_noise_d +
			   _vel_noise_density * generate_wgn() * _vel_noise_amplitude * 1.2f;

	vel_north += _gps_vel_noise_n;
	vel_east += _gps_vel_noise_e;
	vel_down += _gps_vel_noise_d;
}

void GZBridge::navSatCallback(const gz::msgs::NavSat &msg)
{
	const uint64_t timestamp = hrt_absolute_time();

	// initialize gps position
	if (!_pos_ref.isInitialized()) {
		_pos_ref.initReference(msg.latitude_deg(), msg.longitude_deg(), timestamp);
		_alt_ref = msg.altitude();
		return;
	}

	double latitude = msg.latitude_deg();
	double longitude = msg.longitude_deg();
	double altitude = msg.altitude();
	float vel_north = msg.velocity_north();
	float vel_east = msg.velocity_east();
	float vel_down = -msg.velocity_up();

	vehicle_global_position_s gps_truth{};

	// Publish GPS groundtruth
	gps_truth.timestamp = timestamp;
	gps_truth.timestamp_sample = timestamp;
	gps_truth.lat = latitude;
	gps_truth.lon = longitude;
	gps_truth.alt = altitude;
	_gpos_ground_truth_pub.publish(gps_truth);

	// Apply noise model (based on ublox F9P)
	addGpsNoise(latitude, longitude, altitude, vel_north, vel_east, vel_down);

	// Device ID
	device::Device::DeviceId id{};
	id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SIMULATION;
	id.devid_s.devtype = DRV_GPS_DEVTYPE_SIM;
	id.devid_s.bus = 1;
	id.devid_s.address = 1;

	sensor_gps_s sensor_gps{};

	if (_sim_gps_used.get() >= 4) {
		// fix
		sensor_gps.fix_type = 3; // 3D fix
		sensor_gps.s_variance_m_s = 0.4f;
		sensor_gps.c_variance_rad = 0.1f;
		sensor_gps.eph = 0.9f;
		sensor_gps.epv = 1.78f;
		sensor_gps.hdop = 0.7f;
		sensor_gps.vdop = 1.1f;

	} else {
		// no fix
		sensor_gps.fix_type = 0; // No fix
		sensor_gps.s_variance_m_s = 100.f;
		sensor_gps.c_variance_rad = 100.f;
		sensor_gps.eph = 100.f;
		sensor_gps.epv = 100.f;
		sensor_gps.hdop = 100.f;
		sensor_gps.vdop = 100.f;
	}

	sensor_gps.timestamp = timestamp;
	sensor_gps.timestamp_sample = timestamp;
	sensor_gps.time_utc_usec = 0;
	sensor_gps.device_id = id.devid;
	sensor_gps.latitude_deg = latitude;
	sensor_gps.longitude_deg = longitude;
	sensor_gps.altitude_msl_m = altitude;
	sensor_gps.altitude_ellipsoid_m = altitude;
	sensor_gps.noise_per_ms = 0;
	sensor_gps.jamming_indicator = 0;
	sensor_gps.vel_m_s = sqrtf(vel_north * vel_north + vel_east * vel_east);
	sensor_gps.vel_n_m_s = vel_north;
	sensor_gps.vel_e_m_s = vel_east;
	sensor_gps.vel_d_m_s = vel_down;
	sensor_gps.cog_rad = atan2(vel_east, vel_north);
	sensor_gps.timestamp_time_relative = 0;
	sensor_gps.heading = NAN;
	sensor_gps.heading_offset = NAN;
	sensor_gps.heading_accuracy = 0;
	sensor_gps.automatic_gain_control = 0;
	sensor_gps.jamming_state = 0;
	sensor_gps.spoofing_state = 0;
	sensor_gps.vel_ned_valid = true;
	sensor_gps.satellites_used = _sim_gps_used.get();

	_sensor_gps_pub.publish(sensor_gps);
}

void GZBridge::laserScantoLidarSensorCallback(const gz::msgs::LaserScan &msg)
{
	device::Device::DeviceId id{};
	id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SIMULATION;
	id.devid_s.devtype = DRV_DIST_DEVTYPE_SIM;
	id.devid_s.bus = 1;
	id.devid_s.address = 1;

	distance_sensor_s report{};
	report.timestamp = hrt_absolute_time();
	report.device_id = id.devid;
	report.min_distance = static_cast<float>(msg.range_min());
	report.max_distance = static_cast<float>(msg.range_max());
	report.current_distance = static_cast<float>(msg.ranges()[0]);
	report.variance = 0.0f;
	report.signal_quality = -1;
	report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;

	gz::msgs::Quaternion pose_orientation = msg.world_pose().orientation();
	gz::math::Quaterniond q_sensor = gz::math::Quaterniond(
			pose_orientation.w(),
			pose_orientation.x(),
			pose_orientation.y(),
			pose_orientation.z());

	const gz::math::Quaterniond q_left(0.7071068, 0, 0, -0.7071068);

	const gz::math::Quaterniond q_front(0.7071068, 0.7071068, 0, 0);

	const gz::math::Quaterniond q_down(0, 1, 0, 0);

	if (q_sensor.Equal(q_front, 0.03)) {
		report.orientation = distance_sensor_s::ROTATION_FORWARD_FACING;

	} else if (q_sensor.Equal(q_down, 0.03)) {
		report.orientation = distance_sensor_s::ROTATION_DOWNWARD_FACING;

	} else if (q_sensor.Equal(q_left, 0.03)) {
		report.orientation = distance_sensor_s::ROTATION_LEFT_FACING;

	} else {
		report.orientation = distance_sensor_s::ROTATION_CUSTOM;
		report.q[0] = q_sensor.W();
		report.q[1] = q_sensor.X();
		report.q[2] = q_sensor.Y();
		report.q[3] = q_sensor.Z();
	}

	_distance_sensor_pub.publish(report);
}

void GZBridge::laserScanCallback(const gz::msgs::LaserScan &msg)
{
	static constexpr int SECTOR_SIZE_DEG = 5; // PX4 Collision Prevention uses 5 degree sectors

	double angle_min_deg = msg.angle_min() * 180 / M_PI;
	double angle_step_deg = msg.angle_step() * 180 / M_PI;

	int samples_per_sector = std::round(SECTOR_SIZE_DEG / angle_step_deg);
	int number_of_sectors = msg.ranges_size() / samples_per_sector;

	std::vector<double> ds_array(number_of_sectors, UINT16_MAX);

	// Downsample -- take average of samples per sector
	for (int i = 0; i < number_of_sectors; i++) {

		double sum = 0;

		int samples_used_in_sector = 0;

		for (int j = 0; j < samples_per_sector; j++) {

			double distance = msg.ranges()[i * samples_per_sector + j];

			// inf values mean no object
			if (isinf(distance)) {
				continue;
			}

			sum += distance;
			samples_used_in_sector++;
		}

		// If all samples in a sector are inf then it means the sector is clear
		if (samples_used_in_sector == 0) {
			ds_array[i] = msg.range_max();

		} else {
			ds_array[i] = sum / samples_used_in_sector;
		}
	}

	// Publish to uORB
	obstacle_distance_s report {};

	// Initialize unknown
	for (auto &i : report.distances) {
		i = UINT16_MAX;
	}

	report.timestamp = hrt_absolute_time();
	report.frame = obstacle_distance_s::MAV_FRAME_BODY_FRD;
	report.sensor_type = obstacle_distance_s::MAV_DISTANCE_SENSOR_LASER;
	report.min_distance = static_cast<uint16_t>(msg.range_min() * 100.);
	report.max_distance = static_cast<uint16_t>(msg.range_max() * 100.);
	report.angle_offset = static_cast<float>(angle_min_deg);
	report.increment = static_cast<float>(SECTOR_SIZE_DEG);

	// Map samples in FOV into sectors in ObstacleDistance
	int index = 0;

	// Iterate in reverse because array is FLU and we need FRD
	for (std::vector<double>::reverse_iterator i = ds_array.rbegin(); i != ds_array.rend(); ++i) {

		uint16_t distance_cm = (*i) * 100.;

		if (distance_cm >= report.max_distance) {
			report.distances[index] = report.max_distance + 1;

		} else if (distance_cm < report.min_distance) {
			report.distances[index] = 0;

		} else {
			report.distances[index] = distance_cm;
		}

		index++;
	}

	_obstacle_distance_pub.publish(report);
}

void GZBridge::rotateQuaternion(gz::math::Quaterniond &q_FRD_to_NED, const gz::math::Quaterniond q_FLU_to_ENU)
{
	// FLU (ROS) to FRD (PX4) static rotation
	static const auto q_FLU_to_FRD = gz::math::Quaterniond(0, 1, 0, 0);

	/**
	 * @brief Quaternion for rotation between ENU and NED frames
	 *
	 * NED to ENU: +PI/2 rotation about Z (Down) followed by a +PI rotation around X (old North/new East)
	 * ENU to NED: +PI/2 rotation about Z (Up) followed by a +PI rotation about X (old East/new North)
	 * This rotation is symmetric, so q_ENU_to_NED == q_NED_to_ENU.
	 */
	static const auto q_ENU_to_NED = gz::math::Quaterniond(0, 0.70711, 0.70711, 0);

	// final rotation composition
	q_FRD_to_NED = q_ENU_to_NED * q_FLU_to_ENU * q_FLU_to_FRD.Inverse();
}

void GZBridge::Run()
{
	if (should_exit()) {
		ScheduleClear();

		_mixing_interface_esc.stop();
		_mixing_interface_servo.stop();
		_mixing_interface_wheel.stop();
		_gimbal.stop();

		exit_and_cleanup();
		return;
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		updateParams();

		_mixing_interface_esc.updateParams();
		_mixing_interface_servo.updateParams();
		_mixing_interface_wheel.updateParams();
		_gimbal.updateParams();
	}

	ScheduleDelayed(10_ms);
}

int GZBridge::task_spawn(int argc, char *argv[])
{
	std::string world_name;
	std::string model_name;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "w:n:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'w':
			world_name = myoptarg;
			break;

		case 'n':
			model_name = myoptarg;
			break;

		default:
			print_usage();
			return PX4_ERROR;
		}
	}

	PX4_INFO("world: %s, model: %s", world_name.c_str(), model_name.c_str());

	GZBridge *instance = new GZBridge(world_name, model_name);

	if (!instance) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	if (instance->init() != PX4_OK) {
		delete instance;
		_object.store(nullptr);
		_task_id = -1;
		return PX4_ERROR;
	}

	return PX4_OK;
}

int GZBridge::print_status()
{
	PX4_INFO_RAW("ESC outputs:\n");
	_mixing_interface_esc.mixingOutput().printStatus();

	PX4_INFO_RAW("Servo outputs:\n");
	_mixing_interface_servo.mixingOutput().printStatus();

	PX4_INFO_RAW("Wheel outputs:\n");
	_mixing_interface_wheel.mixingOutput().printStatus();

	return 0;
}

int GZBridge::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int GZBridge::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("gz_bridge", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('w', nullptr, nullptr, "World name", true);
	PRINT_MODULE_USAGE_PARAM_STRING('n', nullptr, nullptr, "Model name", false);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int gz_bridge_main(int argc, char *argv[])
{
	return GZBridge::main(argc, argv);
}
