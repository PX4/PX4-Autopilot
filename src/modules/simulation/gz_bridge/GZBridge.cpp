/****************************************************************************
 *
 *   Copyright (c) 2022-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
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

GZBridge::GZBridge(const char *world, const char *name, const char *model,
		   const char *pose_str) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_world_name(world),
	_model_name(name),
	_model_sim(model),
	_model_pose(pose_str)
{
	pthread_mutex_init(&_node_mutex, nullptr);

	updateParams();
}

GZBridge::~GZBridge()
{
	// TODO: unsubscribe

	for (auto &sub_topic : _node.SubscribedTopics()) {
		_node.Unsubscribe(sub_topic);
	}
}

int GZBridge::init()
{
	if (!_model_sim.empty()) {

		// service call to create model
		gz::msgs::EntityFactory req{};
		req.set_sdf_filename(_model_sim + "/model.sdf");

		req.set_name(_model_name); // New name for the entity, overrides the name on the SDF.

		req.set_allow_renaming(false); // allowed to rename the entity in case of overlap with existing entities

		if (!_model_pose.empty()) {
			PX4_INFO("Requested Model Position: %s", _model_pose.c_str());

			std::vector<double> model_pose_v;

			std::stringstream ss(_model_pose);

			while (ss.good()) {
				std::string substr;
				std::getline(ss, substr, ',');
				model_pose_v.push_back(std::stod(substr));
			}

			while (model_pose_v.size() < 6) {
				model_pose_v.push_back(0.0);
			}

			// If model position z is less equal than 0, move above floor to prevent floor glitching
			if (model_pose_v[2] <= 0.0) {
				PX4_INFO("Model position z is less or equal 0.0, moving upwards");
				model_pose_v[2] = 0.5;
			}

			gz::msgs::Pose *p = req.mutable_pose();
			gz::msgs::Vector3d *position = p->mutable_position();
			position->set_x(model_pose_v[0]);
			position->set_y(model_pose_v[1]);
			position->set_z(model_pose_v[2]);

			gz::math::Quaterniond q(model_pose_v[3], model_pose_v[4], model_pose_v[5]);

			q.Normalize();
			gz::msgs::Quaternion *orientation = p->mutable_orientation();
			orientation->set_x(q.X());
			orientation->set_y(q.Y());
			orientation->set_z(q.Z());
			orientation->set_w(q.W());
		}

		//world/$WORLD/create service.
		gz::msgs::Boolean rep;
		bool result;
		std::string create_service = "/world/" + _world_name + "/create";

		bool gz_called = false;
		// Check if PX4_GZ_STANDALONE has been set.
		char *standalone_val = std::getenv("PX4_GZ_STANDALONE");

		if ((standalone_val != nullptr) && (std::strcmp(standalone_val, "1") == 0)) {
			// Check if Gazebo has been called and if not attempt to reconnect.
			while (gz_called == false) {
				if (_node.Request(create_service, req, 1000, rep, result)) {
					if (!rep.data() || !result) {
						PX4_ERR("EntityFactory service call failed");
						return PX4_ERROR;

					} else {
						gz_called = true;
					}
				}

				// If Gazebo has not been called, wait 2 seconds and try again.
				else {
					PX4_WARN("Service call timed out as Gazebo has not been detected.");
					system_usleep(2000000);
				}
			}
		}


		// If PX4_GZ_STANDALONE has been set, you can try to connect but GZ_SIM_RESOURCE_PATH needs to be set correctly to work.
		else {
			if (!callEntityFactoryService(create_service, req)) {
				return PX4_ERROR;
			}

			std::string scene_info_service = "/world/" + _world_name + "/scene/info";
			bool scene_created = false;

			while (scene_created == false) {
				if (!callSceneInfoMsgService(scene_info_service)) {
					PX4_WARN("Service call timed out as Gazebo has not been detected.");
					system_usleep(2000000);

				} else {
					scene_created = true;
				}
			}

			gz::msgs::StringMsg follow_msg{};
			follow_msg.set_data(_model_name);
			callStringMsgService("/gui/follow", follow_msg);
			gz::msgs::Vector3d follow_offset_msg{};
			follow_offset_msg.set_x(-2.0);
			follow_offset_msg.set_y(-2.0);
			follow_offset_msg.set_z(2.0);
			callVector3dService("/gui/follow/offset", follow_offset_msg);
		}
	}

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


	// IMU: /world/$WORLD/model/$MODEL/link/base_link/sensor/imu_sensor/imu
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

int GZBridge::task_spawn(int argc, char *argv[])
{
	const char *world_name = "default";
	const char *model_name = nullptr;
	const char *model_pose = nullptr;
	const char *model_sim = nullptr;
	const char *px4_instance = nullptr;
	std::string model_name_std;


	bool error_flag = false;
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "w:m:p:i:n:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'w':
			// world
			world_name = myoptarg;
			break;

		case 'n':
			// model
			model_name = myoptarg;
			break;

		case 'p':
			// pose
			model_pose = myoptarg;
			break;

		case 'm':
			// pose
			model_sim = myoptarg;
			break;

		case 'i':
			// pose
			px4_instance = myoptarg;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return PX4_ERROR;
	}

	if (!model_pose) {
		model_pose = "";
	}

	if (!model_sim) {
		model_sim = "";
	}

	if (!px4_instance) {
		if (!model_name) {
			model_name = model_sim;
		}

	} else if (!model_name) {
		model_name_std = std::string(model_sim) + "_" + std::string(px4_instance);
		model_name = model_name_std.c_str();
	}

	PX4_INFO("world: %s, model name: %s, simulation model: %s", world_name, model_name, model_sim);

	GZBridge *instance = new GZBridge(world_name, model_name, model_sim, model_pose);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init() == PX4_OK) {

#if defined(ENABLE_LOCKSTEP_SCHEDULER)
			// lockstep scheduler wait for initial clock set before returning
			int sleep_count_limit = 10000;

			while ((instance->world_time_us() == 0) && sleep_count_limit > 0) {
				// wait for first clock message
				system_usleep(1000);
				sleep_count_limit--;
			}

			if (instance->world_time_us() == 0) {
				PX4_ERR("timed out waiting for clock message");
				instance->request_stop();
				instance->ScheduleNow();

			} else {
				return PX4_OK;
			}

#else
			return PX4_OK;
#endif // ENABLE_LOCKSTEP_SCHEDULER

			//return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

bool GZBridge::updateClock(const uint64_t tv_sec, const uint64_t tv_nsec)
{
#if defined(ENABLE_LOCKSTEP_SCHEDULER)
	struct timespec ts;
	ts.tv_sec = tv_sec;
	ts.tv_nsec = tv_nsec;

	if (px4_clock_settime(CLOCK_MONOTONIC, &ts) == 0) {
		_world_time_us.store(ts_to_abstime(&ts));
		return true;
	}

#endif // ENABLE_LOCKSTEP_SCHEDULER

	return false;
}

void GZBridge::clockCallback(const gz::msgs::Clock &clock)
{
	pthread_mutex_lock(&_node_mutex);

	const uint64_t time_us = (clock.sim().sec() * 1000000) + (clock.sim().nsec() / 1000);

	if (time_us > _world_time_us.load()) {
		updateClock(clock.sim().sec(), clock.sim().nsec());
	}

	pthread_mutex_unlock(&_node_mutex);
}

void GZBridge::barometerCallback(const gz::msgs::FluidPressure &air_pressure)
{
	if (hrt_absolute_time() == 0) {
		return;
	}

	pthread_mutex_lock(&_node_mutex);

	const uint64_t time_us = (air_pressure.header().stamp().sec() * 1000000)
				 + (air_pressure.header().stamp().nsec() / 1000);

	// publish
	sensor_baro_s sensor_baro{};
	sensor_baro.timestamp_sample = time_us;
	sensor_baro.device_id = 6620172; // 6620172: DRV_BARO_DEVTYPE_BAROSIM, BUS: 1, ADDR: 4, TYPE: SIMULATION
	sensor_baro.pressure = air_pressure.pressure();
	sensor_baro.temperature = this->_temperature;
	sensor_baro.timestamp = hrt_absolute_time();
	_sensor_baro_pub.publish(sensor_baro);

	pthread_mutex_unlock(&_node_mutex);
}


void GZBridge::airspeedCallback(const gz::msgs::AirSpeed &air_speed)
{
	if (hrt_absolute_time() == 0) {
		return;
	}

	pthread_mutex_lock(&_node_mutex);

	const uint64_t time_us = (air_speed.header().stamp().sec() * 1000000)
				 + (air_speed.header().stamp().nsec() / 1000);

	double air_speed_value = air_speed.diff_pressure();

	differential_pressure_s report{};
	report.timestamp_sample = time_us;
	report.device_id = 1377548; // 1377548: DRV_DIFF_PRESS_DEVTYPE_SIM, BUS: 1, ADDR: 5, TYPE: SIMULATION
	report.differential_pressure_pa = static_cast<float>(air_speed_value); // hPa to Pa;
	report.temperature = static_cast<float>(air_speed.temperature()) + atmosphere::kAbsoluteNullCelsius; // K to C
	report.timestamp = hrt_absolute_time();;
	_differential_pressure_pub.publish(report);

	this->_temperature = report.temperature;

	pthread_mutex_unlock(&_node_mutex);
}

void GZBridge::imuCallback(const gz::msgs::IMU &imu)
{
	if (hrt_absolute_time() == 0) {
		return;
	}

	pthread_mutex_lock(&_node_mutex);

	const uint64_t time_us = (imu.header().stamp().sec() * 1000000) + (imu.header().stamp().nsec() / 1000);

	if (time_us > _world_time_us.load()) {
		updateClock(imu.header().stamp().sec(), imu.header().stamp().nsec());
	}

	// FLU -> FRD
	static const auto q_FLU_to_FRD = gz::math::Quaterniond(0, 1, 0, 0);

	gz::math::Vector3d accel_b = q_FLU_to_FRD.RotateVector(gz::math::Vector3d(
					     imu.linear_acceleration().x(),
					     imu.linear_acceleration().y(),
					     imu.linear_acceleration().z()));

	// publish accel
	sensor_accel_s sensor_accel{};
#if defined(ENABLE_LOCKSTEP_SCHEDULER)
	sensor_accel.timestamp_sample = time_us;
	sensor_accel.timestamp = time_us;
#else
	sensor_accel.timestamp_sample = hrt_absolute_time();
	sensor_accel.timestamp = hrt_absolute_time();
#endif
	sensor_accel.device_id = 1310988; // 1310988: DRV_IMU_DEVTYPE_SIM, BUS: 1, ADDR: 1, TYPE: SIMULATION
	sensor_accel.x = accel_b.X();
	sensor_accel.y = accel_b.Y();
	sensor_accel.z = accel_b.Z();
	sensor_accel.temperature = NAN;
	sensor_accel.samples = 1;
	_sensor_accel_pub.publish(sensor_accel);


	gz::math::Vector3d gyro_b = q_FLU_to_FRD.RotateVector(gz::math::Vector3d(
					    imu.angular_velocity().x(),
					    imu.angular_velocity().y(),
					    imu.angular_velocity().z()));

	// publish gyro
	sensor_gyro_s sensor_gyro{};
#if defined(ENABLE_LOCKSTEP_SCHEDULER)
	sensor_gyro.timestamp_sample = time_us;
	sensor_gyro.timestamp = time_us;
#else
	sensor_gyro.timestamp_sample = hrt_absolute_time();
	sensor_gyro.timestamp = hrt_absolute_time();
#endif
	sensor_gyro.device_id = 1310988; // 1310988: DRV_IMU_DEVTYPE_SIM, BUS: 1, ADDR: 1, TYPE: SIMULATION
	sensor_gyro.x = gyro_b.X();
	sensor_gyro.y = gyro_b.Y();
	sensor_gyro.z = gyro_b.Z();
	sensor_gyro.temperature = NAN;
	sensor_gyro.samples = 1;
	_sensor_gyro_pub.publish(sensor_gyro);

	pthread_mutex_unlock(&_node_mutex);
}

void GZBridge::poseInfoCallback(const gz::msgs::Pose_V &pose)
{
	if (hrt_absolute_time() == 0) {
		return;
	}

	pthread_mutex_lock(&_node_mutex);

	for (int p = 0; p < pose.pose_size(); p++) {
		if (pose.pose(p).name() == _model_name) {

			const uint64_t time_us = (pose.header().stamp().sec() * 1000000) + (pose.header().stamp().nsec() / 1000);

			if (time_us > _world_time_us.load()) {
				updateClock(pose.header().stamp().sec(), pose.header().stamp().nsec());
			}

			const double dt = math::constrain((time_us - _timestamp_prev) * 1e-6, 0.001, 0.1);
			_timestamp_prev = time_us;

			gz::msgs::Vector3d pose_position = pose.pose(p).position();
			gz::msgs::Quaternion pose_orientation = pose.pose(p).orientation();

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
#if defined(ENABLE_LOCKSTEP_SCHEDULER)
			vehicle_attitude_groundtruth.timestamp_sample = time_us;
#else
			vehicle_attitude_groundtruth.timestamp_sample = hrt_absolute_time();
#endif
			vehicle_attitude_groundtruth.q[0] = q_nb.W();
			vehicle_attitude_groundtruth.q[1] = q_nb.X();
			vehicle_attitude_groundtruth.q[2] = q_nb.Y();
			vehicle_attitude_groundtruth.q[3] = q_nb.Z();
			vehicle_attitude_groundtruth.timestamp = hrt_absolute_time();
			_attitude_ground_truth_pub.publish(vehicle_attitude_groundtruth);

			// publish angular velocity groundtruth
			const matrix::Eulerf euler{matrix::Quatf(vehicle_attitude_groundtruth.q)};
			vehicle_angular_velocity_s vehicle_angular_velocity_groundtruth{};
#if defined(ENABLE_LOCKSTEP_SCHEDULER)
			vehicle_angular_velocity_groundtruth.timestamp_sample = time_us;
#else
			vehicle_angular_velocity_groundtruth.timestamp_sample = hrt_absolute_time();
#endif
			const matrix::Vector3f angular_velocity = (euler - _euler_prev) / dt;
			_euler_prev = euler;
			angular_velocity.copyTo(vehicle_angular_velocity_groundtruth.xyz);

			vehicle_angular_velocity_groundtruth.timestamp = hrt_absolute_time();
			_angular_velocity_ground_truth_pub.publish(vehicle_angular_velocity_groundtruth);

			vehicle_local_position_s local_position_groundtruth{};
#if defined(ENABLE_LOCKSTEP_SCHEDULER)
			local_position_groundtruth.timestamp_sample = time_us;
#else
			local_position_groundtruth.timestamp_sample = hrt_absolute_time();
#endif
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

			local_position_groundtruth.timestamp = hrt_absolute_time();
			_lpos_ground_truth_pub.publish(local_position_groundtruth);

			pthread_mutex_unlock(&_node_mutex);
			return;
		}
	}

	pthread_mutex_unlock(&_node_mutex);
}

void GZBridge::odometryCallback(const gz::msgs::OdometryWithCovariance &odometry)
{
	if (hrt_absolute_time() == 0) {
		return;
	}

	pthread_mutex_lock(&_node_mutex);

	const uint64_t time_us = (odometry.header().stamp().sec() * 1000000) + (odometry.header().stamp().nsec() / 1000);

	if (time_us > _world_time_us.load()) {
		updateClock(odometry.header().stamp().sec(), odometry.header().stamp().nsec());
	}

	vehicle_odometry_s odom{};
#if defined(ENABLE_LOCKSTEP_SCHEDULER)
	odom.timestamp_sample = time_us;
	odom.timestamp = time_us;
#else
	odom.timestamp_sample = hrt_absolute_time();
	odom.timestamp = hrt_absolute_time();
#endif

	// gz odometry position is in ENU frame and needs to be converted to NED
	odom.pose_frame = vehicle_odometry_s::POSE_FRAME_NED;
	odom.position[0] = odometry.pose_with_covariance().pose().position().y();
	odom.position[1] = odometry.pose_with_covariance().pose().position().x();
	odom.position[2] = -odometry.pose_with_covariance().pose().position().z();

	// gz odometry orientation is "body FLU->ENU" and needs to be converted in "body FRD->NED"
	gz::msgs::Quaternion pose_orientation = odometry.pose_with_covariance().pose().orientation();
	gz::math::Quaterniond q_gr = gz::math::Quaterniond(
					     pose_orientation.w(),
					     pose_orientation.x(),
					     pose_orientation.y(),
					     pose_orientation.z());
	gz::math::Quaterniond q_nb;
	GZBridge::rotateQuaternion(q_nb, q_gr);
	odom.q[0] = q_nb.W();
	odom.q[1] = q_nb.X();
	odom.q[2] = q_nb.Y();
	odom.q[3] = q_nb.Z();

	// gz odometry linear velocity is in body FLU and needs to be converted in body FRD
	odom.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_BODY_FRD;
	odom.velocity[0] = odometry.twist_with_covariance().twist().linear().x();
	odom.velocity[1] = -odometry.twist_with_covariance().twist().linear().y();
	odom.velocity[2] = -odometry.twist_with_covariance().twist().linear().z();

	// gz odometry angular velocity is in body FLU and need to be converted in body FRD
	odom.angular_velocity[0] = odometry.twist_with_covariance().twist().angular().x();
	odom.angular_velocity[1] = -odometry.twist_with_covariance().twist().angular().y();
	odom.angular_velocity[2] = -odometry.twist_with_covariance().twist().angular().z();

	// VISION_POSITION_ESTIMATE covariance
	//  pose 6x6 cross-covariance matrix
	//  (states: x, y, z, roll, pitch, yaw).
	//  If unknown, assign NaN value to first element in the array.
	odom.position_variance[0] = odometry.pose_with_covariance().covariance().data(7);  // Y  row 1, col 1
	odom.position_variance[1] = odometry.pose_with_covariance().covariance().data(0);  // X  row 0, col 0
	odom.position_variance[2] = odometry.pose_with_covariance().covariance().data(14); // Z  row 2, col 2

	odom.orientation_variance[0] = odometry.pose_with_covariance().covariance().data(21); // R  row 3, col 3
	odom.orientation_variance[1] = odometry.pose_with_covariance().covariance().data(28); // P  row 4, col 4
	odom.orientation_variance[2] = odometry.pose_with_covariance().covariance().data(35); // Y  row 5, col 5

	odom.velocity_variance[0] = odometry.twist_with_covariance().covariance().data(7);  // Y  row 1, col 1
	odom.velocity_variance[1] = odometry.twist_with_covariance().covariance().data(0);  // X  row 0, col 0
	odom.velocity_variance[2] = odometry.twist_with_covariance().covariance().data(14); // Z  row 2, col 2

	// odom.reset_counter = vpe.reset_counter;
	_visual_odometry_pub.publish(odom);

	pthread_mutex_unlock(&_node_mutex);
}

void GZBridge::navSatCallback(const gz::msgs::NavSat &nav_sat)
{
	if (hrt_absolute_time() == 0) {
		return;
	}

	pthread_mutex_lock(&_node_mutex);

	const uint64_t time_us = (nav_sat.header().stamp().sec() * 1000000) + (nav_sat.header().stamp().nsec() / 1000);

	if (time_us > _world_time_us.load()) {
		updateClock(nav_sat.header().stamp().sec(), nav_sat.header().stamp().nsec());
	}

	// initialize gps position
	if (!_pos_ref.isInitialized()) {
		_pos_ref.initReference(nav_sat.latitude_deg(), nav_sat.longitude_deg(), hrt_absolute_time());
		_alt_ref = nav_sat.altitude();

	} else {
		// publish GPS groundtruth
		vehicle_global_position_s global_position_groundtruth{};
#if defined(ENABLE_LOCKSTEP_SCHEDULER)
		global_position_groundtruth.timestamp_sample = time_us;
#else
		global_position_groundtruth.timestamp_sample = hrt_absolute_time();
#endif
		global_position_groundtruth.lat = nav_sat.latitude_deg();
		global_position_groundtruth.lon = nav_sat.longitude_deg();
		global_position_groundtruth.alt = nav_sat.altitude();
		_gpos_ground_truth_pub.publish(global_position_groundtruth);
	}

	pthread_mutex_unlock(&_node_mutex);
}
void GZBridge::laserScantoLidarSensorCallback(const gz::msgs::LaserScan &scan)
{
	if (hrt_absolute_time() == 0) {
		return;
	}

	distance_sensor_s distance_sensor{};
	distance_sensor.timestamp = hrt_absolute_time();
	device::Device::DeviceId id;
	id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SIMULATION;
	id.devid_s.bus = 0;
	id.devid_s.address = 0;
	id.devid_s.devtype = DRV_DIST_DEVTYPE_SIM;
	distance_sensor.device_id = id.devid;
	distance_sensor.min_distance = static_cast<float>(scan.range_min());
	distance_sensor.max_distance = static_cast<float>(scan.range_max());
	distance_sensor.current_distance = static_cast<float>(scan.ranges()[0]);
	distance_sensor.variance = 0.0f;
	distance_sensor.signal_quality = -1;
	distance_sensor.type = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;

	gz::msgs::Quaternion pose_orientation = scan.world_pose().orientation();
	gz::math::Quaterniond q_sensor = gz::math::Quaterniond(
			pose_orientation.w(),
			pose_orientation.x(),
			pose_orientation.y(),
			pose_orientation.z());

	const gz::math::Quaterniond q_front(0.7071068, 0.7071068, 0, 0);
	const gz::math::Quaterniond q_down(0, 1, 0, 0);

	if (q_sensor.Equal(q_front, 0.03)) {
		distance_sensor.orientation = distance_sensor_s::ROTATION_FORWARD_FACING;

	} else if (q_sensor.Equal(q_down, 0.03)) {
		distance_sensor.orientation = distance_sensor_s::ROTATION_DOWNWARD_FACING;

	} else {
		distance_sensor.orientation = distance_sensor_s::ROTATION_CUSTOM;
	}

	_distance_sensor_pub.publish(distance_sensor);
}

void GZBridge::laserScanCallback(const gz::msgs::LaserScan &scan)
{
	static constexpr int SECTOR_SIZE_DEG = 5; // PX4 Collision Prevention uses 5 degree sectors

	double angle_min_deg = scan.angle_min() * 180 / M_PI;
	double angle_step_deg = scan.angle_step() * 180 / M_PI;

	int samples_per_sector = std::round(SECTOR_SIZE_DEG / angle_step_deg);
	int number_of_sectors = scan.ranges_size() / samples_per_sector;

	std::vector<double> ds_array(number_of_sectors, UINT16_MAX);

	// Downsample -- take average of samples per sector
	for (int i = 0; i < number_of_sectors; i++) {

		double sum = 0;

		int samples_used_in_sector = 0;

		for (int j = 0; j < samples_per_sector; j++) {

			double distance = scan.ranges()[i * samples_per_sector + j];

			// inf values mean no object
			if (isinf(distance)) {
				continue;
			}

			sum += distance;
			samples_used_in_sector++;
		}

		// If all samples in a sector are inf then it means the sector is clear
		if (samples_used_in_sector == 0) {
			ds_array[i] = scan.range_max();

		} else {
			ds_array[i] = sum / samples_used_in_sector;
		}
	}

	// Publish to uORB
	obstacle_distance_s obs {};

	// Initialize unknown
	for (auto &i : obs.distances) {
		i = UINT16_MAX;
	}

	obs.timestamp = hrt_absolute_time();
	obs.frame = obstacle_distance_s::MAV_FRAME_BODY_FRD;
	obs.sensor_type = obstacle_distance_s::MAV_DISTANCE_SENSOR_LASER;
	obs.min_distance = static_cast<uint16_t>(scan.range_min() * 100.);
	obs.max_distance = static_cast<uint16_t>(scan.range_max() * 100.);
	obs.angle_offset = static_cast<float>(angle_min_deg);
	obs.increment = static_cast<float>(SECTOR_SIZE_DEG);

	// Map samples in FOV into sectors in ObstacleDistance
	int index = 0;

	// Iterate in reverse because array is FLU and we need FRD
	for (std::vector<double>::reverse_iterator i = ds_array.rbegin(); i != ds_array.rend(); ++i) {

		uint16_t distance_cm = (*i) * 100.;

		if (distance_cm >= obs.max_distance) {
			obs.distances[index] = obs.max_distance + 1;

		} else if (distance_cm < obs.min_distance) {
			obs.distances[index] = 0;

		} else {
			obs.distances[index] = distance_cm;
		}

		index++;
	}

	_obstacle_distance_pub.publish(obs);
}

bool GZBridge::callEntityFactoryService(const std::string &service, const gz::msgs::EntityFactory &req)
{
	bool result;
	gz::msgs::Boolean rep;

	if (_node.Request(service, req, 1000, rep, result)) {
		if (!rep.data() || !result) {
			PX4_ERR("EntityFactory service call failed.");
			return false;
		}

	} else {
		PX4_ERR("Service call timed out. Check GZ_SIM_RESOURCE_PATH is set correctly.");
		return false;
	}

	return true;
}

bool GZBridge::callSceneInfoMsgService(const std::string &service)
{
	bool result;
	gz::msgs::Empty req;
	gz::msgs::Scene rep;

	if (_node.Request(service, req, 1000, rep, result)) {
		if (!result) {
			PX4_ERR("Scene Info service call failed.");
			return false;

		} else {
			return true;
		}

	} else {
		PX4_ERR("Service call timed out. Check GZ_SIM_RESOURCE_PATH is set correctly.");
		return false;
	}

	return true;
}

bool GZBridge::callStringMsgService(const std::string &service, const gz::msgs::StringMsg &req)
{
	bool result;

	gz::msgs::Boolean rep;

	if (_node.Request(service, req, 1000, rep, result)) {
		if (!rep.data() || !result) {
			PX4_ERR("String service call failed");
			return false;

		}
	}

	else {
		PX4_ERR("Service call timed out: %s", service.c_str());
		return false;
	}

	return true;
}

bool GZBridge::callVector3dService(const std::string &service, const gz::msgs::Vector3d &req)
{
	bool result;

	gz::msgs::Boolean rep;

	if (_node.Request(service, req, 1000, rep, result)) {
		if (!rep.data() || !result) {
			PX4_ERR("String service call failed");
			return false;

		}
	}

	else {
		PX4_ERR("Service call timed out: %s", service.c_str());
		return false;
	}

	return true;
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

	pthread_mutex_lock(&_node_mutex);

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

	pthread_mutex_unlock(&_node_mutex);
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
	PRINT_MODULE_USAGE_PARAM_STRING('m', nullptr, nullptr, "Fuel model name", false);
	PRINT_MODULE_USAGE_PARAM_STRING('p', nullptr, nullptr, "Model Pose", false);
	PRINT_MODULE_USAGE_PARAM_STRING('n', nullptr, nullptr, "Model name", false);
	PRINT_MODULE_USAGE_PARAM_STRING('i', nullptr, nullptr, "PX4 instance", false);
	PRINT_MODULE_USAGE_PARAM_STRING('w', nullptr, nullptr, "World name", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int gz_bridge_main(int argc, char *argv[])
{
	return GZBridge::main(argc, argv);
}
