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

#include "SimulatorIgnitionBridge.hpp"

#include <uORB/Subscription.hpp>

#include <lib/mathlib/mathlib.h>

#include <px4_platform_common/getopt.h>

#include <iostream>
#include <string>

SimulatorIgnitionBridge::SimulatorIgnitionBridge(const char *world, const char *model) :
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default),
	_px4_accel(1310988),  // 1310988: DRV_IMU_DEVTYPE_SIM, BUS: 1, ADDR: 1, TYPE: SIMULATION
	_px4_gyro(1310988),   // 1310988: DRV_IMU_DEVTYPE_SIM, BUS: 1, ADDR: 1, TYPE: SIMULATION
	_world_name(world),
	_model_name(model)
{
	_px4_accel.set_range(2000.f); // don't care

	updateParams();
}

SimulatorIgnitionBridge::~SimulatorIgnitionBridge()
{
	// TODO: unsubscribe

	for (auto &sub_topic : _node.SubscribedTopics()) {
		_node.Unsubscribe(sub_topic);
	}
}

int SimulatorIgnitionBridge::init()
{
	// service call to create model
	// ign service -s /world/${PX4_SIM_WORLD}/create --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 1000 --req "sdf_filename: \"${PX4_SIM_MODEL}/model.sdf\""
	ignition::msgs::EntityFactory req{};
	req.set_sdf_filename(_model_name + "/model.sdf");

	// TODO: support model instances?
	// req.set_name("model_instance_name"); // New name for the entity, overrides the name on the SDF.
	req.set_allow_renaming(false); // allowed to rename the entity in case of overlap with existing entities

	// /world/$WORLD/create service.
	ignition::msgs::Boolean rep;
	bool result;
	std::string create_service = "/world/" + _world_name + "/create";

	if (_node.Request(create_service, req, 1000, rep, result)) {
		if (!rep.data() || !result) {
			PX4_ERR("EntityFactory service call failed");
			return PX4_ERROR;
		}

	} else {
		PX4_ERR("Service call timed out");
		return PX4_ERROR;
	}


	// clock
	std::string clock_topic = "/world/" + _world_name + "/clock";

	if (!_node.Subscribe(clock_topic, &SimulatorIgnitionBridge::clockCallback, this)) {
		PX4_ERR("failed to subscribe to %s", clock_topic.c_str());
		return PX4_ERROR;
	}

	// pose: /world/$WORLD/pose/info
	std::string world_pose_topic = "/world/" + _world_name + "/pose/info";

	if (!_node.Subscribe(world_pose_topic, &SimulatorIgnitionBridge::poseInfoCallback, this)) {
		PX4_ERR("failed to subscribe to %s", world_pose_topic.c_str());
		return PX4_ERROR;
	}

	// IMU: /world/$WORLD/model/$MODEL//link/base_link/sensor/imu_sensor/imu
	std::string imu_topic = "/world/" + _world_name + "/model/" + _model_name + "/link/base_link/sensor/imu_sensor/imu";

	if (!_node.Subscribe(imu_topic, &SimulatorIgnitionBridge::imuCallback, this)) {
		PX4_ERR("failed to subscribe to %s", imu_topic.c_str());
		return PX4_ERROR;
	}

	for (auto &sub_topic : _node.SubscribedTopics()) {
		PX4_INFO("subscribed: %s", sub_topic.c_str());
	}

	// output eg /X3/command/motor_speed
	std::string actuator_topic = _model_name + "/command/motor_speed";
	_actuators_pub = _node.Advertise<ignition::msgs::Actuators>(actuator_topic);

	if (!_actuators_pub.Valid()) {
		PX4_ERR("failed to advertise %s", actuator_topic.c_str());
		return PX4_ERROR;
	}

	ScheduleNow();
	return OK;
}

int SimulatorIgnitionBridge::task_spawn(int argc, char *argv[])
{
	const char *world_name = "default";
	const char *model_name = nullptr;

	bool error_flag = false;
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "w:m:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'w':
			// world
			world_name = myoptarg;
			break;

		case 'm':
			// model
			model_name = myoptarg;
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

	PX4_INFO("world: %s, model: %s", world_name, model_name);

	SimulatorIgnitionBridge *instance = new SimulatorIgnitionBridge(world_name, model_name);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init() == PX4_OK) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

void SimulatorIgnitionBridge::clockCallback(const ignition::msgs::Clock &clock)
{
	// uint64_t time_us = clock.sim().sec() * 1e6 + clock.sim().nsec() / 1000;
	// PX4_INFO("clock %lu ", time_us);

	struct timespec ts;
	ts.tv_sec = clock.sim().sec();
	ts.tv_nsec = clock.sim().nsec();
	px4_clock_settime(CLOCK_MONOTONIC, &ts);
}

void SimulatorIgnitionBridge::imuCallback(const ignition::msgs::IMU &imu)
{
	// FLU -> FRD
	static const auto q_FLU_to_FRD = ignition::math::Quaterniond(0, 1, 0, 0);

	ignition::math::Vector3d accel_b = q_FLU_to_FRD.RotateVector(ignition::math::Vector3d(
			imu.linear_acceleration().x(),
			imu.linear_acceleration().y(),
			imu.linear_acceleration().z()));

	ignition::math::Vector3d gyro_b = q_FLU_to_FRD.RotateVector(ignition::math::Vector3d(
			imu.angular_velocity().x(),
			imu.angular_velocity().y(),
			imu.angular_velocity().z()));

	//const uint64_t time_us = imu.header().stamp().sec() * 1e6 + imu.header().stamp().nsec() / 1000;
	const uint64_t time_us = hrt_absolute_time();

	if (time_us != 0) {
		_px4_accel.update(time_us, accel_b.X(), accel_b.Y(), accel_b.Z());
		_px4_gyro.update(time_us, gyro_b.X(), gyro_b.Y(), gyro_b.Z());
	}
}

void SimulatorIgnitionBridge::poseInfoCallback(const ignition::msgs::Pose_V &pose)
{
	//const uint64_t time_us = pose.header().stamp().sec() * 1e6 + pose.header().stamp().nsec() / 1000;
	const uint64_t time_us = hrt_absolute_time();

	if (time_us == 0) {
		return;
	}

	for (int p = 0; p < pose.pose_size(); p++) {
		if (pose.pose(p).name() == _model_name) {

			const double dt = math::constrain((time_us - _timestamp_prev) * 1e-6, 0.001, 0.1);
			_timestamp_prev = time_us;

			ignition::msgs::Vector3d pose_position = pose.pose(p).position();
			ignition::msgs::Quaternion pose_orientation = pose.pose(p).orientation();

			static const auto q_FLU_to_FRD = ignition::math::Quaterniond(0, 1, 0, 0);

			/**
			 * @brief Quaternion for rotation between ENU and NED frames
			 *
			 * NED to ENU: +PI/2 rotation about Z (Down) followed by a +PI rotation around X (old North/new East)
			 * ENU to NED: +PI/2 rotation about Z (Up) followed by a +PI rotation about X (old East/new North)
			 * This rotation is symmetric, so q_ENU_to_NED == q_NED_to_ENU.
			 */
			static const auto q_ENU_to_NED = ignition::math::Quaterniond(0, 0.70711, 0.70711, 0);

			// ground truth
			ignition::math::Quaterniond q_gr = ignition::math::Quaterniond(
					pose_orientation.w(),
					pose_orientation.x(),
					pose_orientation.y(),
					pose_orientation.z());

			ignition::math::Quaterniond q_gb = q_gr * q_FLU_to_FRD.Inverse();
			ignition::math::Quaterniond q_nb = q_ENU_to_NED * q_gb;

			// publish attitude groundtruth
			vehicle_attitude_s vehicle_attitude_groundtruth{};
			vehicle_attitude_groundtruth.timestamp_sample = time_us;
			vehicle_attitude_groundtruth.q[0] = q_nb.W();
			vehicle_attitude_groundtruth.q[1] = q_nb.X();
			vehicle_attitude_groundtruth.q[2] = q_nb.Y();
			vehicle_attitude_groundtruth.q[3] = q_nb.Z();
			vehicle_attitude_groundtruth.timestamp = hrt_absolute_time();
			_attitude_ground_truth_pub.publish(vehicle_attitude_groundtruth);

			// publish angular velocity groundtruth
			const matrix::Eulerf euler{matrix::Quatf(vehicle_attitude_groundtruth.q)};
			vehicle_angular_velocity_s vehicle_angular_velocity_groundtruth{};
			vehicle_angular_velocity_groundtruth.timestamp_sample = time_us;

			const matrix::Vector3f angular_velocity = (euler - _euler_prev) / dt;
			_euler_prev = euler;
			angular_velocity.copyTo(vehicle_angular_velocity_groundtruth.xyz);

			vehicle_angular_velocity_groundtruth.timestamp = hrt_absolute_time();
			_angular_velocity_ground_truth_pub.publish(vehicle_angular_velocity_groundtruth);

			if (!_pos_ref.isInitialized()) {
				_pos_ref.initReference((double)_param_sim_home_lat.get(), (double)_param_sim_home_lon.get(), hrt_absolute_time());
			}

			vehicle_local_position_s local_position_groundtruth{};
			local_position_groundtruth.timestamp_sample = time_us;

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

			local_position_groundtruth.ref_lat = _pos_ref.getProjectionReferenceLat(); // Reference point latitude in degrees
			local_position_groundtruth.ref_lon = _pos_ref.getProjectionReferenceLon(); // Reference point longitude in degrees
			local_position_groundtruth.ref_alt = _param_sim_home_alt.get();
			local_position_groundtruth.ref_timestamp = _pos_ref.getProjectionReferenceTimestamp();

			local_position_groundtruth.timestamp = hrt_absolute_time();
			_lpos_ground_truth_pub.publish(local_position_groundtruth);

			if (_pos_ref.isInitialized()) {
				// publish position groundtruth
				vehicle_global_position_s global_position_groundtruth{};
				global_position_groundtruth.timestamp_sample = time_us;

				_pos_ref.reproject(local_position_groundtruth.x, local_position_groundtruth.y,
						   global_position_groundtruth.lat, global_position_groundtruth.lon);

				global_position_groundtruth.alt = _param_sim_home_alt.get() - static_cast<float>(position(2));
				global_position_groundtruth.timestamp = hrt_absolute_time();
				_gpos_ground_truth_pub.publish(global_position_groundtruth);
			}

			return;
		}
	}
}

bool SimulatorIgnitionBridge::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs,
		unsigned num_control_groups_updated)
{
	ignition::msgs::Actuators rotor_velocity_message;
	rotor_velocity_message.mutable_velocity()->Resize(num_outputs, 0);

	for (unsigned i = 0; i < num_outputs; i++) {
		rotor_velocity_message.set_velocity(i, outputs[i]);
	}

	if (_actuators_pub.Valid()) {
		return _actuators_pub.Publish(rotor_velocity_message);
	}

	return false;
}

void SimulatorIgnitionBridge::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_mixing_output.unregister();

		exit_and_cleanup();
		return;
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		updateParams();
	}

	_mixing_output.update();

	//ScheduleDelayed(1000_us);

	// check at end of cycle (updateSubscriptions() can potentially change to a different WorkQueue thread)
	_mixing_output.updateSubscriptions(true);
}

int SimulatorIgnitionBridge::print_status()
{
	//perf_print_counter(_cycle_perf);
	_mixing_output.printStatus();

	return 0;
}

int SimulatorIgnitionBridge::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int SimulatorIgnitionBridge::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("simulator_ignition_bridge", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('m', nullptr, nullptr, "Model name", false);
	PRINT_MODULE_USAGE_PARAM_STRING('w', nullptr, nullptr, "World name", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int simulator_ignition_bridge_main(int argc, char *argv[])
{
	return SimulatorIgnitionBridge::main(argc, argv);
}
