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

#include "IgnitionSimulator.hpp"

#include <uORB/topics/actuator_outputs.h>
#include <uORB/Subscription.hpp>

#include <iostream>
#include <string>

void IgnitionSimulator::ImuCallback(const ignition::msgs::IMU &imu)
{
	// FLU -> FRD
	_px4_accel.update(hrt_absolute_time(),
			  imu.linear_acceleration().x(),
			  -imu.linear_acceleration().y(),
			  -imu.linear_acceleration().z());

	_px4_gyro.update(hrt_absolute_time(),
			 imu.angular_velocity().x(),
			 -imu.angular_velocity().y(),
			 -imu.angular_velocity().z());
}

void IgnitionSimulator::PoseInfoCallback(const ignition::msgs::Pose_V &pose)
{
	for (int p = 0; p < pose.pose_size(); p++) {
		std::string vehicle_name = "X4";

		if (pose.pose(p).name() == vehicle_name) {

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
			vehicle_attitude_groundtruth.timestamp_sample = hrt_absolute_time();
			vehicle_attitude_groundtruth.q[0] = q_nb.W();
			vehicle_attitude_groundtruth.q[1] = q_nb.X();
			vehicle_attitude_groundtruth.q[2] = q_nb.Y();
			vehicle_attitude_groundtruth.q[3] = q_nb.Z();
			vehicle_attitude_groundtruth.timestamp = hrt_absolute_time();
			_attitude_ground_truth_pub.publish(vehicle_attitude_groundtruth);


			if (!_pos_ref.isInitialized()) {
				_pos_ref.initReference(_param_sim_home_lat.get(), _param_sim_home_lon.get(), hrt_absolute_time());
			}

			vehicle_local_position_s local_position_groundtruth{};
			local_position_groundtruth.timestamp_sample = hrt_absolute_time();

			const double dt = hrt_elapsed_time(&_timestamp_prev) * 1e-6f;
			const matrix::Vector3d position{pose_position.x(), pose_position.y(), pose_position.z()};
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
				global_position_groundtruth.timestamp_sample = hrt_absolute_time();

				_pos_ref.reproject(local_position_groundtruth.x, local_position_groundtruth.y,
						   global_position_groundtruth.lat, global_position_groundtruth.lon);

				global_position_groundtruth.alt = _param_sim_home_alt.get() - static_cast<float>(pose_position.z());
				global_position_groundtruth.timestamp = hrt_absolute_time();
				_gpos_ground_truth_pub.publish(global_position_groundtruth);
			}
		}
	}
}

IgnitionSimulator::IgnitionSimulator() :
	ModuleParams(nullptr),
	_px4_accel(1310988), // 1310988: DRV_IMU_DEVTYPE_SIM, BUS: 1, ADDR: 1, TYPE: SIMULATION
	_px4_gyro(1310988)   // 1310988: DRV_IMU_DEVTYPE_SIM, BUS: 1, ADDR: 1, TYPE: SIMULATION
{
	_px4_accel.set_range(2000.f); // don't care
	_px4_gyro.set_scale(math::radians(2000.f) / static_cast<float>(INT16_MAX - 1)); // 2000 degrees/second max
}

void IgnitionSimulator::run()
{
	ignition::transport::Node node;

	// Subscribe to messages of other plugins.
	node.Subscribe("/world/quadcopter/model/X4/link/base_link/sensor/imu_sensor/imu", &IgnitionSimulator::ImuCallback,
		       this);
	node.Subscribe("/world/quadcopter/pose/info", &IgnitionSimulator::PoseInfoCallback, this);

	std::string topic = "/X4/command/motor_speed";
	auto pub = node.Advertise<ignition::msgs::Actuators>(topic);

	ignition::msgs::Actuators rotor_velocity_message;
	rotor_velocity_message.mutable_velocity()->Resize(6, 0);


	uORB::Subscription actuator_outputs_sub{ORB_ID(actuator_outputs)};

	while (true) {
		actuator_outputs_s actuator_outputs;

		if (actuator_outputs_sub.update(&actuator_outputs)) {
			rotor_velocity_message.set_velocity(0, math::constrain(actuator_outputs.output[0] - 1000, 0.f, 1000.f));
			rotor_velocity_message.set_velocity(1, math::constrain(actuator_outputs.output[1] - 1000, 0.f, 1000.f));
			rotor_velocity_message.set_velocity(2, math::constrain(actuator_outputs.output[2] - 1000, 0.f, 1000.f));
			rotor_velocity_message.set_velocity(3, math::constrain(actuator_outputs.output[3] - 1000, 0.f, 1000.f));
			rotor_velocity_message.set_velocity(4, math::constrain(actuator_outputs.output[4] - 1000, 0.f, 1000.f));
			rotor_velocity_message.set_velocity(5, math::constrain(actuator_outputs.output[5] - 1000, 0.f, 1000.f));

			if (!pub.Publish(rotor_velocity_message)) {
				PX4_ERR("publish failed");
			}
		}

		px4_usleep(1000);
	}

	// TODO: publish


	// Zzzzzz.
	ignition::transport::waitForShutdown();
}

int IgnitionSimulator::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("ignition_simulator",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

IgnitionSimulator *IgnitionSimulator::instantiate(int argc, char *argv[])
{
	IgnitionSimulator *instance = new IgnitionSimulator();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

int IgnitionSimulator::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int IgnitionSimulator::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int IgnitionSimulator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ignition_simulator", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int ignition_simulator_main(int argc, char *argv[])
{
	return IgnitionSimulator::main(argc, argv);
}
