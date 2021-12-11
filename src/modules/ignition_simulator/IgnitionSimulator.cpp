/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
	// TODO: find by name or id?
	//

	//pose.position
	//pose.orientation


	// TODO:
	//  - gps
	//  - magnetometer
	//  - barometer

	//  - groundtruth

	for (int p = 0; p < pose.pose_size(); p++) {
		//std::cout << p << pose.pose(p).name() << std::endl;

		std::string vehicle_name = "x3";

		if (pose.pose(p).name() == vehicle_name) {

			//std::cout << pose.pose(p) << std::endl;

			// ignition.msgs.Header header
			// .ignition.msgs.Vector3d position = 4;
			// .ignition.msgs.Quaternion orientation = 5;

			ignition::msgs::Vector3d position = pose.pose(p).position();
			ignition::msgs::Quaternion orientation = pose.pose(p).orientation();
			// position.x(), position.y(), position.z()
			// orientation. wxyz

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
					last_imu_message_.orientation().w(),
					last_imu_message_.orientation().x(),
					last_imu_message_.orientation().y(),
					last_imu_message_.orientation().z());

			ignition::math::Quaterniond q_gb = q_gr * q_FLU_to_FRD.Inverse();
			ignition::math::Quaterniond q_nb = q_ENU_to_NED * q_gb;

			hil_state_quat.attitude_quaternion[0] = q_nb.W();
			hil_state_quat.attitude_quaternion[1] = q_nb.X();
			hil_state_quat.attitude_quaternion[2] = q_nb.Y();
			hil_state_quat.attitude_quaternion[3] = q_nb.Z();

			//PX4_INFO("matched, position [%.6f, %.6f, %.6f]", (double)position.x(), (double)position.y(), (double)position.z());

			vehicle_local_position_s local_position_groundtruth{};
			local_position_groundtruth.x = position.x();
			local_position_groundtruth.y = position.y();
			local_position_groundtruth.z = position.z();
			local_position_groundtruth.timestamp = hrt_absolute_time();

			_lpos_ground_truth_pub.publish(local_position_groundtruth);


			// _param_sim_home_lat.get()
			//  reproject

		}
	}




// header {
//   stamp {
//     sec: 93
//     nsec: 200000000
//   }
// }
// pose {
//   name: "x3"
//   id: 8
//   position {
//     x: 9.41649363105415e-19
//     y: -1.038857584459745e-19
//     z: 0.054999053759016557
//   }
//   orientation {
//     x: -3.5059616538432739e-20
//     y: -7.2796873500671957e-18
//     z: 1.3844952801031163e-18
//     w: 1
//   }
// }



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
	node.Subscribe("/imu", &IgnitionSimulator::ImuCallback, this);
	node.Subscribe("/world/quadcopter/pose/info", &IgnitionSimulator::PoseInfoCallback, this);


	std::string topic = "/X3/gazebo/command/motor_speed";
	auto pub = node.Advertise<ignition::msgs::Actuators>(topic);

	ignition::msgs::Actuators rotor_velocity_message;
	rotor_velocity_message.mutable_velocity()->Resize(4, 0);


	uORB::Subscription actuator_outputs_sub{ORB_ID(actuator_outputs)};

	while (true) {
		actuator_outputs_s actuator_outputs;

		if (actuator_outputs_sub.update(&actuator_outputs)) {
			rotor_velocity_message.set_velocity(0, math::constrain(actuator_outputs.output[0] - 1000, 0.f, 1000.f));
			rotor_velocity_message.set_velocity(1, math::constrain(actuator_outputs.output[1] - 1000, 0.f, 1000.f));
			rotor_velocity_message.set_velocity(2, math::constrain(actuator_outputs.output[2] - 1000, 0.f, 1000.f));
			rotor_velocity_message.set_velocity(3, math::constrain(actuator_outputs.output[3] - 1000, 0.f, 1000.f));

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
