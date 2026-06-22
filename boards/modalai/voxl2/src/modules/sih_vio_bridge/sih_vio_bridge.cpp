/****************************************************************************
 *
 *   Copyright (c) 2026 ModalAI, Inc. All rights reserved.
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
 * @file sih_vio_bridge.cpp
 *
 * Bridges SIH ground truth pose data to MPA VIO pipes so that other VOXL2
 * services (e.g. voxl-vision-hub) can consume simulated pose as if it were
 * real VIO data.
 *
 * Subscribes to vehicle_local_position_groundtruth and
 * vehicle_attitude_groundtruth published by the SIH simulator module,
 * converts the data to vio_data_t, and writes to the "hitl_vio" and "state"
 * MPA pipes.
 */

#include "mpa.hpp"
#include <time.h>
#include <px4_log.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <uORB/uORB.h>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>

class SihVioBridge : public ModuleBase, public px4::WorkItem
{
public:
	static Descriptor desc;

	SihVioBridge();
	~SihVioBridge() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	int64_t TimeMonotonic_ns();

	uORB::SubscriptionCallbackWorkItem _local_pos_sub{this, ORB_ID(vehicle_local_position_groundtruth)};
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude_groundtruth)};
	uORB::Subscription _angular_vel_sub{ORB_ID(vehicle_angular_velocity_groundtruth)};

	int _vio_pipe_ch{0};
	int _flow_pipe_ch{0};
};

ModuleBase::Descriptor SihVioBridge::desc{task_spawn, custom_command, print_usage};

SihVioBridge::SihVioBridge() :
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

bool SihVioBridge::init()
{
	if (MPA::Initialize() == -1) {
		return false;
	}

	char vio_pipe_name[] = "hitl_vio";
	_vio_pipe_ch = MPA::PipeCreate(vio_pipe_name);

	if (_vio_pipe_ch == -1) {
		PX4_ERR("Pipe create failed for %s", vio_pipe_name);
		return false;
	}

	char flow_pipe_name[] = "state";
	_flow_pipe_ch = MPA::PipeCreate(flow_pipe_name);

	if (_flow_pipe_ch == -1) {
		PX4_ERR("Pipe create failed for %s", flow_pipe_name);
		return false;
	}

	if (!_local_pos_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

int64_t SihVioBridge::TimeMonotonic_ns()
{
	struct timespec ts;

	if (clock_gettime(CLOCK_MONOTONIC, &ts)) {
		PX4_ERR("ERROR calling clock_gettime");
		return -1;
	}

	return (int64_t)ts.tv_sec * 1000000000 + (int64_t)ts.tv_nsec;
}

void SihVioBridge::Run()
{
	if (should_exit()) {
		_local_pos_sub.unregisterCallback();
		exit_and_cleanup(desc);
		return;
	}

	if (_local_pos_sub.updated()) {
		vehicle_local_position_s lpos{};

		if (_local_pos_sub.copy(&lpos)) {
			vio_data_t s;
			memset(&s, 0, sizeof(s));

			s.magic_number = VIO_MAGIC_NUMBER;
			s.error_code = 0;
			s.state = VIO_STATE_OK;
			s.timestamp_ns = TimeMonotonic_ns();

			// Position
			s.T_imu_wrt_vio[0] = lpos.x;
			s.T_imu_wrt_vio[1] = lpos.y;
			s.T_imu_wrt_vio[2] = lpos.z;

			// Velocity
			s.vel_imu_wrt_vio[0] = lpos.vx;
			s.vel_imu_wrt_vio[1] = lpos.vy;
			s.vel_imu_wrt_vio[2] = lpos.vz;

			// Attitude: convert quaternion to rotation matrix
			vehicle_attitude_s att{};

			if (_attitude_sub.copy(&att)) {
				float w = att.q[0], x = att.q[1], y = att.q[2], z = att.q[3];
				float xx = x * x, yy = y * y, zz = z * z;
				float xy = x * y, xz = x * z, yz = y * z;
				float wx = w * x, wy = w * y, wz = w * z;

				s.R_imu_to_vio[0][0] = 1 - 2 * (yy + zz);
				s.R_imu_to_vio[0][1] = 2 * (xy - wz);
				s.R_imu_to_vio[0][2] = 2 * (xz + wy);

				s.R_imu_to_vio[1][0] = 2 * (xy + wz);
				s.R_imu_to_vio[1][1] = 1 - 2 * (xx + zz);
				s.R_imu_to_vio[1][2] = 2 * (yz - wx);

				s.R_imu_to_vio[2][0] = 2 * (xz - wy);
				s.R_imu_to_vio[2][1] = 2 * (yz + wx);
				s.R_imu_to_vio[2][2] = 1 - 2 * (xx + yy);
			}

			// Angular velocity
			vehicle_angular_velocity_s ang_vel{};

			if (_angular_vel_sub.copy(&ang_vel)) {
				s.imu_angular_vel[0] = ang_vel.xyz[0];
				s.imu_angular_vel[1] = ang_vel.xyz[1];
				s.imu_angular_vel[2] = ang_vel.xyz[2];
			}

			// Gravity vector in VIO frame (NED: gravity is +Z)
			s.gravity_vector[0] = 0.f;
			s.gravity_vector[1] = 0.f;
			s.gravity_vector[2] = 1.f;

			// Ground truth has perfect quality
			s.quality = 100;
			s.n_feature_points = 25;

			// Small fixed covariance for ground truth
			s.pose_covariance[0]  = 1e-6f; // x
			s.pose_covariance[6]  = 1e-6f; // y
			s.pose_covariance[11] = 1e-6f; // z
			s.pose_covariance[15] = 1e-6f; // roll
			s.pose_covariance[18] = 1e-6f; // pitch
			s.pose_covariance[20] = 1e-6f; // yaw

			s.velocity_covariance[0]  = 1e-6f; // vx
			s.velocity_covariance[6]  = 1e-6f; // vy
			s.velocity_covariance[11] = 1e-6f; // vz

			if (MPA::PipeWrite(_vio_pipe_ch, (void *)&s, sizeof(vio_data_t)) == -1) {
				PX4_ERR("Pipe %d write failed!", _vio_pipe_ch);
			}

			if (MPA::PipeWrite(_flow_pipe_ch, (void *)&s, sizeof(vio_data_t)) == -1) {
				PX4_ERR("Pipe %d write failed!", _flow_pipe_ch);
			}
		}
	}
}

int SihVioBridge::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int SihVioBridge::task_spawn(int argc, char *argv[])
{
	SihVioBridge *instance = new SihVioBridge();

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;

	return PX4_ERROR;
}

int SihVioBridge::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Bridges SIH ground truth pose data to MPA VIO pipes.

When running SIH simulation on VOXL2, this module publishes the simulated
pose (position, attitude, velocity, angular velocity) to the "hitl_vio" and
"state" MPA pipes in vio_data_t format, allowing other VOXL2 services to
consume simulated VIO data.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("sih_vio_bridge", "simulation");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int sih_vio_bridge_main(int argc, char *argv[])
{
	return ModuleBase::main(SihVioBridge::desc, argc, argv);
}
