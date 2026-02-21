/****************************************************************************
 *
 *   Copyright (c) 2025 ModalAI, inc. All rights reserved.
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
#include "mpa.hpp"
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>
#include <math.h>
#include <px4_log.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <uORB/uORB.h>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/vehicle_local_position.h>

class VehicleLocalPositionBridge : public ModuleBase, public px4::WorkItem
{
public:
	static Descriptor desc;

	VehicleLocalPositionBridge();
	~VehicleLocalPositionBridge() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	uORB::SubscriptionCallbackWorkItem _vehicle_local_position_sub{this, ORB_ID(vehicle_local_position)};

	vehicle_local_position_s _vehicle_local_position{};

	int _pipe_ch{0};

};

ModuleBase::Descriptor VehicleLocalPositionBridge::desc{task_spawn, custom_command, print_usage};

VehicleLocalPositionBridge::VehicleLocalPositionBridge() :
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

bool VehicleLocalPositionBridge::init()
{
	if (MPA::Initialize() == -1) {
		PX4_ERR("MPA init failed");
		return false;
	}

	char pipe_name[] = "px4_vehicle_local_position";
	_pipe_ch = MPA::PipeCreate(pipe_name);

	if (_pipe_ch == -1) {
		PX4_ERR("Pipe create failed for %s", pipe_name);
		return false;
	}

	if (!_vehicle_local_position_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void VehicleLocalPositionBridge::Run()
{
	if (should_exit()) {
		_vehicle_local_position_sub.unregisterCallback();
		exit_and_cleanup(desc);
		return;
	}

	if (_vehicle_local_position_sub.updated()) {
		if (_vehicle_local_position_sub.update(&_vehicle_local_position)) {
			// Only publish if we have valid position data
			if (!_vehicle_local_position.xy_valid && !_vehicle_local_position.z_valid) {
				return;
			}

			pose_vel_6dof_t pose;

			pose.magic_number = POSE_VEL_6DOF_MAGIC_NUMBER;
			pose.timestamp_ns = _vehicle_local_position.timestamp * 1000; // Convert µs to ns

			// Position (NED frame)
			if (_vehicle_local_position.xy_valid) {
				pose.T_child_wrt_parent[0] = _vehicle_local_position.x;
				pose.T_child_wrt_parent[1] = _vehicle_local_position.y;

			} else {
				pose.T_child_wrt_parent[0] = NAN;
				pose.T_child_wrt_parent[1] = NAN;
			}

			if (_vehicle_local_position.z_valid) {
				pose.T_child_wrt_parent[2] = _vehicle_local_position.z;

			} else {
				pose.T_child_wrt_parent[2] = NAN;
			}

			// Rotation matrix from heading (yaw rotation around Z axis)
			// R_z(heading) = [cos(h)  -sin(h)  0]
			//                [sin(h)   cos(h)  0]
			//                [0        0       1]
			float cos_h = cosf(_vehicle_local_position.heading);
			float sin_h = sinf(_vehicle_local_position.heading);
			pose.R_child_to_parent[0][0] = cos_h;
			pose.R_child_to_parent[0][1] = -sin_h;
			pose.R_child_to_parent[0][2] = 0.0f;
			pose.R_child_to_parent[1][0] = sin_h;
			pose.R_child_to_parent[1][1] = cos_h;
			pose.R_child_to_parent[1][2] = 0.0f;
			pose.R_child_to_parent[2][0] = 0.0f;
			pose.R_child_to_parent[2][1] = 0.0f;
			pose.R_child_to_parent[2][2] = 1.0f;

			// Velocity (NED frame)
			if (_vehicle_local_position.v_xy_valid) {
				pose.v_child_wrt_parent[0] = _vehicle_local_position.vx;
				pose.v_child_wrt_parent[1] = _vehicle_local_position.vy;

			} else {
				pose.v_child_wrt_parent[0] = NAN;
				pose.v_child_wrt_parent[1] = NAN;
			}

			if (_vehicle_local_position.v_z_valid) {
				pose.v_child_wrt_parent[2] = _vehicle_local_position.vz;

			} else {
				pose.v_child_wrt_parent[2] = NAN;
			}

			// Angular velocity not available in vehicle_local_position
			pose.w_child_wrt_child[0] = NAN;
			pose.w_child_wrt_child[1] = NAN;
			pose.w_child_wrt_child[2] = NAN;

			if (MPA::PipeWrite(_pipe_ch, (void *)&pose, sizeof(pose_vel_6dof_t)) == -1) {
				PX4_ERR("Pipe %d write failed!", _pipe_ch);
			}
		}
	}
}

int VehicleLocalPositionBridge::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int VehicleLocalPositionBridge::task_spawn(int argc, char *argv[])
{
	VehicleLocalPositionBridge *instance = new VehicleLocalPositionBridge();

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

int VehicleLocalPositionBridge::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Vehicle local position bridge - publishes vehicle_local_position to MPA pipe as pose_vel_6dof_t

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("vehicle_local_position_bridge", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int vehicle_local_position_bridge_main(int argc, char *argv[])
{
	return ModuleBase::main(VehicleLocalPositionBridge::desc, argc, argv);
}
