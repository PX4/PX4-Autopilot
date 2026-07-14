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
#include <px4_log.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <uORB/uORB.h>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>

class SensorImuRawBridge : public ModuleBase, public px4::WorkItem
{
public:
	static Descriptor desc;

	SensorImuRawBridge();
	~SensorImuRawBridge() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	// Subscribe to raw accel topic (triggers the work item)
	uORB::SubscriptionCallbackWorkItem _sensor_accel_sub{this, ORB_ID(sensor_accel)};

	// Poll the raw gyro topic on each accel callback
	uORB::Subscription _sensor_gyro_sub{ORB_ID(sensor_gyro)};

	int imu_raw_pipe_ch{0};

};

ModuleBase::Descriptor SensorImuRawBridge::desc{task_spawn, custom_command, print_usage};

SensorImuRawBridge::SensorImuRawBridge() :
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

bool SensorImuRawBridge::init()
{
	if (MPA::Initialize() == -1) {
		PX4_ERR("MPA init failed");
		return false;
	}

	char imu_raw_pipe_name[] = "px4_sensor_imu_raw";
	imu_raw_pipe_ch = MPA::PipeCreate(imu_raw_pipe_name);

	if (imu_raw_pipe_ch == -1) {
		PX4_ERR("Pipe create failed for %s", imu_raw_pipe_name);
		return false;
	}

	if (!_sensor_accel_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void SensorImuRawBridge::Run()
{
	if (should_exit()) {
		_sensor_accel_sub.unregisterCallback();
		exit_and_cleanup(desc);
		return;
	}

	sensor_accel_s accel_data;

	if (_sensor_accel_sub.update(&accel_data)) {

		imu_data_t imu;
		memset(&imu, 0, sizeof(imu));
		imu.magic_number = IMU_MAGIC_NUMBER;

		// Raw acceleration (m/s^2) - straight from the driver, no TC applied
		imu.accl_ms2[0] = accel_data.x;
		imu.accl_ms2[1] = accel_data.y;
		imu.accl_ms2[2] = accel_data.z;

		// Temperature from accel sensor
		imu.temp_c = accel_data.temperature;

		// Raw angular velocity (rad/s)
		sensor_gyro_s gyro_data;

		if (_sensor_gyro_sub.update(&gyro_data)) {
			imu.gyro_rad[0] = gyro_data.x;
			imu.gyro_rad[1] = gyro_data.y;
			imu.gyro_rad[2] = gyro_data.z;
		}

		imu.timestamp_ns = accel_data.timestamp * 1000; // Convert µs to ns

		if (MPA::PipeWrite(imu_raw_pipe_ch, (void *)&imu, sizeof(imu_data_t)) == -1) {
			PX4_ERR("Pipe %d write failed!", imu_raw_pipe_ch);
		}
	}
}

int SensorImuRawBridge::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int SensorImuRawBridge::task_spawn(int argc, char *argv[])
{
	SensorImuRawBridge *instance = new SensorImuRawBridge();

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

int SensorImuRawBridge::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Raw sensor IMU bridge. Publishes raw accel and gyro data from PX4
to the apps processor via MPA pipe. No temperature compensation
or calibration offsets are applied.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("sensor_imu_raw_bridge", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int sensor_imu_raw_bridge_main(int argc, char *argv[])
{
	return ModuleBase::main(SensorImuRawBridge::desc, argc, argv);
}
