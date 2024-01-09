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

#include "SensorAccelNdk.hpp"
#include <dlfcn.h>
#include <time.h>

#include <drivers/drv_sensor.h>

using namespace matrix;

SensorAccelNdk::SensorAccelNdk() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
	struct timespec ts_startup, ts_epoch;
	clock_gettime(CLOCK_MONOTONIC, &ts_startup);
	clock_gettime(CLOCK_BOOTTIME, &ts_epoch);

	long long nanoseconds_startup = (long long)ts_startup.tv_sec * 1000000000LL + ts_startup.tv_nsec;
	long long nanoseconds_epoch = (long long)ts_epoch.tv_sec * 1000000000LL + ts_epoch.tv_nsec;

	DIFF_BOOTTIME_MONOTONIC = (long long)(nanoseconds_epoch - nanoseconds_startup) / 1000;
	// printf("system boot at epoch: %lu ms\n", DIFF_BOOTTIME_MONOTONIC);

	_px4_accel.set_device_type(DRV_IMU_DEVTYPE_SIM);

	char *kPackageName = NULL;
	sensorManager = AcquireASensorManagerInstance(kPackageName);
	accelerometer = ASensorManager_getDefaultSensor(sensorManager, ASENSOR_TYPE_ACCELEROMETER_UNCALIBRATED);
	looper = ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);
	accelerometerEventQueue = ASensorManager_createEventQueue(sensorManager, looper,
				  LOOPER_ID_USER, NULL, NULL);

	ASensorEventQueue_enableSensor(accelerometerEventQueue, accelerometer);
	ASensorEventQueue_setEventRate(accelerometerEventQueue, accelerometer, SENSOR_REFRESH_PERIOD_US);
}

SensorAccelNdk::~SensorAccelNdk()
{
	perf_free(_loop_perf);
}

ASensorManager *SensorAccelNdk::AcquireASensorManagerInstance(char kPackageName[])
{
	// cppcheck-suppress  cstyleCast
	typedef ASensorManager *(*PF_GETINSTANCEFORPACKAGE)(const char *name);
	void *androidHandle = dlopen("libandroid.so", RTLD_NOW);
	PF_GETINSTANCEFORPACKAGE getInstanceForPackageFunc = (PF_GETINSTANCEFORPACKAGE)
			dlsym(androidHandle, "ASensorManager_getInstanceForPackage");

	if (getInstanceForPackageFunc) {
		return getInstanceForPackageFunc(kPackageName);
	}

	// cppcheck-suppress  cstyleCast
	typedef ASensorManager *(*PF_GETINSTANCE)();
	PF_GETINSTANCE getInstanceFunc = (PF_GETINSTANCE)
					 dlsym(androidHandle, "ASensorManager_getInstance");
	// by all means at this point, ASensorManager_getInstance should be available
	return getInstanceFunc();
}


bool SensorAccelNdk::init()
{
	ScheduleOnInterval(SENSOR_REFRESH_PERIOD_US);
	return true;
}

void SensorAccelNdk::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
	}

	// vehicle_attitude_s attitude;
	// _vehicle_attitude_sub.update(&attitude);
	ALooper_pollAll(0, NULL, NULL, NULL);
	ASensorEvent event;
	float x, y, z;

	while (ASensorEventQueue_getEvents(accelerometerEventQueue, &event, 1) > 0) {

		x = event.uncalibrated_acceleration.y_uncalib;
		y = event.uncalibrated_acceleration.x_uncalib;
		z = event.uncalibrated_acceleration.z_uncalib * -1;
		hrt_abstime now = (uint64_t)event.timestamp / 1000;
		_px4_accel.update(now - DIFF_BOOTTIME_MONOTONIC, x, y, z);
	}

	// PX4_WARN("%ld, %ld", event.timestamp, DIFF_BOOTTIME_MONOTONIC);
	// PX4_WARN("_px4_accel.update %f, %f, %f", (double)x, (double)y, (double)z);

	perf_end(_loop_perf);
}

int SensorAccelNdk::task_spawn(int argc, char *argv[])
{
	SensorAccelNdk *instance = new SensorAccelNdk();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
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

int SensorAccelNdk::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int SensorAccelNdk::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description


)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("sensor_accel_ndk", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int sensor_accel_ndk_main(int argc, char *argv[])
{
	return SensorAccelNdk::main(argc, argv);
}
