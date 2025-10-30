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
#include <uORB/topics/vehicle_air_data.h>

class VehicleAirDataBridge : public ModuleBase<VehicleAirDataBridge>, public px4::WorkItem
{
public:

	VehicleAirDataBridge();
	~VehicleAirDataBridge() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	uORB::SubscriptionCallbackWorkItem _vehicle_air_data_sub{this, ORB_ID(vehicle_air_data)};

	vehicle_air_data_s _vehicle_air_data{};

	int baro_pipe_ch{0};

};

VehicleAirDataBridge::VehicleAirDataBridge() :
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

bool VehicleAirDataBridge::init()
{
	if (MPA::Initialize() == -1) {
		PX4_ERR("MPA init failed");
		return false;
	}

	char baro_pipe_name[] = "px4_vehicle_air_data";
	baro_pipe_ch = MPA::PipeCreate(baro_pipe_name);
	if (baro_pipe_ch == -1) {
		PX4_ERR("Pipe create failed for %s", baro_pipe_name);
		return false;
	}

	if (!_vehicle_air_data_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void VehicleAirDataBridge::Run()
{
	if (should_exit()) {
		_vehicle_air_data_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	if (_vehicle_air_data_sub.updated()) {
		if (_vehicle_air_data_sub.update(&_vehicle_air_data)) {
			baro_data_t baro;
			memset(&baro, 0, sizeof(baro));

			baro.magic_number = BARO_MAGIC_NUMBER;
			baro.pressure_pa = _vehicle_air_data.baro_pressure_pa;
			baro.temp_c = _vehicle_air_data.baro_temp_celcius;
			baro.alt_amsl_m = _vehicle_air_data.baro_alt_meter;
			baro.timestamp_ns = _vehicle_air_data.timestamp * 1000; // Convert µs to ns
			baro.reserved_1 = 0;
			baro.reserved_2 = 0;

			if (MPA::PipeWrite(baro_pipe_ch, (void*)&baro, sizeof(baro_data_t)) == -1) {
				PX4_ERR("Pipe %d write failed!", baro_pipe_ch);
			}
		}
	}
}

int VehicleAirDataBridge::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int VehicleAirDataBridge::task_spawn(int argc, char *argv[])
{
	VehicleAirDataBridge *instance = new VehicleAirDataBridge();

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

int VehicleAirDataBridge::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Vehicle air data bridge

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("vehicle_air_data_bridge", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int vehicle_air_data_bridge_main(int argc, char *argv[])
{
	return VehicleAirDataBridge::main(argc, argv);
}
