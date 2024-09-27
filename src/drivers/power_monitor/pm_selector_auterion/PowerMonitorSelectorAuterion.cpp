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

/**
 * Automatic handling power monitors
 */

#include "PowerMonitorSelectorAuterion.h"
#include "../ina226/ina226.h"

#include <builtin/builtin.h>
#include <sys/wait.h>

PowerMonitorSelectorAuterion::PowerMonitorSelectorAuterion() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

PowerMonitorSelectorAuterion::~PowerMonitorSelectorAuterion() = default;

bool PowerMonitorSelectorAuterion::init()
{
	int32_t sens_en = 0;
	param_get(param_find("SENS_EN_INA226"), &sens_en);

	if (sens_en == 1) {

		sens_en = 0;
		param_set(param_find("SENS_EN_INA226"), &sens_en);
		const char *stop_argv[] {"ina226", "stop", NULL};
		exec_builtin("ina226", (char **)stop_argv, NULL, 0);
	}

	ScheduleNow();
	return true;
}

void PowerMonitorSelectorAuterion::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	actuator_armed_s actuator_armed{};
	_actuator_armed_sub.copy(&actuator_armed);

	if (actuator_armed.armed) {
		exit_and_cleanup();
		return;
	}

	for (uint32_t i = 0U; i <  SENSORS_NUMBER; ++i) {

		if (!_sensors[i].started) {

			int ret_val = ina226_probe(i);

			if (ret_val == PX4_OK) {
				char bus_number[4] = {0};
				itoa(_sensors[i].bus_number, bus_number, 10);
				const char *start_argv[] {
					_sensors[i].name,
					"-X", "-b", bus_number, "-a", _sensors[i].i2c_addr,
					"-t", _sensors[i].id, "-q", "start", NULL
				};

				int status = PX4_ERROR;
				int pid =  exec_builtin(_sensors[i].name, (char **)start_argv, NULL, 0);

				if (pid != -1) {
					waitpid(pid, &status, WUNTRACED);
				}

				if (status == PX4_OK) {
					_sensors[i].started = true;
				}
			}
		}
	}

	ScheduleDelayed(RUN_INTERVAL);
}

int PowerMonitorSelectorAuterion::ina226_probe(uint32_t instance)
{
	struct i2c_master_s *i2c = px4_i2cbus_initialize(_sensors[instance].bus_number);
	int ret = PX4_ERROR;

	if (i2c != nullptr) {

		struct i2c_msg_s msgv[2];

		uint8_t txdata[1] = {0};
		uint8_t rxdata[2] = {0};

		msgv[0].frequency = I2C_SPEED_STANDARD;
		msgv[0].addr = static_cast<uint16_t>(strtol(_sensors[instance].i2c_addr, NULL, 0));
		msgv[0].flags = 0;
		msgv[0].buffer = txdata;
		msgv[0].length = sizeof(txdata);

		msgv[1].frequency = I2C_SPEED_STANDARD;
		msgv[1].addr = static_cast<uint16_t>(strtol(_sensors[instance].i2c_addr, NULL, 0));
		msgv[1].flags = I2C_M_READ;
		msgv[1].buffer = rxdata;
		msgv[1].length = sizeof(rxdata);

		txdata[0] = {INA226_MFG_ID};
		ret = I2C_TRANSFER(i2c, msgv, 2);
		uint16_t value = static_cast<uint16_t>(rxdata[1] | rxdata[0] << 8);

		if (ret != PX4_OK || value != INA226_MFG_ID_TI) {

			ret = PX4_ERROR;

		} else {

			txdata[0] = {INA226_MFG_DIEID};
			ret = I2C_TRANSFER(i2c, msgv, 2);
			value = static_cast<uint16_t>(rxdata[1] | rxdata[0] << 8);

			if (ret != PX4_OK || value != INA226_MFG_DIE) {
				ret = PX4_ERROR;
			}
		}

		px4_i2cbus_uninitialize(i2c);
	}

	return ret;
}

int PowerMonitorSelectorAuterion::task_spawn(int argc, char *argv[])
{
	PowerMonitorSelectorAuterion *instance = new PowerMonitorSelectorAuterion();

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

int PowerMonitorSelectorAuterion::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int PowerMonitorSelectorAuterion::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Driver for starting and auto-detecting different power monitors.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("pm_selector_auterion", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int pm_selector_auterion_main(int argc, char *argv[])
{
	return PowerMonitorSelectorAuterion::main(argc, argv);
}
