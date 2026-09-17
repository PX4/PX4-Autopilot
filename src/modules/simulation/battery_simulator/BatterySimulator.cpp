/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include "BatterySimulator.hpp"
#include <cmath>

ModuleBase::Descriptor BatterySimulator::desc{task_spawn, custom_command, print_usage};

BatterySimulator::BatterySimulator() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
	for (int i = 0; i < battery_status_s::MAX_INSTANCES; i++) {
		_batteries[i] = new Battery(i + 1, this, BATTERY_SIMLATOR_SAMPLE_INTERVAL_US, battery_status_s::SOURCE_POWER_MODULE);

		if (_batteries[i] == nullptr) {
			PX4_ERR("battery %d alloc failed", i + 1);
		}

		char param_name[17]; // 16 chars for parameter name + null terminator

		snprintf(param_name, sizeof(param_name), "SIM_BAT%d_DRAIN", i + 1);
		_drain_handles[i] = param_find(param_name);

		if (_drain_handles[i] == PARAM_INVALID) {
			PX4_ERR("Could not find parameter with name %s", param_name);
		}

		snprintf(param_name, sizeof(param_name), "SIM_BAT%d_MIN_PCT", i + 1);
		_min_pct_handles[i] = param_find(param_name);

		if (_min_pct_handles[i] == PARAM_INVALID) {
			PX4_ERR("Could not find parameter with name %s", param_name);
		}

		_battery_percentage[i] = 1.f;
	}

	updateParams();
}

BatterySimulator::~BatterySimulator()
{
	for (Battery *battery : _batteries) {
		delete battery;
	}

	perf_free(_loop_perf);
}

bool BatterySimulator::init()
{
	for (Battery *battery : _batteries) {
		if (battery == nullptr) {
			return false;
		}
	}

	ScheduleOnInterval(BATTERY_SIMLATOR_SAMPLE_INTERVAL_US);
	return true;
}

void BatterySimulator::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup(desc);
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

	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.copy(&vehicle_status)) {
			_armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
		}
	}

	const hrt_abstime now_us = hrt_absolute_time();

	const float ibatt = -1.0f; // no current sensor in simulation

	for (int i = 0; i < battery_status_s::MAX_INSTANCES; i++) {
		// Limit to +1.0 s to guard against division by 0
		const float discharge_interval_us = math::max(fullDischargeTime(i), 1.0f) * 1000 * 1000;

		if (_armed) {
			if (_last_integration_us != 0) {
				_battery_percentage[i] -= (now_us - _last_integration_us) / discharge_interval_us;
			}

		} else {
			_battery_percentage[i] = 1.f;
		}

		_battery_percentage[i] = math::max(_battery_percentage[i], minimumPercentage(i) / 100.f);

		Battery &battery = *_batteries[i];

		float vbatt = math::interpolate(_battery_percentage[i], 0.f, 1.f, battery.empty_cell_voltage(),
						battery.full_cell_voltage());

		vbatt *= battery.cell_count();

		battery.setConnected(true);
		battery.updateVoltage(vbatt);
		battery.updateCurrent(ibatt);
		battery.updateAndPublishBatteryStatus(now_us);
	}

	// All batteries integrate over the same interval, so this is only advanced once they are all done
	_last_integration_us = _armed ? now_us : 0;

	perf_end(_loop_perf);
}

void BatterySimulator::updateParams()
{
	ModuleParams::updateParams();

	for (int i = 0; i < battery_status_s::MAX_INSTANCES; i++) {
		if (_drain_handles[i] != PARAM_INVALID) {
			param_get(_drain_handles[i], &_drain_override_s[i]);
		}

		if (_min_pct_handles[i] != PARAM_INVALID) {
			param_get(_min_pct_handles[i], &_min_pct_override[i]);
		}
	}
}

float BatterySimulator::fullDischargeTime(int battery_index) const
{
	// A non-positive override means: use the drain time shared by all batteries
	return (_drain_override_s[battery_index] > 0.f) ? _drain_override_s[battery_index] : _param_sim_bat_drain.get();
}

float BatterySimulator::minimumPercentage(int battery_index) const
{
	// Only a negative override means: use the floor shared by all batteries. Zero is a valid floor.
	return (_min_pct_override[battery_index] >= 0.f) ? _min_pct_override[battery_index] : _param_bat_min_pct.get();
}

int BatterySimulator::task_spawn(int argc, char *argv[])
{
	BatterySimulator *instance = new BatterySimulator();

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

int BatterySimulator::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int BatterySimulator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description


)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("battery_simulator", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int battery_simulator_main(int argc, char *argv[])
{
	return ModuleBase::main(BatterySimulator::desc, argc, argv);
}
