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

		snprintf(param_name, sizeof(param_name), "SIM_BAT%d_I_FW", i + 1);
		_i_fw_handles[i] = param_find(param_name);

		if (_i_fw_handles[i] == PARAM_INVALID) {
			PX4_ERR("Could not find parameter with name %s", param_name);
		}

		snprintf(param_name, sizeof(param_name), "SIM_BAT%d_I_MC", i + 1);
		_i_mc_handles[i] = param_find(param_name);

		if (_i_mc_handles[i] == PARAM_INVALID) {
			PX4_ERR("Could not find parameter with name %s", param_name);
		}


		snprintf(param_name, sizeof(param_name), "SIM_BAT%d_MIN_PCT", i + 1);
		_min_pct_handles[i] = param_find(param_name);

		if (_min_pct_handles[i] == PARAM_INVALID) {
			PX4_ERR("Could not find parameter with name %s", param_name);
		}
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
			_is_fw = (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
		}
	}

	const hrt_abstime now_us = hrt_absolute_time();
	const bool disarmed_now = _was_armed && !_armed;
	_was_armed = _armed;

	for (int i = 0; i < battery_status_s::MAX_INSTANCES; i++) {
		Battery &battery = *_batteries[i];

		// The Battery library does the coulomb counting on the published current, its discharged capacity is the
		// simulated state. Without a capacity the battery cannot drain.
		const float capacity = battery.getCapacity();
		const float min_percentage = minimumPercentage(i) / 100.f;
		const bool min_percentage_raised = min_percentage > _last_min_percentage[i];
		_last_min_percentage[i] = min_percentage;

		if (capacity > 0.f) {
			const float min_percentage_discharged_mah = (1.f - min_percentage) * capacity;

			if (disarmed_now) {
				// Fresh battery for the next flight
				battery.resetDischargedMah();

			} else if (min_percentage_raised && battery.getDischargedMah() > min_percentage_discharged_mah) {
				// Regain capacity, e.g. to simulate inaccurate state estimation or in-air charging
				battery.resetDischargedMah(min_percentage_discharged_mah);
			}
		}

		const float percentage = (capacity > 0.f) ? math::max(1.f - battery.getDischargedMah() / capacity, min_percentage) : 1.f;

		// No current while disarmed or once the floor is reached, so the coulomb count stops at SIM_BAT_MIN_PCT
		float ibatt = 0.f;

		if (_armed && (capacity <= 0.f || percentage > min_percentage)) {
			ibatt = avgCurrent(i, _is_fw);

			if (ibatt > 0.f && capacity <= 0.f && battery.isSourceSelected() && !_capacity_warned[i]) {
				PX4_WARN("BAT%d_CAPACITY not set, simulated battery %d will not drain", i + 1, i + 1);
				_capacity_warned[i] = true;
			}
		}

		// Open circuit voltage from the state of charge, minus the load drop the Battery library compensates for
		float vbatt = math::interpolate(percentage, 0.f, 1.f, battery.empty_cell_voltage(), battery.full_cell_voltage());
		vbatt -= battery.internal_resistance() * math::max(ibatt, 0.f);
		vbatt *= battery.cell_count();

		battery.setConnected(true);
		battery.updateVoltage(vbatt);
		battery.updateCurrent(ibatt);
		battery.updateAndPublishBatteryStatus(now_us);
	}

	perf_end(_loop_perf);
}

void BatterySimulator::updateParams()
{
	ModuleParams::updateParams();

	for (int i = 0; i < battery_status_s::MAX_INSTANCES; i++) {
		if (_i_fw_handles[i] != PARAM_INVALID) {
			param_get(_i_fw_handles[i], &_i_fw_override_s[i]);
		}

		if (_i_mc_handles[i] != PARAM_INVALID) {
			param_get(_i_mc_handles[i], &_i_mc_override_s[i]);
		}

		if (_min_pct_handles[i] != PARAM_INVALID) {
			param_get(_min_pct_handles[i], &_min_pct_override[i]);
		}
	}
}

float BatterySimulator::avgCurrent(int battery_index, bool is_fw) const
{
	// A non-positive override means: use the average current time shared by all batteries
	if (is_fw) {
		return (_i_fw_override_s[battery_index] > 0.f) ? _i_fw_override_s[battery_index] : _param_sim_bat_i_fw.get();
	}

	// Rotary wing and every other vehicle type (rover, boat, airship, unspecified) use the MC current
	return (_i_mc_override_s[battery_index] > 0.f) ? _i_mc_override_s[battery_index] : _param_sim_bat_i_mc.get();
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
