/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
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

#include "analog_measurement.hpp"

AnalogMeasurement::AnalogMeasurement() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

bool
AnalogMeasurement::init()
{
	if (!_adc_report_sub.registerCallback()) {
		PX4_ERR("adc_report callback registration failed!");
		return false;
	}

	return true;
}

void
AnalogMeasurement::Run()
{
	if (should_exit()) {
		_adc_report_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	adc_report_s report{};

	if (_adc_report_sub.update(&report)) {

		analog_measurement_s measurement{};

		int adc1_unit = _adc1_unit.get();
		int adc2_unit = _adc2_unit.get();
		int adc3_unit = _adc3_unit.get();
		int adc4_unit = _adc4_unit.get();

		// TODO: determine size. Only read first 4 ADCs for now.
		if (adc1_unit) {
			measurement.values[0] = report.raw_data[0] *_adc1_scale.get();
			measurement.unit_type[0] = adc1_unit;
		}

		if (adc2_unit) {
			measurement.values[1] = report.raw_data[1] *_adc2_scale.get();
			measurement.unit_type[1] = adc2_unit;
		}

		if (adc3_unit) {
			measurement.values[2] = report.raw_data[2] *_adc3_scale.get();
			measurement.unit_type[2] = adc3_unit;
		}

		if (adc4_unit) {
			measurement.values[3] = report.raw_data[3] *_adc4_scale.get();
			measurement.unit_type[3] = adc4_unit;
		}

		_analog_measurement_pub.publish(measurement);
	}
}

int AnalogMeasurement::task_spawn(int argc, char *argv[])
{
	AnalogMeasurement *instance = new AnalogMeasurement();

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

int AnalogMeasurement::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int AnalogMeasurement::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module reads the adc_report and converts the raw adc values into voltage,
current, or temperature depending on the parameter ADC[N]_UNIT and applies the
scale factor ADC[N]_SCALE.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("analog_measurement", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int analog_measurement_main(int argc, char *argv[])
{
	return AnalogMeasurement::main(argc, argv);
}