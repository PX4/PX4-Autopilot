/****************************************************************************
 *
 *   Copyright (c) 2013-2021 PX4 Development Team. All rights reserved.
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

#include "AnalogDifferentialPressure.hpp"

#include <px4_platform_common/getopt.h>

AnalogDifferentialPressure::AnalogDifferentialPressure(int16_t adc_channel) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_adc_channel(adc_channel)
{
}

int AnalogDifferentialPressure::init()
{
	_adc_report_sub.registerCallback();
	ScheduleNow();

	return PX4_OK;
}

void AnalogDifferentialPressure::Run()
{
	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
	}

	perf_begin(_sample_perf);

	if (_param_sens_dpres_ansc.get() > 0.0f) {
		adc_report_s adc;

		if (_adc_report_sub.update(&adc)) {
			/* Read add channels we got */
			for (unsigned i = 0; i < PX4_MAX_ADC_CHANNELS; i++) {
				if (_adc_channel == adc.channel_id[i]) {

					// The scale factor defined by HW's resistive divider (Rt+Rb)/ Rb
#if defined(ADC_DP_V_DIV)
					static constexpr float scale = ADC_DP_V_DIV;
#else
					static constexpr float scale = 1.0f;
#endif

					/* calculate airspeed, raw is the difference from */
					const float voltage = (float)(adc.raw_data[i]) * adc.v_ref / adc.resolution * scale;

					/**
					 * The voltage divider pulls the signal down, only act on
					 * a valid voltage from a connected sensor. Also assume a non-
					 * zero offset from the sensor if its connected.
					 *
					 * Notice: This won't work on devices which have PGA controlled
					 * vref. Those devices require no divider at all.
					 */
					if (voltage > 0.4f) {
						const float differential_pressure_pa = voltage * _param_sens_dpres_ansc.get();

						if (fabsf(differential_pressure_pa - _differential_pressure_pa_last) > 0.01f) {
							// only publish changes
							sensor_differential_pressure_s diff_pres{};
							diff_pres.device_id = 0xAA; // TODO DRV_DIFF_PRESS_DEVTYPE_MS4525
							diff_pres.timestamp_sample = adc.timestamp;
							diff_pres.differential_pressure_pa = differential_pressure_pa;
							diff_pres.temperature = NAN;
							diff_pres.timestamp = hrt_absolute_time();
							_diff_pres_pub.publish(diff_pres);

							_differential_pressure_pa_last = differential_pressure_pa;
						}
					}

					break;
				}
			}

			PX4_DEBUG("ADC channel %d not found", _adc_channel);
		}
	}

	perf_end(_sample_perf);
}

int AnalogDifferentialPressure::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int AnalogDifferentialPressure::task_spawn(int argc, char *argv[])
{
#if defined(ADC_AIRSPEED_VOLTAGE_CHANNEL)
	int16_t adc_channel = ADC_AIRSPEED_VOLTAGE_CHANNEL;
#else
	int16_t adc_channel = -1;
#endif

	bool error_flag = false;
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "c:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'c':
			adc_channel = strtoul(myoptarg, nullptr, 10);
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return PX4_ERROR;
	}

	AnalogDifferentialPressure *instance = new AnalogDifferentialPressure(adc_channel);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init() == PX4_OK) {
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

int AnalogDifferentialPressure::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
ADC driver.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("analog_diffpres", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_INT('c', -1, 0, 32, "ADC channel", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int analog_diffpres_main(int argc, char *argv[])
{
	return AnalogDifferentialPressure::main(argc, argv);
}
