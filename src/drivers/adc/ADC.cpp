/****************************************************************************
 *
 *   Copyright (C) 2012-2019 PX4 Development Team. All rights reserved.
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

#include "ADC.hpp"

static constexpr unsigned ADC_TOTAL_CHANNELS = 32;

ADC::ADC() :
	ScheduledWorkItem(px4::wq_configurations::hp_default),
	_sample_perf(perf_alloc(PC_ELAPSED, "adc: sample"))
{
	for (int i = 0; i < adc_report_s::MAX_CHANNELS; i++) {
		_samples.channel_id[i] = -1;
	}

	uint32_t channels = ADC_CHANNELS;

	// allocate the sample array
	_samples.channels = 0;

	for (unsigned i = 0; i < ADC_TOTAL_CHANNELS; i++) {
		if (channels & (1 << i)) {
			_samples.channels++;
		}
	}

	// prefill the channel numbers in the sample array
	unsigned index = 0;

	for (unsigned i = 0; i < ADC_TOTAL_CHANNELS; i++) {
		if (channels & (1 << i)) {
			_samples.channel_id[index] = i;
			_samples.channel_value[index] = 0;
			index++;
		}

		if (index >= adc_report_s::MAX_CHANNELS) {
			break;
		}
	}
}

ADC::~ADC()
{
	ScheduleClear();
	perf_free(_sample_perf);
}

int
ADC::init()
{
	int rv = board_adc_init();

	if (rv < 0) {
		PX4_DEBUG("sample timeout");
		return rv;
	}

	return start();
}

int
ADC::start()
{
	// schedule regular updates
	ScheduleOnInterval(TICKRATE);
	return PX4_OK;
}

int
ADC::stop()
{
	ScheduleClear();
	return PX4_OK;
}

void
ADC::Run()
{
	perf_begin(_sample_perf);

	// scan the channel set and sample each
	_samples.timestamp = hrt_absolute_time();

	for (unsigned i = 0; i < _samples.channels; i++) {

		uint16_t result = board_adc_sample(_samples.channel_id[i]);

		if (result == 0xffff) {
			_samples.channel_value[i] = 0;
			PX4_ERR("sample timeout");

		} else {
			_samples.channel_value[i] = result;
		}
	}

	_adc_report_pub.publish(_samples);

	perf_end(_sample_perf);
}

int
ADC::print_status()
{
	PX4_INFO("Running");

	perf_print_counter(_sample_perf);

	return 0;
}

int
ADC::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int
ADC::task_spawn(int argc, char *argv[])
{
	ADC *instance = new ADC();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init() == PX4_OK) {
			return instance->start();
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int
ADC::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
ADC driver.
)DESCR_STR");

	PRINT_MODULE_USAGE_COMMAND("start");

	PRINT_MODULE_USAGE_NAME("adc", "driver");

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int adc_main(int argc, char *argv[]);

int adc_main(int argc, char *argv[])
{
	return ADC::main(argc, argv);
}
