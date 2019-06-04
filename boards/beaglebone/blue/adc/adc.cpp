/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file bbblue_adc.cpp
 *
 * BBBlue ADC Driver
 *
 */

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_config.h>
#include <px4_getopt.h>
#include <px4_log.h>
#include <px4_module.h>
#include <px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/adc_report.h>
#include <uORB/topics/system_power.h>

#ifdef __DF_BBBLUE
#include <robotcontrol.h>
#include <board_config.h>
#endif

#define BBBLUE_MAX_ADC_CHANNELS 	            (6)
#define BBBLUE_MAX_ADC_USER_CHANNELS 	        (4)
#define BBBLUE_ADC_DC_JACK_VOLTAGE_CHANNEL 		(5)
#define BBBLUE_ADC_LIPO_BATTERY_VOLTAGE_CHANNEL (6)

class ADC : public ModuleBase<ADC>, public px4::ScheduledWorkItem
{
public:
	ADC();
	~ADC();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	int		start();
	int		stop();

private:
	void			Run() override;

	void			update_adc_report(const hrt_abstime &now);

	static constexpr uint32_t TICKRATE{10_ms};	/**< 100Hz base rate */

	perf_counter_t				_sample_perf;
	adc_report_s				_samples{};		/**< sample buffer */

	uORB::Publication<adc_report_s>		_adc_report_pub{ORB_ID(adc_report)};
	uORB::Publication<system_power_s>	_system_power_pub{ORB_ID(system_power)};
};

ADC::ADC() :
	ScheduledWorkItem(px4::wq_configurations::hp_default),
	_sample_perf(perf_alloc(PC_ELAPSED, "adc_sample"))
{
	for (int i = 0; i < adc_report_s::MAX_CHANNELS; i++) {
		_samples.channel_id[i] = -1;
	}
}

ADC::~ADC()
{
	ScheduleClear();
	perf_free(_sample_perf);
}

int
ADC::start()
{
	/* and schedule regular updates */
	ScheduleOnInterval(TICKRATE);
	return PX4_OK;
}

void
ADC::Run()
{
	px4_adc_msg_t tmp_samples[BBBLUE_MAX_ADC_CHANNELS];

	for (int i = 0; i < BBBLUE_MAX_ADC_USER_CHANNELS; ++i) {
		tmp_samples[i].am_channel = i;
		tmp_samples[i].am_data = rc_adc_read_raw(i);
	}

	tmp_samples[BBBLUE_MAX_ADC_USER_CHANNELS].am_channel = BBBLUE_ADC_LIPO_BATTERY_VOLTAGE_CHANNEL;
	tmp_samples[BBBLUE_MAX_ADC_USER_CHANNELS].am_data    = rc_adc_read_raw(BBBLUE_ADC_LIPO_BATTERY_VOLTAGE_CHANNEL);

	tmp_samples[BBBLUE_ADC_DC_JACK_VOLTAGE_CHANNEL].am_channel = BBBLUE_ADC_DC_JACK_VOLTAGE_CHANNEL;
	tmp_samples[BBBLUE_ADC_DC_JACK_VOLTAGE_CHANNEL].am_data    = rc_adc_read_raw(BBBLUE_ADC_DC_JACK_VOLTAGE_CHANNEL);

	_samples.timestamp = hrt_absolute_time();
	_adc_report_pub.publish(_samples);
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
	ADC *instance = new ADC(ADC_CHANNELS);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->start() == PX4_OK) {
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
