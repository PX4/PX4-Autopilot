/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include <drivers/drv_hrt.h>
#include <drivers/drv_adc.h>
#include <parameters/param.h>
#include <perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/topics/adc_report.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>

using namespace time_literals;

static constexpr uint32_t SCHEDULE_INTERVAL{10_ms};	/**< The schedule interval in usec (100 Hz) */

class ADC_DifferentialPressure_Wrapper : public ModuleBase<ADC_DifferentialPressure_Wrapper>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:

	ADC_DifferentialPressure_Wrapper();

	~ADC_DifferentialPressure_Wrapper() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

private:

	void Run() override;

	void init();

	perf_counter_t _perf_elapsed{};

	differential_pressure_s _dp_report = {};

	uORB::Subscription	_parameter_update_sub{ORB_ID(parameter_update)};				/**< notification of parameter updates */
	uORB::Subscription	_adc_report_update_sub{ORB_ID(adc_report)};

	uORB::PublicationMulti<differential_pressure_s>	_diff_pres_sub{ORB_ID(differential_pressure)};			/**< raw differential pressure subscription */

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::ADC_DP_CH>) _param_input_channel,
		(ParamInt<px4::params::ADC_DP_MAIN_ID>) _param_input_devid,

		(ParamFloat<px4::params::SENS_DPRES_ANSC>) _param_dp_scale,
		(ParamFloat<px4::params::SENS_DPRES_OFF>) _param_dp_offset
	)

};

ADC_DifferentialPressure_Wrapper::ADC_DifferentialPressure_Wrapper():
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::att_pos_ctrl)
{
	_perf_elapsed = perf_alloc(PC_ELAPSED, MODULE_NAME": elapsed");
}

ADC_DifferentialPressure_Wrapper::~ADC_DifferentialPressure_Wrapper()
{
	ScheduleClear();

	perf_free(_perf_elapsed);
}

int
ADC_DifferentialPressure_Wrapper::task_spawn(int argc, char *argv[])
{
	ADC_DifferentialPressure_Wrapper *dev = new ADC_DifferentialPressure_Wrapper();

	if (!dev) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(dev);

	dev->init();

	dev->ScheduleOnInterval(SCHEDULE_INTERVAL, 10000);
	_task_id = task_id_is_work_queue;
	return PX4_OK;

}

void
ADC_DifferentialPressure_Wrapper::init()
{
	_dp_report.differential_pressure_filtered_pa = 2.5f;
}

void
ADC_DifferentialPressure_Wrapper::Run()
{
	perf_begin(_perf_elapsed);

	parameter_update_s param_update;

	if (_parameter_update_sub.update(&param_update)) {
		updateParams();
	}

	adc_report_s adc_report;

	if (_adc_report_update_sub.update(&adc_report)) {   // only if adc report updated
		int32_t dp_channel = _param_input_channel.get();

		union {
			int32_t i;
			uint32_t u;
		} u2i;
		u2i.i = _param_input_devid.get();

		if (adc_report.device_id == u2i.u) {   // if device_id match
			float diff_pres_pa_raw = adc_report.raw_data[dp_channel]
						 * adc_report.v_ref[dp_channel]
						 / adc_report.resolution[dp_channel];
			diff_pres_pa_raw = diff_pres_pa_raw * _param_dp_scale.get() / _param_dp_offset.get();

			_dp_report.timestamp = adc_report.timestamp;
			_dp_report.differential_pressure_raw_pa = diff_pres_pa_raw;
			_dp_report.differential_pressure_filtered_pa = (_dp_report.differential_pressure_filtered_pa * 0.9f) +
					(diff_pres_pa_raw * 0.1f);
			_dp_report.temperature = -1000.0f;

			_diff_pres_sub.publish(_dp_report);
		}
	}

	perf_end(_perf_elapsed);

	if (should_exit()) {
		exit_and_cleanup();
	}
}

int ADC_DifferentialPressure_Wrapper::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		int ret = ADC_DifferentialPressure_Wrapper::task_spawn(argc, argv);

		if (ret) {
			return ret;
		}
	}

	return print_usage("unknown command");
}

int ADC_DifferentialPressure_Wrapper::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module converts ADC result to differential pressure topic.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("adc_dp_wrapper", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int adc_dp_wrapper_main(int argc, char *argv[])
{
	return ADC_DifferentialPressure_Wrapper::main(argc, argv);
}