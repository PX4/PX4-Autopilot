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
 * @file sensors.cpp
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Julian Oes <julian@oes.ch>
 * @author Thomas Gubler <thomas@px4.io>
 * @author Anton Babushkin <anton@px4.io>
 * @author Beat Küng <beat-kueng@gmx.net>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/adc_report.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include "analog_battery.h"

using namespace time_literals;

/**
 * The channel definitions (e.g., ADC_BATTERY_VOLTAGE_CHANNEL, ADC_BATTERY_CURRENT_CHANNEL, and ADC_AIRSPEED_VOLTAGE_CHANNEL) are defined in board_config.h
 */

class BatteryStatus : public ModuleBase<BatteryStatus>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	explicit BatteryStatus(int num_bricks);
	~BatteryStatus() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	uORB::Subscription	_actuator_ctrl_0_sub{ORB_ID(actuator_controls_0)};		/**< attitude controls sub */
	uORB::Subscription	_parameter_update_sub{ORB_ID(parameter_update)};				/**< notification of parameter updates */
	uORB::Subscription	_adc_report_sub{ORB_ID(adc_report)};

	AnalogBattery *_batterys[2] = {nullptr};
	int _num_bricks = 0;

	int 			_battery_pub_intance0ndx {0}; /**< track the index of instance 0 */

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	/**
	 * Check for changes in parameters.
	 */
	void 		parameter_update_poll(bool forced = false);

	/**
	 * Poll the ADC and update readings to suit.
	 *
	 * @param raw			Combined sensor data structure into which
	 *				data should be returned.
	 */
	void		adc_poll();
};

BatteryStatus::BatteryStatus(int num_bricks = 1) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),  // TODO: hp_default doesn't get scheduled
	_num_bricks(num_bricks),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME))
{
	for (int i = 0; i < _num_bricks; ++i) {
		_batterys[i] = new AnalogBattery(i + 1, this);
	}
}

BatteryStatus::~BatteryStatus()
{
	ScheduleClear();

	for (int i = 0; i < _num_bricks; ++i) {
		delete _batterys[i];
	}
}

void
BatteryStatus::parameter_update_poll(bool forced)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || forced) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
	}
}

// TODO: bug: if voltage and current come from different adc device then there cannot generate correct report.
void
BatteryStatus::adc_poll()
{
	adc_report_s adc_report = {};

	if (_adc_report_sub.update(&adc_report)) {  // only update if adc report ready.
		for (int i = 0; i < _num_bricks; ++i) { // execute update per brick
			bool voltage_updated = false, current_updated = false;
			float adc_v_v = -1, adc_v_c = -1; // adc raw voltage

			if (adc_report.device_id == _batterys[i]->get_voltage_deviceid()) {
				int aimed_channel = _batterys[i]->get_voltage_channel();

				if (adc_report.channel_id[aimed_channel] !=
				    i) { // adc_report doesn't contain such a valid channel (channel id not match)
					PX4_DEBUG("Invalid v_mon channel from ADC device %.4x", adc_report.device_id);

				} else {
					adc_v_v = adc_report.raw_data[aimed_channel] * adc_report.v_ref[aimed_channel] / adc_report.resolution[aimed_channel];
					voltage_updated = true;
				}
			}

			if (adc_report.device_id == _batterys[i]->get_current_deviceid()) {
				int aimed_channel = _batterys[i]->get_current_channel();

				if (adc_report.channel_id[aimed_channel] !=
				    i) { // adc_report doesn't contain such a valid channel (channel id not match)
					PX4_DEBUG("Invalid c_mon channel from ADC device %.4x", adc_report.device_id);

				} else {
					adc_v_c = adc_report.raw_data[aimed_channel] * adc_report.v_ref[aimed_channel] / adc_report.resolution[aimed_channel];
					current_updated = true;
				}
			}

			if (voltage_updated || current_updated) {
				actuator_controls_s ctrl{};
				_actuator_ctrl_0_sub.copy(&ctrl);

				_batterys[i]->updateBatteryStatusRawADC(
					hrt_absolute_time(),    // maybe adc report timestamp is better?
					adc_v_v,
					adc_v_c,
					true,
					i,
					ctrl.control[actuator_controls_s::INDEX_THROTTLE]
				);
			}
		}
	}
}

void
BatteryStatus::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	/* check parameters for updates */
	parameter_update_poll();

	/* check battery voltage */
	adc_poll();

	perf_end(_loop_perf);
}

int
BatteryStatus::task_spawn(int argc, char *argv[])
{
	int num_of_bricks = 1;

	int ch = -1;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "n:", &myoptind, &myoptarg)) != EOF) {
		PX4_ERR("%d", ch);

		switch (ch) {
		case 'n':
			num_of_bricks = atoi(myoptarg);
			break;

		case '?':
			PX4_WARN("Unsupported args");
			return -EINVAL;

		default:
			break;
		}
	}

	BatteryStatus *instance = new BatteryStatus(num_of_bricks);

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

bool
BatteryStatus::init()
{
	parameter_update_poll(true);

	ScheduleOnInterval(10_ms, 100_ms); // 100 Hz

	return true;
}

int BatteryStatus::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int BatteryStatus::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

The provided functionality includes:
- Read the output from adc_report topic (via uORB interface) and publish `battery_status`.


### Implementation
It runs in its own thread.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("battery_status", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_PARAM_INT('n',1,1,2,"number of bricks",true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int battery_status_main(int argc, char *argv[])
{
	return BatteryStatus::main(argc, argv);
}
