/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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

#include "SystemPower.hpp"

SystemPower::SystemPower() :
	ScheduledWorkItem(px4::wq_configurations::hp_default),
	_sample_perf(perf_alloc(PC_ELAPSED, "system_power: sample"))
{
}

SystemPower::~SystemPower()
{
	ScheduleClear();
	perf_free(_sample_perf);
}

int
SystemPower::init()
{
	return start();
}

int
SystemPower::start()
{
	// schedule regular updates
	ScheduleOnInterval(TICKRATE);
	return PX4_OK;
}

int
SystemPower::stop()
{
	ScheduleClear();
	return PX4_OK;
}

void
SystemPower::Run()
{
	perf_begin(_sample_perf);

	adc_report_s adc_report{};

	if (_adc_report_sub.update(&adc_report)) {
		system_power_s system_power {};

		for (unsigned i = 0; i < adc_report_s::MAX_CHANNELS; i++) {

#if defined(ADC_SCALED_V5_SENSE)

			if (adc_report.channel_id[i] == ADC_SCALED_V5_SENSE) {
				// it is 2:1 scaled
				system_power.v5_valid = true;
				system_power.voltage5v_v = adc_report.channel_value[i] * (ADC_V5_V_FULL_SCALE / 4096.0f);
			}

#endif // ADC_SCALED_V5_SENSE

#if defined(ADC_SCALED_V3V3_SENSORS_SENSE)

			if (adc_report.channel_id[i] == ADC_SCALED_V3V3_SENSORS_SENSE) {
				// it is 2:1 scaled
				system_power.v3v3_valid = true;
				system_power.voltage3v3_v = adc_report.channel_value[i] * (ADC_3V3_SCALE * (3.3f / 4096.0f));
			}

#endif // ADC_SCALED_V3V3_SENSORS_SENSE
		}

#if defined (BOARD_ADC_USB_CONNECTED)
		/* Note once the board_config.h provides BOARD_ADC_USB_CONNECTED,
		 * It must provide the true logic GPIO BOARD_ADC_xxxx macros.
		 */

		system_power.usb_connected = BOARD_ADC_USB_CONNECTED;


#if defined(BOARD_ADC_USB_VALID)
		// If provided used the Valid signal from HW
		system_power.usb_valid = BOARD_ADC_USB_VALID;
#else
		// If not provided then use connected
		system_power.usb_valid = system_power.usb_connected;
#endif // BOARD_ADC_USB_VALID


#if !defined(BOARD_NUMBER_DIGITAL_BRICKS)
		// The valid signals (HW dependent) are associated with each brick
		bool valid_chan[BOARD_NUMBER_BRICKS] = BOARD_BRICK_VALID_LIST;

		for (int b = 0; b < BOARD_NUMBER_BRICKS; b++) {
			system_power.brick_valid |=  valid_chan[b] ? 1 << b : 0;
		}

#endif // BOARD_NUMBER_DIGITAL_BRICKS


#ifdef BOARD_ADC_SERVO_VALID
		system_power.servo_valid = BOARD_ADC_SERVO_VALID;
#endif // BOARD_ADC_SERVO_VALID


#ifdef BOARD_ADC_PERIPH_5V_OC
		// OC pins are active low
		system_power.periph_5v_oc = BOARD_ADC_PERIPH_5V_OC;
#endif // BOARD_ADC_PERIPH_5V_OC


#ifdef BOARD_ADC_HIPOWER_5V_OC
		system_power.hipower_5v_oc = BOARD_ADC_HIPOWER_5V_OC;
#endif // BOARD_ADC_HIPOWER_5V_OC


#endif // BOARD_ADC_USB_CONNECTED

		system_power.timestamp = hrt_absolute_time();
		_system_power_pub.publish(system_power);
	}

	perf_end(_sample_perf);
}

int
SystemPower::print_status()
{
	PX4_INFO("Running");

	perf_print_counter(_sample_perf);

	return 0;
}

int
SystemPower::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int
SystemPower::task_spawn(int argc, char *argv[])
{
	SystemPower *instance = new SystemPower();

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
SystemPower::print_usage(const char *reason)
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

	PRINT_MODULE_USAGE_NAME("system_power", "driver");

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int system_power_main(int argc, char *argv[]);

int system_power_main(int argc, char *argv[])
{
	return SystemPower::main(argc, argv);
}
