/****************************************************************************
 *
 *   Copyright (C) 2012-2020 PX4 Development Team. All rights reserved.
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

#include <uORB/Subscription.hpp>

#include "SystemPower.hpp"

#if defined(CONFIG_DEV_GPIO)
# include <nuttx/ioexpander/gpio.h>
#endif // CONFIG_DEV_GPIO

SystemPower::SystemPower() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

SystemPower::~SystemPower()
{
	DataReadyInterruptDisable();
	ScheduleClear();

	perf_free(_cycle_perf);

	CloseGpioDevices();
}

int SystemPower::init()
{
	DataReadyInterruptConfigure();
	ScheduleDelayed(1_s);
	return PX4_OK;
}

void SystemPower::OpenGpioDevices()
{
#if defined(BOARD_GPIO_VDD_5V_COMP_VALID)
	_5v_comp_valid_fd = open(BOARD_GPIO_VDD_5V_COMP_VALID, O_RDONLY);
#endif // BOARD_GPIO_VDD_5V_COMP_VALID

#if defined(BOARD_GPIO_VDD_5V_CAN1_GPS1_VALID)
	_5v_can1_gps1_valid_fd = open(BOARD_GPIO_VDD_5V_CAN1_GPS1_VALID, O_RDONLY);
#endif // BOARD_GPIO_VDD_5V_CAN1_GPS1_VALID
}

void SystemPower::CloseGpioDevices()
{
#if defined(BOARD_GPIO_VDD_5V_COMP_VALID)
	close(_5v_comp_valid_fd);
#endif // BOARD_GPIO_VDD_5V_COMP_VALID

#if defined(BOARD_GPIO_VDD_5V_CAN1_GPS1_VALID)
	close(_5v_can1_gps1_valid_fd);
#endif // BOARD_GPIO_VDD_5V_CAN1_GPS1_VALID
}

uint8_t SystemPower::ReadGpioValue(int fd)
{
#if defined(CONFIG_DEV_GPIO)

	if (fd == -1) {
		return 0xff;
	}

	bool value;

	if (ioctl(fd, GPIOC_READ, (long)&value) != 0) {
		return 0xff;
	}

	return value;
#else
	return 0xff;
#endif // CONFIG_DEV_GPIO
}

int SystemPower::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<SystemPower *>(arg)->DataReady();
	return 0;
}

void SystemPower::DataReady()
{
	ScheduleNow();
}

bool SystemPower::DataReadyInterruptConfigure()
{
	// Setup data ready on rising and falling edge
#if defined(GPIO_OTGFS_VBUS)
	px4_arch_gpiosetevent(GPIO_OTGFS_VBUS, true, true, true, &DataReadyInterruptCallback, this);
#endif // GPIO_OTGFS_VBUS

#if defined(GPIO_nVDD_USB_VALID)
	px4_arch_gpiosetevent(GPIO_nVDD_USB_VALID, true, true, true, &DataReadyInterruptCallback, this);
#endif // GPIO_nVDD_USB_VALID

#if defined(GPIO_VDD_USB_VALID)
	px4_arch_gpiosetevent(GPIO_VDD_USB_VALID, true, true, true, &DataReadyInterruptCallback, this);
#endif // GPIO_VDD_USB_VALID

#if defined(GPIO_VDD_SERVO_VALID)
	px4_arch_gpiosetevent(GPIO_VDD_SERVO_VALID, true, true, true, &DataReadyInterruptCallback, this);
#endif // GPIO_VDD_SERVO_VALID

#if defined(GPIO_nVDD_5V_PERIPH_OC)
	px4_arch_gpiosetevent(GPIO_nVDD_5V_PERIPH_OC, true, true, true, &DataReadyInterruptCallback, this);
#endif // GPIO_nVDD_5V_PERIPH_OC

#if defined(GPIO_VDD_5V_HIPOWER_OC)
	px4_arch_gpiosetevent(GPIO_VDD_5V_HIPOWER_OC, true, true, true, &DataReadyInterruptCallback, this);
#endif // GPIO_VDD_5V_HIPOWER_OC

#if defined(GPIO_nVDD_BRICK1_VALID)
	px4_arch_gpiosetevent(GPIO_nVDD_BRICK1_VALID, true, true, true, &DataReadyInterruptCallback, this);
#endif // GPIO_nVDD_BRICK1_VALID

#if defined(GPIO_nVDD_BRICK2_VALID)
	px4_arch_gpiosetevent(GPIO_nVDD_BRICK2_VALID, true, true, true, &DataReadyInterruptCallback, this);
#endif // GPIO_nVDD_BRICK2_VALID

#if defined(GPIO_nVDD_BRICK3_VALID)
	px4_arch_gpiosetevent(GPIO_nVDD_BRICK3_VALID, true, true, true, &DataReadyInterruptCallback, this);
#endif // GPIO_nVDD_BRICK3_VALID

#if defined(GPIO_nVDD_BRICK4_VALID)
	px4_arch_gpiosetevent(GPIO_nVDD_BRICK4_VALID, true, true, true, &DataReadyInterruptCallback, this);
#endif // GPIO_nVDD_BRICK4_VALID

	return true;
}

bool SystemPower::DataReadyInterruptDisable()
{
#if defined(GPIO_OTGFS_VBUS)
	px4_arch_gpiosetevent(GPIO_OTGFS_VBUS, false, false, false, nullptr, nullptr);
#endif // GPIO_OTGFS_VBUS

#if defined(GPIO_nVDD_USB_VALID)
	px4_arch_gpiosetevent(GPIO_nVDD_USB_VALID, false, false, false, nullptr, nullptr);
#endif // GPIO_nVDD_USB_VALID

#if defined(GPIO_VDD_USB_VALID)
	px4_arch_gpiosetevent(GPIO_VDD_USB_VALID, false, false, false, nullptr, nullptr);
#endif // GPIO_VDD_USB_VALID

#if defined(GPIO_VDD_SERVO_VALID)
	px4_arch_gpiosetevent(GPIO_VDD_SERVO_VALID, false, false, false, nullptr, nullptr);
#endif // GPIO_VDD_SERVO_VALID

#if defined(GPIO_nVDD_5V_PERIPH_OC)
	px4_arch_gpiosetevent(GPIO_nVDD_5V_PERIPH_OC, false, false, false, nullptr, nullptr);
#endif // GPIO_nVDD_5V_PERIPH_OC

#if defined(GPIO_VDD_5V_HIPOWER_OC)
	px4_arch_gpiosetevent(GPIO_VDD_5V_HIPOWER_OC, false, false, false, nullptr, nullptr);
#endif // GPIO_VDD_5V_HIPOWER_OC

#if defined(GPIO_nVDD_BRICK1_VALID)
	px4_arch_gpiosetevent(GPIO_nVDD_BRICK1_VALID, false, false, false, nullptr, nullptr);
#endif // GPIO_nVDD_BRICK1_VALID

#if defined(GPIO_nVDD_BRICK2_VALID)
	px4_arch_gpiosetevent(GPIO_nVDD_BRICK2_VALID, false, false, false, nullptr, nullptr);
#endif // GPIO_nVDD_BRICK2_VALID

#if defined(GPIO_nVDD_BRICK3_VALID)
	px4_arch_gpiosetevent(GPIO_nVDD_BRICK3_VALID, false, false, false, nullptr, nullptr);
#endif // GPIO_nVDD_BRICK3_VALID

#if defined(GPIO_nVDD_BRICK4_VALID)
	px4_arch_gpiosetevent(GPIO_nVDD_BRICK4_VALID, false, false, false, nullptr, nullptr);
#endif // GPIO_nVDD_BRICK4_VALID

	return true;
}

void SystemPower::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	if (_first_run) {
		OpenGpioDevices();
		_first_run = false;
	}

	perf_begin(_cycle_perf);
	ScheduleDelayed(500_ms);

	bool updated = false;

	system_power_s system_power{};

	for (auto &v : system_power.sensors3v3) {
		v = NAN;
	}

	for (auto &v : system_power.brick_voltage_adc) {
		v = NAN;
	}

	for (auto &v : system_power.brick_current_adc) {
		v = NAN;
	}

#if defined(ADC_SCALED_V5_SENSE) || defined(ADC_SCALED_V3V3_SENSORS_SENSE) || defined(BOARD_NUMBER_BRICKS)
	adc_report_s adc_report;

	if (_adc_report_sub.copy(&adc_report)) {
		if (hrt_elapsed_time(&adc_report.timestamp) < 1_s) {
			for (unsigned adc_index = 0; adc_index < (sizeof(adc_report.channel_id) / sizeof(adc_report.channel_id[0]));
			     adc_index++) {
#  if defined(ADC_SCALED_V5_SENSE)

				if (adc_report.channel_id[adc_index] == ADC_SCALED_V5_SENSE) {
					system_power.voltage5v_v = adc_report.raw_data[adc_index] * (ADC_V5_V_FULL_SCALE / adc_report.resolution);
					system_power.voltage5v_available = true;
					updated = true;
				}

#  endif // ADC_SCALED_V5_SENSE

#  if defined(ADC_SCALED_V3V3_SENSORS_SENSE)
				const int sensors_channels[ADC_SCALED_V3V3_SENSORS_COUNT] = ADC_SCALED_V3V3_SENSORS_SENSE;
				static_assert(sizeof(system_power.sensors3v3) / sizeof(system_power.sensors3v3[0]) >= ADC_SCALED_V3V3_SENSORS_COUNT,
					      "array too small");

				for (int sensor_index = 0; sensor_index < ADC_SCALED_V3V3_SENSORS_COUNT; sensor_index++) {
					if (adc_report.channel_id[adc_index] == sensors_channels[sensor_index]) {
						// it is 2:1 scaled
						system_power.sensors3v3[sensor_index] = adc_report.raw_data[adc_index] * (ADC_3V3_SCALE *
											(3.3f / adc_report.resolution));
						system_power.sensors3v3_available[sensor_index] = true;
						updated = true;
					}
				}

#  endif // ADC_SCALED_V3V3_SENSORS_SENSE


# if defined(BOARD_NUMBER_BRICKS)
				const float scale = adc_report.v_ref / adc_report.resolution;

				for (int brick_index = 0; brick_index < BOARD_NUMBER_BRICKS; brick_index++) {
					// brick voltage ADC reading
					static constexpr int batt_v_list[BOARD_NUMBER_BRICKS] = BOARD_BATT_V_LIST;

					if (adc_report.channel_id[adc_index] == batt_v_list[brick_index]) {
						system_power.brick_voltage_adc[brick_index] = adc_report.raw_data[adc_index] * scale;
					}

					// brick current ADC reading
					static constexpr int batt_i_list[BOARD_NUMBER_BRICKS] = BOARD_BATT_I_LIST;

					if (adc_report.channel_id[adc_index] == batt_i_list[brick_index]) {
						system_power.brick_current_adc[brick_index] = adc_report.raw_data[adc_index] * scale;
					}
				}

#endif // BOARD_NUMBER_BRICKS
			}
		}
	}

#endif // ADC_SCALED_V5_SENSE || ADC_SCALED_V3V3_SENSORS_SENSE || BOARD_NUMBER_BRICKS


#if defined(BOARD_ADC_USB_CONNECTED)
	system_power.usb_connected = BOARD_ADC_USB_CONNECTED;
	updated = true;
#endif // BOARD_ADC_USB_CONNECTED

#if defined(BOARD_ADC_USB_VALID)
	// If provided used the Valid signal from HW
	system_power.usb_power_valid = BOARD_ADC_USB_VALID;
	updated = true;
#else
	// If not provided then use connected
	system_power.usb_power_valid = system_power.usb_connected;
#endif

#if defined(BOARD_BRICK_VALID_LIST)
	system_power.bricks_supported = BOARD_NUMBER_BRICKS;

	// The valid signals (HW dependent) are associated with each brick
	bool valid_chan[BOARD_NUMBER_BRICKS] = BOARD_BRICK_VALID_LIST;

	for (int b = 0; b < BOARD_NUMBER_BRICKS; b++) {
		system_power.brick_valid[b] = valid_chan[b];
		updated = true;
	}

#endif // BOARD_BRICK_VALID_LIST

#if defined(BOARD_ADC_SERVO_VALID)
	system_power.servo_power_available = true;
	system_power.servo_power_valid = BOARD_ADC_SERVO_VALID;
	updated = true;
#endif // BOARD_ADC_SERVO_VALID

#if defined(BOARD_ADC_PERIPH_5V_OC)
	system_power.peripheral_5v_available = true;
	system_power.peripheral_5v_overcurrent = BOARD_ADC_PERIPH_5V_OC;
	updated = true;
#endif // BOARD_ADC_PERIPH_5V_OC

#if defined(BOARD_ADC_HIPOWER_5V_OC)
	system_power.hipower_5v_available = true;
	system_power.hipower_5v_overcurrent = BOARD_ADC_HIPOWER_5V_OC;
	updated = true;
#endif // BOARD_ADC_HIPOWER_5V_OC

#if defined(BOARD_GPIO_VDD_5V_COMP_VALID)
	system_power.comp_5v_availabled = true;
	system_power.comp_5v_valid = ReadGpioValue(_5v_comp_valid_fd);
	updated = true;
#endif // BOARD_GPIO_VDD_5V_COMP_VALID

#if defined(BOARD_GPIO_VDD_5V_CAN1_GPS1_VALID)
	system_power.can1_gps1_5v_available = true;
	system_power.can1_gps1_5v_valid = ReadGpioValue(_5v_can1_gps1_valid_fd);
	updated = true;
#endif // BOARD_GPIO_VDD_5V_CAN1_GPS1_VALID

	if (updated) {
		system_power.timestamp = hrt_absolute_time();
		_system_power_pub.publish(system_power);
	}

	perf_end(_cycle_perf);
}

int SystemPower::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int SystemPower::task_spawn(int argc, char *argv[])
{
	SystemPower *instance = new SystemPower();

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

int SystemPower::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
ADC driver.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("system_power", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int system_power_main(int argc, char *argv[])
{
	return SystemPower::main(argc, argv);
}
