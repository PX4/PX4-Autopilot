/****************************************************************************
 *
 *   Copyright (c) 2024 Technology Innovation Institute. All rights reserved.
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

#include "tvs.hpp"

#include <drivers/drv_hrt.h>

TempVoltSensor::TempVoltSensor() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	_tvs_pub.advertise();
}

TempVoltSensor::~TempVoltSensor()
{
	_tvs_pub.unadvertise();
}

int TempVoltSensor::task_spawn(int argc, char *argv[])
{
	TempVoltSensor *instance = new TempVoltSensor();

	if (!instance) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	instance->init();

	return PX4_OK;
}

int TempVoltSensor::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int TempVoltSensor::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION("Temperature and Voltage Sensor");
	PRINT_MODULE_USAGE_NAME("tvs", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int TempVoltSensor::print_status()
{
	PX4_INFO("TVS voltage 1.0 V: %.3f mV", double(_volt_1_0));
	PX4_INFO("TVS voltage 1.8 V: %.3f mV", double(_volt_1_8));
	PX4_INFO("TVS voltage 2.5 V: %.3f mV", double(_volt_2_5));
	PX4_INFO("TVS temperature:   %.3f Celcius", double(_temperature));
	return 0;
}

uint32_t TempVoltSensor::get_dev_id()
{
	device::Device::DeviceId id;
	id.devid_s.devtype = DRV_DEVTYPE_UNUSED;
	id.devid_s.bus_type = device::Device::DeviceBusType_UNKNOWN;
	id.devid_s.bus = 0;
	id.devid_s.address = 1;

	return id.devid;
}

void TempVoltSensor::init()
{
	ScheduleNow();
}

void TempVoltSensor::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	publish_report();

	// reschedule timeout
	ScheduleDelayed(MEAS_INTERVAL);
}

void TempVoltSensor::publish_report()
{
	tvs_get_voltage(0, &_volt_1_0);
	tvs_get_voltage(1, &_volt_1_8);
	tvs_get_voltage(2, &_volt_2_5);
	tvs_get_temperature(&_temperature);

	sensor_tvs_s report{};

	report.timestamp = hrt_absolute_time();
	report.device_id = get_dev_id();
	report.temperature = _temperature;
	report.voltage[0] = _volt_1_0;
	report.voltage[1] = _volt_1_8;
	report.voltage[2] = _volt_2_5;

	_tvs_pub.publish(report);
}

extern "C" __EXPORT int tvs_main(int argc, char *argv[])
{
	return TempVoltSensor::main(argc, argv);
}

