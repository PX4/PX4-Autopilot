/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * @file core_heater.cpp
 *
 */

#include "core_heater.h"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_io_heater.h>

#  ifndef GPIO_CORE_HEATER_OUTPUT
#  error "To use the heater driver, the board_config.h must define and initialize GPIO_CORE_HEATER_OUTPUT"
#  endif

Core_Heater::Core_Heater() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	_heater_status_pub.advertise();
}

Core_Heater::~Core_Heater()
{
	disable_core_heater();
}

int Core_Heater::custom_command(int argc, char *argv[])
{
	// Check if the driver is running.
	if (!is_running()) {
		PX4_INFO("not running");
		return PX4_ERROR;
	}

	return print_usage("Unrecognized command.");
}

void Core_Heater::disable_core_heater()
{
	// Reset heater to off state.
	px4_arch_unconfiggpio(GPIO_CORE_HEATER_OUTPUT);
}

void Core_Heater::initialize_core_heater_io()
{
	// Initialize heater to off state.
	px4_arch_configgpio(GPIO_CORE_HEATER_OUTPUT);
}

void Core_Heater::core_heater_off()
{
	CORE_HEATER_OUTPUT_EN(false);
}

void Core_Heater::core_heater_on()
{
	CORE_HEATER_OUTPUT_EN(true);
}

bool Core_Heater::initialize_topics()
{
	for (uint8_t i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
		uORB::SubscriptionData<sensor_accel_s> sensor_accel_sub{ORB_ID(sensor_accel), i};

		if (sensor_accel_sub.get().timestamp != 0 &&
		    sensor_accel_sub.get().device_id != 0 &&
		    PX4_ISFINITE(sensor_accel_sub.get().temperature)) {

			// If the correct ID is found, exit the for-loop with _sensor_accel_sub pointing to the correct instance.
			if (sensor_accel_sub.get().device_id == (uint32_t)_param_core_temp_id.get()) {
				_sensor_accel_sub.ChangeInstance(i);
				_sensor_device_id = sensor_accel_sub.get().device_id;
				initialize_core_heater_io();
				return true;
			}
		}
	}

	return false;
}

void Core_Heater::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	update_params();

	if (_sensor_device_id == 0) {
		if (!initialize_topics()) {
			// if sensor still not found try again in 1 second
			ScheduleDelayed(1_s);
			return;
		}
	}

	sensor_accel_s sensor_accel;
	float temperature_delta {0.f};

	if (_core_heater_on) {
		// Turn the heater off.
		_core_heater_on = false;
		core_heater_off();
		ScheduleDelayed(_controller_period_usec - _controller_time_on_usec);

	} else if (_sensor_accel_sub.update(&sensor_accel)) {
		// Update the current IMU sensor temperature if valid.
		if (PX4_ISFINITE(sensor_accel.temperature)) {
			temperature_delta = _param_core_imu_temp.get() - sensor_accel.temperature;
			_temperature_last = sensor_accel.temperature;
		}

		_proportional_value = temperature_delta * _param_core_imu_temp_p.get();
		_integrator_value += temperature_delta * _param_core_imu_temp_i.get();

		_integrator_value = math::constrain(_integrator_value, -0.25f, 0.25f);

		_controller_time_on_usec = static_cast<int>((_param_core_imu_temp_ff.get() + _proportional_value +
					   _integrator_value) * static_cast<float>(_controller_period_usec));

		_controller_time_on_usec = math::constrain(_controller_time_on_usec, 0, _controller_period_usec);

		if (fabsf(temperature_delta) < TEMPERATURE_TARGET_THRESHOLD) {
			_temperature_target_met = true;

		} else {

			_temperature_target_met = false;
		}

		_core_heater_on = true;
		core_heater_on();
		ScheduleDelayed(_controller_time_on_usec);
	}

	publish_status();
}

void Core_Heater::publish_status()
{
	heater_status_s status{};
	status.device_id               = _sensor_device_id;
	status.heater_on               = _core_heater_on;
	status.temperature_sensor      = _temperature_last;
	status.temperature_target      = _param_core_imu_temp.get();
	status.temperature_target_met  = _temperature_target_met;
	status.controller_period_usec  = _controller_period_usec;
	status.controller_time_on_usec = _controller_time_on_usec;
	status.proportional_value      = _proportional_value;
	status.integrator_value        = _integrator_value;
	status.feed_forward_value      = _param_core_imu_temp_ff.get();

	status.mode = heater_status_s::MODE_GPIO;

	status.timestamp = hrt_absolute_time();
	_heater_status_pub.publish(status);
}

int Core_Heater::start()
{
	// Exit the driver if the sensor ID does not match the desired sensor.
	if (_param_core_temp_id.get() == 0) {
		PX4_ERR("Valid CORE_TEMP_ID required");
		request_stop();
		return PX4_ERROR;
	}

	update_params(true);
	ScheduleNow();
	return PX4_OK;
}

int Core_Heater::task_spawn(int argc, char *argv[])
{
	Core_Heater *core_heater = new Core_Heater();

	if (!core_heater) {
		PX4_ERR("driver allocation failed");
		return PX4_ERROR;
	}

	_object.store(core_heater);
	_task_id = task_id_is_work_queue;

	core_heater->start();
	return 0;
}

void Core_Heater::update_params(const bool force)
{
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		// update parameters from storage
		ModuleParams::updateParams();
	}
}

int Core_Heater::print_usage(const char *reason)
{
	if (reason) {
		printf("%s\n\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Background process running periodically on the LP work queue to regulate IMU temperature at a setpoint.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("core_heater", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int core_heater_main(int argc, char *argv[])
{
	return Core_Heater::main(argc, argv);
}
