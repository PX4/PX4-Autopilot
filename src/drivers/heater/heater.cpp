/****************************************************************************
 *
 *   Copyright (c) 2018-20 PX4 Development Team. All rights reserved.
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
 * @file heater.cpp
 *
 * @author Mark Sauder <mcsauder@gmail.com>
 * @author Alex Klimaj <alexklimaj@gmail.com>
 * @author Jake Dahl <dahl.jakejacob@gmail.com>
 * @author Mohammed Kabir <mhkabir@mit.edu>
 * @author Jacob Crabill <jacob@flyvoly.com>
 */

#include "heater.h"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_io_heater.h>

#if defined(BOARD_USES_PX4IO) and defined(PX4IO_HEATER_ENABLED)
// Heater on some boards is on IO MCU
// Use ioctl calls to IO driver to turn heater on/off
#  define HEATER_PX4IO
#else
// Use direct calls to turn GPIO pin on/off
#  ifndef GPIO_HEATER_OUTPUT
#  error "To use the heater driver, the board_config.h must define and initialize GPIO_HEATER_OUTPUT"
#  endif
#  define HEATER_GPIO
#endif

Heater::Heater() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
#ifdef HEATER_PX4IO
	_io_fd = px4_open(IO_HEATER_DEVICE_PATH, O_RDWR);

	if (_io_fd < 0) {
		PX4_ERR("Unable to open heater device path");
		return;
	}

#endif

	// Initialize heater to off state
	heater_enable();
}

Heater::~Heater()
{
	// Reset heater to off state
	heater_disable();

#ifdef HEATER_PX4IO
	px4_close(_io_fd);
#endif
}

void Heater::heater_enable()
{
#ifdef HEATER_PX4IO
	px4_ioctl(_io_fd, PX4IO_HEATER_CONTROL, HEATER_MODE_OFF);
#endif
#ifdef HEATER_GPIO
	px4_arch_configgpio(GPIO_HEATER_OUTPUT);
#endif
}

void Heater::heater_disable()
{
#ifdef HEATER_PX4IO
	px4_ioctl(_io_fd, PX4IO_HEATER_CONTROL, HEATER_MODE_DISABLED);
#endif
#ifdef HEATER_GPIO
	px4_arch_configgpio(GPIO_HEATER_OUTPUT);
#endif
}

void Heater::heater_on()
{
#ifdef HEATER_PX4IO
	px4_ioctl(_io_fd, PX4IO_HEATER_CONTROL, HEATER_MODE_ON);
#endif
#ifdef HEATER_GPIO
	px4_arch_gpiowrite(GPIO_HEATER_OUTPUT, 1);
#endif
}

void Heater::heater_off()
{
#ifdef HEATER_PX4IO
	px4_ioctl(_io_fd, PX4IO_HEATER_CONTROL, HEATER_MODE_OFF);
#endif
#ifdef HEATER_GPIO
	px4_arch_gpiowrite(GPIO_HEATER_OUTPUT, 0);
#endif
}

int Heater::custom_command(int argc, char *argv[])
{
	// Check if the driver is running.
	if (!is_running()) {
		PX4_INFO("not running");
		return PX4_ERROR;
	}

	return print_usage("Unrecognized command.");
}

void Heater::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	if (_heater_on) {
		// Turn the heater off.
		heater_off();
		_heater_on = false;

	} else {
		update_params(false);

		_sensor_accel_sub.update(&_sensor_accel);

		// Obtain the current IMU sensor temperature.
		_sensor_temperature = _sensor_accel.temperature;

		// Calculate the temperature delta between the setpoint and reported temperature.
		float temperature_delta = _param_sens_imu_temp.get() - _sensor_temperature;

		// Modulate the heater time on with a feedforward/PI controller.
		_proportional_value = temperature_delta * _param_sens_imu_temp_p.get();
		_integrator_value += temperature_delta * _param_sens_imu_temp_i.get();

		// Constrain the integrator value to no more than 25% of the duty cycle.
		_integrator_value = math::constrain(_integrator_value, -0.25f, 0.25f);

		// Calculate the duty cycle. This is a value between 0 and 1.
		float duty = _proportional_value + _integrator_value;

		_controller_time_on_usec = (int)(duty * (float)_controller_period_usec);

		// Constrain the heater time within the allowable duty cycle.
		_controller_time_on_usec = math::constrain(_controller_time_on_usec, 0, _controller_period_usec);

		// Turn the heater on.
		_heater_on = true;
		heater_on();
	}

	// Schedule the next cycle.
	if (_heater_on) {
		ScheduleDelayed(_controller_time_on_usec);

	} else {
		ScheduleDelayed(_controller_period_usec - _controller_time_on_usec);
	}
}

void Heater::initialize_topics()
{
	// Get the total number of accelerometer instances.
	uint8_t number_of_imus = orb_group_count(ORB_ID(sensor_accel));

	// Check each instance for the correct ID.
	for (uint8_t x = 0; x < number_of_imus; x++) {
		_sensor_accel_sub = uORB::Subscription{ORB_ID(sensor_accel), x};

		if (!_sensor_accel_sub.advertised()) {
			continue;
		}

		_sensor_accel_sub.copy(&_sensor_accel);

		// If the correct ID is found, exit the for-loop with _sensor_accel_sub pointing to the correct instance.
		if (_sensor_accel.device_id == (uint32_t)_param_sens_temp_id.get()) {
			break;
		}
	}

	// Exit the driver if the sensor ID does not match the desired sensor.
	if (_sensor_accel.device_id != (uint32_t)_param_sens_temp_id.get()) {
		request_stop();
		PX4_ERR("Could not identify IMU sensor.");
	}
}

int Heater::print_status()
{
	PX4_INFO("Sensor ID: %d - Temperature: %3.3fC, Setpoint: %3.2fC, Heater State: %s",
		 _sensor_accel.device_id,
		 (double)_sensor_temperature,
		 (double)_param_sens_imu_temp.get(),
		 _heater_on ? "On" : "Off");

	return PX4_OK;
}

int Heater::start()
{
	update_params(true);
	initialize_topics();

	ScheduleNow();

	return PX4_OK;
}

int Heater::task_spawn(int argc, char *argv[])
{
	Heater *heater = new Heater();

	if (!heater) {
		PX4_ERR("driver allocation failed");
		return PX4_ERROR;
	}

	_object.store(heater);
	_task_id = task_id_is_work_queue;

	heater->start();

	return 0;
}

void Heater::update_params(const bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		ModuleParams::updateParams();
	}
}

int Heater::print_usage(const char *reason)
{
	if (reason) {
		printf("%s\n\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Background process running periodically on the LP work queue to regulate IMU temperature at a setpoint.

This task can be started at boot from the startup scripts by setting SENS_EN_THERMAL or via CLI.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("heater", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

/**
 * Main entry point for the heater driver module
 */
int heater_main(int argc, char *argv[])
{
	return Heater::main(argc, argv);
}
