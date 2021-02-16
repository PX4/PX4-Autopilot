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

	heater_enable();
}

Heater::~Heater()
{
	heater_disable();

#ifdef HEATER_PX4IO
	px4_close(_io_fd);
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

uint32_t Heater::get_sensor_id()
{
	return _sensor_accel.device_id;
}

void Heater::heater_disable()
{
	// Reset heater to off state.
#ifdef HEATER_PX4IO
	px4_ioctl(_io_fd, PX4IO_HEATER_CONTROL, HEATER_MODE_DISABLED);
#endif
#ifdef HEATER_GPIO
	px4_arch_configgpio(GPIO_HEATER_OUTPUT);
#endif
}

void Heater::heater_enable()
{
	// Initialize heater to off state.
#ifdef HEATER_PX4IO
	px4_ioctl(_io_fd, PX4IO_HEATER_CONTROL, HEATER_MODE_OFF);
#endif
#ifdef HEATER_GPIO
	px4_arch_configgpio(GPIO_HEATER_OUTPUT);
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

void Heater::heater_on()
{
#ifdef HEATER_PX4IO
	px4_ioctl(_io_fd, PX4IO_HEATER_CONTROL, HEATER_MODE_ON);
#endif
#ifdef HEATER_GPIO
	px4_arch_gpiowrite(GPIO_HEATER_OUTPUT, 1);
#endif
}

void Heater::initialize_topics()
{
	// Get the total number of accelerometer instances.
	uint8_t number_of_imus = orb_group_count(ORB_ID(sensor_accel));

	// Get the total number of accelerometer instances and check each instance for the correct ID.
	for (uint8_t x = 0; x < number_of_imus; x++) {
		_sensor_accel.device_id = 0;

		while (_sensor_accel.device_id == 0) {
			_sensor_accel_sub = uORB::Subscription{ORB_ID(sensor_accel), x};

			if (!_sensor_accel_sub.advertised()) {
				px4_usleep(100);
				continue;
			}

			_sensor_accel_sub.copy(&_sensor_accel);
		}

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
	float _feedforward_value = _param_sens_imu_temp_ff.get();

	PX4_INFO("Sensor ID: %d,\tSetpoint: %3.2fC,\t Sensor Temperature: %3.2fC,\tDuty Cycle (usec): %d",
		 _sensor_accel.device_id,
		 static_cast<double>(_param_sens_imu_temp.get()),
		 static_cast<double>(_sensor_accel.temperature),
		 _controller_period_usec);
	PX4_INFO("Feed Forward control effort: %3.2f%%,\tProportional control effort: %3.2f%%,\tIntegrator control effort: %3.3f%%,\t Heater cycle: %3.2f%%",
		 static_cast<double>(_feedforward_value * 100),
		 static_cast<double>(_proportional_value * 100),
		 static_cast<double>(_integrator_value * 100),
		 static_cast<double>(static_cast<float>(_controller_time_on_usec) / static_cast<float>(_controller_period_usec) * 100));

	return PX4_OK;
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

		float temperature_delta {0.f};

		// Update the current IMU sensor temperature if valid.
		if (!isnan(_sensor_accel.temperature)) {
			temperature_delta = _param_sens_imu_temp.get() - _sensor_accel.temperature;
		}

		_proportional_value = temperature_delta * _param_sens_imu_temp_p.get();
		_integrator_value += temperature_delta * _param_sens_imu_temp_i.get();

		if (fabs(_param_sens_imu_temp_i.get()) <= 0.0001) {
			_integrator_value = 0.f;
		}

		// Guard against integrator wind up.
		_integrator_value = math::constrain(_integrator_value, -0.25f, 0.25f);

		_controller_time_on_usec = static_cast<int>((_param_sens_imu_temp_ff.get() + _proportional_value +
						             _integrator_value) * static_cast<float>(_controller_period_usec));

		_controller_time_on_usec = math::constrain(_controller_time_on_usec, 0, _controller_period_usec);

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

int Heater::start()
{
	update_params(true);
	initialize_topics();

	// Allow sufficient time for all additional sensors and processes to start.
	ScheduleDelayed(100000);
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

/**
 * Main entry point for the heater driver module
 */
int heater_main(int argc, char *argv[])
{
	return Heater::main(argc, argv);
}
