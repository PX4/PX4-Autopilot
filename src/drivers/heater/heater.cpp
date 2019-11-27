/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 */

#include "heater.h"

#include <px4_getopt.h>
#include <px4_log.h>
#include <drivers/drv_hrt.h>

#ifndef GPIO_HEATER_INPUT
#error "To use the heater driver, the board_config.h must define and initialize GPIO_HEATER_INPUT and GPIO_HEATER_OUTPUT"
#endif

struct work_s Heater::_work = {};

Heater::Heater() :
	ModuleParams(nullptr)
{
	px4_arch_configgpio(GPIO_HEATER_OUTPUT);
	px4_arch_gpiowrite(GPIO_HEATER_OUTPUT, 0);
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
}

Heater::~Heater()
{
	// Drive the heater GPIO pin low.
	px4_arch_gpiowrite(GPIO_HEATER_OUTPUT, 0);

	// Verify if GPIO is low, and if not, configure it as an input pulldown then reconfigure as an output.
	if (px4_arch_gpioread(GPIO_HEATER_OUTPUT)) {
		px4_arch_configgpio(GPIO_HEATER_INPUT);
		px4_arch_configgpio(GPIO_HEATER_OUTPUT);
		px4_arch_gpiowrite(GPIO_HEATER_OUTPUT, 0);
	}

	// Unsubscribe from uORB topics.
	orb_unsubscribe(_params_sub);

	if (_sensor_accel_sub >= 0) {
		orb_unsubscribe(_sensor_accel_sub);
	}
}

int Heater::controller_period(char *argv[])
{
	if (argv[1]) {
		_controller_period_usec = atoi(argv[1]);
	}

	PX4_INFO("controller period (usec):  %i", _controller_period_usec);
	return _controller_period_usec;
}

int Heater::custom_command(int argc, char *argv[])
{
	// Check if the driver is running.
	if (!is_running()) {
		PX4_INFO("not running");
		return PX4_ERROR;
	}

	const char *arg_v = argv[0];

	// Display/Set the heater controller period value (usec).
	if (strcmp(arg_v, "controller_period") == 0) {
		return get_instance()->controller_period(argv);
	}

	// Display/Set the heater driver integrator gain value.
	if (strcmp(arg_v, "integrator") == 0) {
		return get_instance()->integrator(argv);
	}

	// Display/Set the heater driver proportional gain value.
	if (strcmp(arg_v, "proportional") == 0) {
		return get_instance()->proportional(argv);
	}

	// Display the id of the sensor we are controlling temperature on.
	if (strcmp(arg_v, "sensor_id") == 0) {
		return get_instance()->sensor_id();
	}

	// Displays/Set the current IMU temperature setpoint.
	if (strcmp(arg_v, "setpoint") == 0) {
		return get_instance()->temperature_setpoint(argv);
	}

	get_instance()->print_usage("Unrecognized command.");
	return PX4_OK;
}

void Heater::cycle()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	if (_heater_on) {
		// Turn the heater off.
		px4_arch_gpiowrite(GPIO_HEATER_OUTPUT, 0);
		_heater_on = false;

		// Check if GPIO is stuck on, and if so, configure it as an input pulldown then reconfigure as an output.
		if (px4_arch_gpioread(GPIO_HEATER_OUTPUT)) {
			px4_arch_configgpio(GPIO_HEATER_INPUT);
			px4_arch_configgpio(GPIO_HEATER_OUTPUT);
			px4_arch_gpiowrite(GPIO_HEATER_OUTPUT, 0);
		}

	} else {
		update_params(false);

		orb_update(ORB_ID(sensor_accel), _sensor_accel_sub, &_sensor_accel);

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

		px4_arch_gpiowrite(GPIO_HEATER_OUTPUT, 1);
	}

	// Schedule the next cycle.
	if (_heater_on) {
		work_queue(LPWORK, &_work, (worker_t)&Heater::cycle_trampoline, this,
			   USEC2TICK(_controller_time_on_usec));

	} else {
		work_queue(LPWORK, &_work, (worker_t)&Heater::cycle_trampoline, this,
			   USEC2TICK(_controller_period_usec - _controller_time_on_usec));
	}
}

void Heater::cycle_trampoline(void *argv)
{
	Heater *obj = reinterpret_cast<Heater *>(argv);
	obj->cycle();
}

void Heater::initialize_topics()
{
	// Get the total number of accelerometer instances.
	size_t number_of_imus = orb_group_count(ORB_ID(sensor_accel));

	// Check each instance for the correct ID.
	for (size_t x = 0; x < number_of_imus; x++) {
		_sensor_accel_sub = orb_subscribe_multi(ORB_ID(sensor_accel), (int)x);

		if (_sensor_accel_sub < 0) {
			continue;
		}

		while (orb_update(ORB_ID(sensor_accel), _sensor_accel_sub, &_sensor_accel) != PX4_OK) {
			usleep(200000);
		}

		// If the correct ID is found, exit the for-loop with _sensor_accel_sub pointing to the correct instance.
		if (_sensor_accel.device_id == (uint32_t)_param_sens_temp_id.get()) {
			PX4_INFO("IMU sensor identified.");
			break;
		}

		orb_unsubscribe(_sensor_accel_sub);
	}

	PX4_INFO("Device ID:  %d", _sensor_accel.device_id);

	// Exit the driver if the sensor ID does not match the desired sensor.
	if (_sensor_accel.device_id != (uint32_t)_param_sens_temp_id.get()) {
		request_stop();
		PX4_ERR("Could not identify IMU sensor.");
	}
}

void Heater::initialize_trampoline(void *argv)
{
	Heater *heater = new Heater();

	if (!heater) {
		PX4_ERR("driver allocation failed");
		return;
	}

	_object.store(heater);
	heater->start();
}

float Heater::integrator(char *argv[])
{
	if (argv[1]) {
		_param_sens_imu_temp_i.set(atof(argv[1]));
	}

	PX4_INFO("Integrator gain:  %2.5f", (double)_param_sens_imu_temp_i.get());
	return _param_sens_imu_temp_i.get();
}

int Heater::orb_update(const struct orb_metadata *meta, int handle, void *buffer)
{
	bool newData = false;

	// Check if there is new data to obtain.
	if (orb_check(handle, &newData) != OK) {
		return PX4_ERROR;
	}

	if (!newData) {
		return PX4_ERROR;
	}

	if (orb_copy(meta, handle, buffer) != OK) {
		return PX4_ERROR;
	}

	return PX4_OK;
}

int Heater::print_status()
{
	PX4_INFO("Temperature: %3.3fC - Setpoint: %3.2fC - Heater State: %s",
		 (double)_sensor_temperature,
		 (double)_param_sens_imu_temp.get(),
		 _heater_on ? "On" : "Off");

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
	PRINT_MODULE_USAGE_COMMAND_DESCR("controller_period", "Reports the heater driver cycle period value, (us), and sets it if supplied an argument.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("integrator", "Sets the integrator gain value if supplied an argument and reports the current value.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("proportional", "Sets the proportional gain value if supplied an argument and reports the current value.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("sensor_id", "Reports the current IMU the heater is temperature controlling.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("setpoint", "Reports the current IMU temperature.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Starts the IMU heater driver as a background task");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Reports the current IMU temperature, temperature setpoint, and heater on/off status.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stops the IMU heater driver.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("temp", "Reports the current IMU temperature.");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

float Heater::proportional(char *argv[])
{
	if (argv[1]) {
		_param_sens_imu_temp_p.set(atof(argv[1]));
	}

	PX4_INFO("Proportional gain:  %2.5f", (double)_param_sens_imu_temp_p.get());
	return _param_sens_imu_temp_p.get();
}

uint32_t Heater::sensor_id()
{
	PX4_INFO("Sensor ID:  %d", _sensor_accel.device_id);
	return _sensor_accel.device_id;
}

int Heater::start()
{
	if (is_running()) {
		PX4_INFO("Driver already running.");
		return PX4_ERROR;
	}

	update_params(true);
	initialize_topics();

	// Kick off the cycling. We can call it directly because we're already in the work queue context
	cycle();

	PX4_INFO("Driver started successfully.");

	return PX4_OK;
}

int Heater::task_spawn(int argc, char *argv[])
{
	int ret = work_queue(LPWORK, &_work, (worker_t)&Heater::initialize_trampoline, nullptr, 0);

	if (ret < 0) {
		return ret;
	}

	ret = wait_until_running();

	if (ret < 0) {
		return ret;
	}

	_task_id = task_id_is_work_queue;
	return 0;
}

float Heater::temperature_setpoint(char *argv[])
{
	if (argv[1]) {
		_param_sens_imu_temp.set(atof(argv[1]));
	}

	PX4_INFO("Target temp:  %3.3f", (double)_param_sens_imu_temp.get());
	return _param_sens_imu_temp.get();
}

void Heater::update_params(const bool force)
{
	bool updated;
	parameter_update_s param_update;

	orb_check(_params_sub, &updated);

	if (updated || force) {
		ModuleParams::updateParams();
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);
	}
}


/**
 * Main entry point for the heater driver module
 */
int heater_main(int argc, char *argv[])
{
	return Heater::main(argc, argv);
}
