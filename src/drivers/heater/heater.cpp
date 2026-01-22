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


#if defined(BOARD_USES_PX4IO_VERSION) and defined(PX4IO_HEATER_ENABLED)
// Heater on some boards is on IO MCU
// Use ioctl calls to IO driver to turn heater on/off
// TODO: Multi-instance heater for PX4IO to be implemented
#  define HEATER_PX4IO
#else
// Use direct calls to turn GPIO pin on/off
#  ifndef GPIO_HEATER_OUTPUT
#  error "To use the heater driver, the board_config.h must define and initialize GPIO_HEATER_OUTPUT"
#  endif
#  if !(HEATER_NUM >=1 && HEATER_NUM <=3)
#    error "HEATER_NUM must be defined 1, 2, or 3 in the board_config.h"
#  endif
#  if HEATER_NUM >= 1
#    ifndef GPIO_HEATER1_OUTPUT
#      error "The board_config.h must define every heater's GPIO"
#    endif
#  endif
#  if HEATER_NUM >= 2
#    ifndef GPIO_HEATER2_OUTPUT
#      error "The board_config.h must define every heater's GPIO"
#    endif
#  endif
#  if HEATER_NUM == 3
#    ifndef GPIO_HEATER3_OUTPUT
#      error "The board_config.h must define every heater's GPIO"
#    endif
#  endif
#  define HEATER_GPIO
#endif

Heater *Heater::g_heater[HEATER_MAX_INSTANCES] {};

Heater::Heater(uint8_t instance, const px4::wq_config_t &wq) :
	ScheduledWorkItem(MODULE_NAME, wq),
	ModuleParams(nullptr),
	_instance(instance)
{
	initialize_heater_io();

	char name[32];

	// Dynamically locate the parameter handle corresponding to the current instance
	snprintf(name, sizeof(name), "HEATER%u_IMU_ID", (unsigned)_instance);
	_param_handles.imu_id = param_find(name); //

	snprintf(name, sizeof(name), "HEATER%u_TEMP", (unsigned)_instance);
	_param_handles.temp = param_find(name); //

	snprintf(name, sizeof(name), "HEATER%u_TEMP_P", (unsigned)_instance);
	_param_handles.temp_p = param_find(name); //

	snprintf(name, sizeof(name), "HEATER%u_TEMP_I", (unsigned)_instance);
	_param_handles.temp_i = param_find(name); //

	snprintf(name, sizeof(name), "HEATER%u_TEMP_FF", (unsigned)_instance);
	_param_handles.temp_ff = param_find(name); //

	_heater_status_pub.advertise(); //

	// Initialization parameter values
	update_params(true);
}

Heater::~Heater()
{
	disable_heater();
	ScheduleClear();

}

void Heater::disable_heater()
{
	// Reset heater to off state.
#ifdef HEATER_PX4IO
	// TODO: Multi-instance heater for PX4IO to be implemented
	if (_io_fd >= 0) {
		px4_ioctl(_io_fd, PX4IO_HEATER_CONTROL, HEATER_MODE_DISABLED);
	}

#endif

#ifdef HEATER_GPIO

	switch (_instance) {
	case 1 :
#ifdef GPIO_HEATER1_OUTPUT
		px4_arch_configgpio(GPIO_HEATER1_OUTPUT);
#endif
		break;

	case 2 :
#ifdef GPIO_HEATER2_OUTPUT
		px4_arch_configgpio(GPIO_HEATER2_OUTPUT);
#endif
		break;

	case 3 :
#ifdef GPIO_HEATER3_OUTPUT
		px4_arch_configgpio(GPIO_HEATER3_OUTPUT);
#endif
		break;

	default:
		break;
	}

#endif
}

void Heater::initialize_heater_io()
{
	// Initialize heater to off state.
#ifdef HEATER_PX4IO
	// TODO: Multi-instance heater for PX4IO to be implemented
	if (_io_fd < 0) {
		_io_fd = px4_open(IO_HEATER_DEVICE_PATH, O_RDWR);
	}

	if (_io_fd >= 0) {
		px4_ioctl(_io_fd, PX4IO_HEATER_CONTROL, HEATER_MODE_OFF);
	}

#endif

#ifdef HEATER_GPIO

	switch (_instance) {
	case 1 :
#ifdef GPIO_HEATER1_OUTPUT
		px4_arch_configgpio(GPIO_HEATER1_OUTPUT);
#endif
		break;

	case 2 :
#ifdef GPIO_HEATER2_OUTPUT
		px4_arch_configgpio(GPIO_HEATER2_OUTPUT);
#endif
		break;

	case 3 :
#ifdef GPIO_HEATER3_OUTPUT
		px4_arch_configgpio(GPIO_HEATER3_OUTPUT);
#endif
		break;

	default:
		break;
	}

#endif
}

void Heater::heater_off()
{
#ifdef HEATER_PX4IO

	// TODO: Multi-instance heater for PX4IO to be implemented
	if (_io_fd >= 0) {
		px4_ioctl(_io_fd, PX4IO_HEATER_CONTROL, HEATER_MODE_OFF);
	}

#endif

#ifdef HEATER_GPIO

	switch (_instance) {
	case 1 :
#ifdef GPIO_HEATER1_OUTPUT
		HEATER1_OUTPUT_EN(false);
		// px4_arch_gpiowrite(GPIO_HEATER1_OUTPUT, false);
#endif
		break;

	case 2 :
#ifdef GPIO_HEATER2_OUTPUT
		HEATER2_OUTPUT_EN(false);
		// px4_arch_gpiowrite(GPIO_HEATER2_OUTPUT, false);
#endif
		break;

	case 3 :
#ifdef GPIO_HEATER3_OUTPUT
		HEATER3_OUTPUT_EN(false);
		// px4_arch_gpiowrite(GPIO_HEATER3_OUTPUT, false);
#endif
		break;

	default:
		break;
	}
#endif
}

void Heater::heater_on()
{
#ifdef HEATER_PX4IO

	// TODO: Multi-instance heater for PX4IO to be implemented
	if (_io_fd >= 0) {
		px4_ioctl(_io_fd, PX4IO_HEATER_CONTROL, HEATER_MODE_ON);
	}

#endif

#ifdef HEATER_GPIO

	switch (_instance) {
	case 1 :
#  ifdef GPIO_HEATER1_OUTPUT
		HEATER1_OUTPUT_EN(true);
		// px4_arch_gpiowrite(GPIO_HEATER1_OUTPUT, true);
#  endif
		break;

	case 2 :
#  ifdef GPIO_HEATER2_OUTPUT
		HEATER2_OUTPUT_EN(true);
		// px4_arch_gpiowrite(GPIO_HEATER2_OUTPUT, true);
#  endif
		break;

	case 3 :
#  ifdef GPIO_HEATER3_OUTPUT
		HEATER3_OUTPUT_EN(true);
		// px4_arch_gpiowrite(GPIO_HEATER3_OUTPUT, true);
#  endif
		break;

	default:
		break;
	}
#endif
}

bool Heater::initialize_topics()
{
	// Force a single read of the parameters to ensure _params.imu_id is already up to date.
	update_params(true);

	if (!_heater_initialized) {
		PX4_ERR("heater %u: params not initialized", (unsigned)_instance);
		return false;
	}

	const int32_t target = _params.imu_id;

	// Scan multiple instances of accel, matching device_id or auto-select
	int8_t selected_instance = -1;
	sensor_accel_s accel{};


	for (uint8_t i = 0; i < HEATER_MAX_INSTANCES; i++) {
		uORB::Subscription s{ORB_ID(sensor_accel), i};

		if (!s.advertised()) {
			continue;
		}

		sensor_accel_s a{};

		if (!s.copy(&a)) {
			continue;
		}

		if (target == 0) {
			// Select the first available option
			selected_instance = i;
			accel = a;
			break;
		}

		if ((uint32_t)target == a.device_id) {
			selected_instance = i;
			accel = a;
			break;
		}
	}

	if (selected_instance < 0) {
		if (target == 0) {
			PX4_ERR("heater %u: no sensor_accel instances available", (unsigned)_instance);

		} else {
			PX4_ERR("heater %u: no accel matches device_id=%ld", (unsigned)_instance, (long)target);
		}

		return false;
	}

	_sensor_device_id = accel.device_id;

	// Switch subscription to this instance
	_sensor_accel_sub.ChangeInstance(selected_instance);
	PX4_INFO("heater %u bound accel instance %d (device_id=%lu)",
		 (unsigned)_instance, selected_instance, (unsigned long)_sensor_device_id);

	return true;
}



void Heater::Run()
{
	if (_should_exit) {
#if defined(HEATER_PX4IO)

		// must be closed from wq thread
		if (_io_fd >= 0) {
			px4_close(_io_fd);
		}

#endif
		ScheduleClear();

		// Ensure heating is turned off (avoid leaving it in the ON position).
		heater_off();

		delete Heater::g_heater[_instance - 1];
		Heater::g_heater[_instance - 1] = nullptr;
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

	if (_heater_on) {
		// Turn the heater off.
		_heater_on = false;
		heater_off();
		ScheduleDelayed(_controller_period_usec - _controller_time_on_usec);

	} else if (_sensor_accel_sub.update(&sensor_accel)) {

		// Update the current IMU sensor temperature if valid.
		if (PX4_ISFINITE(sensor_accel.temperature)) {
			temperature_delta = _params.temp - sensor_accel.temperature;
			_temperature_last = sensor_accel.temperature;
		}

		_proportional_value = temperature_delta * _params.temp_p;
		_integrator_value += temperature_delta * _params.temp_i;

		_integrator_value = math::constrain(_integrator_value, -0.25f, 0.25f);

		_controller_time_on_usec = static_cast<int>((_params.temp_ff + _proportional_value +
					   _integrator_value) * static_cast<float>(_controller_period_usec));

		_controller_time_on_usec = math::constrain(_controller_time_on_usec, 0, _controller_period_usec);

		if (fabsf(temperature_delta) < TEMPERATURE_TARGET_THRESHOLD) {
			_temperature_target_met = true;

		} else {

			_temperature_target_met = false;
		}

		_heater_on = true;
		heater_on();
		ScheduleDelayed(_controller_time_on_usec);
	}

	publish_status();
}

void Heater::publish_status()
{
	heater_status_s status{};
	status.device_id               = _sensor_device_id;
	status.heater_on               = _heater_on;
	status.temperature_sensor      = _temperature_last;
	status.temperature_target      = _params.temp;
	status.temperature_target_met  = _temperature_target_met;
	status.controller_period_usec  = _controller_period_usec;
	status.controller_time_on_usec = _controller_time_on_usec;
	status.proportional_value      = _proportional_value;
	status.integrator_value        = _integrator_value;
	status.feed_forward_value      = _params.temp_ff;

#ifdef HEATER_PX4IO
	status.mode = heater_status_s::MODE_PX4IO;
#endif
#ifdef HEATER_GPIO
	status.mode = heater_status_s::MODE_GPIO;
#endif

	status.timestamp = hrt_absolute_time();
	_heater_status_pub.publish(status);
}

int Heater::start()
{


	if (_params.imu_id == 0) {
		// Exit the driver if the sensor ID does not match the desired sensor.
		switch (_instance) {
		case 1:

			PX4_ERR("Valid HEATER1_IMU_ID required");
			break;

		case 2:
			PX4_ERR("Valid HEATER2_IMU_ID required");
			break;

		case 3:
			PX4_ERR("Valid HEATER3_IMU_ID required");
			break;

		default:
			break;
		}

		stop();
		return PX4_ERROR;

	}

	update_params(true);
	ScheduleNow();
	return PX4_OK;
}

void Heater::stop()
{
	_should_exit = true;
	heater_off();
	ScheduleNow();
}

int Heater::status(uint8_t instance ){

	if(instance > 0 && instance < HEATER_MAX_INSTANCES){
		if (Heater::is_running_instance(instance)) {
			PX4_INFO("instance %u: running", (unsigned)instance);
			PX4_INFO("instance %u: IMU ID is %lu",(unsigned)instance , Heater::g_heater[instance - 1]->_params.imu_id);
		}
	}
	else{
		for (instance = 1; instance <= HEATER_MAX_INSTANCES; instance++) {
			if (Heater::is_running_instance(instance)) {
				PX4_INFO("instance %u: running", (unsigned)instance);
				PX4_INFO("instance %u: IMU ID is %lu",(unsigned)instance , Heater::g_heater[instance - 1]->_params.imu_id);
			}
		}
	}
	return PX4_OK;
}

int Heater::stop_all()
{
	for (uint8_t i = 0; i < HEATER_MAX_INSTANCES; i++) {
		if (Heater::g_heater[i]) {
			Heater::g_heater[i]->stop();
		}
	}

	PX4_INFO("All heater stoped");

	return PX4_OK;
}

bool Heater::is_running_instance(uint8_t instance)
{
	if (instance < 1 || instance > HEATER_MAX_INSTANCES) {
		return false;
	}

	return Heater::g_heater[instance - 1] != nullptr;
}

bool Heater::is_running_any()
{
	for (uint8_t i = 0; i < HEATER_MAX_INSTANCES; i++) {
		if (Heater::g_heater[i] != nullptr) { return true; }
	}

	return false;
}

int Heater::start_instance(uint8_t instance)
{
	if (instance < 1 || instance > HEATER_MAX_INSTANCES) {
		PX4_ERR("invalid instance %u", (unsigned)instance);
		return PX4_ERROR;
	}

	if (Heater::is_running_instance(instance)) {
		PX4_WARN("heater %u already running", (unsigned)instance);
		return PX4_OK;
	}

	const px4::wq_config_t &wq = px4::ins_instance_to_wq((int)instance - 1);

	Heater *h = new Heater(instance, wq);

	if (!h) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	int ret = h->start();

	if (ret != PX4_OK) {
		h->stop();
		delete h;
		return ret;
	}

	Heater::g_heater[instance - 1] = h;
	return PX4_OK;
}

void Heater::update_params(const bool force)
{
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		// update parameters from storage
		if (_param_handles.imu_id != PARAM_INVALID) {
			param_get(_param_handles.imu_id, &_params.imu_id);
		}

		if (_param_handles.temp != PARAM_INVALID) {
			param_get(_param_handles.temp, &_params.temp);
		}

		if (_param_handles.temp_p != PARAM_INVALID) {
			param_get(_param_handles.temp_p, &_params.temp_p);
		}

		if (_param_handles.temp_i != PARAM_INVALID) {
			param_get(_param_handles.temp_i, &_params.temp_i);
		}

		if (_param_handles.temp_ff != PARAM_INVALID) {
			param_get(_param_handles.temp_ff, &_params.temp_ff);
		}

		_heater_initialized = true;
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
Background process running periodically on the INS{i} queue to regulate IMU temperature at a setpoint.

This task can be started at boot from the startup scripts by setting SENS_EN_THERMAL or via CLI.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("heater", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}


extern "C" __EXPORT int heater_main(int argc, char *argv[])
{
    	if (argc < 2) {
		PX4_INFO("usage: heater {start|stop|status} [-i N]");
		return PX4_ERROR;
    	}

    	int ch;
	int myoptind = 2;
	const char *myoptarg = nullptr;
	int instance = -1;
	while ((ch = px4_getopt(argc, argv, "i:", &myoptind, &myoptarg)) != EOF) {
	    if (ch == 'i') {
	        instance = (int)strtol(myoptarg, nullptr, 10);
	    }
	}

	if (!strcmp(argv[1], "start")) {
		// Compatibility: Without parameters, only the first one is applied by default.
		if (instance > 0) {
			return Heater::start_instance((uint8_t)instance);
		} else {
			PX4_INFO("Heater numbers start from 1, trying to start all Heaters");
			for(uint8_t i = 0; i < HEATER_NUM; i++){
				Heater::start_instance(i + 1);
			}

			return PX4_OK;
		}
	}

	if (!strcmp(argv[1], "stop")) {

		// Compatibility: Without parameters, only the first one is applied by default.
		if (instance > 0 && instance <= HEATER_MAX_INSTANCES) {
		if (Heater::g_heater[instance - 1]) {
			Heater::g_heater[instance - 1]->stop();
			return PX4_OK;
		}
		return PX4_ERROR;
		} else {
			return Heater::stop_all();
		}


	}

	if (!strcmp(argv[1], "status")) {

		if (!Heater::is_running_any()) {
			PX4_INFO("not running");
			return PX4_OK;
		}

		Heater::status(instance);
		return PX4_OK;
	}

	PX4_INFO("unknown command");
	return PX4_ERROR;
}

