/****************************************************************************
 *
 *   Copyright (c) 2018-2026 PX4 Development Team. All rights reserved.
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

Heater *Heater::g_heater[HEATER_MAX_INSTANCES] {}; //! 0-based

template<typename T>
static int8_t find_sensor_instance(orb_id_t id, int32_t target, uint32_t &out_device_id)
{
	for (uint8_t i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
		uORB::Subscription s{id, i};

		if (!s.advertised()) { continue; }

		T msg{};

		if (!s.copy(&msg)) { continue; }

		if (target == 0 || (uint32_t)target == msg.device_id) {
			out_device_id = msg.device_id;
			return i;
		}
	}

	return -1;
}

Heater::Heater(uint8_t instance) :
	ScheduledWorkItem(heater_instance_name(instance), px4::wq_configurations::lp_default),
	ModuleParams(nullptr),
	_instance(instance)
{
	initialize_heater_io();

	char name[32];

	// Dynamically locate the parameter handle corresponding to the current instance
	snprintf(name, sizeof(name), "HEATER%u_SENS_ID", (unsigned)_instance);
	_param_handles.sens_id = param_find(name);

	snprintf(name, sizeof(name), "HEATER%u_TEMP", (unsigned)_instance);
	_param_handles.temp = param_find(name); //

	snprintf(name, sizeof(name), "HEATER%u_TEMP_P", (unsigned)_instance);
	_param_handles.temp_p = param_find(name); //

	snprintf(name, sizeof(name), "HEATER%u_TEMP_I", (unsigned)_instance);
	_param_handles.temp_i = param_find(name); //

	snprintf(name, sizeof(name), "HEATER%u_TEMP_FF", (unsigned)_instance);
	_param_handles.temp_ff = param_find(name); //

	snprintf(name, sizeof(name), "HEATER%u_IMAX", (unsigned)_instance);
	_param_handles.temp_imax = param_find(name);

	snprintf(name, sizeof(name), "HEATER%u_TEMP_SRC", (unsigned)_instance);
	_param_handles.temp_src = param_find(name);

	snprintf(name, sizeof(name), "HEATER%u_TEMP_ACT", (unsigned)_instance);
	_param_handles.temp_activation_threshold = param_find(name);

	snprintf(name, sizeof(name), "HEATER%u_NOM_V", (unsigned)_instance);
	_param_handles.nom_v = param_find(name);

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
	// Force a single read of the parameters to ensure _params.sens_id is up to date.
	update_params(true);

	if (!_heater_initialized) {
		PX4_ERR("heater %u: params not initialized", (unsigned)_instance);
		return false;
	}

	const int32_t target = _params.sens_id;
	const bool use_hygro = (_params.temp_src == heater_status_s::TEMPERATURE_SOURCE_HYGRO);

	int8_t selected_instance = -1;
	uint32_t selected_device_id = 0;

	if (use_hygro) {
		selected_instance = find_sensor_instance<sensor_hygrometer_s>(ORB_ID(sensor_hygrometer), target, selected_device_id);

	} else {
		selected_instance = find_sensor_instance<sensor_accel_s>(ORB_ID(sensor_accel), target, selected_device_id);
	}

	if (selected_instance < 0) {
		if (target == 0) {
			PX4_ERR("heater %u: no %s instances available", (unsigned)_instance,
				use_hygro ? "sensor_hygrometer" : "sensor_accel");

		} else {
			PX4_ERR("heater %u: no %s matches device_id=%ld", (unsigned)_instance,
				use_hygro ? "hygrometer" : "accel", (long)target);
		}

		return false;
	}

	_sensor_device_id = selected_device_id;

	if (use_hygro) {
		_sensor_hygrometer_sub.ChangeInstance(selected_instance);

	} else {
		_sensor_accel_sub.ChangeInstance(selected_instance);
	}

	PX4_INFO("heater %u bound %s instance %d (device_id=%lu)",
		 (unsigned)_instance, use_hygro ? "hygrometer" : "accel",
		 selected_instance, (unsigned long)_sensor_device_id);

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

	float temperature_delta {0.f};
	bool temperature_updated = false;
	uint32_t schedule_delay = CONTROLLER_PERIOD_DEFAULT;

	// Update input voltage/current monitoring
	battery_status_s battery_status;

	if (_battery_status_sub.update(&battery_status)) {
		_supply_voltage = battery_status.voltage_v;
		_heater_current = battery_status.current_a;
		_battery_status_last_update_time = hrt_absolute_time();
	}

	if (_heater_on) {
		// Turn the heater off.
		_heater_on = false;
		heater_off();
		schedule_delay = CONTROLLER_PERIOD_DEFAULT - _controller_time_on_usec;

	} else {
		float current_temp = NAN;
		const bool use_hygro = (_params.temp_src == heater_status_s::TEMPERATURE_SOURCE_HYGRO);

		if (use_hygro) {
			sensor_hygrometer_s sensor_hygrometer;

			if (_sensor_hygrometer_sub.update(&sensor_hygrometer)) {
				current_temp = sensor_hygrometer.temperature;
			}

		} else {
			sensor_accel_s sensor_accel;

			if (_sensor_accel_sub.update(&sensor_accel)) {
				current_temp = sensor_accel.temperature;
			}
		}

		if (PX4_ISFINITE(current_temp)) {
			temperature_delta = _params.temp - current_temp;
			_temperature_last = current_temp;
			temperature_updated = true;
#ifdef CONFIG_HEATER_FAST_UPDATE_MODE
			_temperature_last_update_time = hrt_absolute_time();
#endif
		}

		// Latch heating on once the temperature threshold is crossed; stays on until reset.
		if (_temperature_activation_threshold_met
		    || (PX4_ISFINITE(current_temp) && current_temp < _params.temp_activation_threshold)) {

			_temperature_activation_threshold_met = true;

#ifdef CONFIG_HEATER_FAST_UPDATE_MODE

			// No fresh sensor update: use cached temperature if still within a certain window.
			if (!temperature_updated
			    && _temperature_last_update_time != 0
			    && hrt_elapsed_time(&_temperature_last_update_time) < 500_ms) {
				temperature_delta = _params.temp - _temperature_last;
				temperature_updated = true;
			}

#endif

			if (temperature_updated) {
				_proportional_value = temperature_delta * _params.temp_p;

				_integrator_value += temperature_delta * _params.temp_i;

				_integrator_value = math::constrain(_integrator_value, -_params.temp_imax, _params.temp_imax);

				_controller_time_on_usec = static_cast<int>((_params.temp_ff + _proportional_value +
							   _integrator_value) * static_cast<float>(CONTROLLER_PERIOD_DEFAULT));

				_controller_time_on_usec = math::constrain(_controller_time_on_usec, 0, CONTROLLER_PERIOD_DEFAULT);

				const float nom_v = _params.nom_v;

				if (nom_v > 0.f) {
					if (_battery_status_last_update_time == 0 || hrt_elapsed_time(&_battery_status_last_update_time) > 500_ms) {
						_controller_time_on_usec = 0;

					} else if (PX4_ISFINITE(_supply_voltage) && _supply_voltage > nom_v) {
						// Scale duty cycle by (V_nom/V)^2 so delivered power is the same regardless of supply voltage.
						const float ratio = nom_v / _supply_voltage;
						_nominal_multiplier = ratio * ratio;
						_controller_time_on_usec = static_cast<int>(_controller_time_on_usec * _nominal_multiplier);
					}
				}

				_temperature_target_met = fabsf(temperature_delta) < TEMPERATURE_TARGET_THRESHOLD;

				if (_controller_time_on_usec > 0) {
					_heater_on = true;
					heater_on();
					schedule_delay = _controller_time_on_usec;
				}
			}

		}
	}

	ScheduleDelayed(schedule_delay);
	publish_status();
}

void Heater::publish_status()
{
	heater_status_s status{};
	status.device_id                 	    = _sensor_device_id;
	status.heater_on                 	    = _heater_on;
	status.temperature_sensor        	    = _temperature_last;
	status.temperature_target        	    = _params.temp;
	status.temperature_target_met    	    = _temperature_target_met;
	status.controller_period_usec    	    = CONTROLLER_PERIOD_DEFAULT;
	status.controller_time_on_usec   	    = _controller_time_on_usec;
	status.proportional_value        	    = _proportional_value;
	status.integrator_value          	    = _integrator_value;
	status.feed_forward_value        	    = _params.temp_ff;
	status.supply_voltage            	    = _supply_voltage;
	status.heater_current            	    = _heater_current;
	status.nominal_multiplier        	    = _nominal_multiplier;
	status.temperature_activation_threshold_met = _temperature_activation_threshold_met;
	status.temperature_source                   = _params.temp_src;

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
	update_params(true);

	const int32_t target = _params.sens_id;

	// Disabled instance
	if (target < 0) {
		PX4_INFO("heater %u disabled (HEATER%u_SENS_ID=%ld)",
			 (unsigned)_instance, (unsigned)_instance, (long)target);
		return PX4_OK;
	}

	// Auto-select only allowed for legacy single-heater setups
	if ((target == 0) && (HEATER_NUM > 1)) {
		PX4_INFO("heater %u disabled (HEATER%u_SENS_ID=0 not allowed when HEATER_NUM>1)",
			 (unsigned)_instance, (unsigned)_instance);
		return PX4_OK;
	}

	ScheduleNow();
	return PX4_OK;
}

void Heater::stop()
{
	_should_exit = true;
	heater_off();
	ScheduleNow();
}

int Heater::status(uint8_t instance)
{

	if (instance > 0 && instance <= HEATER_MAX_INSTANCES) {
		if (Heater::is_running_instance(instance)) {
			PX4_INFO("instance %u: running", (unsigned)instance);
			PX4_INFO("instance %u: Sensor ID is %lu", (unsigned)instance, Heater::g_heater[instance - 1]->_sensor_device_id);
			PX4_INFO("instance %u: Sensor Temperature is %f", (unsigned)instance, (double)Heater::g_heater[instance - 1]->_temperature_last);
			PX4_INFO("instance %u: Set Temperature is %f", (unsigned)instance, (double)Heater::g_heater[instance - 1]->_params.temp);

		}

	} else {
		for (instance = 1; instance <= HEATER_MAX_INSTANCES; instance++) {
			if (Heater::is_running_instance(instance)) {
				PX4_INFO("instance %u: running", (unsigned)instance);
				PX4_INFO("instance %u: Sensor ID is %lu", (unsigned)instance, Heater::g_heater[instance - 1]->_sensor_device_id);
				PX4_INFO("instance %u: Sensor Temperature is %f", (unsigned)instance, (double)Heater::g_heater[instance - 1]->_temperature_last);
				PX4_INFO("instance %u: Set Temperature is %f", (unsigned)instance, (double)Heater::g_heater[instance - 1]->_params.temp);
			}
		}
	}

	return PX4_OK;
}

const char *Heater::heater_instance_name(uint8_t inst)
{
	switch (inst) {
	case 1: return "heater_1";

	case 2: return "heater_2";

	case 3: return "heater_3";

	default: return "heater";
	}
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
	if (instance < 1 || instance > HEATER_NUM) {
		PX4_ERR("invalid instance %u", (unsigned)instance);
		return PX4_ERROR;
	}

	if (Heater::is_running_instance(instance)) {
		PX4_WARN("heater %u already running", (unsigned)instance);
		return PX4_OK;
	}

	Heater *h = new Heater(instance);

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
		if (_param_handles.sens_id != PARAM_INVALID) {
			param_get(_param_handles.sens_id, &_params.sens_id);
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

		if (_param_handles.temp_imax != PARAM_INVALID) {
			param_get(_param_handles.temp_imax, &_params.temp_imax);
		}

		if (_param_handles.temp_src != PARAM_INVALID) {
			param_get(_param_handles.temp_src, &_params.temp_src);
		}

		if (_param_handles.temp_activation_threshold != PARAM_INVALID) {
			param_get(_param_handles.temp_activation_threshold, &_params.temp_activation_threshold);
		}

		if (_param_handles.nom_v != PARAM_INVALID) {
			param_get(_param_handles.nom_v, &_params.nom_v);
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
Background process running periodically on the INS{i} queue to regulate temperature at a setpoint.

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
