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
 * @author CaFeZn <1837781998@qq.com>
 */

#include "heater.h"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_io_heater.h>

#if defined(BOARD_USES_PX4IO_VERSION) and defined(PX4IO_HEATER_ENABLED)
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

// 多 IMU 加热引脚（必须在 board_config.h 中定义）
#ifndef GPIO_HEATER1
#  error "Please define GPIO_HEATER1 in board_config.h"
#endif
#ifndef GPIO_HEATER2
#  error "Please define GPIO_HEATER2 in board_config.h"
#endif

#ifdef HEATER_GPIO
#    define HEATER_OUTPUT_EN_1(state) px4_arch_gpiowrite(GPIO_HEATER1, state)
#    define HEATER_OUTPUT_EN_2(state) px4_arch_gpiowrite(GPIO_HEATER2, state)
#    define HEATER_OUTPUT_EN(state) \
       do { if (_instance == 1) HEATER_OUTPUT_EN_1(state); \
            else if (_instance == 2) HEATER_OUTPUT_EN_2(state); } while(0)
#endif

Heater::Heater(int instance) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default),
	_heater_status_pub(ORB_ID(heater_status)),
	_instance(instance)
{
	// 根据实例选择对应引脚并初始化
	if (_instance == 1) {
		px4_arch_configgpio(GPIO_HEATER1);
		px4_arch_gpiowrite(GPIO_HEATER1, 0);  // 初始关
	} else if (_instance == 2) {
		px4_arch_configgpio(GPIO_HEATER2);
		px4_arch_gpiowrite(GPIO_HEATER2, 0);
	}

	// 根据实例获取参数句柄（只执行一次）
	if (_instance == 1) {
		_param_imu_temp_ff = param_find("IMU_1_TEMP_FF");
		_param_imu_temp_i = param_find("IMU_1_TEMP_I");
		_param_imu_temp_p = param_find("IMU_1_TEMP_P");
		_param_imu_temp = param_find("IMU_1_TEMP");
		_param_sens_temp_id = param_find("IMU_1_ID");
	} else if (_instance == 2) {
		_param_imu_temp_ff = param_find("IMU_2_TEMP_FF");
		_param_imu_temp_i = param_find("IMU_2_TEMP_I");
		_param_imu_temp_p = param_find("IMU_2_TEMP_P");
		_param_imu_temp = param_find("IMU_2_TEMP");
		_param_sens_temp_id = param_find("IMU_2_ID");
	}

	// 初始化参数缓存值
	update_params(true);

	_heater_status_pub.advertise();
}

Heater::~Heater()
{
	disable_heater();
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

void Heater::disable_heater()
{
	// Reset heater to off state.
#ifdef HEATER_PX4IO
	if (_io_fd >= 0) {
		px4_ioctl(_io_fd, PX4IO_HEATER_CONTROL, HEATER_MODE_DISABLED);
	}

#endif

#ifdef HEATER_GPIO
	if (_instance == 1) {
		px4_arch_configgpio(GPIO_HEATER1);
		px4_arch_gpiowrite(GPIO_HEATER1, 0);
	} else if (_instance == 2) {
		px4_arch_configgpio(GPIO_HEATER2);
		px4_arch_gpiowrite(GPIO_HEATER2, 0);
	}
#endif
}

void Heater::initialize_heater_io()
{
	// Initialize heater to off state.
#ifdef HEATER_PX4IO
	if (_io_fd < 0) {
		_io_fd = px4_open(IO_HEATER_DEVICE_PATH, O_RDWR);
	}

	if (_io_fd >= 0) {
		px4_ioctl(_io_fd, PX4IO_HEATER_CONTROL, HEATER_MODE_OFF);
	}

#endif

#ifdef HEATER_GPIO
	// 多 IMU：根据实例初始化对应引脚
	if (_instance == 1) {
		px4_arch_configgpio(GPIO_HEATER1);
		px4_arch_gpiowrite(GPIO_HEATER1, 0);
	} else if (_instance == 2) {
		px4_arch_configgpio(GPIO_HEATER2);
		px4_arch_gpiowrite(GPIO_HEATER2, 0);
	}
#endif
}

void Heater::heater_off()
{
#ifdef HEATER_PX4IO

	if (_io_fd >= 0) {
		px4_ioctl(_io_fd, PX4IO_HEATER_CONTROL, HEATER_MODE_OFF);
	}

#endif

#ifdef HEATER_GPIO
	HEATER_OUTPUT_EN(false);
#endif
}

void Heater::heater_on()
{
#ifdef HEATER_PX4IO

	if (_io_fd >= 0) {
		px4_ioctl(_io_fd, PX4IO_HEATER_CONTROL, HEATER_MODE_ON);
	}

#endif

#ifdef HEATER_GPIO
	HEATER_OUTPUT_EN(true);
#endif
}

bool Heater::initialize_topics()
{
	for (uint8_t i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
		uORB::SubscriptionData<sensor_accel_s> sensor_accel_sub{ORB_ID(sensor_accel), i};

		if (sensor_accel_sub.get().timestamp != 0 &&
		    sensor_accel_sub.get().device_id != 0 &&
		    PX4_ISFINITE(sensor_accel_sub.get().temperature)) {

			// 根据实例选择对应 IMU 的 device_id
			int32_t target_id = _param_cached_sens_id;

			if (sensor_accel_sub.get().device_id == (uint32_t)target_id) {
				_sensor_accel_sub.ChangeInstance(i);
				_sensor_device_id = sensor_accel_sub.get().device_id;
				initialize_heater_io();
				return true;
			}
		}
	}

	return false;
}

void Heater::Run()
{
	if (should_exit()) {
#if defined(HEATER_PX4IO)

		// must be closed from wq thread
		if (_io_fd >= 0) {
			px4_close(_io_fd);
		}

#endif
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

	if (_heater_on) {
		// Turn the heater off.
		_heater_on = false;
		heater_off();
		ScheduleDelayed(CONTROLLER_PERIOD_DEFAULT - _controller_time_on_usec);

	} else if (_sensor_accel_sub.update(&sensor_accel)) {

		// Update the current IMU sensor temperature if valid.
		if (PX4_ISFINITE(sensor_accel.temperature)) {
			temperature_delta = _param_cached_temp - sensor_accel.temperature;
			_temperature_last = sensor_accel.temperature;
		}

		// 使用缓存的参数值（高效）
		float p_gain = _param_cached_temp_p;
		float i_gain = _param_cached_temp_i;
		float ff_val = _param_cached_temp_ff;

		// PID积分项计算
		_integrator_value += i_gain * temperature_delta;
		_integrator_value = math::constrain(_integrator_value, -0.25f, 0.25f);

		_proportional_value = p_gain * temperature_delta;

		_controller_time_on_usec = static_cast<int>((ff_val + _proportional_value +
					   _integrator_value) * static_cast<float>(CONTROLLER_PERIOD_DEFAULT));

		_controller_time_on_usec = math::constrain(_controller_time_on_usec, 0, CONTROLLER_PERIOD_DEFAULT);

		if (fabsf(temperature_delta) < TEMPERATURE_TARGET_THRESHOLD) {
			_temperature_target_met = true;

		} else {
			_temperature_target_met = false;
		}

		if (_controller_time_on_usec > 0) {
			// Turn the heater on.
			_heater_on = true;
			heater_on();
			ScheduleDelayed(_controller_time_on_usec);

		} else {
			// Turn the heater off.
			ScheduleDelayed(CONTROLLER_PERIOD_DEFAULT);
		}
	}

	publish_status();
}

void Heater::publish_status()
{
	heater_status_s status{};
	status.device_id               = _sensor_device_id;
	status.heater_on               = _heater_on;
	status.temperature_sensor      = _temperature_last;
	status.temperature_target      = _param_cached_temp;
	status.temperature_target_met  = _temperature_target_met;
	status.controller_period_usec  = CONTROLLER_PERIOD_DEFAULT;
	status.controller_time_on_usec = _controller_time_on_usec;
	status.proportional_value      = _proportional_value;
	status.integrator_value        = _integrator_value;
	status.feed_forward_value      = _param_cached_temp_ff;

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
	ScheduleNow();
	return PX4_OK;
}

int Heater::task_spawn(int argc, char *argv[])
{
	int instance = 1;
	if (argc >= 3 && strcmp(argv[1], "-i") == 0) {
		instance = atoi(argv[2]);
		if (instance != 1 && instance != 2) instance = 1;
	}
	Heater *heater = new Heater(instance);

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
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		// update parameters from storage
		ModuleParams::updateParams();

		// 更新缓存的参数值
		if (_param_imu_temp_ff != PARAM_INVALID) {
			param_get(_param_imu_temp_ff, &_param_cached_temp_ff);
		}
		if (_param_imu_temp_i != PARAM_INVALID) {
			param_get(_param_imu_temp_i, &_param_cached_temp_i);
		}
		if (_param_imu_temp_p != PARAM_INVALID) {
			param_get(_param_imu_temp_p, &_param_cached_temp_p);
		}
		if (_param_imu_temp != PARAM_INVALID) {
			param_get(_param_imu_temp, &_param_cached_temp);
		}
		if (_param_sens_temp_id != PARAM_INVALID) {
			param_get(_param_sens_temp_id, &_param_cached_sens_id);
		}
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

extern "C" __EXPORT int heater_main(int argc, char *argv[])
{
	return Heater::main(argc, argv);
}
