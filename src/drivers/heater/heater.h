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
 * @file heater.h
 *
 * @author Mark Sauder <mcsauder@gmail.com>
 * @author Alex Klimaj <alexklimaj@gmail.com>
 * @author Jake Dahl <dahl.jakejacob@gmail.com>
 * @author Jacob Crabill <jacob@flyvoly.com>
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/heater_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>
#include <parameters/param.h>

#include <mathlib/mathlib.h>

using namespace time_literals;

#define CONTROLLER_PERIOD_DEFAULT    10000
#define TEMPERATURE_TARGET_THRESHOLD 2.5f
#define HEATER_MAX_INSTANCES 3 	// If changed, also need to change `max_num_config_instances` in module.yaml

class Heater : px4::ScheduledWorkItem, public ModuleParams
{
public:
	Heater(uint8_t instance, const px4::wq_config_t &wq);

	virtual ~Heater();

	/**
	 * @brief Initiates the heater driver work queue, starts a new background task,
	 *        and fails if it is already running.
	 * @return Returns 1 iff start was successful.
	 */
	int start();

	void stop();

	/**
	 * @see ModuleBase::print_usage().
	 * @brief Prints the module usage to the nuttshell console.
	 * @param reason The requested reason for printing to console.
	 */
	static int print_usage(const char *reason = nullptr);

	static bool is_running_instance(uint8_t instance);
	static bool is_running_any();

	static int start_instance(uint8_t instance);
	static int stop_all();
	static int status(uint8_t instance);

	static Heater *g_heater[HEATER_MAX_INSTANCES];

private:

	/** Disables the heater (either by GPIO or PX4IO). */
	void disable_heater();

	/** Turns the heater on (either by GPIO or PX4IO). */
	void heater_on();

	/** Turns the heater off (either by GPIO or PX4IO). */
	void heater_off();

	void initialize();

	/** Enables / configures the heater (either by GPIO or PX4IO). */
	void initialize_heater_io();

	/** @brief Called once to initialize uORB topics. */
	bool initialize_topics();

	void publish_status();

	/** @brief Calculates the heater element on/off time and schedules the next cycle. */
	void Run() override;

	/**
	 * @brief Updates and checks for updated uORB parameters.
	 * @param force Boolean to determine if an update check should be forced.
	 */
	void update_params(const bool force = false);

	/** Work queue struct for the scheduler. */
	static struct work_s _work;

	/** File descriptor for PX4IO for heater ioctl's */
#if defined(PX4IO_HEATER_ENABLED)
	int _io_fd {-1};
#endif

	bool _heater_initialized     = false;
	bool _heater_on              = false;
	bool _temperature_target_met = false;

	int _controller_period_usec = CONTROLLER_PERIOD_DEFAULT;
	int _controller_time_on_usec = 0;

	float _integrator_value   = 0.0f;
	float _proportional_value = 0.0f;

	uORB::Publication<heater_status_s> _heater_status_pub{ORB_ID(heater_status)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _sensor_accel_sub{ORB_ID(sensor_accel)};

	uint32_t _sensor_device_id{0};

	float _temperature_last{NAN};

	const uint8_t _instance; // 1,2,3

	volatile bool _should_exit{false};
	struct {
		param_t imu_id;
		param_t temp;
		param_t temp_p;
		param_t temp_i;
		param_t temp_ff;
	} _param_handles;

	struct {
		int32_t imu_id;   // HEATER<i>_IMU_ID: <0 disable, 0 auto, >0 match device_id
		float   temp;     // target temperature
		float   temp_p;
		float   temp_i;
		float   temp_ff;
	} _params;

};


