/****************************************************************************
 *
 *   Copyright (c) 2012-2022 PX4 Development Team. All rights reserved.
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
#pragma once

#include <drivers/device/device.h>
#include <lib/led/led.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <lib/perf/perf_counter.h>

#include <px4_log.h>
#include <px4_platform_common/module.h>

#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/led_control.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/actuator_test.h>

#include "vertiq_telemetry_manager.hpp"
#include "vertiq_client_manager.hpp"
#include "vertiq_serial_interface.hpp"
#include "ifci.hpp"

#include "iq-module-communication-cpp/inc/propeller_motor_control_client.hpp"
#include "iq-module-communication-cpp/inc/brushless_drive_client.hpp"
#include "iq-module-communication-cpp/inc/arming_handler_client.hpp"

class VertiqIo : public ModuleBase<VertiqIo>, public OutputModuleInterface
{

public:

	/**
	* @brief Create a new VertiqIo object
	*/
	VertiqIo();

	/**
	* @brief destruct a VertiqIo object
	*/
	~VertiqIo() override;

	/**
	* @brief initialize the VertiqIo object. This will be called by the task_spawn function. Makes sure that the thread gets scheduled.
	*/
	bool init();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	int print_status() override;

	/** @see ModuleBase::run() */ //I do not think this actually comes from ModuleBase. it should come from scheduled work item
	void Run() override;

	/** @see OutputModuleInterface */
	bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated) override;

private:
	static const uint8_t MAX_SUPPORTABLE_IFCI_CVS = 16;

	enum DISARM_BEHAVIORS {TRIGGER_MOTOR_DISARM, COAST_MOTOR, SEND_PREDEFINED_THROTTLE};
	enum ARM_BEHAVIORS {USE_MOTOR_ARMING, FORCE_ARMING};

	//Variables and functions necessary for properly configuring the serial interface
	//Determines whether or not we should initialize or re-initialize the serial connection
	static px4::atomic_bool _request_telemetry_init;

	MixingOutput _mixing_output{"VERTIQ_IO", MAX_SUPPORTABLE_IFCI_CVS, *this, MixingOutput::SchedulingPolicy::Auto, false, false};

	//The name of the device we're connecting to. this will be something like /dev/ttyS3
	static char _telemetry_device[20];

	//Counters/timers to track our status
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": output update interval")};

	//We need a serial handler in order to talk over the serial port
	VertiqSerialInterface _serial_interface;

	//We need someone who can manage our clients
	VertiqClientManager _client_manager;

	//We need a telemetry handler
	VertiqTelemetryManager _telem_manager;

	//Pointer to the IFCI handler
	IFCI *_motor_interface_ptr;

	//Store the number of control variables that we're using
	uint8_t _cvs_in_use = 0;

	//Store the telemetry bitmask for who we want to get telemetry from
	uint16_t _telem_bitmask = 0;

	//This is the variable we're actually going to use in the brodcast packed control message
	//We set and use it to _current_telemetry_target_module_id until we send the first
	//broadcast message with this as the tail byte. after that first transmission, set it
	//to an impossible module ID
	uint16_t _telemetry_request_id = 0;

	static const uint8_t _impossible_module_id = 255;

	bool _send_forced_arm = true;

	//We want to publish our ESC Status to anyone who will listen
	uORB::Publication<esc_status_s> _esc_status_pub{ORB_ID(esc_status)};

	/**
	 * Check for parameter changes and update them if needed.
	 */
	void parameters_update();

	// Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	//We need to bring in the parameters that we define in module.yaml in order to view them in the
	//control station, as well as to use them in the firmware
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::VERTIQ_BAUD>) _param_vertiq_baud,
		(ParamInt<px4::params::VERTIQ_NUM_CVS>) _param_vertiq_number_of_cvs,
		(ParamInt<px4::params::VERTIQ_TEL_MSK>) _param_vertiq_telemetry_mask,
		(ParamInt<px4::params::DISARM_THROTTLE>) _param_vertiq_disarm_throttle,
		(ParamInt<px4::params::DISARM_BEHAVE>) _param_vertiq_disarm_behavior,
		(ParamInt<px4::params::ARMING_BEHAVE>) _param_vertiq_arm_behavior,
		(ParamInt<px4::params::TARGET_MODULE_ID>) _param_vertiq_target_module_id
#ifdef CONFIG_USE_IQUART_MODULE_ENTRIES
		, (ParamBool<px4::params::TRIGGER_READ>) _param_vertiq_trigger_read
		, (ParamInt<px4::params::CONTROL_MODE>) _param_vertiq_control_mode
		, (ParamFloat<px4::params::MAX_VELOCITY>) _param_vertiq_max_velo
		, (ParamFloat<px4::params::MAX_VOLTS>) _param_vertiq_max_volts
		, (ParamInt<px4::params::VERTIQ_MOTOR_DIR>) _param_vertiq_motor_direction
		, (ParamInt<px4::params::VERTIQ_FC_DIR>) _param_vertiq_fc_direction
#ifdef CONFIG_USE_IFCI_CONFIGURATION
		, (ParamInt<px4::params::THROTTLE_CVI>) _param_vertiq_throttle_cvi
#ifdef CONFIG_USE_PULSING_CONFIGURATION
		, (ParamInt<px4::params::PULSE_VOLT_MODE>) _param_vertiq_pulse_volt_mode
		, (ParamInt<px4::params::X_CVI>) _param_vertiq_pulse_x_cvi
		, (ParamInt<px4::params::Y_CVI>) _param_vertiq_pulse_y_cvi
		, (ParamFloat<px4::params::ZERO_ANGLE>) _param_vertiq_pulse_zero_angle
		, (ParamFloat<px4::params::VELOCITY_CUTOFF>) _param_vertiq_pulse_velo_cutoff
		, (ParamFloat<px4::params::TORQUE_OFF_ANGLE>) _param_vertiq_pulse_torque_offset_angle
		, (ParamFloat<px4::params::PULSE_VOLT_LIM>) _param_vertiq_pulse_voltage_limit
#endif //CONFIG_USE_IQUART_MODULE_ENTRIES
#endif //CONFIG_USE_PULSING_CONFIGURATION
#endif //CONFIG_USE_IFCI_CONFIGURATION
	)
};




