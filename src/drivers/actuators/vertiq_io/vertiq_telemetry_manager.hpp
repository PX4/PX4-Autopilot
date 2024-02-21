/****************************************************************************
 *
 *   Copyright (c) 2012-2024 PX4 Development Team. All rights reserved.
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

#ifndef VERTIQ_TELEMETRY_MANAGER_HPP
#define VERTIQ_TELEMETRY_MANAGER_HPP

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

#include "ifci.hpp"

enum vertiq_telemetry_pause_states {
	PAUSED,
	UNPAUSED
};

class VertiqTelemetryManager
{
public:

	/**
	* @brief Construct a new VertiqTelemetryManager object with a pointer to an IFCI handler
	* @param motor_interface A pointer to and IFCI interface
	*/
	VertiqTelemetryManager(IFCI *motor_interface);

	/**
	* @brief Initialize the telemetry manager with the bitmask set in the PX4 parameters
	* @param telem_bitmask The bitmask set in the PX4 parameters as VERTIQ_TEL_MSK
	*/
	void Init(uint64_t telem_bitmask);

	/**
	* @brief Start publishing the ESC statuses to the uORB esc_status topic
	*/
	void StartPublishing(uORB::Publication<esc_status_s> *esc_status_pub);

	/**
	* @brief Part of initialization. Find the first and last positions that indicate modules to grab telemetry from
	*/
	void FindTelemetryModuleIds();

	/**
	* @brief Attempt to grab the telemetry response from the currently targeted module. Handles
	* updating when to start requesting telemetry from the next module.
	*/
	uint16_t UpdateTelemetry();

	/**
	* @brief Determing the next module to request telemetry from
	*/
	uint16_t FindNextMotorForTelemetry();

	/**
	* @brief return access to our esc_status_s handler
	* @return _esc_status: our instance of an esc_status_s
	*/
	esc_status_s GetEscStatus();

	void PauseTelemetry();
	void UnpauseTelemetry();
	vertiq_telemetry_pause_states GetTelemetryPauseState();

private:

	vertiq_telemetry_pause_states _telem_state;

	IFCI *_motor_interface;

	//We want to publish our ESC Status to anyone who will listen
	esc_status_s		_esc_status;

	//The max number of module IDs that we can support
	static const uint8_t MAX_SUPPORTABLE_MODULE_IDS = 63; //[0, 62]

	//The max number of esc status entries we can keep track of (per the esc_status_s type)
	static const uint8_t MAX_ESC_STATUS_ENTRIES = 8;

	//We need a way to store the module IDs that we're supposed to ask for telemetry from. We can have as many as 63.
	uint8_t _module_ids_in_use[MAX_ESC_STATUS_ENTRIES];
	uint8_t _number_of_module_ids_for_telem = 0;
	uint8_t _current_module_id_target_index = 0;

	//Store the telemetry bitmask for who we want to get telemetry from
	uint64_t _telem_bitmask = 0;

	//The amount of time (in ms) that we'll wait for a telemetry response
	static const hrt_abstime _telem_timeout = 50_ms;

	//The system time the last time that we got telemetry
	hrt_abstime _time_of_last_telem_request = 0;

	static const uint8_t _impossible_module_id = 255;
};

#endif
