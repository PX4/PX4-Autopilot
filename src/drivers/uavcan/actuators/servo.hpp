/****************************************************************************
 *
 *   Copyright (C) 2021 PX4 Development Team. All rights reserved.
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

#include <uavcan/uavcan.hpp>
#include <uavcan/equipment/actuator/Status.hpp>
#include <uavcan/equipment/actuator/ArrayCommand.hpp>
#include <uavcan/equipment/device/Temperature.hpp>
#include <uavcan/equipment/power/CircuitStatus.hpp>
#include <lib/perf/perf_counter.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/servo_status.h>
#include <drivers/drv_hrt.h>
#include <lib/mixer_module/mixer_module.hpp>

class UavcanServoController
{
public:
	static constexpr int MAX_ACTUATORS = 8;
	static constexpr unsigned MAX_RATE_HZ = 50;
	static constexpr unsigned UAVCAN_COMMAND_TRANSFER_PRIORITY = 6;	///< 0..31, inclusive, 0 - highest, 31 - lowest
	static_assert(uavcan::equipment::actuator::ArrayCommand::FieldTypes::commands::MaxSize >= MAX_ACTUATORS,
		      "Too many actuators");

	UavcanServoController(uavcan::INode &node);
	~UavcanServoController() = default;

	int init();
	void set_servo_count(uint8_t count);
	void update_outputs(uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs);

	servo_status_s &servo_status() { return _servo_status; }


private:

	/**
	 * Servo status message reception will be reported via this callback.
	 */
	void servo_status_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::actuator::Status> &msg);

	/**
	 * Servo temperature message reception will be reported via this callback.
	 */
	void servo_temperature_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::device::Temperature> &msg);

	/**
	 * Servo circuit status message reception will be reported via this callback.
	 */
	void servo_circuit_status_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::power::CircuitStatus> &msg);


	/**
	 * Checks all the servo freshness based on timestamp, if an servo exceeds the timeout then is flagged offline.
	 */
	uint8_t check_servos_status();

	typedef uavcan::MethodBinder<UavcanServoController *,
		void (UavcanServoController::*)(const uavcan::ReceivedDataStructure<uavcan::equipment::actuator::Status>&)>
		StatusCbBinder;

	typedef uavcan::MethodBinder<UavcanServoController *,
		void (UavcanServoController::*)(const uavcan::ReceivedDataStructure<uavcan::equipment::device::Temperature>&)>
		TemperatureCbBinder;

	typedef uavcan::MethodBinder<UavcanServoController *,
		void (UavcanServoController::*)(const uavcan::ReceivedDataStructure<uavcan::equipment::power::CircuitStatus>&)>
		CircuitStatusCbBinder;

	servo_status_s	_servo_status{};
	uint8_t _servo_count{0};

	uORB::PublicationMulti<servo_status_s> _servo_status_pub{ORB_ID(servo_status)};

	/*
	 * libuavcan related things
	 */

	uavcan::INode								&_node;
	uavcan::Publisher<uavcan::equipment::actuator::ArrayCommand> _uavcan_pub_array_cmd;
	uavcan::Subscriber<uavcan::equipment::actuator::Status, StatusCbBinder>	_uavcan_sub_status;
	uavcan::Subscriber<uavcan::equipment::device::Temperature, TemperatureCbBinder> _uavcan_sub_temperature;
	uavcan::Subscriber<uavcan::equipment::power::CircuitStatus, CircuitStatusCbBinder> _uavcan_sub_circuit_status;


	float _last_temperature[servo_status_s::CONNECTED_SERVO_MAX];
	uint8_t _last_temperature_error_flag[servo_status_s::CONNECTED_SERVO_MAX];
	float _last_voltage[servo_status_s::CONNECTED_SERVO_MAX];
	float _last_current[servo_status_s::CONNECTED_SERVO_MAX];
	uint8_t _last_power_error_flag[servo_status_s::CONNECTED_SERVO_MAX];
	int _servo_temperature_counter{0};
	int _servo_power_counter{0};

};
