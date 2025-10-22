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

#include "servo.hpp"
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))

using namespace time_literals;

UavcanServoController::UavcanServoController(uavcan::INode &node) :
	_node(node),
	_uavcan_pub_array_cmd(node),
	_uavcan_sub_status(node),
	_uavcan_sub_temperature(node),
	_uavcan_sub_circuit_status(node)
{
	/* Ensure that future changes do not cause any out-of-bounds access */
	static_assert(ARRAY_SIZE(_servo_status.servo) == servo_status_s::CONNECTED_SERVO_MAX);
	static_assert(ARRAY_SIZE(_last_temperature) == servo_status_s::CONNECTED_SERVO_MAX);
	static_assert(ARRAY_SIZE(_last_temperature_error_flag) == servo_status_s::CONNECTED_SERVO_MAX);
	static_assert(ARRAY_SIZE(_last_voltage) == servo_status_s::CONNECTED_SERVO_MAX);
	static_assert(ARRAY_SIZE(_last_current) == servo_status_s::CONNECTED_SERVO_MAX);
	static_assert(ARRAY_SIZE(_last_power_error_flag) == servo_status_s::CONNECTED_SERVO_MAX);
	static_assert(ARRAY_SIZE(_servo_temperature_counter) == servo_status_s::CONNECTED_SERVO_MAX);
	static_assert(ARRAY_SIZE(_servo_power_counter) == servo_status_s::CONNECTED_SERVO_MAX);

	_uavcan_pub_array_cmd.setPriority(UAVCAN_COMMAND_TRANSFER_PRIORITY);

	/* Init the arrays to zero */
	memset(_last_temperature, 0, sizeof(_last_temperature));
	memset(_last_temperature_error_flag, 0, sizeof(_last_temperature_error_flag));
	memset(_last_voltage, 0, sizeof(_last_voltage));
	memset(_last_current, 0, sizeof(_last_current));
	memset(_last_power_error_flag, 0, sizeof(_last_power_error_flag));
	memset(_servo_temperature_counter, 0, sizeof(_servo_temperature_counter));
	memset(_servo_power_counter, 0, sizeof(_servo_power_counter));
}

int
UavcanServoController::init()
{
	// Servo status subscription
	int res = _uavcan_sub_status.start(StatusCbBinder(this, &UavcanServoController::servo_status_sub_cb));

	if (res < 0) {
		PX4_ERR("Servo status sub failed %i", res);
		return res;
	}

	// Servo temperature subscription
	res = _uavcan_sub_temperature.start(TemperatureCbBinder(this, &UavcanServoController::servo_temperature_sub_cb));

	if (res < 0) {
		PX4_ERR("Servo Temperature sub failed %i", res);
		return res;
	}

	// Servo circuit status subscription
	res = _uavcan_sub_circuit_status.start(CircuitStatusCbBinder(this, &UavcanServoController::servo_circuit_status_sub_cb));

	if (res < 0) {
		PX4_ERR("Servo circuit status sub failed %i", res);
		return res;
	}

	_servo_status_pub.advertise(); // advertise to ensure messages are logged

	return res; // return if both subscriptions are successful
}

void
UavcanServoController::set_servo_count(uint8_t count)
{
	_servo_count = count;
}

void
UavcanServoController::update_outputs(uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs)
{
	uavcan::equipment::actuator::ArrayCommand msg;

	for (unsigned i = 0; i < num_outputs; ++i) {
		uavcan::equipment::actuator::Command cmd;
		cmd.actuator_id = i;
		cmd.command_type = uavcan::equipment::actuator::Command::COMMAND_TYPE_UNITLESS;
		cmd.command_value = (float)outputs[i] / 500.f - 1.f; // [-1, 1]

		msg.commands.push_back(cmd);
	}

	_uavcan_pub_array_cmd.broadcast(msg);
}

void
UavcanServoController::servo_temperature_sub_cb(const
		uavcan::ReceivedDataStructure<uavcan::equipment::device::Temperature>
		&msg)
{
	for (int i = 0; i < servo_status_s::CONNECTED_SERVO_MAX; i++) {
		auto &ref = _servo_status.servo[i];

		const bool is_servo_online = _servo_status.servo_online_flags & (1 << i);
		const bool is_servo_matching = ref.servo_node_id == msg.getSrcNodeID().get();

		if (is_servo_online && is_servo_matching) {
			_servo_temperature_counter[i] += 1;
			_last_temperature[i] = msg.temperature;
			_last_temperature_error_flag[i] = msg.error_flags;
			break;
		}
	}
}

void
UavcanServoController::servo_circuit_status_sub_cb(const
		uavcan::ReceivedDataStructure<uavcan::equipment::power::CircuitStatus>
		&msg)
{
	for (int i = 0; i < servo_status_s::CONNECTED_SERVO_MAX; i++) {
		auto &ref = _servo_status.servo[i];

		const bool is_servo_online = _servo_status.servo_online_flags & (1 << i);
		const bool is_servo_matching = ref.servo_node_id == msg.getSrcNodeID().get();

		if (is_servo_online && is_servo_matching) {
			_servo_power_counter[i] += 1;
			_last_voltage[i] = msg.voltage;
			_last_current[i] = msg.current;
			_last_power_error_flag[i] = msg.error_flags;
			break;
		}
	}
}

void
UavcanServoController::servo_status_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::actuator::Status>
		&msg)
{
	if (msg.actuator_id < servo_status_s::CONNECTED_SERVO_MAX) {
		auto &ref = _servo_status.servo[msg.actuator_id];

		ref.timestamp = hrt_absolute_time();
		ref.servo_node_id = msg.getSrcNodeID().get();
		ref.servo_actuator_id = msg.actuator_id;
		ref.servo_position = msg.position;
		ref.servo_force = msg.force;
		ref.servo_speed = msg.speed;
		ref.servo_power_rating_pct = msg.power_rating_pct;

		// Add servo temperature data
		ref.servo_temperature_counter = _servo_temperature_counter[msg.actuator_id];
		ref.servo_temperature = _last_temperature[msg.actuator_id];
		ref.servo_temperature_error_flags = _last_temperature_error_flag[msg.actuator_id];

		// Add servo power data
		ref.servo_power_counter = _servo_power_counter[msg.actuator_id];
		ref.servo_voltage = _last_voltage[msg.actuator_id];
		ref.servo_current = _last_current[msg.actuator_id];
		ref.servo_power_error_flags = _last_power_error_flag[msg.actuator_id];

		_servo_status.counter += 1;
		_servo_status.servo_count = _servo_count;
		_servo_status.servo_connectiontype = servo_status_s::SERVO_CONNECTION_TYPE_CAN;
		_servo_status.servo_online_flags = check_servos_status();
		_servo_status.timestamp = hrt_absolute_time();
		_servo_status_pub.publish(_servo_status);
	}
}

uint8_t
UavcanServoController::check_servos_status()
{
	int servo_status_flags = 0;
	const hrt_abstime now = hrt_absolute_time();

	for (int index = 0; index < servo_status_s::CONNECTED_SERVO_MAX; index++) {

		if (_servo_status.servo[index].timestamp > 0 && now - _servo_status.servo[index].timestamp < 1200_ms) {
			servo_status_flags |= (1 << index);
		}
	}

	return servo_status_flags;
}
