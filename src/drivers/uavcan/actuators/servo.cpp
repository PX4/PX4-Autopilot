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

using namespace time_literals;

UavcanServoController::UavcanServoController(uavcan::INode &node) :
	_node(node),
	_uavcan_pub_array_cmd(node),
	_uavcan_sub_status(node)
{
	_uavcan_pub_array_cmd.setPriority(UAVCAN_COMMAND_TRANSFER_PRIORITY);
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

	return res;
}

void
UavcanServoController::set_servo_count(uint8_t count)
{
	_servo_count = count;
}

void
UavcanServoController::update_outputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs)
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
UavcanServoController::servo_status_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::actuator::Status>
		&msg)
{
	// make sure your ID number set on your uavcan servo is between 0-8 otherwise it wont be picked up here.
	if (msg.actuator_id < servo_status_s::CONNECTED_SERVO_MAX) {
		auto &ref = _servo_status.servo[msg.actuator_id];

		ref.timestamp       	= hrt_absolute_time();
		ref.servo_position 	= msg.position;
		ref.servo_acceleration  = msg.force;
		ref.servo_speed     	= msg.speed;
		ref.load_pct     	= msg.power_rating_pct;

		_servo_status.timestamp = hrt_absolute_time();
		_servo_status.servo_count = _servo_count;
		_servo_status.counter += 1;

		// this is only helpfull if you have more than 1 UAVCAN servo connected, since if you have only one,
		// and it gets disconnected, this callback function will not execute, thus not updating the
		// check_servo_status()
		_servo_status.servo_online_flags = check_servos_status();
		_servo_status_pub.publish(_servo_status);
	}
}

uint8_t
UavcanServoController::check_servos_status()
{
	int servo_status_flags = 0;
	const hrt_abstime now = hrt_absolute_time();

	for (int index = 0; index < servo_status_s::CONNECTED_SERVO_MAX; index++) {

		// this timeout value can be too short for a given servo periodic stream rate, and may result in a servo being timeout.
		if (_servo_status.servo[index].timestamp > 0 && now - _servo_status.servo[index].timestamp < 1200_ms) {
			servo_status_flags |= (1 << index);
		}

	}

	return servo_status_flags;
}
