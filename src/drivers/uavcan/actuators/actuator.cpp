/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
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
 * @file actuator.cpp
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 * @author Jacob Crabill <jacob@flyvoly.com>
 */

#include "actuator.hpp"
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

#define MOTOR_BIT(x) (1<<(x))

using namespace time_literals;

UavcanActuatorController::UavcanActuatorController(uavcan::INode &node) :
	_node(node),
	_uavcan_pub_array_cmd(node),
	_uavcan_sub_status(node),
	_orb_timer(node)
{
	_uavcan_pub_array_cmd.setPriority(UAVCAN_COMMAND_TRANSFER_PRIORITY);
}

UavcanActuatorController::~UavcanActuatorController()
{
}

int
UavcanActuatorController::init()
{
	param_get(param_find("UAVCAN_SERVO_RATE"), &_actuator_update_rate);

	// ESC status subscription
	int res = _uavcan_sub_status.start(StatusCbBinder(this, &UavcanActuatorController::actuator_status_sub_cb));

	if (res < 0) {
		PX4_ERR("ESC status sub failed %i", res);
		return res;
	}

	// ESC status will be relayed from UAVCAN bus into ORB at this rate
	_orb_timer.setCallback(TimerCbBinder(this, &UavcanActuatorController::orb_pub_timer_cb));
	_orb_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(1000 / actuator_STATUS_UPDATE_RATE_HZ));

	return res;
}

void
UavcanActuatorController::update_outputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs)
{
	if (num_outputs > uavcan::equipment::actuator::ArrayCommand::FieldTypes::cmd::MaxSize) {
		num_outputs = uavcan::equipment::actuator::ArrayCommand::FieldTypes::cmd::MaxSize;
	}

	if (num_outputs > servo_status_s::CONNECTED_ACTUATOR_MAX) {
		num_outputs = servo_status_s::CONNECTED_ACTUATOR_MAX;
	}

	/*
	 * Rate limiting - we don't want to congest the bus
	 */
	const auto timestamp = _node.getMonotonicTime();

	if ((timestamp - _prev_cmd_pub).toUSec() < (1000000 / _actuator_update_rate)) {
		return;
	}

	_prev_cmd_pub = timestamp;

	/*
	 * Fill the command message
	 * If unarmed, we publish an empty message anyway
	 */
	uavcan::equipment::actuator::ArrayCommand msg;

	for (unsigned i = 0; i < num_outputs; i++) {
		if (stop_motors || outputs[i] == DISARMED_OUTPUT_VALUE) {
			msg.cmd.push_back(static_cast<unsigned>(0));

		} else {
			msg.cmd.push_back(static_cast<int>(outputs[i]));
		}
	}

	/*
	 * Remove channels that are always zero.
	 * The objective of this optimization is to avoid broadcasting multi-frame transfers when a single frame
	 * transfer would be enough. This is a valid optimization as the UAVCAN specification implies that all
	 * non-specified ESC setpoints should be considered zero.
	 * The positive outcome is a (marginally) lower bus traffic and lower CPU load.
	 *
	 * From the standpoint of the PX4 architecture, however, this is a hack. It should be investigated why
	 * the mixer returns more outputs than are actually used.
	 */
	for (int index = int(msg.cmd.size()) - 1; index >= _max_number_of_nonzero_outputs; index--) {
		if (msg.cmd[index] != 0) {
			_max_number_of_nonzero_outputs = index + 1;
			break;
		}
	}

	msg.cmd.resize(_max_number_of_nonzero_outputs);

	/*
	 * Publish the command message to the bus
	 * Note that for a quadrotor it takes one CAN frame
	 */
	_uavcan_pub_array_cmd.broadcast(msg);
}

void
UavcanActuatorController::actuator_status_sub_cb(const
		uavcan::ReceivedDataStructure<uavcan::equipment::actuator::Status> &msg)
{
	if (msg.actuator_index < servo_status_s::CONNECTED_ACTUATOR_MAX) {
		auto &ref = _servo_status.servo[msg.actuator_index];

		ref.actuator_address = msg.getSrcNodeID().get();
		ref.timestamp       = hrt_absolute_time();
		ref.servo_voltage     = msg.voltage;
		ref.servo_current     = msg.current;
		ref.servo_temperature = msg.temperature;
		ref.servo_errorcount  = msg.error_count;
	}
}

void
UavcanActuatorController::orb_pub_timer_cb(const uavcan::TimerEvent &)
{
	_servo_status.timestamp = hrt_absolute_time();
	_servo_status.servo_count = _actuator_count;
	_servo_status.counter += 1;
	_servo_status.servo_connectiontype = servo_status_s::SERVO_CONNECTION_TYPE_CAN;
	_servo_status.servo_online_flags = check_actuators_status();
	_servo_status.servo_armed_flags = (1 << _actuator_count) - 1;

	_servo_status_pub.publish(_servo_status);
}

uint8_t
UavcanActuatorController::check_actuators_status()
{
	int actuator_status_flags = 0;
	const hrt_abstime now = hrt_absolute_time();

	for (int index = 0; index < servo_status_s::CONNECTED_SERVO_MAX; index++) {

		if (_servo_status.servo[index].timestamp > 0 && now - _servo_status.servo[index].timestamp < 1200_ms) {
			actuator_status_flags |= (1 << index);
		}

	}

	return actuator_status_flags;
}
