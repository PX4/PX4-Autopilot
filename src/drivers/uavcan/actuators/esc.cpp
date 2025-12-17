/****************************************************************************
 *
 *   Copyright (C) 2014-2025 PX4 Development Team. All rights reserved.
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
 * @file esc.cpp
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include "esc.hpp"
#include <systemlib/err.h>
#include <parameters/param.h>
#include <drivers/drv_hrt.h>
#include <lib/atmosphere/atmosphere.h>

#define MOTOR_BIT(x) (1<<(x))

using namespace time_literals;

UavcanEscController::UavcanEscController(uavcan::INode &node) :
	_node(node),
	_uavcan_pub_raw_cmd(node),
	_uavcan_pub_hobbywing_raw_cmd(node),
	_uavcan_sub_status(node),
	_uavcan_hobbywing_status_msg1(node),
	_uavcan_hobbywing_status_msg2(node),
	_uavcan_hobbywing_status_msg3(node)
{
	_uavcan_pub_raw_cmd.setPriority(uavcan::TransferPriority::NumericallyMin); // Highest priority
	_uavcan_pub_hobbywing_raw_cmd.setPriority(uavcan::TransferPriority::NumericallyMin);
}

int
UavcanEscController::init()
{
	_esc_status_pub.advertise();

	int32_t iface_mask{0xFF};
	int res = 0;

	if (param_get(param_find("UAVCAN_ESC_IFACE"), &iface_mask) == OK && param_get(param_find("UAVCAN_ESC_TYPE"), &_esc_type) == OK) {
		switch (_esc_type) {
		case 0:
			res = _uavcan_sub_status.start(StatusCbBinder(this, &UavcanEscController::esc_status_sub_cb));

			if (res < 0) {
				PX4_ERR("ESC status sub failed %i", res);
				return res;
			}

			_uavcan_pub_raw_cmd.getTransferSender().setIfaceMask(iface_mask);
			break;

		case 1:
			res = _uavcan_hobbywing_status_msg1.start(StatusMsg1Binder(this, &UavcanEscController::esc_status_msg1_cb));

			if (res < 0) {
				PX4_ERR("ESC status sub failed %i", res);
				return res;
			}

			res = _uavcan_hobbywing_status_msg2.start(StatusMsg2Binder(this, &UavcanEscController::esc_status_msg2_cb));

			if (res < 0) {
				PX4_ERR("ESC status sub failed %i", res);
				return res;
			}

			res = _uavcan_hobbywing_status_msg3.start(StatusMsg3Binder(this, &UavcanEscController::esc_status_msg3_cb));

			if (res < 0) {
				PX4_ERR("ESC status sub failed %i", res);
				return res;
			}

			_uavcan_pub_hobbywing_raw_cmd.getTransferSender().setIfaceMask(iface_mask);
			break;

		default:
			PX4_ERR("ESC type failed");
			return res;
			break;
		}
	}

	_initialized = true;

	return res;
}

void
UavcanEscController::update_outputs(uint16_t outputs[MAX_ACTUATORS], uint8_t output_array_size)
{
	// TODO: configurable rate limit
	const auto timestamp = _node.getMonotonicTime();

	if ((timestamp - _prev_cmd_pub).toUSec() < (1000000 / MAX_RATE_HZ)) {
		return;
	}

	_prev_cmd_pub = timestamp;

	switch (_esc_type) {
	case 0: {
			uavcan::equipment::esc::RawCommand msg = {};

			for (unsigned i = 0; i < output_array_size; i++) {
				msg.cmd.push_back(static_cast<int>(outputs[i]));
			}

			_uavcan_pub_raw_cmd.broadcast(msg);
			break;
		}

	case 1: {
			com::hobbywing::esc::RawCommand msg = {};

			for (unsigned i = 0; i < output_array_size; i++) {
				msg.command.push_back(static_cast<int>(outputs[i]));
			}

			_uavcan_pub_hobbywing_raw_cmd.broadcast(msg);
			break;
		}

	default: {
			PX4_ERR("ESC type failed");
			break;
		}
	}

}

void
UavcanEscController::set_rotor_count(uint8_t count)
{
	_rotor_count = count;
}

void
UavcanEscController::esc_status_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::Status> &msg)
{
	if (msg.esc_index < esc_status_s::CONNECTED_ESC_MAX) {
		auto &ref = _esc_status.esc[msg.esc_index];

		ref.timestamp       = hrt_absolute_time();
		ref.esc_address = msg.getSrcNodeID().get();
		ref.esc_voltage     = msg.voltage;
		ref.esc_current     = msg.current;
		ref.esc_temperature = msg.temperature + atmosphere::kAbsoluteNullCelsius; // Kelvin to Celsius
		ref.esc_rpm         = msg.rpm;
		ref.esc_errorcount  = msg.error_count;

		_esc_status.esc_count = _rotor_count;
		_esc_status.counter += 1;
		_esc_status.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_CAN;
		_esc_status.esc_online_flags = check_escs_status();
		_esc_status.esc_armed_flags = (1 << _rotor_count) - 1;
		_esc_status.timestamp = hrt_absolute_time();
		_esc_status_pub.publish(_esc_status);
	}
}

void
UavcanEscController::esc_status_msg1_cb(const uavcan::ReceivedDataStructure<com::hobbywing::esc::StatusMsg1> &msg)
{
	if (msg.getSrcNodeID().get() < esc_status_s::CONNECTED_ESC_MAX) {
		auto &ref = _esc_status.esc[msg.getSrcNodeID().get() - 1];

		ref.timestamp       = hrt_absolute_time();
		ref.esc_address = msg.getSrcNodeID().get();
		ref.esc_rpm         = msg.rpm;

		_esc_status.esc_count = _rotor_count;
		_esc_status.counter += 1;
		_esc_status.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_CAN;
		_esc_status.esc_online_flags = check_escs_status();
		_esc_status.esc_armed_flags = (1 << _rotor_count) - 1;
		_esc_status.timestamp = hrt_absolute_time();
		_esc_status_pub.publish(_esc_status);
	}
}

void
UavcanEscController::esc_status_msg2_cb(const uavcan::ReceivedDataStructure<com::hobbywing::esc::StatusMsg2> &msg)
{
	if (msg.getSrcNodeID().get() < esc_status_s::CONNECTED_ESC_MAX) {
		auto &ref = _esc_status.esc[msg.getSrcNodeID().get() - 1];

		ref.timestamp       = hrt_absolute_time();
		ref.esc_address = msg.getSrcNodeID().get();
		ref.esc_voltage     = msg.input_voltage / 10;
		ref.esc_current     = msg.current;
		ref.esc_temperature = (float)msg.temperature / 100; // In the Mavlink system, the temperature is multiplied by 100.

		_esc_status.esc_count = _rotor_count;
		_esc_status.counter += 1;
		_esc_status.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_CAN;
		_esc_status.esc_online_flags = check_escs_status();
		_esc_status.esc_armed_flags = (1 << _rotor_count) - 1;
		_esc_status.timestamp = hrt_absolute_time();
		_esc_status_pub.publish(_esc_status);
	}
}

void
UavcanEscController::esc_status_msg3_cb(const uavcan::ReceivedDataStructure<com::hobbywing::esc::StatusMsg3> &msg)
{
	if (msg.getSrcNodeID().get() < esc_status_s::CONNECTED_ESC_MAX) {
		auto &ref = _esc_status.esc[msg.getSrcNodeID().get() - 1];

		ref.timestamp       = hrt_absolute_time();
		ref.esc_address = msg.getSrcNodeID().get();

		_esc_status.esc_count = _rotor_count;
		_esc_status.counter += 1;
		_esc_status.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_CAN;
		_esc_status.esc_online_flags = check_escs_status();
		_esc_status.esc_armed_flags = (1 << _rotor_count) - 1;
		_esc_status.timestamp = hrt_absolute_time();
		_esc_status_pub.publish(_esc_status);
	}
}

uint8_t
UavcanEscController::check_escs_status()
{
	int esc_status_flags = 0;
	const hrt_abstime now = hrt_absolute_time();

	for (int index = 0; index < esc_status_s::CONNECTED_ESC_MAX; index++) {

		if (_esc_status.esc[index].timestamp > 0 && now - _esc_status.esc[index].timestamp < 1200_ms) {
			esc_status_flags |= (1 << index);
		}

	}

	return esc_status_flags;
}
