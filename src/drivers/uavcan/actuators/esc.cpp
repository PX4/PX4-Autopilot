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

using namespace time_literals;

UavcanEscController::UavcanEscController(uavcan::INode &node) :
	_node(node),
	_uavcan_pub_raw_cmd(node),
	_uavcan_sub_status(node),
	_uavcan_sub_status_extended(node)
{
	_uavcan_pub_raw_cmd.setPriority(uavcan::TransferPriority::NumericallyMin); // Highest priority
}

int UavcanEscController::init()
{
	// ESC status subscription
	int res = _uavcan_sub_status.start(StatusCbBinder(this, &UavcanEscController::esc_status_sub_cb));

	if (res < 0) {
		PX4_ERR("ESC status sub failed %i", res);
		return res;
	}

	// ESC Status Extended subscription
	res = _uavcan_sub_status_extended.start(StatusExtendedCbBinder(this, &UavcanEscController::esc_status_extended_sub_cb));

	if (res < 0) {
		PX4_ERR("ESC status extended sub failed %i", res);
		return res;
	}

	_esc_status_pub.advertise();

	int32_t iface_mask{0xFF};

	if (param_get(param_find("UAVCAN_ESC_IFACE"), &iface_mask) == OK) {
		_uavcan_pub_raw_cmd.getTransferSender().setIfaceMask(iface_mask);
	}

	_initialized = true;

	return res;
}

void UavcanEscController::update_outputs(uint16_t outputs[MAX_ACTUATORS], uint8_t output_array_size)
{
	// TODO: configurable rate limit
	const auto timestamp = _node.getMonotonicTime();

	if ((timestamp - _prev_cmd_pub).toUSec() < (1000000 / MAX_RATE_HZ)) {
		return;
	}

	_prev_cmd_pub = timestamp;

	uavcan::equipment::esc::RawCommand msg{};

	for (unsigned i = 0; i < output_array_size; i++) {
		msg.cmd.push_back(static_cast<int>(outputs[i]));
	}

	_uavcan_pub_raw_cmd.broadcast(msg);
}

void UavcanEscController::set_rotor_count(uint8_t count)
{
	_rotor_count = count;
}

void UavcanEscController::esc_status_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::Status> &msg)
{
	if (msg.esc_index < esc_status_s::CONNECTED_ESC_MAX) {
		esc_report_s &esc_report = _esc_status.esc[msg.esc_index];
		esc_report.timestamp = hrt_absolute_time();
		esc_report.esc_address = msg.getSrcNodeID().get();
		esc_report.esc_voltage = msg.voltage;
		esc_report.esc_current = msg.current;
		esc_report.esc_temperature = msg.temperature + atmosphere::kAbsoluteNullCelsius; // Kelvin to Celsius
		// esc_report.motor_temperature is filled in the extended status callback
		esc_report.esc_rpm = msg.rpm;
		esc_report.esc_errorcount = msg.error_count;
		esc_report.failures = get_failures(msg.esc_index);

		_esc_status.esc_count = _rotor_count;
		_esc_status.counter += 1;
		_esc_status.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_CAN;
		_esc_status.esc_online_flags = check_escs_status();
		_esc_status.esc_armed_flags = (1 << _rotor_count) - 1;
		_esc_status.timestamp = hrt_absolute_time();

		_esc_status_pub.publish(_esc_status);
	}

	// Register device capability for each ESC channel
	if (_node_info_publisher != nullptr) {
		uint8_t node_id = msg.getSrcNodeID().get();
		uint32_t device_id = msg.esc_index;
		_node_info_publisher->registerDeviceCapability(node_id, device_id, NodeInfoPublisher::DeviceCapability::ESC);
	}
}

void UavcanEscController::esc_status_extended_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::StatusExtended> &msg)
{
	uint8_t index = msg.esc_index;

	if (index < esc_status_s::CONNECTED_ESC_MAX) {
		esc_report_s &esc_report = _esc_status.esc[index];
		// published with the non-extended esc::Status
		esc_report.motor_temperature = msg.motor_temperature_degC;
		esc_report.esc_power = msg.input_pct;
	}
}

uint8_t UavcanEscController::check_escs_status()
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

uint32_t UavcanEscController::get_failures(uint8_t esc_index)
{
	esc_report_s &esc_report = _esc_status.esc[esc_index];
	uint32_t failures = 0;

	// Update device vendor/model information from device_information topic
	device_information_s device_information{};

	char esc_name[80] {};

	if (_device_information_sub.copy(&device_information)
	    && device_information.device_type == device_information_s::DEVICE_TYPE_ESC
	    && device_information.device_id == esc_index) {
		strncpy(esc_name, device_information.name, sizeof(esc_name) - 1);
		esc_name[sizeof(esc_name) - 1] = '\0';
	}

	// Update node health from all available dronecan_node_status messages
	dronecan_node_status_s node_status {};
	uint8_t node_health{dronecan_node_status_s::HEALTH_OK};
	uint16_t vendor_specific_status_code{0};

	for (auto &dronecan_node_status_sub : _dronecan_node_status_subs) {
		if (dronecan_node_status_sub.copy(&node_status)) {
			if (node_status.node_id == esc_report.esc_address) {
				node_health = node_status.health;
				vendor_specific_status_code = node_status.vendor_specific_status_code;
				break;
			}
		}
	}

	if (esc_report.esc_address <= uavcan::NodeID::Max && (node_health == dronecan_node_status_s::HEALTH_OK ||
			node_health == dronecan_node_status_s::HEALTH_WARNING)) {
		failures = 0;

	} else if (strstr(esc_name, "iq_motion") != nullptr) {
		// Parse iq_motion ESC errors
		static const struct {
			uint8_t bit;
			uint8_t failure_type;
		} bit_to_failure_map[] = {
			{0,  esc_report_s::FAILURE_OVER_VOLTAGE},
			{1,  esc_report_s::FAILURE_OVER_VOLTAGE},
			{2,  esc_report_s::FAILURE_OVER_VOLTAGE},
			{3,  esc_report_s::FAILURE_OVER_CURRENT},
			{4,  esc_report_s::FAILURE_OVER_CURRENT},
			{5,  esc_report_s::FAILURE_OVER_ESC_TEMPERATURE},
			{6,  esc_report_s::FAILURE_MOTOR_OVER_TEMPERATURE},
			{7,  esc_report_s::FAILURE_GENERIC},
			{8,  esc_report_s::FAILURE_OVER_RPM},
			{9,  esc_report_s::FAILURE_WARN_ESC_TEMPERATURE},
			{10, esc_report_s::FAILURE_MOTOR_WARN_TEMPERATURE},
			{11, esc_report_s::FAILURE_OVER_VOLTAGE},
		};

		for (const auto &mapping : bit_to_failure_map) {
			if (vendor_specific_status_code & (1 << mapping.bit)) {
				failures |= (1 << mapping.failure_type);
			}
		}

	} else {
		// Generic parsing
		failures |= (1 << esc_report_s::FAILURE_GENERIC);
	}

	return failures;
}
