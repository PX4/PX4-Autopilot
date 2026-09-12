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
	_uavcan_sub_status_extended(node),
	_uavcan_param_client(node)
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

	_uavcan_param_client.setCallback(GetSetCbBinder(this, &UavcanEscController::error_count_meaning_cb));

	_esc_status_pub.advertise();

	int32_t iface_mask{0xFF};

	if (param_get(param_find("UAVCAN_ESC_IFACE"), &iface_mask) == OK) {
		_uavcan_pub_raw_cmd.getTransferSender().setIfaceMask(iface_mask);
	}

	int32_t rate_max{400};

	if (param_get(param_find("UAVCAN_ESC_RTMAX"), &rate_max) == OK) {
		_max_rate_hz = (unsigned)rate_max;
	}

	param_get(param_find("UAVCAN_QUIRKS"), &_param_uavcan_quirks);

	_initialized = true;

	return res;
}

void UavcanEscController::update_outputs(float outputs[MAX_ACTUATORS], uint8_t output_array_size)
{
	const auto timestamp = _node.getMonotonicTime();

	if ((timestamp - _prev_cmd_pub).toUSec() < (1000000 / _max_rate_hz)) {
		return;
	}

	_prev_cmd_pub = timestamp;

	uavcan::equipment::esc::RawCommand msg{};

	for (unsigned i = 0; i < output_array_size; i++) {
		msg.cmd.push_back(static_cast<int>(lroundf(outputs[i])));
	}

	_uavcan_pub_raw_cmd.broadcast(msg);
}

void UavcanEscController::set_rotor_count(uint8_t count)
{
	_rotor_count = count;
	_seen_status_mask = 0; // the set of reporting ESC indices may change with the mixer configuration
}

void UavcanEscController::esc_status_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::Status> &msg)
{
	uint8_t esc_index = msg.esc_index;

	if (_param_uavcan_quirks & static_cast<int32_t>(Quirk::HobbywingEscIdx1)) {
		if (msg.esc_index == 0) {
			// non-compliant ESC firmware: esc_index 0 is not expected, ignore
			return;
		}

		esc_index -= 1;
	}

	if (esc_index < esc_status_s::CONNECTED_ESC_MAX) {
		esc_report_s &esc_report = _esc_status.esc[esc_index];
		esc_report.timestamp = hrt_absolute_time();
		esc_report.esc_voltage = msg.voltage;
		esc_report.esc_current = msg.current;
		esc_report.esc_temperature = msg.temperature + atmosphere::kAbsoluteNullCelsius; // Kelvin to Celsius
		// esc_report.motor_temperature is filled in the extended status callback
		esc_report.esc_rpm = msg.rpm;
		esc_report.esc_errorcount = msg.error_count;
		esc_report.esc_errorcount_type = error_count_type(msg.getSrcNodeID().get());
		esc_report.failures = get_failures(esc_index, msg.getSrcNodeID().get());

		// A repeated ESC index marks the start of a new round; publish once per round.
		const uint16_t index_bit = 1u << msg.esc_index;

		if (_seen_status_mask & index_bit) {
			_seen_status_mask = 0;
			_esc_status.esc_count = _rotor_count;
			_esc_status.counter += 1;
			_esc_status.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_CAN;
			_esc_status.esc_online_flags = check_escs_status();
			_esc_status.esc_armed_flags = (1 << _rotor_count) - 1;
			_esc_status.timestamp = esc_report.timestamp;

			_failure_config.update();
			_esc_status_pub.publish(failure_injection::process_esc(_failure_config, _esc_status));
		}

		_seen_status_mask |= index_bit;
	}

	// Register device capability for each ESC channel
	if (_node_info_publisher != nullptr) {
		uint8_t node_id = msg.getSrcNodeID().get();
		uint32_t device_id = esc_index;
		_node_info_publisher->registerDeviceCapability(node_id, device_id, NodeInfoPublisher::DeviceCapability::ESC);
	}

	process_error_count_meaning_retries();
}

void UavcanEscController::handleNodeInfoRetrieved(uavcan::NodeID node_id,
		const uavcan::protocol::GetNodeInfo::Response &node_info)
{
	ErrorCountMeaning *entry = find_error_count_meaning(node_id.get());

	// Only Vertiq modules make the error_count meaning configurable, everything else follows the DroneCAN definition
	if (strncmp(node_info.name.c_str(), VERTIQ_NODE_NAME_PREFIX, strlen(VERTIQ_NODE_NAME_PREFIX)) != 0) {
		if (entry != nullptr) {
			*entry = {};
		}

		return;
	}

	if (entry == nullptr) {
		entry = allocate_error_count_meaning(node_id.get());
	}

	if (entry == nullptr) {
		PX4_WARN("ESC node %d: no slot to track error count meaning, assuming ESC faults", node_id.get());
		return;
	}

	// Node info is retrieved again after a node restart, and the parameter may have changed in between
	*entry = {};
	entry->node_id = node_id.get();
	request_error_count_meaning(*entry);
}

const UavcanEscController::ErrorCountMeaning *UavcanEscController::find_error_count_meaning(uint8_t node_id) const
{
	for (const ErrorCountMeaning &entry : _error_count_meanings) {
		if (entry.state != ErrorCountMeaning::State::Unused && entry.node_id == node_id) {
			return &entry;
		}
	}

	return nullptr;
}

UavcanEscController::ErrorCountMeaning *UavcanEscController::find_error_count_meaning(uint8_t node_id)
{
	return const_cast<ErrorCountMeaning *>(static_cast<const UavcanEscController *>(this)->find_error_count_meaning(node_id));
}

UavcanEscController::ErrorCountMeaning *UavcanEscController::allocate_error_count_meaning(uint8_t node_id)
{
	for (ErrorCountMeaning &entry : _error_count_meanings) {
		if (entry.state == ErrorCountMeaning::State::Unused) {
			entry.node_id = node_id;
			return &entry;
		}
	}

	return nullptr;
}

void UavcanEscController::request_error_count_meaning(ErrorCountMeaning &entry)
{
	uavcan::protocol::param::GetSet::Request req;
	req.name = VERTIQ_ERROR_MEANING_PARAM;

	entry.attempts++;

	if (_uavcan_param_client.call(entry.node_id, req) < 0) {
		fail_error_count_meaning_attempt(entry);

	} else {
		entry.state = ErrorCountMeaning::State::Pending;
	}
}

void UavcanEscController::fail_error_count_meaning_attempt(ErrorCountMeaning &entry)
{
	if (entry.attempts >= ERROR_MEANING_MAX_ATTEMPTS) {
		// Give up and fall back to the generic interpretation
		entry.state = ErrorCountMeaning::State::Resolved;
		entry.type = esc_report_s::ERRORCOUNT_TYPE_ESC_FAULTS;
		PX4_WARN("ESC node %d: %s not readable, assuming ESC fault count", entry.node_id,
			 VERTIQ_ERROR_MEANING_PARAM);

	} else {
		entry.state = ErrorCountMeaning::State::Retry;
		entry.next_attempt = hrt_absolute_time() + ERROR_MEANING_RETRY_INTERVAL_US;
	}
}

void UavcanEscController::process_error_count_meaning_retries()
{
	const hrt_abstime now = hrt_absolute_time();

	for (ErrorCountMeaning &entry : _error_count_meanings) {
		if (entry.state == ErrorCountMeaning::State::Retry && now >= entry.next_attempt) {
			request_error_count_meaning(entry);
		}
	}
}

void UavcanEscController::error_count_meaning_cb(const uavcan::ServiceCallResult<uavcan::protocol::param::GetSet>
		&result)
{
	ErrorCountMeaning *entry = find_error_count_meaning(result.getCallID().server_node_id.get());

	if (entry == nullptr || entry->state != ErrorCountMeaning::State::Pending) {
		return;
	}

	if (!result.isSuccessful()) {
		fail_error_count_meaning_attempt(*entry);
		return;
	}

	uavcan::protocol::param::GetSet::Response resp = result.getResponse(); // Value::is()/to() are not const
	entry->state = ErrorCountMeaning::State::Resolved;

	if (resp.name.empty()) {
		// Parameter does not exist: speed firmware before v0.3.0 always reports the live CAN TX error counter
		entry->type = esc_report_s::ERRORCOUNT_TYPE_CAN_TEC;
		return;
	}

	if (!resp.value.is(uavcan::protocol::param::Value::Tag::integer_value)) {
		// The parameter exists but is not what we expect: better not to judge the count at all
		entry->type = esc_report_s::ERRORCOUNT_TYPE_UNKNOWN;
		return;
	}

	switch (resp.value.to<uavcan::protocol::param::Value::Tag::integer_value>()) {
	case TecErrorCounter:
		entry->type = esc_report_s::ERRORCOUNT_TYPE_CAN_TEC;
		break;

	case RecErrorCounter:
		entry->type = esc_report_s::ERRORCOUNT_TYPE_CAN_REC;
		break;

	case MaxErrorCounter:
		entry->type = esc_report_s::ERRORCOUNT_TYPE_CAN_TEC_REC_MAX;
		break;

	case PackedErrorCounters:
		entry->type = esc_report_s::ERRORCOUNT_TYPE_CAN_TEC_REC_PACKED;
		break;

	case CumulativeErrors:
		entry->type = esc_report_s::ERRORCOUNT_TYPE_CAN_ERRORS;
		break;

	default:
		// A mode added by a newer firmware: its meaning is not known, so the count is not evaluated
		entry->type = esc_report_s::ERRORCOUNT_TYPE_UNKNOWN;
		break;
	}
}

uint8_t UavcanEscController::error_count_type(uint8_t node_id) const
{
	const ErrorCountMeaning *entry = find_error_count_meaning(node_id);

	if (entry == nullptr || entry->state != ErrorCountMeaning::State::Resolved) {
		// Generic interpretation, also used while a vendor specific lookup is still in progress
		return esc_report_s::ERRORCOUNT_TYPE_ESC_FAULTS;
	}

	return entry->type;
}

void UavcanEscController::esc_status_extended_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::StatusExtended> &msg)
{
	uint8_t index = msg.esc_index;

	if (_param_uavcan_quirks & static_cast<int32_t>(Quirk::HobbywingEscIdx1)) {
		if (msg.esc_index == 0) {
			// non-compliant ESC firmware: esc_index 0 is not expected, ignore
			return;
		}

		index -= 1;
	}

	if (index < esc_status_s::CONNECTED_ESC_MAX) {
		esc_report_s &esc_report = _esc_status.esc[index];
		// published with the non-extended esc::Status
		esc_report.motor_temperature = msg.motor_temperature_degC;
		esc_report.esc_power = msg.input_pct;
	}
}

uint16_t UavcanEscController::check_escs_status()
{
	uint16_t esc_status_flags = 0;
	const hrt_abstime now = hrt_absolute_time();

	for (int index = 0; index < esc_status_s::CONNECTED_ESC_MAX; index++) {

		if (_esc_status.esc[index].timestamp > 0 && now - _esc_status.esc[index].timestamp < 1200_ms) {
			esc_status_flags |= (1 << index);
		}

	}

	return esc_status_flags;
}

uint32_t UavcanEscController::get_failures(uint8_t esc_index, uint8_t node_id)
{
	// Check DroneCAN node health of the ESC
	dronecan_node_status_s node_status{};
	uint8_t esc_node_id = node_id;
	uint8_t node_health = dronecan_node_status_s::HEALTH_OK;
	uint16_t vendor_specific_status_code = 0;

	for (auto &dronecan_node_status_sub : _dronecan_node_status_subs) {
		if (dronecan_node_status_sub.copy(&node_status)) {
			if (node_status.node_id == esc_node_id) {
				node_health = node_status.health;
				vendor_specific_status_code = node_status.vendor_specific_status_code;
				break;
			}
		}
	}

	uint32_t failures = 0;

	if ((node_health == dronecan_node_status_s::HEALTH_ERROR)
	    || (node_health == dronecan_node_status_s::HEALTH_CRITICAL)) {
		// Parse VertiQ = iq_motion ESC error flags
		device_information_s device_information{};

		if (_device_information_sub.copy(&device_information)
		    && device_information.device_type == device_information_s::DEVICE_TYPE_ESC
		    && device_information.device_id == esc_index
		    && strstr(device_information.name, "iq_motion") != nullptr) {
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
		}

		if (failures == 0) { // no specific error parsed
			failures = (1 << esc_report_s::FAILURE_GENERIC);
		}
	}

	return failures;
}
