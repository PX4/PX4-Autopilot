/****************************************************************************
 *
 *   Copyright (C) 2024-2026 PX4 Development Team. All rights reserved.
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
 * @file esc_hobbywing.cpp
 *
 * DroneCAN driver for HobbyWing ESCs (com.hobbywing.esc namespace).
 *
 * Protocol overview:
 *   Discovery (disarmed only):
 *     FC broadcasts GetEscID (DTID 20013) with payload[0]=0 every 1 second.
 *     Each ESC responds with payload[0]=its_node_id, payload[1]=throttle_channel (1-based).
 *     The node_id -> motor_slot table is built from these responses.
 *
 *   Command:
 *     com.hobbywing.esc.RawCommand (DTID 20100), int14[<=8] per-ESC commands.
 *     All ESCs receive the same broadcast; each takes the command at its
 *     throttle_channel index.
 *
 *   Telemetry:
 *     StatusMsg1 (DTID 20050): rpm, pwm, status
 *     StatusMsg2 (DTID 20051): input_voltage (0.1V), current (0.1A), temperature (degC)
 *     StatusMsg3 (DTID 20052): MOS_T, CAP_T, Motor_T (all degC)
 */

#include <px4_platform_common/px4_config.h>

#ifdef CONFIG_UAVCAN_HOBBYWING_ESC

#include "esc_hobbywing.hpp"
#include <systemlib/err.h>
#include <parameters/param.h>
#include <drivers/drv_hrt.h>
#include <cmath>

using namespace time_literals;

UavcanHobbyWingEscController::UavcanHobbyWingEscController(uavcan::INode &node) :
	_node(node),
	_pub_raw_cmd(node),
	_pub_get_esc_id(node),
	_sub_get_esc_id(node),
	_sub_status_msg1(node),
	_sub_status_msg2(node),
	_sub_status_msg3(node),
	_get_esc_id_timer(node)
{
	memset(_node_id_to_slot, SLOT_UNMAPPED, sizeof(_node_id_to_slot));
	_pub_raw_cmd.setPriority(uavcan::TransferPriority::NumericallyMin);  // Highest priority
}

int UavcanHobbyWingEscController::init()
{
	// Subscribe to GetEscID — handles both our own echoed requests (len=1, ignored)
	// and ESC responses (len=2: [node_id, throttle_channel])
	int res = _sub_get_esc_id.start(GetEscIDCbBinder(this, &UavcanHobbyWingEscController::handle_GetEscID));

	if (res < 0) {
		PX4_ERR("HobbyWing: GetEscID sub failed %i", res);
		return res;
	}

	res = _sub_status_msg1.start(StatusMsg1CbBinder(this, &UavcanHobbyWingEscController::handle_StatusMsg1));

	if (res < 0) {
		PX4_ERR("HobbyWing: StatusMsg1 sub failed %i", res);
		return res;
	}

	res = _sub_status_msg2.start(StatusMsg2CbBinder(this, &UavcanHobbyWingEscController::handle_StatusMsg2));

	if (res < 0) {
		PX4_ERR("HobbyWing: StatusMsg2 sub failed %i", res);
		return res;
	}

	res = _sub_status_msg3.start(StatusMsg3CbBinder(this, &UavcanHobbyWingEscController::handle_StatusMsg3));

	if (res < 0) {
		PX4_ERR("HobbyWing: StatusMsg3 sub failed %i", res);
		return res;
	}

	// Apply CAN interface mask from parameter (same as standard ESC driver)
	int32_t iface_mask{0xFF};

	if (param_get(param_find("UAVCAN_ESC_IFACE"), &iface_mask) == OK) {
		_pub_raw_cmd.getTransferSender().setIfaceMask(iface_mask);
		_pub_get_esc_id.getTransferSender().setIfaceMask(iface_mask);
	}

	// Start 1 Hz GetEscID discovery timer. The callback skips broadcasting
	// while the vehicle is armed to prevent mid-flight ESC stutter.
	_get_esc_id_timer.setCallback(TimerCbBinder(this, &UavcanHobbyWingEscController::get_esc_id_timer_cb));
	_get_esc_id_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(1000));

	_esc_status_pub.advertise();

	_initialized = true;
	return 0;
}

// ---------------------------------------------------------------------------
// GetEscID discovery
// ---------------------------------------------------------------------------

void UavcanHobbyWingEscController::get_esc_id_timer_cb(const uavcan::TimerEvent &)
{
	// Stop discovery broadcasts while armed — avoids any transient command glitch
	actuator_armed_s armed{};

	if (_actuator_armed_sub.copy(&armed) && armed.armed) {
		return;
	}

	com::hobbywing::esc::GetEscID msg{};
	msg.payload.push_back(0);  // 1-byte request; ESC replies with [node_id, channel]
	_pub_get_esc_id.broadcast(msg);
}

void UavcanHobbyWingEscController::handle_GetEscID(
	const uavcan::ReceivedDataStructure<com::hobbywing::esc::GetEscID> &msg)
{
	// ESC responses have 2 bytes: [node_id, throttle_channel (1-based)]
	// Our own broadcast requests echo back with 1 byte — ignore those
	if (msg.payload.size() != 2) {
		return;
	}

	const uint8_t esc_node_id      = msg.getSrcNodeID().get();
	const uint8_t throttle_channel = msg.payload[1];

	if (esc_node_id == 0 || esc_node_id >= sizeof(_node_id_to_slot)) {
		return;
	}

	if (throttle_channel == 0 || throttle_channel > MAX_ACTUATORS) {
		PX4_WARN("HobbyWing: node %u reported invalid throttle_channel %u",
			 esc_node_id, throttle_channel);
		return;
	}

	const uint8_t slot = throttle_channel - 1;  // convert to 0-based motor slot

	if (_node_id_to_slot[esc_node_id] != slot) {
		PX4_INFO("HobbyWing: ESC node %u -> motor slot %u", esc_node_id, slot);
		_node_id_to_slot[esc_node_id] = slot;
	}
}

// ---------------------------------------------------------------------------
// Command output
// ---------------------------------------------------------------------------

void UavcanHobbyWingEscController::update_outputs(float outputs[MAX_ACTUATORS], uint8_t output_array_size)
{
	const auto timestamp = _node.getMonotonicTime();

	if ((timestamp - _prev_cmd_pub).toUSec() < (1000000 / MAX_RATE_HZ)) {
		return;
	}

	_prev_cmd_pub = timestamp;

	com::hobbywing::esc::RawCommand msg{};

	for (unsigned i = 0; i < output_array_size; i++) {
		msg.command.push_back(static_cast<int>(lroundf(outputs[i])));
	}

	_pub_raw_cmd.broadcast(msg);
}

void UavcanHobbyWingEscController::set_rotor_count(uint8_t count)
{
	_rotor_count = count;
}

// ---------------------------------------------------------------------------
// Node ID -> slot helper
// ---------------------------------------------------------------------------

int8_t UavcanHobbyWingEscController::node_id_to_slot_idx(uint8_t node_id) const
{
	if (node_id == 0 || node_id >= sizeof(_node_id_to_slot)) {
		return -1;
	}

	const uint8_t slot = _node_id_to_slot[node_id];

	return (slot == SLOT_UNMAPPED || slot >= MAX_ACTUATORS) ? -1 : static_cast<int8_t>(slot);
}

// ---------------------------------------------------------------------------
// Telemetry callbacks
// ---------------------------------------------------------------------------

void UavcanHobbyWingEscController::handle_StatusMsg1(
	const uavcan::ReceivedDataStructure<com::hobbywing::esc::StatusMsg1> &msg)
{
	const int8_t slot = node_id_to_slot_idx(msg.getSrcNodeID().get());

	if (slot < 0) {
		return;  // ESC not yet discovered via GetEscID
	}

	esc_report_s &ref = _esc_status.esc[slot];
	ref.timestamp   = hrt_absolute_time();
	ref.esc_rpm     = static_cast<int32_t>(msg.rpm);
	// HobbyWing 'pwm' is a throttle echo, nominally 0-1000; scale to esc_power [0-100%]
	ref.esc_power   = static_cast<int8_t>(msg.pwm > 1000u ? 100 : msg.pwm / 10u);

	_esc_status.esc_count          = _rotor_count;
	_esc_status.counter            += 1;
	_esc_status.esc_connectiontype  = esc_status_s::ESC_CONNECTION_TYPE_CAN;
	_esc_status.esc_online_flags    = check_escs_status();
	_esc_status.esc_armed_flags     = 0;  // Not encoded in HobbyWing protocol
	_esc_status.timestamp           = ref.timestamp;
	_esc_status_pub.publish(_esc_status);
}

void UavcanHobbyWingEscController::handle_StatusMsg2(
	const uavcan::ReceivedDataStructure<com::hobbywing::esc::StatusMsg2> &msg)
{
	const int8_t slot = node_id_to_slot_idx(msg.getSrcNodeID().get());

	if (slot < 0) {
		return;
	}

	esc_report_s &ref   = _esc_status.esc[slot];
	ref.timestamp       = hrt_absolute_time();
	ref.esc_voltage     = msg.input_voltage * 0.1f;        // 0.1V units -> Volts
	ref.esc_current     = msg.current * 0.1f;              // 0.1A units -> Amps
	ref.esc_temperature = static_cast<float>(msg.temperature);  // degC (direct, no conversion)

	_esc_status.esc_count          = _rotor_count;
	_esc_status.counter            += 1;
	_esc_status.esc_connectiontype  = esc_status_s::ESC_CONNECTION_TYPE_CAN;
	_esc_status.esc_online_flags    = check_escs_status();
	_esc_status.timestamp           = ref.timestamp;
	_esc_status_pub.publish(_esc_status);
}

void UavcanHobbyWingEscController::handle_StatusMsg3(
	const uavcan::ReceivedDataStructure<com::hobbywing::esc::StatusMsg3> &msg)
{
	const int8_t slot = node_id_to_slot_idx(msg.getSrcNodeID().get());

	if (slot < 0) {
		return;
	}

	esc_report_s &ref     = _esc_status.esc[slot];
	ref.timestamp         = hrt_absolute_time();
	// Use the highest temperature among MOSFET and capacitor as the ESC temperature
	ref.esc_temperature   = static_cast<float>(msg.MOS_T > msg.CAP_T ? msg.MOS_T : msg.CAP_T);
	ref.motor_temperature = static_cast<int16_t>(msg.Motor_T);

	// Timestamp update keeps the freshness checker happy; StatusMsg1 drives the publish cycle.
}

// ---------------------------------------------------------------------------
// Online check
// ---------------------------------------------------------------------------

uint16_t UavcanHobbyWingEscController::check_escs_status()
{
	uint16_t online_flags = 0;
	const hrt_abstime now = hrt_absolute_time();

	for (int i = 0; i < MAX_ACTUATORS; i++) {
		if (_esc_status.esc[i].timestamp > 0 && now - _esc_status.esc[i].timestamp < 1200_ms) {
			online_flags |= (1 << i);
		}
	}

	return online_flags;
}

#endif // CONFIG_UAVCAN_HOBBYWING_ESC
