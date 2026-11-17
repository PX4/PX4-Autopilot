/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
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

#include "can_forwarder.hpp"

#include <lib/mathlib/mathlib.h>
#include <math.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
#include <systemlib/mavlink_log.h>
#include <uavcan/protocol/GetNodeInfo.hpp>
#include <uavcan/protocol/NodeStatus.hpp>
#include <uavcan/protocol/RestartNode.hpp>
#include <uavcan/protocol/file/BeginFirmwareUpdate.hpp>
#include <uavcan/protocol/file/GetInfo.hpp>
#include <uavcan/protocol/file/Read.hpp>
#include <uavcan/protocol/param/ExecuteOpcode.hpp>
#include <uavcan/protocol/param/GetSet.hpp>

// frames a ground station may send in configuration only mode (node info, parameters, restart, firmware update): no commands
static bool isConfigurationFrame(uint32_t can_id)
{
	const uint32_t id = can_id & can_forward::CanMaskExtId;

	if (!(can_id & can_forward::CanFlagEff) || (id & 0x7FU) == 0) {
		return false;
	}

	if (id & 0x80U) {
		switch (static_cast<uint8_t>(id >> 16)) {
		case uavcan::protocol::GetNodeInfo::DefaultDataTypeID:
		case uavcan::protocol::RestartNode::DefaultDataTypeID:
		case uavcan::protocol::param::ExecuteOpcode::DefaultDataTypeID:
		case uavcan::protocol::param::GetSet::DefaultDataTypeID:
		case uavcan::protocol::file::BeginFirmwareUpdate::DefaultDataTypeID:
		case uavcan::protocol::file::GetInfo::DefaultDataTypeID:
		case uavcan::protocol::file::Read::DefaultDataTypeID:
			return true;

		default:
			return false;
		}
	}

	return static_cast<uint16_t>(id >> 8) == uavcan::protocol::NodeStatus::DefaultDataTypeID;
}

void CanForwarder::init(uavcan::Dispatcher &dispatcher)
{
	_dispatcher = &dispatcher;
	_param_can_fwd = param_find("UAVCAN_CAN_FWD");
	_tx_sub.registerCallback();
}

void CanForwarder::deinit()
{
	_tx_sub.unregisterCallback();

	if (_dispatcher != nullptr) {
		_session.stop();
		applySession(hrt_absolute_time());
		_dispatcher = nullptr;
	}
}

void CanForwarder::handleRxFrame(const uavcan::CanRxFrame &frame, uavcan::CanIOFlags flags)
{
	if (!(flags & uavcan::CanIOFlagLoopback) && frame.iface_index == _session.bus()) {
		mirror(frame);
	}
}

void CanForwarder::handleTxFrame(const uavcan::CanFrame &frame, uint8_t iface_mask)
{
	if (_session.active() && (iface_mask & (1U << _session.bus()))) {
		mirror(frame);
	}
}

void CanForwarder::mirror(const uavcan::CanFrame &frame)
{
	if (!_filter.matches(frame.id)) {
		return;
	}

	const uint8_t i = _rx_batch.count;

	if (i == 0) {
		_rx_batch.timestamp = hrt_absolute_time();
	}

	_rx_batch.bus = _session.bus();
	_rx_batch.id[i] = frame.id;
	// the CAN drivers pass the DLC of the wire, up to 15
	_rx_batch.len[i] = math::min(frame.dlc, can_forward::CanMaxDataLen);
	memcpy(&_rx_batch.data[i * 8], frame.data, _rx_batch.len[i]);
	_rx_batch.count++;
	_frames_to_gcs++;

	if (_rx_batch.count >= can_forward_frames_s::MAX_FRAMES) {
		flushRx(hrt_absolute_time());
	}
}

void CanForwarder::publish(hrt_abstime now)
{
	if (_rx_batch.count > 0 && now >= _rx_batch.timestamp + RxBatchAge) {
		flushRx(now);
	}
}

void CanForwarder::flushRx(hrt_abstime now)
{
	if (_rx_batch.count > 0) {
		_rx_batch.timestamp = now;
		_rx_pub.publish(_rx_batch);
		_rx_batch.count = 0;
	}
}

void CanForwarder::handleCommand(const vehicle_command_s &cmd, hrt_abstime now)
{
	int32_t mode = 0;
	(void)param_get(_param_can_fwd, &mode);
	_config_only = (mode != 2);

	const int bus = PX4_ISFINITE(cmd.param1) ? static_cast<int>(roundf(cmd.param1)) : -1;
	const bool allowed = mode != 0 && bus > 0 && bus <= _dispatcher->getCanIOManager().getNumIfaces()
			     && _dispatcher->getCanIOManager().getCanDriver().getIface(bus - 1) != nullptr;

	vehicle_command_ack_s ack{};
	ack.command = cmd.command;
	ack.target_system = cmd.source_system;
	ack.target_component = cmd.source_component;
	ack.result = _session.handleCommand(bus, cmd.source_system, cmd.source_component, allowed, now)
		     ? vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED : vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED;
	ack.timestamp = hrt_absolute_time();
	_ack_pub.publish(ack);
}

void CanForwarder::applySession(hrt_abstime now)
{
	const bool active = _session.active();
	const bool same = _session.bus() == _status.bus && _session.system() == _status.client_system
			  && _session.component() == _status.client_component;

	if (active == _status.active && (!active || same)) {
		return;
	}

	// nothing is kept for the next bus or client
	_filter.clear();
	_rx_batch.count = 0;

	if (active) {
		// the CAN path is only touched during a session, which cannot be active while armed
		_dispatcher->installRxFrameListener(this);
		_dispatcher->installTxFrameListener(this);
		mavlink_log_info(&_mavlink_log_pub, "CAN forwarding active on CAN%d", _session.bus() + 1);

	} else {
		_dispatcher->removeRxFrameListener();
		_dispatcher->removeTxFrameListener();
		mavlink_log_info(&_mavlink_log_pub, "CAN forwarding stopped");
	}

	_status.active = active;
	_status.bus = active ? _session.bus() : 0;
	_status.client_system = _session.system();
	_status.client_component = _session.component();
	_status.timestamp = now;
	_status_pub.publish(_status);
}

void CanForwarder::update(hrt_abstime now)
{
	if (_dispatcher == nullptr) {
		return;
	}

	actuator_armed_s armed;

	if (_armed_sub.update(&armed)) {
		_armed = armed.armed;
	}

	_session.update(now, _armed);

	vehicle_command_s cmd;

	while (_vcmd_sub.update(&cmd)) {
		if (cmd.command == vehicle_command_s::VEHICLE_CMD_CAN_FORWARD) {
			handleCommand(cmd, now);
		}
	}

	applySession(now);

	can_forward_filter_s filter;

	while (_filter_sub.update(&filter)) {
		if (filter.bus == _session.bus()) {
			_filter.apply(filter.operation, filter.ids, math::min(filter.num_ids, static_cast<uint8_t>(16)));
		}
	}

	pushFramesToBus(now);
}

bool CanForwarder::nextTxBatch()
{
	const unsigned last_generation = _tx_sub.get_last_generation();

	if (!_tx_sub.update(&_tx_batch)) {
		return false;
	}

	if (_tx_seen) {
		_lost_from_gcs += _tx_sub.get_last_generation() - last_generation - 1;
	}

	_tx_seen = true;
	_tx_index = 0;
	_tx_pending = true;
	return true;
}

void CanForwarder::pushFramesToBus(hrt_abstime now)
{
	uavcan::CanIOManager &canio = _dispatcher->getCanIOManager();
	uint8_t sent = 0;

	while (_tx_pending || nextTxBatch()) {
		const uint8_t count = math::min(_tx_batch.count, can_forward_frames_s::MAX_FRAMES);
		const uint8_t bus = _tx_batch.bus;

		if (bus != _session.bus() || now > _tx_batch.timestamp + MaxBatchAge) {
			_dropped_from_gcs += count - _tx_index;
			_tx_pending = false;
			continue;
		}

		// one frame at a time, so multi-frame transfers keep their order
		while (_tx_index < count) {
			// the controller's hardware TX FIFO accepts many frames: limit the burst
			if (sent >= MaxFramesPerUpdate || (canio.makePendingTxMask() & (1U << bus))) {
				return;
			}

			if (_config_only && !isConfigurationFrame(_tx_batch.id[_tx_index])) {
				_tx_index++;
				_dropped_from_gcs++;
				continue;
			}

			const uint8_t len = math::min(_tx_batch.len[_tx_index], can_forward::CanMaxDataLen);
			const uavcan::CanFrame frame(_tx_batch.id[_tx_index], &_tx_batch.data[_tx_index * 8], len);
			(void)canio.send(frame, uavcan::MonotonicTime::fromUSec(now + TxDeadline), uavcan::MonotonicTime(),
					 static_cast<uint8_t>(1U << bus), uavcan::CanTxQueue::Persistent, 0);

			_tx_index++;
			_frames_from_gcs++;
			sent++;
		}

		_tx_pending = false;
	}
}

void CanForwarder::printStatus() const
{
	if (_session.active()) {
		printf("CAN forwarding over MAVLink: CAN%d%s, client %" PRIu32 "/%u\n", _session.bus() + 1,
		       _config_only ? " (configuration only)" : "", _session.system(), unsigned(_session.component()));

	} else {
		printf("CAN forwarding over MAVLink: idle\n");
	}

	printf("\tto GCS: %" PRIu32 ", from GCS: %" PRIu32 ", dropped from GCS: %" PRIu32 ", lost batches from GCS: %" PRIu32
	       ", filter IDs: %u\n", _frames_to_gcs, _frames_from_gcs, _dropped_from_gcs, _lost_from_gcs, unsigned(_filter.size()));
}
