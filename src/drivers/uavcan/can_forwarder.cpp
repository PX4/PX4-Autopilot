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

ForwardingCanDriver::ForwardingCanDriver(uavcan::ICanDriver &inner) : _inner(inner)
{
	_num_ifaces = math::min(inner.getNumIfaces(), static_cast<uint8_t>(uavcan::MaxCanIfaces));

	for (uint8_t i = 0; i < _num_ifaces; i++) {
		_ifaces[i].owner = this;
		_ifaces[i].index = i;
	}
}

uavcan::ICanIface *ForwardingCanDriver::getIface(uint8_t iface_index)
{
	if (iface_index >= _num_ifaces) {
		return nullptr;
	}

	_ifaces[iface_index].inner = _inner.getIface(iface_index);
	return (_ifaces[iface_index].inner != nullptr) ? &_ifaces[iface_index] : nullptr;
}

int16_t ForwardingCanDriver::Iface::send(const uavcan::CanFrame &frame, uavcan::MonotonicTime tx_deadline,
		uavcan::CanIOFlags flags)
{
	CanForwarder *forwarder = owner->_forwarder;

	if (flags & CanIOFlagForwarded) {
		if (forwarder == nullptr || !forwarder->allowInject(index)) {
			return 1;
		}

		return inner->send(frame, tx_deadline, flags & ~CanIOFlagForwarded);
	}

	const int16_t res = inner->send(frame, tx_deadline, flags);

	if (res > 0 && forwarder != nullptr) {
		forwarder->mirror(index, frame);
	}

	return res;
}

int16_t ForwardingCanDriver::Iface::receive(uavcan::CanFrame &out_frame, uavcan::MonotonicTime &out_ts_monotonic,
		uavcan::UtcTime &out_ts_utc, uavcan::CanIOFlags &out_flags)
{
	const int16_t res = inner->receive(out_frame, out_ts_monotonic, out_ts_utc, out_flags);

	if (res > 0 && !(out_flags & uavcan::CanIOFlagLoopback) && owner->_forwarder != nullptr) {
		owner->_forwarder->mirror(index, out_frame);
	}

	return res;
}

void CanForwarder::init(ForwardingCanDriver *driver, bool config_only)
{
	_driver = driver;
	_config_only = config_only;

	if (_driver != nullptr) {
		_driver->setForwarder(this);
		_tx_sub.registerCallback();
	}
}

void CanForwarder::deinit()
{
	_tx_sub.unregisterCallback();

	if (_driver != nullptr) {
		_driver->setForwarder(nullptr);
		_driver = nullptr;
	}

	_bus = -1;
}

void CanForwarder::mirror(uint8_t bus, const uavcan::CanFrame &frame)
{
	if (bus != _bus || !_filter.matches(frame.id)) {
		return;
	}

	if (_rx_batch.count > 0 && _rx_batch.bus != bus) {
		flushRx(hrt_absolute_time());
	}

	const uint8_t i = _rx_batch.count;

	if (i == 0) {
		_rx_batch.timestamp = hrt_absolute_time();
	}

	_rx_batch.bus = bus;
	_rx_batch.id[i] = frame.id;
	_rx_batch.len[i] = frame.dlc;
	memcpy(&_rx_batch.data[i * 8], frame.data, frame.dlc);
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
	vehicle_command_ack_s ack{};
	ack.command = cmd.command;
	ack.target_system = cmd.source_system;
	ack.target_component = cmd.source_component;
	ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED;

	const int bus = PX4_ISFINITE(cmd.param1) ? static_cast<int>(roundf(cmd.param1)) : -1;

	if (bus == 0) {
		if (active()) {
			PX4_INFO("CAN forwarding stopped");
		}

		_bus = -1;
		ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;

	} else if (_driver != nullptr && bus > 0 && bus <= _driver->getNumIfaces() && _driver->getIface(bus - 1) != nullptr) {
		if (_bus != bus - 1) {
			PX4_INFO("CAN forwarding on CAN%d to %" PRIu32 "/%u", bus, cmd.source_system, unsigned(cmd.source_component));
			_bus = bus - 1;
			_filter.clear();
		}

		_last_keepalive = now;
		ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
	}

	ack.timestamp = hrt_absolute_time();
	_ack_pub.publish(ack);
}

void CanForwarder::update(hrt_abstime now, uavcan::CanIOManager &canio)
{
	vehicle_command_s cmd;

	while (_vcmd_sub.update(&cmd)) {
		if (cmd.command == vehicle_command_s::VEHICLE_CMD_CAN_FORWARD) {
			handleCommand(cmd, now);
		}
	}

	if (_driver == nullptr) {
		return;
	}

	actuator_armed_s armed;

	if (_armed_sub.update(&armed)) {
		_armed = armed.armed || armed.prearmed;
	}

	if (active() && now > _last_keepalive + KeepaliveTimeout) {
		PX4_INFO("CAN forwarding timed out");
		_bus = -1;
	}

	can_forward_filter_s filter;

	while (_filter_sub.update(&filter)) {
		if (filter.bus == _bus) {
			_filter.apply(filter.operation, filter.ids, math::min(filter.num_ids, static_cast<uint8_t>(16)));
		}
	}

	pushFramesToBus(now, canio);
}

void CanForwarder::pushFramesToBus(hrt_abstime now, uavcan::CanIOManager &canio)
{
	uint8_t sent = 0;

	while (true) {
		if (!_tx_pending) {
			if (!_tx_sub.update(&_tx_batch)) {
				return;
			}

			_tx_index = 0;
			_tx_pending = true;
		}

		const uint8_t count = math::min(_tx_batch.count, can_forward_frames_s::MAX_FRAMES);
		const uint8_t bus = _tx_batch.bus;

		if (!allowInject(bus) || now > _tx_batch.timestamp + MaxBatchAge) {
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
					 static_cast<uint8_t>(1U << bus), uavcan::CanTxQueue::Persistent, ForwardingCanDriver::CanIOFlagForwarded);
			_tx_index++;
			_frames_from_gcs++;
			sent++;
		}

		_tx_pending = false;
	}
}

void CanForwarder::printStatus() const
{
	if (_driver == nullptr) {
		printf("CAN forwarding over MAVLink: disabled (UAVCAN_CAN_FWD)\n");
		return;
	}

	if (active()) {
		printf("CAN forwarding over MAVLink: CAN%d%s", _bus + 1, _armed ? " (armed, monitor only)" : "");

	} else {
		printf("CAN forwarding over MAVLink: idle");
	}

	printf("%s\n", _config_only ? " (configuration only)" : "");

	printf("\tto GCS: %" PRIu32 ", from GCS: %" PRIu32 ", dropped from GCS: %" PRIu32 ", filter IDs: %u\n",
	       _frames_to_gcs, _frames_from_gcs, _dropped_from_gcs, unsigned(_filter.size()));
}
