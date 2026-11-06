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

#include <px4_platform_common/log.h>

using can_forward::CanForwardSession;

void CanForwarder::init(can_forward::ForwardingCanDriver *driver)
{
	_driver = driver;
	_num_buses = (driver != nullptr) ? driver->getNumIfaces() : 0;
	_tx_sub.registerCallback();
}

void CanForwarder::deinit()
{
	_tx_sub.unregisterCallback();

	if (_driver != nullptr) {
		_driver->configure(-1, false, nullptr, nullptr, 0);
	}
}

void CanForwarder::onFrameToGcs(uint8_t iface_index, const uavcan::CanFrame &frame)
{
	if (_rx_batch.count > 0 && _rx_batch.bus != iface_index) {
		flushRx(hrt_absolute_time());
	}

	const uint8_t i = _rx_batch.count;
	_rx_batch.bus = iface_index;
	_rx_batch.id[i] = frame.id;
	_rx_batch.len[i] = frame.dlc;
	memcpy(&_rx_batch.data[i * 8], frame.data, frame.dlc);
	_rx_batch.count++;

	if (_rx_batch.count >= can_forward_frames_s::MAX_FRAMES) {
		flushRx(hrt_absolute_time());
	}
}

void CanForwarder::flushRx(hrt_abstime now)
{
	if (_rx_batch.count > 0) {
		_rx_batch.timestamp = now;
		_rx_pub.publish(_rx_batch);
		_rx_batch.count = 0;
		memset(_rx_batch.data, 0, sizeof(_rx_batch.data));
	}
}

void CanForwarder::handleCommand(const vehicle_command_s &cmd, hrt_abstime now, bool blocked)
{
	const CanForwardSession::State previous = _session.state();
	CanForwardSession::Result result = CanForwardSession::Result::Denied;

	// an interface the board does not enable (no driver behind it) cannot be forwarded
	const bool disabled_bus = cmd.param1 >= 0.5f && cmd.param1 < _num_buses + 0.5f
				  && _driver->getIface(static_cast<uint8_t>(cmd.param1 + 0.5f) - 1) == nullptr;

	if (!disabled_bus) {
		result = _session.request(cmd.param1, _num_buses, blocked, now, cmd.source_system, cmd.source_component);
	}

	vehicle_command_ack_s ack{};
	ack.command = cmd.command;
	ack.target_system = cmd.source_system;
	ack.target_component = cmd.source_component;

	switch (result) {
	case CanForwardSession::Result::Accepted:
		ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
		break;

	case CanForwardSession::Result::TemporarilyRejected:
		ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
		break;

	case CanForwardSession::Result::Denied:
	default:
		ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED;
		break;
	}

	ack.timestamp = hrt_absolute_time();
	_ack_pub.publish(ack);

	if (_session.state() != previous) {
		_status_changed = true;

		if (_session.active() && previous == CanForwardSession::State::Idle) {
			PX4_INFO("CAN forwarding to %" PRIu32 "/%" PRIu8 " on CAN%" PRIu8, _session.sysid(), _session.compid(),
				 static_cast<uint8_t>(_session.bus() + 1));

		} else if (!_session.active()) {
			PX4_INFO("CAN forwarding stopped");
		}
	}
}

void CanForwarder::update(hrt_abstime now, bool blocked, uint8_t local_node_id, uavcan::CanIOManager &canio)
{
	if (_driver == nullptr) {
		return;
	}

	_blocked = blocked;

	// handled outside the driver's command gate, so keepalives are never held back
	vehicle_command_s cmd;

	while (_vcmd_sub.update(&cmd)) {
		if (cmd.command == vehicle_command_s::VEHICLE_CMD_CAN_FORWARD) {
			handleCommand(cmd, now, blocked);
		}
	}

	if (_session.update(blocked, now)) {
		_status_changed = true;

		if (!_session.active()) {
			PX4_INFO("CAN forwarding timed out");
		}
	}

	can_forward_filter_s filter;

	while (_filter_sub.update(&filter)) {
		if (filter.bus >= _num_buses) {
			continue;
		}

		const auto result = _filter.apply(filter.operation, filter.ids, filter.num_ids);

		if (result != can_forward::CanForwardFilter::Result::Ok) {
			PX4_DEBUG("CAN forward filter rejected (%d)", static_cast<int>(result));
		}

		_status_changed = true;
	}

	configureDriver(local_node_id);
	pushFramesToBus(now, canio);
}

void CanForwarder::configureDriver(uint8_t local_node_id)
{
	const int8_t bus = _session.active() ? static_cast<int8_t>(_session.bus()) : -1;
	_driver->configure(bus, _session.allowInject(_session.bus()), &_filter, this, local_node_id);
}

void CanForwarder::pushFramesToBus(hrt_abstime now, uavcan::CanIOManager &canio)
{
	while (true) {
		if (!_tx_pending) {
			const unsigned last_generation = _tx_sub.get_last_generation();

			if (!_tx_sub.update(&_tx_batch)) {
				return;
			}

			// the subscription skips batches the queue could not hold
			const unsigned lost = _tx_sub.get_last_generation() - last_generation - 1;

			if (_tx_received_any && lost > 0 && lost < 1000) {
				_dropped_queue += lost;
			}

			_tx_received_any = true;
			_tx_index = 0;
			_tx_pending = true;
		}

		const uint8_t count = (_tx_batch.count < can_forward_frames_s::MAX_FRAMES) ? _tx_batch.count
				      : can_forward_frames_s::MAX_FRAMES;

		if (!_session.allowInject(_tx_batch.bus) || canio.getCanDriver().getIface(_tx_batch.bus) == nullptr) {
			_dropped_blocked += count - _tx_index;
			_tx_pending = false;
			continue;
		}

		if (now > _tx_batch.timestamp + MaxBatchAge) {
			_dropped_stale += count - _tx_index;
			_tx_pending = false;
			continue;
		}

		can_forward::Frame frames[can_forward_frames_s::MAX_FRAMES];

		for (uint8_t i = _tx_index; i < count; i++) {
			frames[i].id = _tx_batch.id[i];
			frames[i].len = (_tx_batch.len[i] < can_forward::CanMaxDataLen) ? _tx_batch.len[i] : can_forward::CanMaxDataLen;
			memcpy(frames[i].data, &_tx_batch.data[i * 8], sizeof(frames[i].data));
		}

		const uint8_t sent = can_forward::pumpToBus(canio, _tx_batch.bus, &frames[_tx_index], count - _tx_index,
				     uavcan::MonotonicTime::fromUSec(now + TxDeadline));
		_frames_from_gcs += sent;
		_tx_index += sent;

		if (_tx_index < count) {
			// the bus is busy, continue in the next cycle
			return;
		}

		_tx_pending = false;
	}
}

void CanForwarder::publish(hrt_abstime now)
{
	if (_driver == nullptr) {
		return;
	}

	flushRx(now);

	if (_status_changed || (_session.active() && now >= _last_status_pub + 1000 * 1000)) {
		const auto &counters = _driver->counters();

		can_forward_status_s status{};
		status.state = static_cast<uint8_t>(_session.state());
		status.bus = _session.bus();
		status.num_buses = _num_buses;
		status.requester_sysid = _session.sysid();
		status.requester_compid = _session.compid();
		status.num_filter_ids = _filter.size();
		status.frames_to_gcs = counters.frames_to_gcs;
		status.frames_from_gcs = _frames_from_gcs;
		status.dropped_blocked = _dropped_blocked + counters.dropped_blocked;
		status.dropped_stale = _dropped_stale;
		status.dropped_queue = _dropped_queue;
		status.dropped_inject = counters.dropped_inject_queue;
		status.timestamp = now;
		_status_pub.publish(status);

		_status_changed = false;
		_last_status_pub = now;
	}
}

void CanForwarder::printStatus() const
{
	static constexpr const char *States[] = {"idle", "active", "monitor only (armed)"};

	printf("CAN forwarding over MAVLink: %s", States[static_cast<uint8_t>(_session.state()) % 3]);

	if (_session.active()) {
		printf(" on CAN%u to %" PRIu32 "/%u", unsigned(_session.bus() + 1), _session.sysid(), unsigned(_session.compid()));
	}

	printf("\n");

	if (_driver != nullptr) {
		const auto &counters = _driver->counters();
		printf("\tfilter IDs: %u\n", unsigned(_filter.size()));
		printf("\tto GCS: %" PRIu32 " frames, from GCS: %" PRIu32 " frames\n", counters.frames_to_gcs, _frames_from_gcs);
		printf("\tdropped from GCS: %" PRIu32 " blocked/inactive, %" PRIu32 " stale, %" PRIu32 " lost batches, %" PRIu32
		       " not fed to local node\n", _dropped_blocked + counters.dropped_blocked, _dropped_stale, _dropped_queue,
		       counters.dropped_inject_queue);
	}
}
