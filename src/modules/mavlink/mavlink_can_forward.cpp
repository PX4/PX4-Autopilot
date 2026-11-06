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

#include "mavlink_can_forward.h"

#include <drivers/drv_hrt.h>

px4::atomic<uint32_t> MavlinkCanForward::_request_sequence{0};

MavlinkCanForward::MavlinkCanForward()
{
	pthread_mutex_init(&_mutex, nullptr);
}

MavlinkCanForward::~MavlinkCanForward()
{
	pthread_mutex_destroy(&_mutex);
}

bool MavlinkCanForward::bound() const
{
	pthread_mutex_lock(&_mutex);
	const bool bound = _bound;
	pthread_mutex_unlock(&_mutex);
	return bound;
}

MavlinkCanForward::Counters MavlinkCanForward::counters() const
{
	pthread_mutex_lock(&_mutex);
	const Counters counters = _counters;
	pthread_mutex_unlock(&_mutex);
	return counters;
}

void MavlinkCanForward::handleCommand(uint32_t sysid, uint8_t compid, float param1)
{
	pthread_mutex_lock(&_mutex);
	_pending = true;
	_pending_sysid = sysid;
	_pending_compid = compid;
	// 1-based bus, validated by the driver; only used until the driver's status arrives
	_pending_bus = (param1 >= 1.f && param1 < 256.f) ? static_cast<uint8_t>(param1 + 0.5f) - 1 : 0;
	_pending_stop = !(param1 >= 0.5f); // also NaN, which the driver denies
	_pending_sequence = _request_sequence.fetch_add(1) + 1;
	pthread_mutex_unlock(&_mutex);
}

void MavlinkCanForward::handleAck(const vehicle_command_ack_s &ack)
{
	pthread_mutex_lock(&_mutex);

	if (_pending && ack.target_system == _pending_sysid && ack.target_component == _pending_compid) {
		if (_pending_sequence != _request_sequence.load()) {
			// a newer request came in on another instance, that one gets the frames
			_bound = false;
			_pending = false;

		} else if (ack.result == vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED && _pending_stop) {
			_bound = false;

		} else if (ack.result == vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED) {
			if (!_bound || _sysid != _pending_sysid || _compid != _pending_compid) {
				_bound_since = ack.timestamp;
			}

			_bound = true;
			_sysid = _pending_sysid;
			_compid = _pending_compid;
			_bus = _pending_bus;
		}

		// otherwise (denied, armed) the binding follows the session in can_forward_status
	}

	pthread_mutex_unlock(&_mutex);
}

bool MavlinkCanForward::isOwner(uint32_t sysid, uint8_t compid) const
{
	return _bound && sysid == _sysid && compid == _compid;
}

void MavlinkCanForward::unbind()
{
	_bound = false;
}

void MavlinkCanForward::handleMessage(const mavlink_message_t &msg)
{
	switch (msg.msgid) {
	case MAVLINK_MSG_ID_CAN_FRAME: {
			mavlink_can_frame_t frame;
			mavlink_msg_can_frame_decode(&msg, &frame);
			handleCanFrame(msg.sysid, msg.compid, frame.bus, frame.id, frame.len, frame.data, sizeof(frame.data));
			break;
		}

	case MAVLINK_MSG_ID_CANFD_FRAME: {
			// classic frames only: the DroneCAN stack has no CAN FD support, longer frames are rejected
			mavlink_canfd_frame_t frame;
			mavlink_msg_canfd_frame_decode(&msg, &frame);
			handleCanFrame(msg.sysid, msg.compid, frame.bus, frame.id, frame.len, frame.data, sizeof(frame.data));
			break;
		}

	case MAVLINK_MSG_ID_CAN_FILTER_MODIFY: {
			mavlink_can_filter_modify_t filter;
			mavlink_msg_can_filter_modify_decode(&msg, &filter);
			handleFilter(msg.sysid, msg.compid, filter);
			break;
		}

	default:
		break;
	}
}

void MavlinkCanForward::handleCanFrame(uint32_t sysid, uint8_t compid, uint8_t bus, uint32_t id, uint8_t len,
				       const uint8_t *data, uint8_t data_size)
{
	pthread_mutex_lock(&_mutex);
	const bool owner = isOwner(sysid, compid);
	const uint8_t session_bus = _bus;

	if (!owner) {
		_counters.dropped_not_owner++;
		pthread_mutex_unlock(&_mutex);
		return;
	}

	can_forward::Frame frame;

	if (bus != session_bus
	    || can_forward::decodeFrame(id, len, data, data_size, frame) != can_forward::DecodeResult::Ok) {
		_counters.dropped_invalid++;
		pthread_mutex_unlock(&_mutex);
		return;
	}

	_counters.frames_from_gcs++;
	pthread_mutex_unlock(&_mutex);

	if (_tx_batch.count > 0 && _tx_batch.bus != bus) {
		flushToBus(hrt_absolute_time());
	}

	const uint8_t i = _tx_batch.count;
	_tx_batch.bus = bus;
	_tx_batch.id[i] = frame.id;
	_tx_batch.len[i] = frame.len;
	memcpy(&_tx_batch.data[i * 8], frame.data, sizeof(frame.data));
	_tx_batch.count++;

	if (_tx_batch.count >= can_forward_frames_s::MAX_FRAMES) {
		flushToBus(hrt_absolute_time());
	}
}

void MavlinkCanForward::handleFilter(uint32_t sysid, uint8_t compid, const mavlink_can_filter_modify_t &filter)
{
	pthread_mutex_lock(&_mutex);
	const bool owner = isOwner(sysid, compid);
	const bool pending = _pending && sysid == _pending_sysid && compid == _pending_compid;

	// clients send the filter right after the request, possibly before the ack binds them
	if (!owner && !pending) {
		_counters.dropped_not_owner++;
		pthread_mutex_unlock(&_mutex);
		return;
	}

	pthread_mutex_unlock(&_mutex);

	// bus is 1-based in CAN_FILTER_MODIFY
	if (filter.bus == 0 || filter.num_ids > sizeof(filter.ids) / sizeof(filter.ids[0])) {
		pthread_mutex_lock(&_mutex);
		_counters.dropped_invalid++;
		pthread_mutex_unlock(&_mutex);
		return;
	}

	can_forward_filter_s out{};
	out.bus = filter.bus - 1;
	out.operation = filter.operation;
	out.num_ids = filter.num_ids;
	static_assert(sizeof(out.ids) == sizeof(filter.ids), "CAN_FILTER_MODIFY ids size mismatch");
	memcpy(out.ids, filter.ids, sizeof(out.ids));
	out.timestamp = hrt_absolute_time();
	_filter_pub.publish(out);
}

void MavlinkCanForward::flushToBus(hrt_abstime now)
{
	if (_tx_batch.count > 0) {
		_tx_batch.timestamp = now;
		_tx_pub.publish(_tx_batch);
		_tx_batch.count = 0;
		memset(_tx_batch.data, 0, sizeof(_tx_batch.data));
	}
}

void MavlinkCanForward::update(Sender &sender)
{
	can_forward_status_s status;

	while (_status_sub.update(&status)) {
		pthread_mutex_lock(&_mutex);

		// a status from before the ack that bound us does not end the binding
		if (_bound && status.timestamp >= _bound_since) {
			if (status.state == can_forward_status_s::STATE_IDLE
			    || status.requester_sysid != _sysid || status.requester_compid != _compid) {
				unbind();

			} else {
				_bus = status.bus;
			}
		}

		pthread_mutex_unlock(&_mutex);
	}

	pthread_mutex_lock(&_mutex);
	const bool bound = _bound;
	const uint32_t sysid = _sysid;
	const uint8_t compid = _compid;
	const hrt_abstime bound_since = _bound_since;
	pthread_mutex_unlock(&_mutex);

	if (!bound) {
		// keep up with the topic, so a later binding does not start with a burst of old frames
		_rx_pending = false;

		while (_rx_sub.update(&_rx_batch)) {}

		return;
	}

	uint32_t sent = 0;
	uint32_t lost = 0;

	while (true) {
		if (!_rx_pending) {
			const unsigned last_generation = _rx_sub.get_last_generation();

			if (!_rx_sub.update(&_rx_batch)) {
				break;
			}

			const unsigned gap = _rx_sub.get_last_generation() - last_generation - 1;

			if (gap > 0 && gap < 1000) {
				lost += gap;
			}

			// frames from before the binding were meant for an earlier client
			if (_rx_batch.timestamp < bound_since) {
				continue;
			}

			_rx_index = 0;
			_rx_pending = true;
		}

		const uint8_t count = (_rx_batch.count < can_forward_frames_s::MAX_FRAMES) ? _rx_batch.count
				      : can_forward_frames_s::MAX_FRAMES;

		while (_rx_index < count) {
			if (sender.freeTxBuf() < CanFrameMessageSize) {
				// link is full, continue with this frame next time
				break;
			}

			mavlink_can_frame_t msg{};
			msg.target_component = compid;
			msg.bus = _rx_batch.bus;
			msg.id = _rx_batch.id[_rx_index];
			msg.len = (_rx_batch.len[_rx_index] <= sizeof(msg.data)) ? _rx_batch.len[_rx_index] : sizeof(msg.data);
			memcpy(msg.data, &_rx_batch.data[_rx_index * 8], msg.len);
			sender.sendCanFrame(msg, sysid);
			_rx_index++;
			sent++;
		}

		if (_rx_index < count) {
			break;
		}

		_rx_pending = false;
	}

	pthread_mutex_lock(&_mutex);
	_counters.frames_to_gcs += sent;
	_counters.dropped_to_gcs += lost;
	pthread_mutex_unlock(&_mutex);
}
