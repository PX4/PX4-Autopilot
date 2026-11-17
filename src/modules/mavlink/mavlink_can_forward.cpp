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
#include <lib/can_forward/CanForward.hpp>
#include <lib/mathlib/mathlib.h>

px4::atomic<MavlinkCanForward *> MavlinkCanForward::_link{nullptr};

MavlinkCanForward::~MavlinkCanForward()
{
	MavlinkCanForward *self = this;
	_link.compare_exchange(&self, nullptr);
}

bool MavlinkCanForward::claimLink()
{
	MavlinkCanForward *expected = nullptr;
	return _link.compare_exchange(&expected, this) || expected == this;
}

bool MavlinkCanForward::isClient(uint8_t system, uint8_t component) const
{
	return _receiver_status.active && _receiver_status.client_system == system
	       && _receiver_status.client_component == component;
}

bool MavlinkCanForward::handleCommand(float param1, uint8_t system, uint8_t component)
{
	_receiver_status_sub.update(&_receiver_status);

	if (_receiver_status.active) {
		// the session stays on its link: a keepalive from another link would keep it alive there
		return isClient(system, component) ? claimLink() : _link.load() == this;
	}

	if (param1 >= 0.5f) {
		// the first link to ask carries the session, if the driver accepts it
		claimLink();
	}

	return true;
}

void MavlinkCanForward::handleMessage(const mavlink_message_t &msg)
{
	_receiver_status_sub.update(&_receiver_status);

	if (!isClient(msg.sysid, msg.compid) || !claimLink()) {
		return;
	}

	switch (msg.msgid) {
	case MAVLINK_MSG_ID_CAN_FRAME: {
			mavlink_can_frame_t frame;
			mavlink_msg_can_frame_decode(&msg, &frame);
			queueFrame(frame.bus, frame.id, frame.len, frame.data);
			break;
		}

	case MAVLINK_MSG_ID_CANFD_FRAME: {
			mavlink_canfd_frame_t frame;
			mavlink_msg_canfd_frame_decode(&msg, &frame);
			queueFrame(frame.bus, frame.id, frame.len, frame.data);
			break;
		}

	case MAVLINK_MSG_ID_CAN_FILTER_MODIFY: {
			mavlink_can_filter_modify_t filter;
			mavlink_msg_can_filter_modify_decode(&msg, &filter);

			if (filter.bus > 0 && filter.num_ids <= sizeof(filter.ids) / sizeof(filter.ids[0])) {
				can_forward_filter_s out{};
				out.bus = filter.bus - 1;
				out.operation = filter.operation;
				out.num_ids = filter.num_ids;
				static_assert(sizeof(out.ids) == sizeof(filter.ids), "CAN_FILTER_MODIFY ids size mismatch");
				memcpy(out.ids, filter.ids, sizeof(out.ids));
				out.timestamp = hrt_absolute_time();
				_filter_pub.publish(out);
			}

			break;
		}

	default:
		break;
	}
}

void MavlinkCanForward::queueFrame(uint8_t bus, uint32_t id, uint8_t len, const uint8_t *data)
{
	if (!can_forward::isValidFrame(id, len)) {
		return;
	}

	if (_tx_batch.count > 0 && _tx_batch.bus != bus) {
		flushToBus();
	}

	const uint8_t i = _tx_batch.count;
	_tx_batch.bus = bus;
	_tx_batch.id[i] = id;
	_tx_batch.len[i] = len;
	memcpy(&_tx_batch.data[i * 8], data, len);
	_tx_batch.count++;

	if (_tx_batch.count >= can_forward_frames_s::MAX_FRAMES) {
		flushToBus();
	}
}

void MavlinkCanForward::flushToBus()
{
	if (_tx_batch.count > 0) {
		_tx_batch.timestamp = hrt_absolute_time();
		_tx_pub.publish(_tx_batch);
		_tx_batch.count = 0;
	}
}

bool MavlinkCanForward::update()
{
	_status_sub.update(&_status);

	if (_link.load() == this) {
		if (_status.active) {
			_carried = true;
			_claimed_since = 0;
			return true;
		}

		const hrt_abstime now = hrt_absolute_time();

		if (_claimed_since == 0) {
			_claimed_since = now;
		}

		// free the link for the next session: when this one ended, or when the request was not accepted
		if (_carried || now > _claimed_since + ClaimTimeout) {
			MavlinkCanForward *self = this;
			_link.compare_exchange(&self, nullptr);
			_carried = false;
			_claimed_since = 0;
		}

	} else {
		_carried = false;
		_claimed_since = 0;
	}

	// not for this link: drop
	_rx_pending = false;

	while (_rx_sub.update(&_rx_batch)) {
		_rx_seen = true;
	}

	return false;
}

bool MavlinkCanForward::nextRxBatch()
{
	const unsigned last_generation = _rx_sub.get_last_generation();

	if (!_rx_sub.update(&_rx_batch)) {
		return false;
	}

	if (_rx_seen) {
		_rx_lost += _rx_sub.get_last_generation() - last_generation - 1;
	}

	_rx_seen = true;
	_rx_index = 0;
	_rx_pending = true;
	return true;
}

void MavlinkCanForward::sendToGcs(mavlink_channel_t channel, unsigned message_size, unsigned &budget)
{
	while (_rx_pending || nextRxBatch()) {
		const uint8_t count = math::min(_rx_batch.count, can_forward_frames_s::MAX_FRAMES);

		for (; _rx_index < count; _rx_index++) {
			if (budget < message_size) {
				return;
			}

			mavlink_can_frame_t msg{};
			msg.target_component = static_cast<uint8_t>(_status.client_component);
			msg.bus = _rx_batch.bus;
			msg.id = _rx_batch.id[_rx_index];
			msg.len = math::min(_rx_batch.len[_rx_index], static_cast<uint8_t>(sizeof(msg.data)));
			memcpy(msg.data, &_rx_batch.data[_rx_index * 8], msg.len);
			mavlink_msg_send_struct_target(channel, CAN_FRAME, &msg, _status.client_system);
			budget -= message_size;
		}

		_rx_pending = false;
	}
}
