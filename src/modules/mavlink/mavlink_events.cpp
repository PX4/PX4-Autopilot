/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include "mavlink_events.h"
#include "mavlink_main.h"

#include <px4_log.h>
#include <errno.h>

namespace events
{

EventBuffer::EventBuffer(int capacity)
	: _capacity(capacity)
{
	pthread_mutex_init(&_mutex, nullptr);
}

EventBuffer::~EventBuffer()
{
	delete[](_events);
	pthread_mutex_destroy(&_mutex);
}

int EventBuffer::init()
{
	if (_events) { return 0; }

	_events = new Event[_capacity];

	if (!_events) {
		return -ENOMEM;
	}

	return 0;
}

void EventBuffer::insert_event(const Event &event)
{
	pthread_mutex_lock(&_mutex);
	_events[_next] = event;
	_next = (_next + 1) % _capacity;

	if (_size < _capacity) {
		++_size;
	}

	_latest_sequence.store(event.sequence);
	pthread_mutex_unlock(&_mutex);
}

uint16_t EventBuffer::get_oldest_sequence_after(uint16_t sequence) const
{
	pthread_mutex_lock(&_mutex);
	uint16_t sequence_ret = _latest_sequence.load();
	uint16_t min_diff = UINT16_MAX;

	for (int i = 0; i < _size; ++i) {
		uint16_t event_seq = _events[i].sequence;
		uint16_t diff = event_seq - sequence;

		// this handles wrap-arounds correctly
		if (event_seq != sequence && diff < min_diff) {
			min_diff = diff;
			sequence_ret = event_seq;
		}
	}

	pthread_mutex_unlock(&_mutex);
	return sequence_ret;
}
bool EventBuffer::get_event(uint16_t sequence, Event &event) const
{
	pthread_mutex_lock(&_mutex);

	for (int count = 0; count < _size; ++count) {
		int index = (_next - 1 - count + _size) % _size;

		if (_events[index].sequence == sequence) {
			event = _events[index];
			pthread_mutex_unlock(&_mutex);
			return true;
		}
	}

	pthread_mutex_unlock(&_mutex);
	return false;
}

int EventBuffer::size() const
{
	pthread_mutex_lock(&_mutex);
	int size = _size;
	pthread_mutex_unlock(&_mutex);
	return size;
}

SendProtocol::SendProtocol(EventBuffer &buffer, Mavlink &mavlink)
	: _buffer(buffer), _latest_sequence(buffer.get_latest_sequence()), _mavlink(mavlink)
{
}

void SendProtocol::update(const hrt_abstime &now)
{
	// check for new events in the buffer
	uint16_t buffer_sequence = _buffer.get_latest_sequence();
	int num_drops = 0;

	if (_first_event && buffer_sequence != _latest_sequence) {
		// If events are published before the first mavlink instance starts, they would be flagged as dropped
		// events as the buffer initializes the sequence to the global events::initial_event_sequence,
		// therefore reset the sequence.
		_latest_sequence = buffer_sequence - 1;
		_first_event = false;
		PX4_DEBUG("Setting initial sequence to %i", _latest_sequence);
		// Send sequence - only needed for SITL to ensure we send the sequence reset flag initially
		send_current_sequence(now, true);
	}

	while (_latest_sequence != buffer_sequence) {
		// only send if enough tx buffer space available
		if (_mavlink.get_free_tx_buf() < MAVLINK_MSG_ID_EVENT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) {
			break;
		}

		PX4_DEBUG("Changed seq: %i, latest: %i (mavlink instance: %i)", buffer_sequence, _latest_sequence,
			  _mavlink.get_instance_id());
		++_latest_sequence;
		Event e;

		if (_buffer.get_event(_latest_sequence, e)) {
			send_event(e);

		} else {
			if (num_drops == 0) { // avoid console spamming
				// This happens if either an event dropped in uORB or update() is not called fast enough
				PX4_WARN("Event dropped (%i, %i)", (int)_latest_sequence, buffer_sequence);
			}

			++num_drops;
		}
	}

	if (num_drops > 1) {
		PX4_WARN("Dropped %i events (seq=%i)", num_drops, _latest_sequence);
	}

	if (now - _last_current_sequence_sent > current_sequence_interval) {
		send_current_sequence(now);
	}
}

void SendProtocol::handle_request_event(const mavlink_message_t &msg) const
{
	mavlink_request_event_t request_event;
	mavlink_msg_request_event_decode(&msg, &request_event);
	Event e;

	const uint16_t end_sequence = request_event.last_sequence + 1;

	for (uint16_t sequence = request_event.first_sequence; sequence != end_sequence; ++sequence) {
		if (_buffer.get_event(sequence, e)) {
			PX4_DEBUG("sending requested event %i", sequence);
			send_event(e);

		} else {
			mavlink_response_event_error_t event_error{};
			event_error.target_system = msg.sysid;
			event_error.target_component = msg.compid;
			event_error.sequence = sequence;
			event_error.sequence_oldest_available = _buffer.get_oldest_sequence_after(sequence);
			event_error.reason = MAV_EVENT_ERROR_REASON_UNAVAILABLE;
			PX4_DEBUG("Event unavailable (seq=%i oldest=%i)", sequence, event_error.sequence_oldest_available);
			mavlink_msg_response_event_error_send_struct(_mavlink.get_channel(), &event_error);
		}
	}
}

void SendProtocol::send_event(const Event &event) const
{
	mavlink_event_t event_msg{};
	event_msg.event_time_boot_ms = event.timestamp_ms;
	event_msg.destination_component = MAV_COMP_ID_ALL;
	event_msg.destination_system = 0;
	event_msg.id = event.id;
	event_msg.sequence = event.sequence;
	event_msg.log_levels = event.log_levels;
	static_assert(sizeof(event_msg.arguments) >= sizeof(event.arguments), "MAVLink message arguments buffer too small");
	memcpy(&event_msg.arguments, event.arguments, sizeof(event.arguments));
	mavlink_msg_event_send_struct(_mavlink.get_channel(), &event_msg);

}

void SendProtocol::on_gcs_connected()
{
	send_current_sequence(hrt_absolute_time());
}

void SendProtocol::send_current_sequence(const hrt_abstime &now, bool force_reset)
{
	// only send if enough tx buffer space available
	if (_mavlink.get_free_tx_buf() < MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) {
		return;
	}

	_last_current_sequence_sent = now;
	mavlink_current_event_sequence_t current_event_seq;
	current_event_seq.sequence = _buffer.get_latest_sequence();
	current_event_seq.flags = (_buffer.size() == 0 || force_reset) ? MAV_EVENT_CURRENT_SEQUENCE_FLAGS_RESET : 0;
	mavlink_msg_current_event_sequence_send_struct(_mavlink.get_channel(), &current_event_seq);
}

} /* namespace events */
