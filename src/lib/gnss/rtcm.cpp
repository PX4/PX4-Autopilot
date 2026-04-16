/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * @file rtcm.cpp
 *
 * RTCM3 protocol parsing implementation.
 */

#include "rtcm.h"
#include <cstring>

namespace gnss
{

uint32_t rtcm3_crc24q(const uint8_t *data, size_t len)
{
	uint32_t crc = 0;

	for (size_t i = 0; i < len; i++) {
		crc ^= static_cast<uint32_t>(data[i]) << 16;

		for (int j = 0; j < 8; j++) {
			crc <<= 1;

			if (crc & 0x1000000) {
				crc ^= RTCM3_CRC24Q_POLY;
			}
		}
	}

	return crc & 0xFFFFFF;
}

const uint8_t *GpsRtcmMessageAssembler::addPacket(uint8_t flags, const uint8_t *data, size_t len, uint64_t timestamp,
		size_t &out_len)
{
	out_len = 0;

	if (data == nullptr) {
		reset();
		return nullptr;
	}

	if (!gps_rtcm_is_fragmented(flags)) {
		reset();

		if (len > sizeof(_assembled_message)) {
			return nullptr;
		}

		memcpy(_assembled_message, data, len);
		out_len = len;

		return _assembled_message;
	}

	if (len > GPS_RTCM_MAX_FRAGMENT_LEN) {
		resetActiveState();
		return nullptr;
	}

	const uint8_t sequence_id = gps_rtcm_sequence_id(flags);
	const uint8_t fragment_id = gps_rtcm_fragment_id(flags);
	const bool is_non_full_fragment = len < GPS_RTCM_MAX_FRAGMENT_LEN;

	// Timeout, clear stale partial state.
	if (_active_sequence.active && (_active_sequence.timestamp != 0)
	    && (timestamp > _active_sequence.timestamp + GPS_RTCM_FRAGMENT_TIMEOUT_US)) {
		resetActiveState();
	}

	if (!_active_sequence.active || sequence_id != _active_sequence.sequence_id) {
		// A new sequence ID always starts with a clean assembly buffer.
		resetActiveState();
		_active_sequence.active = true;
		_active_sequence.sequence_id = sequence_id;
	}

	const bool last_fragment_known = (_active_sequence.last_fragment_id >= 0);

	// Once a non-full final fragment is known, any later fragment index is invalid.
	if (last_fragment_known && (fragment_id > _active_sequence.last_fragment_id)) {
		resetActiveState();
		return nullptr;
	}

	// A non-full fragment before the known end would contradict the current
	// end-of-message boundary. That can only come from malformed sender state.
	if (last_fragment_known && (fragment_id < _active_sequence.last_fragment_id) && is_non_full_fragment) {
		resetActiveState();
		return nullptr;
	}

	// The first fragment with a non-full payload defines the completion
	// boundary. A higher buffered fragment would contradict that boundary.
	if (is_non_full_fragment && hasFragmentAfter(fragment_id)) {
		resetActiveState();
		return nullptr;
	}

	FragmentSlot &fragment = _fragments[fragment_id];

	if (fragment.present && ((fragment.len != len) || (memcmp(fragment.data, data, len) != 0))) {
		// Reusing a slot with different bytes means the current buffer no longer
		// matches the incoming fragments. Drop the old partial state and treat
		// this packet as the start of a new buffer for the same sequence ID.
		resetActiveState();
		_active_sequence.active = true;
		_active_sequence.sequence_id = sequence_id;
	}

	memcpy(fragment.data, data, len);
	fragment.len = len;
	fragment.present = true;
	_active_sequence.timestamp = timestamp;

	// The buffer is complete when either all 4 fragments are present, or when
	// the first fragment with a non-full payload has been received and every
	// lower fragment ID is already present. Recording fragment 3 as the
	// boundary covers the "all 4 fragments" case.
	if (is_non_full_fragment || (fragment_id == GPS_RTCM_MAX_FRAGMENTS - 1)) {
		_active_sequence.last_fragment_id = fragment_id;
	}

	if (!isComplete()) {
		return nullptr;
	}

	size_t assembled_len = 0;

	for (size_t i = 0; i <= lastFragmentIndex(); i++) {
		const FragmentSlot &slot = _fragments[i];
		memcpy(&_assembled_message[assembled_len], slot.data, slot.len);
		assembled_len += slot.len;
	}

	out_len = assembled_len;

	resetActiveState();
	return _assembled_message;
}

void GpsRtcmMessageAssembler::reset()
{
	resetActiveState();
}

void GpsRtcmMessageAssembler::resetActiveState()
{
	for (FragmentSlot &fragment : _fragments) {
		fragment.present = false;
		fragment.len = 0;
	}

	_active_sequence = {};
}

bool GpsRtcmMessageAssembler::hasFragmentAfter(uint8_t fragment_id) const
{
	for (size_t i = fragment_id + 1; i < GPS_RTCM_MAX_FRAGMENTS; i++) {
		if (_fragments[i].present) {
			return true;
		}
	}

	return false;
}

bool GpsRtcmMessageAssembler::isComplete() const
{
	for (size_t i = 0; i <= lastFragmentIndex(); i++) {
		if (!_fragments[i].present) {
			return false;
		}
	}

	return true;
}

size_t GpsRtcmMessageAssembler::lastFragmentIndex() const
{
	// Per the MAVLink GPS_RTCM_DATA rule, a buffer is complete once either all
	// 4 fragments are present, or the first fragment with a non-full payload
	// has been received and every lower fragment ID is present. Until such a
	// fragment arrives, the receiver must assume that all 4 fragment slots may
	// still be used. A sender therefore needs an extra zero-length fragment
	// when a message is shorter than 720 bytes but its length is still an
	// exact multiple of 180.
	return (_active_sequence.last_fragment_id >= 0) ? static_cast<size_t>(_active_sequence.last_fragment_id) :
	       (GPS_RTCM_MAX_FRAGMENTS - 1);
}

bool GpsRtcmMessageFragmenter::startMessage(const uint8_t *data, size_t len)
{
	resetActiveState();

	if ((data == nullptr) || (len == 0) || (len > GPS_RTCM_MAX_MESSAGE_LEN)) {
		return false;
	}

	memcpy(_message, data, len);
	_active.len = len;
	_active.total_packets = packetCountForLength(len);
	_active.sequence_id = _next_sequence_id;
	_next_sequence_id = (_next_sequence_id + 1) & GPS_RTCM_FLAG_SEQUENCE_ID_MASK;

	return true;
}

bool GpsRtcmMessageFragmenter::nextPacket(uint8_t &out_flags, const uint8_t *&out_data, size_t &out_len)
{
	out_flags = 0;
	out_data = nullptr;
	out_len = 0;

	if (!active()) {
		return false;
	}

	out_flags = ((_active.next_fragment_id & GPS_RTCM_FLAG_FRAGMENT_ID_MASK) << GPS_RTCM_FLAG_FRAGMENT_ID_SHIFT) |
		    ((_active.sequence_id & GPS_RTCM_FLAG_SEQUENCE_ID_MASK) << GPS_RTCM_FLAG_SEQUENCE_ID_SHIFT);

	if (_active.total_packets > 1) {
		out_flags |= GPS_RTCM_FLAG_FRAGMENTED;
	}

	if (_active.total_packets == 1) {
		out_data = _message;
		out_len = _active.len;
		resetActiveState();
		return true;
	}

	const size_t offset = static_cast<size_t>(_active.next_fragment_id) * GPS_RTCM_MAX_FRAGMENT_LEN;

	if (offset < _active.len) {
		const size_t remaining = _active.len - offset;
		out_len = (remaining > GPS_RTCM_MAX_FRAGMENT_LEN) ? GPS_RTCM_MAX_FRAGMENT_LEN : remaining;
		out_data = &_message[offset];

	} else {
		// Exact multiples of 180 bytes below 720 bytes need a zero-length
		// fragment so the receiver can observe the first non-full payload and
		// close the buffer early.
		out_data = _message;
	}

	_active.next_fragment_id++;

	if (_active.next_fragment_id >= _active.total_packets) {
		resetActiveState();
	}

	return true;
}

bool GpsRtcmMessageFragmenter::active() const
{
	return _active.total_packets > 0;
}

uint8_t GpsRtcmMessageFragmenter::packetCountForLength(size_t len)
{
	if (len <= GPS_RTCM_MAX_FRAGMENT_LEN) {
		return 1;
	}

	uint8_t packets = static_cast<uint8_t>((len + GPS_RTCM_MAX_FRAGMENT_LEN - 1) / GPS_RTCM_MAX_FRAGMENT_LEN);

	// Exact multiples of 180 bytes below 720 bytes emit a final zero-length fragment so receivers
	// can apply the MAVLink completion rule: a fragmented payload is complete
	// once either all 4 fragments are present, or the first fragment with a
	// non-full payload has been received and every lower fragment ID is present.
	if ((len < GPS_RTCM_MAX_MESSAGE_LEN) && ((len % GPS_RTCM_MAX_FRAGMENT_LEN) == 0)) {
		packets++;
	}

	return packets;
}

void GpsRtcmMessageFragmenter::resetActiveState()
{
	_active = {};
}

size_t Rtcm3Parser::addData(const uint8_t *data, size_t len)
{
	size_t space_available = BUFFER_SIZE - _buffer_len;
	size_t to_copy = (len < space_available) ? len : space_available;

	if (to_copy > 0) {
		memcpy(&_buffer[_buffer_len], data, to_copy);
		_buffer_len += to_copy;
	}

	return to_copy;
}

const uint8_t *Rtcm3Parser::getNextMessage(size_t *out_len)
{
	while (_buffer_len > 0) {
		int to_drop = 0;

		// Find preamble
		for (size_t i = 0; i < _buffer_len; i++) {
			if (_buffer[i] == RTCM3_PREAMBLE) {
				break;
			}

			to_drop++;
		}

		// Drop everything not being the preamble
		if (to_drop > 0) {
			_bytes_discarded += to_drop;
			discardBytes(to_drop);
		}

		// Need at least header to check length
		if (_buffer_len < RTCM3_HEADER_LEN) {
			return nullptr;
		}

		size_t payload_len = rtcm3_payload_length(_buffer);

		if (payload_len > RTCM3_MAX_PAYLOAD_LEN) {
			// Invalid length - not a valid frame, discard preamble
			_bytes_discarded++;
			discardBytes(1);
			continue;
		}

		size_t frame_len = RTCM3_HEADER_LEN + payload_len + RTCM3_CRC_LEN;

		// Check if we have the complete frame
		if (_buffer_len < frame_len) {
			return nullptr;
		}

		uint32_t calculated_crc = rtcm3_crc24q(_buffer, RTCM3_HEADER_LEN + payload_len);
		uint32_t received_crc = (static_cast<uint32_t>(_buffer[frame_len - 3]) << 16) |
					(static_cast<uint32_t>(_buffer[frame_len - 2]) << 8) |
					_buffer[frame_len - 1];

		if (calculated_crc != received_crc) {
			_crc_errors++;
			discardBytes(1);
			continue;
		}

		*out_len = frame_len;
		return _buffer;
	}

	return nullptr;
}

void Rtcm3Parser::consumeMessage(size_t len)
{
	discardBytes(len);
	_messages_parsed++;
	_total_frame_bytes += len;
}

void Rtcm3Parser::discardBytes(size_t count)
{
	if (count >= _buffer_len) {
		_buffer_len = 0;

	} else {
		memmove(_buffer, &_buffer[count], _buffer_len - count);
		_buffer_len -= count;
	}
}

} // namespace gnss
