/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file correction_framer.cpp
 *
 * GNSS correction stream framing implementation.
 */

#include "correction_framer.h"

#include <cstring>

namespace gnss
{

bool CorrectionFramer::isPreamble(uint8_t byte)
{
	if (byte == RTCM3_PREAMBLE) {
		return true;
	}

#if defined(CONFIG_GPS_SPARTN)

	if (byte == SPARTN_PREAMBLE) {
		return true;
	}

#endif

	return false;
}

size_t CorrectionFramer::addData(const uint8_t *data, size_t len)
{
	const size_t space_available = BUFFER_SIZE - _buffer_len;
	const size_t to_copy = (len < space_available) ? len : space_available;

	if (to_copy > 0) {
		memcpy(&_buffer[_buffer_len], data, to_copy);
		_buffer_len += to_copy;
	}

	return to_copy;
}

CorrectionFramer::Probe CorrectionFramer::probeRtcm3(size_t *frame_len) const
{
	// Need at least the header to read the length
	if (_buffer_len < RTCM3_HEADER_LEN) {
		return Probe::NeedMoreData;
	}

	// The 10-bit field cannot exceed RTCM3_MAX_PAYLOAD_LEN, so the length alone
	// never rejects a candidate: CRC-24Q is what validates the frame.
	const size_t payload_len = rtcm3_payload_length(_buffer);
	const size_t len = RTCM3_HEADER_LEN + payload_len + RTCM3_CRC_LEN;

	if (_buffer_len < len) {
		return Probe::NeedMoreData;
	}

	const uint32_t calculated_crc = rtcm3_crc24q(_buffer, RTCM3_HEADER_LEN + payload_len);
	const uint32_t received_crc = (static_cast<uint32_t>(_buffer[len - 3]) << 16) |
				      (static_cast<uint32_t>(_buffer[len - 2]) << 8) |
				      _buffer[len - 1];

	if (calculated_crc != received_crc) {
		return Probe::CrcError;
	}

	*frame_len = len;
	return Probe::Complete;
}

#if defined(CONFIG_GPS_SPARTN)
CorrectionFramer::Probe CorrectionFramer::probeSpartn(size_t *frame_len) const
{
	// Need TF002..TF006 to read the length and CRC type
	if (_buffer_len < SPARTN_FRAME_START_LEN) {
		return Probe::NeedMoreData;
	}

	const uint8_t b1 = _buffer[1];
	const uint8_t b2 = _buffer[2];
	const uint8_t b3 = _buffer[3];

	// TF002 is the only header field with a checkable range. The 10-bit length
	// spans its full range and TF006 is a 4-bit CRC over a non-byte-aligned
	// field, so this is the only cheap guard before the message CRC.
	if (!spartn_message_type_defined(b1 >> 1)) {
		return Probe::InvalidHeader;
	}

	const uint16_t payload_len = static_cast<uint16_t>(((b1 & 0x01) << 9) | (b2 << 1) | (b3 >> 7));
	const bool eaf = ((b3 >> 6) & 0x01) != 0;
	const uint8_t crc_type = (b3 >> 4) & 0x03;
	const size_t crc_bytes = static_cast<size_t>(crc_type) + 1;

	// TF008 (time tag type) sets the description block length
	if (_buffer_len < SPARTN_FRAME_START_LEN + 1) {
		return Probe::NeedMoreData;
	}

	const bool time_tag_32bit = ((_buffer[4] >> 3) & 0x01) != 0;
	const size_t desc_len = spartn_desc_length_bytes(eaf, time_tag_32bit);

	if (_buffer_len < SPARTN_FRAME_START_LEN + desc_len) {
		return Probe::NeedMoreData;
	}

	size_t auth_bytes = 0;

	if (eaf) {
		// TF014/TF015 occupy the low 6 bits of the last description byte
		const uint8_t last = _buffer[SPARTN_FRAME_START_LEN + desc_len - 1];
		auth_bytes = spartn_auth_length_bytes(true, (last >> 3) & 0x07, last & 0x07);
	}

	const size_t len = SPARTN_FRAME_START_LEN + desc_len + payload_len + auth_bytes + crc_bytes;

	if (_buffer_len < len) {
		return Probe::NeedMoreData;
	}

	// Message CRC (TF018) covers TF002..TF017
	const size_t core_len = len - 1 - crc_bytes;
	const uint32_t calculated_crc = spartn_message_crc(&_buffer[1], core_len, crc_type);
	uint32_t received_crc = 0;

	for (size_t i = 0; i < crc_bytes; i++) {
		received_crc = (received_crc << 8) | _buffer[len - crc_bytes + i];
	}

	if (calculated_crc != received_crc) {
		return Probe::CrcError;
	}

	*frame_len = len;
	return Probe::Complete;
}
#endif // CONFIG_GPS_SPARTN

const uint8_t *CorrectionFramer::getNextMessage(size_t *out_len, CorrectionProtocol *out_protocol)
{
	while (_buffer_len > 0) {
		// Drop everything that cannot start a frame of either protocol
		size_t to_drop = 0;

		while ((to_drop < _buffer_len) && !isPreamble(_buffer[to_drop])) {
			to_drop++;
		}

		if (to_drop > 0) {
			_bytes_discarded += to_drop;
			discardBytes(to_drop);
		}

		if (_buffer_len == 0) {
			return nullptr;
		}

		size_t frame_len = 0;
		CorrectionProtocol protocol = CorrectionProtocol::Rtcm3;
		Probe result;

#if defined(CONFIG_GPS_SPARTN)

		if (_buffer[0] == SPARTN_PREAMBLE) {
			protocol = CorrectionProtocol::Spartn;
			result = probeSpartn(&frame_len);

		} else {
			result = probeRtcm3(&frame_len);
		}

#else
		result = probeRtcm3(&frame_len);
#endif

		switch (result) {
		case Probe::NeedMoreData:
			return nullptr;

		case Probe::InvalidHeader:
			_bytes_discarded++;
			discardBytes(1);
			continue;

		case Probe::CrcError:
			_crc_errors++;
			discardBytes(1);
			continue;

		case Probe::Complete:
			_pending_protocol = protocol;
			*out_len = frame_len;

			if (out_protocol != nullptr) {
				*out_protocol = protocol;
			}

			return _buffer;
		}
	}

	return nullptr;
}

void CorrectionFramer::consumeMessage(size_t len)
{
	discardBytes(len);
	_messages_parsed++;
	_total_frame_bytes += len;

	if (_pending_protocol == CorrectionProtocol::Rtcm3) {
		_rtcm3_messages++;

	} else {
		_spartn_messages++;
	}
}

void CorrectionFramer::discardBytes(size_t count)
{
	if (count >= _buffer_len) {
		_buffer_len = 0;

	} else {
		memmove(_buffer, &_buffer[count], _buffer_len - count);
		_buffer_len -= count;
	}
}

} // namespace gnss
