/****************************************************************************
 *
 *	Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
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

#include "sky_fall.hpp"

#include <cctype>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/vehicle_status.h>

#include <cstring>

namespace payload_board
{
SkyFall::SkyFall(uORB::Subscription &input_rc_sub, uORB::Subscription &vehicle_status_sub)
	: _input_rc_sub{input_rc_sub}, _vehicle_status_sub{vehicle_status_sub}
{
}

int SkyFall::update_state()
{
	if (_msp_message.getCommand() != MSPMessage::MSP_SET_NAME) { return PX4_OK; }

	if (_msp_message.copyDataTo(_msp_message_text, sizeof(_msp_message_text)) == PX4_ERROR) { return PX4_ERROR; }

	_msp_message_text[sizeof(_msp_message_text) - 1] = '\0'; // ensure correct string

	if (message_contains("INIC")) {
		_last_state = payload_response_s::STATE_INACTIVE;

	} else if (message_contains("FLIGHT")) {
		_last_state = payload_response_s::STATE_CHARGING;

	} else if (message_is_blank_or_empty() || message_contains("WAIT FUSE2") || message_contains("SRV MOVE") ||
		   message_contains("DTN READY") || message_contains("****")) {
		_last_state = payload_response_s::STATE_READY;

	} else if (message_contains("DTN")) {
		_last_state = payload_response_s::STATE_ACTIVE;

	} else {
		_last_state = payload_response_s::STATE_ERR;
	}

	return PX4_OK;
}

bool SkyFall::should_reply()
{
	switch (_msp_message.getCommand()) {
	case MSPMessage::MSP_RAW_RC:
	case MSPMessage::MSP_STATUS:
		return true;

	default:
		return false;
	}
}

/* TODO: General MSP handling should be moved to a more general scope */
void SkyFall::create_reply()
{
	memset(_tx_buf, 0, sizeof(_tx_buf));

	switch (_msp_message.getCommand()) {
	case MSPMessage::MSP_RAW_RC: {
			constexpr size_t kRcChannelsCount{16};
			constexpr size_t kDataLength = kRcChannelsCount * sizeof(uint16_t);
			static_assert(sizeof(input_rc_s::values) >= kDataLength,
				      "input_rc_s::values array must be at least the size of 16 channels times 2 bytes");

			uint8_t data[kDataLength] {};

			if (_input_rc_sub.updated()) {
				input_rc_s input_rc{};
				_input_rc_sub.copy(&input_rc);

				if (!input_rc.rc_lost) { memcpy(data, input_rc.values, kDataLength); }
			}

			const auto message = MSPMessage::response(MSPMessage::MSP_RAW_RC, data, kDataLength);
			message.copyTo(_tx_buf);
			_transmit_length = message.getTotalLength();
			break;
		}

	case MSPMessage::MSP_STATUS: {
			constexpr size_t kDataLength{24};
			uint8_t data[kDataLength] {};
			// flight mode flags
			data[6] = 0;
			vehicle_status_s vehicle_status{};
			_vehicle_status_sub.copy(&vehicle_status);

			if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) { data[6] |= 3; }

			const auto message = MSPMessage::response(MSPMessage::MSP_STATUS, data, kDataLength);
			message.copyTo(_tx_buf);
			_transmit_length = message.getTotalLength();
			break;
		}

	default:
		break;
	}
}

int SkyFall::get_custom_message(uint8_t *message, const size_t size)
{
	const size_t msp_message_text_length = strlen(reinterpret_cast<const char *>(_msp_message_text));

	if (size < msp_message_text_length) {
		return PX4_ERROR;
	}

	memcpy(message, _msp_message_text, msp_message_text_length);
	return PX4_OK;
}

void SkyFall::begin_read()
{
	Protocol::begin_read();
	_msp_message.clearData();
}

bool SkyFall::message_contains(const char *message) const
{
	return strstr(reinterpret_cast<const char *>(_msp_message_text), message) != nullptr;
}

bool SkyFall::message_is_blank_or_empty() const
{
	if (*_msp_message_text == '\0') {
		return true;
	}

	for (const uint8_t *c = _msp_message_text; *c != '\0'; ++c) {
		if (!isspace(*c)) {
			return false;
		}
	}

	return true;
}
}  // namespace payload_board
