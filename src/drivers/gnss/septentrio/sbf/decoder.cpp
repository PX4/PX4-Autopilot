/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * @file decoder.cpp
 *
 * Decoding logic for the Septentrio Binary Format (SBF).
 *
 * @author Thomas Frans
 */

#include "decoder.h"

#include <cstring>
#include <mathlib/math/Limits.hpp>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>

#include "../module.h"
#include "../util.h"
#include "drivers/gnss/septentrio/sbf/messages.h"

namespace septentrio
{

namespace sbf
{

Decoder::State Decoder::add_byte(uint8_t byte)
{
	uint8_t *message = reinterpret_cast<uint8_t *>(&_message);

	switch (_state) {
	case State::SearchingSync1:
		if (byte == (uint8_t)k_sync1) {
			// Sync is always same, so we don't store it.
			_current_index++;
			_state = State::SearchingSync2;
		}

		break;

	case State::SearchingSync2:
		if (byte == (uint8_t)k_sync2) {
			// Sync is always same, so we don't store it.
			_current_index++;
			_state = State::Busy;

		} else {
			reset();
		}

		break;

	case State::Busy:
		message[_current_index] = byte;
		_current_index++;

		if (done()) {
			_state = State::Done;
		}

		break;

	case State::Done:
		SEP_WARN("SBF: Discarding excess byte");
		break;
	}

	return _state;
}

BlockID Decoder::id() const
{
	return _state == State::Done ? _message.header.id_number : BlockID::Invalid;
}

int Decoder::parse(Header *header) const
{
	if (can_parse()) {
		memcpy(header, &_message.header, sizeof(Header));
		return PX4_OK;
	}

	return PX4_ERROR;
}

int Decoder::parse(DOP *message) const
{
	if (can_parse() && id() == BlockID::DOP) {
		memcpy(message, _message.payload, sizeof(DOP));
		return PX4_OK;
	}

	return PX4_ERROR;
}

int Decoder::parse(PVTGeodetic *message) const
{
	if (can_parse() && id() == BlockID::PVTGeodetic) {
		memcpy(message, _message.payload, sizeof(PVTGeodetic));
		return PX4_OK;
	}

	return PX4_ERROR;
}

int Decoder::parse(ReceiverStatus *message) const
{
	if (can_parse() && id() == BlockID::ReceiverStatus) {
		memcpy(message, _message.payload, sizeof(ReceiverStatus));
		return PX4_OK;
	}

	return PX4_ERROR;
}

int Decoder::parse(QualityInd *message) const
{
	if (can_parse() && id() == BlockID::QualityInd) {
		// Safe to copy entire size of the message as it is smaller than the maximum expected SBF message size.
		// It's up to the user of the parsed message to ignore the invalid fields.
		memcpy(message, _message.payload, sizeof(QualityInd));
		return PX4_OK;
	}

	return PX4_ERROR;
}

int Decoder::parse(RFStatus *message) const
{
	if (can_parse() && id() == BlockID::PVTGeodetic) {
		memcpy(message, _message.payload, sizeof(RFStatus) - sizeof(RFStatus::rf_band));

		for (uint8_t i = 0; i < math::min(message->n, k_max_rfband_blocks); i++) {
			memcpy(&message->rf_band[i], &_message.payload[sizeof(RFStatus) - sizeof(RFStatus::rf_band) + i *
					message->sb_length], sizeof(RFBand));
		}

		return PX4_OK;
	}

	return PX4_ERROR;
}

int Decoder::parse(GALAuthStatus *message) const
{
	if (can_parse() && id() == BlockID::GALAuthStatus) {
		memcpy(message, _message.payload, sizeof(GALAuthStatus));
		return PX4_OK;
	}

	return PX4_ERROR;
}

int Decoder::parse(VelCovGeodetic *message) const
{
	if (can_parse() && id() == BlockID::VelCovGeodetic) {
		memcpy(message, _message.payload, sizeof(VelCovGeodetic));
		return PX4_OK;
	}

	return PX4_ERROR;
}

int Decoder::parse(GEOIonoDelay *message) const
{
	if (can_parse() && id() == BlockID::GEOIonoDelay) {
		memcpy(message, _message.payload, sizeof(GEOIonoDelay) - sizeof(GEOIonoDelay::idc));

		for (size_t i = 0; i < math::min(message->n, (uint8_t)4); i++) {
			memcpy(&message->idc[i], &_message.payload[sizeof(GEOIonoDelay) - sizeof(GEOIonoDelay::idc) + i *
					message->sb_length], sizeof(IDC));
		}

		return PX4_OK;
	}

	return PX4_ERROR;
}

int Decoder::parse(AttEuler *message) const
{
	if (can_parse() && id() == BlockID::AttEuler) {
		memcpy(message, _message.payload, sizeof(AttEuler));
		return PX4_OK;
	}

	return PX4_ERROR;
}

int Decoder::parse(AttCovEuler *message) const
{
	if (can_parse() && id() == BlockID::AttCovEuler) {
		memcpy(message, _message.payload, sizeof(AttCovEuler));
		return PX4_OK;
	}

	return PX4_ERROR;
}

void Decoder::reset()
{
	_current_index = 0;
	_state = State::SearchingSync1;
	memset(&_message, 0, sizeof(_message));
}

bool Decoder::done() const
{
	return (_current_index >= 14 && _current_index >= _message.header.length) || _current_index >= sizeof(_message);
}

bool Decoder::can_parse() const
{
	return done()
	       && _message.header.crc == buffer_crc16(reinterpret_cast<const uint8_t *>(&_message) + 4, _message.header.length - 4);
}

} // namespace sbf

} // namespace septentrio
