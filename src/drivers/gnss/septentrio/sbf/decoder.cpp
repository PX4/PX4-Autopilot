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
	return _state == State::Done ? static_cast<BlockID>(_message.header.id_number) : BlockID::Invalid;
}

int Decoder::parse(Header *header) const
{
	if (can_parse()) {
		memcpy(header, &_message.header, sizeof(Header));
		return PX4_OK;
	}

	return PX4_ERROR;
}

int Decoder::parse_block(BlockID block_id, void *message, size_t size) const
{
	if (can_parse() && id() == block_id) {
		memcpy(message, _message.payload, size);
		return PX4_OK;
	}

	return PX4_ERROR;
}

int Decoder::parse(DOP *message) const
{
	static_assert(sizeof(*message) <= sizeof(_message.payload), "Buffer too small");
	return parse_block(BlockID::DOP, message, sizeof(*message));
}

int Decoder::parse(PVTGeodetic *message) const
{
	static_assert(sizeof(*message) <= sizeof(_message.payload), "Buffer too small");
	return parse_block(BlockID::PVTGeodetic, message, sizeof(*message));
}

int Decoder::parse(ReceiverStatus *message) const
{
	static_assert(sizeof(*message) <= sizeof(_message.payload), "Buffer too small");
	return parse_block(BlockID::ReceiverStatus, message, sizeof(*message));
}

int Decoder::parse(QualityInd *message) const
{
	static_assert(sizeof(*message) <= sizeof(_message.payload), "Buffer too small");
	// Safe to copy entire size of the message as it is smaller than the maximum expected SBF message size.
	// It's up to the user of the parsed message to ignore the invalid fields.
	return parse_block(BlockID::QualityInd, message, sizeof(*message));
}

int Decoder::parse(RFStatus *message) const
{
	static_assert(sizeof(*message) <= sizeof(_message.payload), "Buffer too small");

	if (parse_block(BlockID::RFStatus, message, sizeof(RFStatus) - sizeof(RFStatus::rf_band)) != PX4_OK) {
		return PX4_ERROR;
	}

	for (uint8_t i = 0; i < math::min(message->n, k_max_rfband_blocks); i++) {
		const unsigned offset = sizeof(RFStatus) - sizeof(RFStatus::rf_band) + i *
					message->sb_length;

		if (offset + sizeof(RFBand) <= sizeof(_message.payload)) {
			memcpy(&message->rf_band[i], &_message.payload[offset], sizeof(RFBand));
		}
	}

	return PX4_OK;
}

int Decoder::parse(GALAuthStatus *message) const
{
	static_assert(sizeof(*message) <= sizeof(_message.payload), "Buffer too small");
	return parse_block(BlockID::GALAuthStatus, message, sizeof(*message));
}

int Decoder::parse(VelCovGeodetic *message) const
{
	static_assert(sizeof(*message) <= sizeof(_message.payload), "Buffer too small");
	return parse_block(BlockID::VelCovGeodetic, message, sizeof(*message));
}

int Decoder::parse(GEOIonoDelay *message) const
{
	static_assert(sizeof(*message) <= sizeof(_message.payload), "Buffer too small");

	if (parse_block(BlockID::GEOIonoDelay, message, sizeof(GEOIonoDelay) - sizeof(GEOIonoDelay::idc)) != PX4_OK) {
		return PX4_ERROR;
	}

	for (size_t i = 0; i < math::min(message->n, (uint8_t)(sizeof(GEOIonoDelay::idc) / sizeof(GEOIonoDelay::idc[0])));
	     i++) {
		const unsigned offset = sizeof(GEOIonoDelay) - sizeof(GEOIonoDelay::idc) + i *
					message->sb_length;

		if (offset + sizeof(IDC) <= sizeof(_message.payload)) {
			memcpy(&message->idc[i], &_message.payload[offset], sizeof(IDC));
		}
	}

	return PX4_OK;
}

int Decoder::parse(AttEuler *message) const
{
	static_assert(sizeof(*message) <= sizeof(_message.payload), "Buffer too small");
	return parse_block(BlockID::AttEuler, message, sizeof(*message));
}

int Decoder::parse(AttCovEuler *message) const
{
	static_assert(sizeof(*message) <= sizeof(_message.payload), "Buffer too small");
	return parse_block(BlockID::AttCovEuler, message, sizeof(*message));
}

int Decoder::parse(BaseVectorGeod *message) const
{
	static_assert(sizeof(*message) <= sizeof(_message.payload), "Buffer too small");

	// Sub-blocks can grow in later block revisions; the fields read here are at their start
	if (parse_block(BlockID::BaseVectorGeod, message, sizeof(*message)) == PX4_OK
	    && message->n > 0 && message->sb_length >= sizeof(VectorInfoGeod)
	    && sizeof(Header) + sizeof(BaseVectorGeod) <= _message.header.length) {
		return PX4_OK;
	}

	return PX4_ERROR;
}

int Decoder::parse(AuxAntPositions *message) const
{
	static_assert(sizeof(*message) <= sizeof(_message.payload), "Buffer too small");

	if (parse_block(BlockID::AuxAntPositions, message, sizeof(*message)) == PX4_OK
	    && message->n > 0 && message->sb_length >= sizeof(AuxAntPosSub)
	    && sizeof(Header) + sizeof(AuxAntPositions) <= _message.header.length) {
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
	const bool precondition = done() && _message.header.length <= sizeof(_message) && _message.header.length > 4;
#ifdef FUZZING_BUILD_MODE_UNSAFE_FOR_PRODUCTION
	// When fuzzing, disable the CRC check as it is hard for the fuzzer to find msgs with valid CRC.
	return precondition;
#else
	return precondition &&
	       _message.header.crc == buffer_crc16(reinterpret_cast<const uint8_t *>(&_message) + 4, _message.header.length - 4);
#endif
}

} // namespace sbf

} // namespace septentrio
