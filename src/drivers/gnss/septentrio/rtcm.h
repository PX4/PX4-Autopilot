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
 * @file rtcm.h
 *
 * @author Thomas Frans
 */

#pragma once

#include <cstdint>
#include <math.h>

namespace septentrio
{

namespace rtcm
{

constexpr uint8_t PREAMBLE = 0xD3;
constexpr uint8_t HEADER_SIZE = 3;                         ///< Total number of bytes in a message header.
constexpr uint8_t CRC_SIZE = 3;                            ///< Total number of bytes in the CRC.
constexpr uint8_t LENGTH_FIELD_BITS = 10;                  ///< Total number of bits used for the length.
constexpr uint16_t MAX_BODY_SIZE = 1 << LENGTH_FIELD_BITS; ///< Maximum allowed size of the message body.

class Decoder
{
public:
	enum class State {
		/// Searching for the first byte of an RTCM message.
		SearchingPreamble,

		/// Getting the complete header of an RTCM message.
		GettingHeader,

		/// Getting a complete RTCM message.
		Busy,

		/// Complete RTCM message is available.
		Done,
	};

	Decoder();
	~Decoder();

	/**
	 * Add a byte to the current message.
	 *
	 * @param byte The new byte.
	 *
	 * @return true if message complete (use @message to get it)
	 */
	State add_byte(uint8_t b);

	/**
	 * @brief Reset the parser to a clean state.
	 */
	void reset();

	uint8_t *message() const { return _message; }

	/**
	 * @brief Number of received bytes of the current message.
	*/
	uint16_t received_bytes() const;

	/**
	 * @brief The id of the current message.
	 *
	 * This should only be called if the message has been received completely.
	 *
	 * @return The id of the current complete message.
	*/
	uint16_t message_id() const;

private:
	static constexpr uint16_t INITIAL_BUFFER_LENGTH = 300;

	/**
	 * @brief Parse the message lentgh of the current message.
	 *
	 * @return The expected length of the current message without header and CRC.
	*/
	uint16_t parse_message_length() const;

	/**
	 * @brief Check whether the full header has been received.
	 *
	 * @return `true` if the full header is available, `false` otherwise.
	*/
	bool header_received() const;

	uint8_t  *_message{nullptr};
	uint16_t _message_length;       ///< The total length of the message excluding header and CRC (3 bytes each).
	uint16_t _current_index;        ///< The current index of the byte we expect to read into the buffer.
	State _state{State::SearchingPreamble}; ///< Current state of the parser.
};

} // namespace rtcm

} // namespace septentrio
