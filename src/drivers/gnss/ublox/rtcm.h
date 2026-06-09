/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#pragma once

#include <cstdint>

/* RTCM3 */
#define RTCM3_PREAMBLE					0xD3
#define RTCM_INITIAL_BUFFER_LENGTH			300		/**< initial maximum message length of an RTCM message */


class RTCMParsing
{
public:
	RTCMParsing();
	~RTCMParsing();

	/**
	 * reset the parsing state
	 */
	void reset();

	/**
	 * add a byte to the message
	 * @param b byte to add
	 * @return true, if a message is complete (use @message to get it). The user needs to reset the parser in this case.
	 * 	false, if more data is needed or the parser failed. In this case no resetting of the parser is needed.
	 */
	bool addByte(uint8_t b);

	uint8_t *message() const { return _buffer; }
	uint16_t messageLength() const { return _pos; }
	uint16_t messageId() const { return (_buffer[3] << 4) | (_buffer[4] >> 4); }

private:
	uint32_t crc24(const uint8_t *buffer, const uint16_t len);

	uint8_t			*_buffer{nullptr};
	uint16_t		_buffer_len{};
	uint16_t		_pos{};						///< next position in buffer
	uint16_t		_message_length{};				///< message length without header & CRC (both 3 bytes)
	bool			_preamble_received{false};
};
