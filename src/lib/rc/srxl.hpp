/****************************************************************************
 *
 *	Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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

/**
 * @file srxl.h
 *
 * RC protocol definition for Spektrum SRXL
 *
 * @author Kurt Kiefer <kekiefer@gmail.com>
 * @author Daniel Williams <equipoise@gmail.com>
 */

#pragma once

// "Cannot find the standard library!?"
// #include <array>

#include <cstdint>

class SRXLCodec
{
// ====== ====== ====== ====== Public Constants ====== ====== ====== ======
public:

	// Version 1 specific
	constexpr static const uint8_t SRXL1_FRAME_HEADER = 	0xA5;
	constexpr static const uint8_t SRXL1_MAX_LENGTH =	64;

	// Version 2 specific
	constexpr const static uint8_t SRXL2_FRAME_HEADER = 	0xA6;
	constexpr const static uint8_t SRXL2_MIN_LENGTH =	5;
	constexpr const static uint8_t SRXL2_MAX_LENGTH =	80;

	// Packet Types
	enum frame_type_t {
		FRAME_TYPE_HANDSHAKE = 0x21,
		FRAME_TYPE_BIND = 0x41,
		FRAME_TYPE_PARAMETER = 0x50,
		FRAME_TYPE_SIGNAL_QUALITY = 0x55,
		FRAME_TYPE_TELEMETRY = 0x80,
		FRAME_TYPE_CONTROL = 0xCD,
	};

	// apply to all versions
	constexpr static const uint8_t SRXL_FRAME_HEADER_SIZE = 3;
	constexpr static const uint8_t SRXL_MAX_FRAME_LENGTH = SRXL2_MAX_LENGTH;
	constexpr static const uint8_t SRXL_MAX_PAYLOAD_LENGTH = SRXL2_MAX_LENGTH - SRXL_FRAME_HEADER_SIZE;


// ====== ====== ====== ====== Public Types ====== ====== ====== ======
public:
	typedef uint8_t srxl_buffer_t[SRXL_MAX_FRAME_LENGTH];

#pragma pack(push,1)
	struct srxl_frame_t {
		uint8_t header;
		uint8_t version;
		uint8_t length;
		uint8_t payload[SRXL_MAX_PAYLOAD_LENGTH];
	};
#pragma pack(pop)
	static_assert(sizeof(srxl_frame_t) == SRXL_MAX_FRAME_LENGTH,
		      "Inconsistent frame-struct size!! This is a developer error!");


// ====== ====== ====== ====== Public API ====== ====== ====== ======
public:
	SRXLCodec() = default;
	~SRXLCodec() = default;

	static void set_payload(uint8_t *payload, size_t length, srxl_frame_t &frame);

	srxl_frame_t &get_frame();

	void get_buffer(uint8_t *&buf_ref, size_t &buf_size);

	static constexpr bool is_srxl_telemetry(srxl_frame_t &frame);

	static constexpr uint8_t get_frame_length(srxl_frame_t &frame);

private:
	// raw buffer; will match bytes on/to/from the wire
	srxl_buffer_t _receive_buffer;

	// maybe should overlay the byte-buffer, above?
	srxl_frame_t _frame_buffer;

	size_t _bytes_received;

	size_t _bytes_to_transmit;

};
