/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: Anton Babushkin <anton.babushkin@me.com>
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
 * @file sdlog2_format.h
 *
 * General log format structures and macro.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

/*
Format characters in the format string for binary log messages
  b   : int8_t
  B   : uint8_t
  h   : int16_t
  H   : uint16_t
  i   : int32_t
  I   : uint32_t
  f   : float
  n   : char[4]
  N   : char[16]
  Z   : char[64]
  c   : int16_t * 100
  C   : uint16_t * 100
  e   : int32_t * 100
  E   : uint32_t * 100
  L   : int32_t latitude/longitude
  M   : uint8_t flight mode

  q   : int64_t
  Q   : uint64_t
 */

#ifndef SDLOG2_FORMAT_H_
#define SDLOG2_FORMAT_H_

#define LOG_PACKET_HEADER_LEN	   3
#define LOG_PACKET_HEADER	       uint8_t head1, head2, msg_type;
#define LOG_PACKET_HEADER_INIT(id) .head1 = HEAD_BYTE1, .head2 = HEAD_BYTE2, .msg_type = id

// once the logging code is all converted we will remove these from
// this header
#define HEAD_BYTE1  0xA3    // Decimal 163
#define HEAD_BYTE2  0x95    // Decimal 149

struct log_format_s {
	uint8_t type;
	uint8_t length;		// full packet length including header
	char name[4];
	char format[16];
	char labels[64];
};

#define LOG_FORMAT(_name, _format, _labels) { \
		.type = LOG_##_name##_MSG, \
			.length = sizeof(struct log_##_name##_s) + LOG_PACKET_HEADER_LEN, \
				  .name = #_name, \
					  .format = _format, \
						    .labels = _labels \
	}

#define LOG_FORMAT_S(_name, _struct_name, _format, _labels) { \
		.type = LOG_##_name##_MSG, \
			.length = sizeof(struct log_##_struct_name##_s) + LOG_PACKET_HEADER_LEN, \
				  .name = #_name, \
					  .format = _format, \
						    .labels = _labels \
	}

#define LOG_FORMAT_MSG	  0x80

#define LOG_PACKET_SIZE(_name)	LOG_PACKET_HEADER_LEN + sizeof(struct log_##_name##_s)

#endif /* SDLOG2_FORMAT_H_ */
