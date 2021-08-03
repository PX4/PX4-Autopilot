/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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


#include <stdint.h>

#define CRTP_PORT_CONSOLE 0x00
#define CRTP_PORT_PARAM	0x02
#define CRTP_PORT_COMMANDER	0x03
#define CRTP_PORT_MEM 0x04
#define CRTP_PORT_LOG 0x05

#define CRTP_PORT_MAVLINK 0x08 // Non-standard port for transmitting mavlink messages

#define CRTP_PORT_PLATFORM 0x0D
#define CRTP_PORT_DEBUG 0x0E
#define CRTP_PORT_LINK 0x0F

#define CRTP_NULL(x) (((x).header & 0xf3) == 0xf3)

// 1 byte header + 31 bytes data = 32 (max ESB packet size)
// With the NRF51, this could be increased to ~250, but the Crazyradio PA uses a NRF24 which can't do this
#define CRTP_MAX_DATA_SIZE 31

typedef struct {
	uint8_t size; // Total size of this message, including the header (placed here to overlap with syslink length field)
	union {
		uint8_t header;
		struct {
			uint8_t channel : 2;
			uint8_t link : 2;
			uint8_t port : 4;
		};
	};

	uint8_t data[CRTP_MAX_DATA_SIZE];
} __attribute__((packed)) crtp_message_t;


typedef struct {
	float roll; // -20 to 20
	float pitch;
	float yaw; // -150 to 150
	uint16_t thrust;
} crtp_commander;
