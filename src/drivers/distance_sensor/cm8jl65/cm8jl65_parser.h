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

/**
 * @file cm8jl65.cpp
 * @author Claudio Micheli <claudio@auterion.com>
 *
 * Parser declarations for Lanbao PSK-CM8JL65-CC5 distance sensor
 */

#pragma once

#include <stdint.h>
// frame start delimiter
#define START_FRAME_DIGIT1      0xA5
#define START_FRAME_DIGIT2      0x5A


// Frame format definition
//
//   1B     1B      1B              1B            2B
// | 0xA5 | 0x5A | distance-MSB | distance-LSB | crc-16 |
//
// Frame data saved for CRC calculation
#define DISTANCE_MSB_POS   2
#define DISTANCE_LSB_POS   3
#define PARSER_BUF_LENGTH  4


enum CM8JL65_PARSE_STATE {
	STATE0_WAITING_FRAME = 0,
	STATE1_GOT_DIGIT1,
	STATE2_GOT_DIGIT2,
	STATE3_GOT_MSB_DATA,
	STATE4_GOT_LSB_DATA,
	STATE5_GOT_CHKSUM1,
	STATE6_GOT_CHKSUM2,
};



int cm8jl65_parser(uint8_t c, uint8_t parserbuf[PARSER_BUF_LENGTH], enum CM8JL65_PARSE_STATE &state, uint16_t &crc16,
		   int &dist);
