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
 * @file tf02pro_parser.h
 *
 * Parser for the Benewake TF02 Pro distance sensor.
 */

#pragma once
#include <stdint.h>

enum class TF02PRO_PARSE_STATE {
	STATE0_UNSYNC = 0,
	STATE1_SYNC_1,          // received first 0x59
	STATE2_SYNC_2,          // received second 0x59
	STATE3_GOT_DIST_L,      // byte[2] Dist_L collected
	STATE4_GOT_DIST_H,      // byte[3] Dist_H collected
	STATE5_GOT_STRENGTH_L,  // byte[4] Strength_L collected
	STATE6_GOT_STRENGTH_H,  // byte[5] Strength_H collected
	STATE7_GOT_TEMP_L,      // byte[6] Temp_L collected
	STATE8_GOT_TEMP_H,      // byte[7] Temp_H collected
	STATE9_GOT_CHECKSUM     // byte[8] checksum validated; ready to re-sync
};

/**
 * Parse one byte of the TF02 Pro 9-byte binary stream.
 *
 * Frame layout: 0x59 0x59 Dist_L Dist_H Str_L Str_H Temp_L Temp_H Checksum
 * Checksum = lower 8 bits of (sum of bytes 0..7).
 *
 * @param c       incoming byte
 * @param buf     9-byte working buffer (caller-owned, persistent across calls)
 * @param buf_idx pointer to current buffer index (reset on frame complete or error)
 * @param state   pointer to parser state (persistent across calls)
 * @param dist    output: distance in metres (valid only when returns 0)
 * @return 0 on valid complete frame, -1 otherwise
 */
int tf02pro_parse(uint8_t c, uint8_t *buf, unsigned *buf_idx,
		  TF02PRO_PARSE_STATE *state, float *dist);
