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
 * @file tf02pro_parser.cpp
 *
 * Parser for the Benewake TF02 Pro distance sensor.
 */

#include "tf02pro_parser.h"

int tf02pro_parse(uint8_t c, uint8_t *buf, unsigned *buf_idx,
		  TF02PRO_PARSE_STATE *state, float *dist)
{
	int ret = -1;

	switch (*state) {

	case TF02PRO_PARSE_STATE::STATE9_GOT_CHECKSUM:

	// Fall through — re-sync immediately after a completed frame
	case TF02PRO_PARSE_STATE::STATE0_UNSYNC:
		if (c == 0x59) {
			*state    = TF02PRO_PARSE_STATE::STATE1_SYNC_1;
			buf[0]    = c;
			*buf_idx  = 1;

		} else {
			*state   = TF02PRO_PARSE_STATE::STATE0_UNSYNC;
			*buf_idx = 0;
		}

		break;

	case TF02PRO_PARSE_STATE::STATE1_SYNC_1:
		if (c == 0x59) {
			*state = TF02PRO_PARSE_STATE::STATE2_SYNC_2;
			buf[(*buf_idx)++] = c;   // buf[1] = 0x59

		} else {
			*state   = TF02PRO_PARSE_STATE::STATE0_UNSYNC;
			*buf_idx = 0;
		}

		break;

	case TF02PRO_PARSE_STATE::STATE2_SYNC_2:         // collect Dist_L → buf[2]
		buf[(*buf_idx)++] = c;
		*state = TF02PRO_PARSE_STATE::STATE3_GOT_DIST_L;
		break;

	case TF02PRO_PARSE_STATE::STATE3_GOT_DIST_L:     // collect Dist_H → buf[3]
		buf[(*buf_idx)++] = c;
		*state = TF02PRO_PARSE_STATE::STATE4_GOT_DIST_H;
		break;

	case TF02PRO_PARSE_STATE::STATE4_GOT_DIST_H:     // collect Strength_L → buf[4]
		buf[(*buf_idx)++] = c;
		*state = TF02PRO_PARSE_STATE::STATE5_GOT_STRENGTH_L;
		break;

	case TF02PRO_PARSE_STATE::STATE5_GOT_STRENGTH_L: // collect Strength_H → buf[5]
		buf[(*buf_idx)++] = c;
		*state = TF02PRO_PARSE_STATE::STATE6_GOT_STRENGTH_H;
		break;

	case TF02PRO_PARSE_STATE::STATE6_GOT_STRENGTH_H: // collect Temp_L → buf[6]
		buf[(*buf_idx)++] = c;
		*state = TF02PRO_PARSE_STATE::STATE7_GOT_TEMP_L;
		break;

	case TF02PRO_PARSE_STATE::STATE7_GOT_TEMP_L:     // collect Temp_H → buf[7]
		buf[(*buf_idx)++] = c;
		*state = TF02PRO_PARSE_STATE::STATE8_GOT_TEMP_H;
		break;

	case TF02PRO_PARSE_STATE::STATE8_GOT_TEMP_H: {   // receive + validate checksum (byte[8])
			uint8_t cksm = 0;

			for (int i = 0; i < 8; i++) {
				cksm += buf[i];
			}

			if (c == cksm) {
				uint16_t distance_cm = (uint16_t)buf[3] << 8 | buf[2];
				uint16_t strength    = (uint16_t)buf[5] << 8 | buf[4];

				// Both filters from the existing I2C collect()
				if (strength >= 60u && distance_cm < 4500u) {
					*dist = distance_cm * 1e-2f;
					ret   = 0;
				}

				*state = TF02PRO_PARSE_STATE::STATE9_GOT_CHECKSUM;

			} else {
				// Checksum mismatch — discard frame, attempt re-sync
				*state = TF02PRO_PARSE_STATE::STATE0_UNSYNC;
			}

			*buf_idx = 0;
			break;
		}

	} // end switch

	return ret;
}
