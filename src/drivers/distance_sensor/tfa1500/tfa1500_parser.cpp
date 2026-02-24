/****************************************************************************
 *
 *   Copyright (c) 2017-2019 PX4 Development Team. All rights reserved.
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
 * @file tfmini_parser.cpp
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Chuong Nguyen <chnguye7@asu.edu>
 * @author Ayush Gaud <ayush.gaud@gmail.com>
 *
 * Declarations of parser for the Benewake TFmini laser rangefinder series
 */

#include "tfa1500_parser.h"
#include <string.h>
#include <stdlib.h>

// #define TFA1500_DEBUG
#define TFA1500_FRAME_LEN 5
#ifdef TFA1500_DEBUG
#include <stdio.h>

const char *parser_state[] = {
	"0_UNSYNC",
	"1_SYNC_1",
	"2_GOT_DIST_LOW",
	"3_GOT_DIST_MID",
	"4_GOT_DIST_HIGH",
	"5_GOT_CHECKSUM"};
#endif

int tfa1500_parse(char c, char *parserbuf, unsigned *parserbuf_index, TFA1500_PARSE_STATE *state, float *dist)
{
	int ret = -1;
	// char *end;
	if (*state == TFA1500_PARSE_STATE::STATE0_UNSYNC && *parserbuf_index > 0)
	{
		*parserbuf_index = 0;
		memset(parserbuf, 0, TFA1500_FRAME_LEN); // 清空残留数据
	}
	switch (*state)
	{
	case TFA1500_PARSE_STATE::STATE5_GOT_CHECKSUM:
		if (c == 0x5C && *parserbuf_index == 0)
		{
			*state = TFA1500_PARSE_STATE::STATE1_GOT_HEADER;
			parserbuf[*parserbuf_index] = c;
			(*parserbuf_index)++;
		}
		else
		{
			*state = TFA1500_PARSE_STATE::STATE0_UNSYNC;
		}

		break;

	case TFA1500_PARSE_STATE::STATE0_UNSYNC:
		if (c == 0x5C && *parserbuf_index == 0)
		{
			*state = TFA1500_PARSE_STATE::STATE1_GOT_HEADER;
			parserbuf[*parserbuf_index] = c;
			(*parserbuf_index)++;
		}

		break;

	case TFA1500_PARSE_STATE::STATE1_GOT_HEADER:
		*state = TFA1500_PARSE_STATE::STATE2_GOT_DIST_LOW;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;

		break;

	case TFA1500_PARSE_STATE::STATE2_GOT_DIST_LOW:
		*state = TFA1500_PARSE_STATE::STATE3_GOT_DIST_MID;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;

		break;

	case TFA1500_PARSE_STATE::STATE3_GOT_DIST_MID:
		*state = TFA1500_PARSE_STATE::STATE4_GOT_DIST_HIGH;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;

		break;

	case TFA1500_PARSE_STATE::STATE4_GOT_DIST_HIGH:

		// Find the checksum
		unsigned char cksm = 0;

		for (int i = 1; i < TFA1500_FRAME_LEN - 1; i++)
		{
			cksm += parserbuf[i];
		}
		cksm = ~cksm;
		if (c == cksm)
		{
			parserbuf[*parserbuf_index] = '\0';
			unsigned int distance_cm = parserbuf[1] | parserbuf[2] << 8 | parserbuf[3] << 16;
			*dist = (float)distance_cm / 100.0f; // m

			*state = TFA1500_PARSE_STATE::STATE5_GOT_CHECKSUM;
			*parserbuf_index = 0;
			ret = 0;
		}
		else
		{
			*state = TFA1500_PARSE_STATE::STATE0_UNSYNC;
			*parserbuf_index = 0;
		}

		break;
	}

#ifdef TFA1500_DEBUG
	printf("state: TFA1500_PARSE_STATE%s\n", parser_state[*state]);
#endif

	return ret;
}
