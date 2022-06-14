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
 * @file modified from sf0x_parser.cpp
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Chuong Nguyen <chnguye7@asu.edu>
 * @author Ayush Gaud <ayush.gaud@gmail.com>
 * @author Zebulon <zebulon-86@outlook.com>
 *
 * Declarations of parser for the Benewake Rds02UF rangefinder series
 */

#include "rds02uf_parser.h"
#include <string.h>
#include <stdlib.h>

#define RDS02UF_HEAD_LEN 2
#define RDS02UF_PRE_DATA_LEN 6
#define RDS02UF_DATA_LEN 10
// #define RDS02UF_DEBUG

#ifdef RDS02UF_DEBUG
#include <stdio.h>

const char *parser_state[] = {

	"0_STATE1_SYNC_1",
	"1_STATE2_SYNC_2",
	"2_STATE3_ADDRESS",
	"3_STATE4_ERROR_CODE",
	"4_STATE5_FC_CODE_L",
	"5_STATE6_FC_CODE_H",
	"6_STATE7_LENGTH_L",
	"7_STATE8_LENGTH_H",
	"8_STATE9_REAL_DATA",
	"9_STATE10_CRC",
	"10_STATE11_END_1",
	"11_STATE12_END_2"
};
#endif

int rds02uf_parse(char c, char *parserbuf, unsigned *parserbuf_index, RDS02UF_PARSE_STATE *state, float *dist)
{
	int ret = -1;
	char data = c;
	uint8_t crc_data = 0;

	switch (*state) {
	case RDS02UF_PARSE_STATE::STATE0_SYNC_1:
		if (data == RDS02_HEAD1)
		{
			parserbuf[*parserbuf_index] = data;
			(*parserbuf_index)++;
			*state = RDS02UF_PARSE_STATE::STATE1_SYNC_2;
		}
		break;
	case RDS02UF_PARSE_STATE::STATE1_SYNC_2:
		if (data == RDS02_HEAD2)
		{
			parserbuf[*parserbuf_index] = data;
			(*parserbuf_index)++;
			*state = RDS02UF_PARSE_STATE::STATE2_ADDRESS;
		}else{
			*parserbuf_index = 0;
			*state = RDS02UF_PARSE_STATE::STATE0_SYNC_1;
		}
		break;
	case RDS02UF_PARSE_STATE::STATE2_ADDRESS: // address
		parserbuf[*parserbuf_index] = data;
		(*parserbuf_index)++;
		*state = RDS02UF_PARSE_STATE::STATE3_ERROR_CODE;
		break;
	case RDS02UF_PARSE_STATE::STATE3_ERROR_CODE: // error_code
		parserbuf[*parserbuf_index] = data;
		(*parserbuf_index)++;
		*state = RDS02UF_PARSE_STATE::STATE4_FC_CODE_L;
		break;
	case RDS02UF_PARSE_STATE::STATE4_FC_CODE_L: // fc_code low
		parserbuf[*parserbuf_index] = data;
		(*parserbuf_index)++;
		*state = RDS02UF_PARSE_STATE::STATE5_FC_CODE_H;
		break;
	case RDS02UF_PARSE_STATE::STATE5_FC_CODE_H: // fc_code high
		parserbuf[*parserbuf_index] = data;
		(*parserbuf_index)++;
		*state = RDS02UF_PARSE_STATE::STATE6_LENGTH_L;
		break;
	case RDS02UF_PARSE_STATE::STATE6_LENGTH_L: // lengh_low
		parserbuf[*parserbuf_index] = data;
		(*parserbuf_index)++;
		*state = RDS02UF_PARSE_STATE::STATE7_LENGTH_H;
		break;
	case RDS02UF_PARSE_STATE::STATE7_LENGTH_H: // lengh_high
		{
			uint8_t read_len = data << 8 | parserbuf[*parserbuf_index-1];
			if ( read_len == RDS02UF_DATA_LEN)	// rds02uf data length is 10
			{
				parserbuf[*parserbuf_index] = data;
				(*parserbuf_index)++;
				*state = RDS02UF_PARSE_STATE::STATE8_REAL_DATA;
			}else{
				*parserbuf_index = 0;
				*state = RDS02UF_PARSE_STATE::STATE0_SYNC_1;
			}
			break;
		}

	case RDS02UF_PARSE_STATE::STATE8_REAL_DATA: // real_data
		parserbuf[*parserbuf_index] = data;
		(*parserbuf_index)++;
		if ((*parserbuf_index) == (RDS02UF_HEAD_LEN + RDS02UF_PRE_DATA_LEN + RDS02UF_DATA_LEN))
		{
			*state = RDS02UF_PARSE_STATE::STATE9_CRC;
		}
		break;
	case RDS02UF_PARSE_STATE::STATE9_CRC: // crc
		crc_data = crc8(&parserbuf[2], RDS02UF_PRE_DATA_LEN + RDS02UF_DATA_LEN);
		parserbuf[*parserbuf_index] = data;
		if (crc_data == data || data == 0xff)
		{
			(*parserbuf_index)++;
			*state = RDS02UF_PARSE_STATE::STATE10_END_1;
		}else{
			*parserbuf_index = 0;
			*state = RDS02UF_PARSE_STATE::STATE0_SYNC_1;
		}
		break;
	case RDS02UF_PARSE_STATE::STATE10_END_1: //
		if (data == RDS02_END)
		{
			parserbuf[*parserbuf_index] = data;
			(*parserbuf_index)++;
			*state = RDS02UF_PARSE_STATE::STATE11_END_2;
		}else{
			*parserbuf_index = 0;
			*state = RDS02UF_PARSE_STATE::STATE0_SYNC_1;
		}

		break;
	case RDS02UF_PARSE_STATE::STATE11_END_2: //
		{
			uint16_t fc_code = (parserbuf[STATE5_FC_CODE_H] << 8 | parserbuf[STATE4_FC_CODE_L]);
			uint8_t err_code = parserbuf[STATE3_ERROR_CODE];
			if (data == RDS02_END)
			{
				if (fc_code == 0x03ff && err_code == 0)	// get targer information
				{
					if(parserbuf[RDS02_DATA_START_INDEX] == RDS02_TARGET_INFO)
					{
						float distance = (parserbuf[RDS02_DATA_Y_INDEX + 1] * 256 + parserbuf[RDS02_DATA_Y_INDEX]) / 100.0f;
						*dist = distance;
						ret = true;
					}
				}
			}

			*parserbuf_index = 0;
			*state = RDS02UF_PARSE_STATE::STATE0_SYNC_1;

			break;
		}
	}

#ifdef RDS02UF_DEBUG
	static u_int16_t cnt;
	cnt += 1;
	static RDS02UF_PARSE_STATE last_state = RDS02UF_PARSE_STATE::STATE12_END_2;
	if (*state != last_state || cnt > 500)
	{
		printf("state: %s,read: %02x\n", parser_state[*state],data);
		last_state = *state;
		cnt = 0;
	}
#endif

	return ret;
}

uint8_t crc8(char* pbuf, int32_t len)
{
     char* data = pbuf;
     uint8_t crc = 0;
     while ( len-- )
     crc = crc8_table[crc^*(data++)];
     return crc;
}
