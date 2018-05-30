/****************************************************************************
 *
 *   Copyright (c) 2012-2018 PX4 Development Team. All rights reserved.
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

#ifndef _DYNAMIXEL_MESSAGE_H
#define _DYNAMIXEL_MESSAGE_H

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define NUM_DYNAMIXELS 8

#define OPERATING_MODE_ADDRESS 0x0B
#define ENABLE_ADDRESS 0x40
#define LED_ADDRESS 0x41
#define RETURN_LEVEL_SET_ADDRESS 0x44
#define POSITION_D_ADDRESS 0x50
#define POSITION_I_ADDRESS 0x52
#define POSITION_P_ADDRESS 0x54
#define GOAL_POSITION_ADDRESS 0x74
#define CURRENT_POSITION_ADDRESS 0x84

#define EXTENDED_POSITION_CONTROL_MODE 0x04

#define HEADER_BYTE_0 0xFF
#define HEADER_BYTE_1 0xFF
#define HEADER_BYTE_2 0xFD
#define RESERVED_BYTE 0x00
#define BROADCAST_BYTE 0xFE

#define READ_INSTRUCTION 0x82
#define WRITE_INSTRUCTION 0x83

#define STATUS_ID_BYTE 0x04
#define STATUS_ERROR_BYTE 0x08
#define STATUS_DATA_START_BYTE 0x09

#define MESSAGE_HEADER_START_BYTE 0
#define MESSAGE_RESERVED_START_BYTE \
	(MESSAGE_HEADER_START_BYTE + 3 * sizeof(uint8_t))
#define MESSAGE_TARGET_START_BYTE \
	(MESSAGE_RESERVED_START_BYTE + sizeof(uint8_t))
#define MESSAGE_SIZE_START_BYTE (MESSAGE_TARGET_START_BYTE + sizeof(uint8_t))
#define MESSAGE_INSTRUCTION_START_BYTE \
	(MESSAGE_SIZE_START_BYTE + sizeof(uint16_t))
#define MESSAGE_ADDRESS_START_BYTE \
	(MESSAGE_INSTRUCTION_START_BYTE + sizeof(uint8_t))
#define MESSAGE_DATA_LENGTH_START_BYTE \
	(MESSAGE_ADDRESS_START_BYTE + sizeof(uint16_t))
#define MESSAGE_BODY_START_BYTE \
	(MESSAGE_DATA_LENGTH_START_BYTE + sizeof(uint16_t))

#define PACKET_HEADER_SIZE sizeof(uint8_t)
#define READ_PACKET_SIZE PACKET_HEADER_SIZE
#define WRITE_32_BIT_PACKET_SIZE (PACKET_HEADER_SIZE + sizeof(int32_t))
#define WRITE_16_BIT_PACKET_SIZE (PACKET_HEADER_SIZE + sizeof(uint16_t))
#define WRITE_8_BIT_PACKET_SIZE (PACKET_HEADER_SIZE + sizeof(uint8_t))

#define BASE_MESSAGE_SIZE (MESSAGE_BODY_START_BYTE + sizeof(uint16_t))
#define READ_MESSAGE_SIZE \
	(BASE_MESSAGE_SIZE + NUM_DYNAMIXELS * READ_PACKET_SIZE)
#define WRITE_32_BIT_MESSAGE_SIZE \
	(BASE_MESSAGE_SIZE + NUM_DYNAMIXELS * WRITE_32_BIT_PACKET_SIZE)
#define WRITE_16_BIT_MESSAGE_SIZE \
	(BASE_MESSAGE_SIZE + NUM_DYNAMIXELS * WRITE_16_BIT_PACKET_SIZE)
#define WRITE_8_BIT_MESSAGE_SIZE \
	(BASE_MESSAGE_SIZE + NUM_DYNAMIXELS * WRITE_8_BIT_PACKET_SIZE)
#define STATUS_MESSAGE_SIZE 15

uint16_t calc_crc(uint8_t *data_blk_ptr, uint16_t data_blk_size);
void add_message_header(uint8_t *msg);
void add_message_reserved_byte(uint8_t *msg);
void add_message_broadcast_target(uint8_t *msg);
void add_message_instruction(uint8_t instruction, uint8_t *msg);
void add_message_address(uint16_t message_address, uint8_t *msg);
void add_message_data_length(uint16_t data_length, uint8_t *msg);
void add_message_read_data_packet(uint8_t target_id, uint8_t *msg);
void add_message_32_bit_write_data_packet(uint8_t target_id, int32_t data,
		uint8_t *msg);
void add_message_16_bit_write_data_packet(uint8_t target_id, uint16_t data,
		uint8_t *msg);
void add_message_8_bit_write_data_packet(uint8_t target_id, uint8_t data,
		uint8_t *msg);
void add_message_size_and_crc(uint16_t packet_size, uint8_t *msg);
int calc_message_size(uint16_t packet_size);

void build_read_message(uint16_t read_address, uint8_t *msg);
void build_write_32_bit_message(uint16_t write_address, int32_t *data,
				uint8_t *msg);
void build_write_16_bit_message(uint16_t write_address, uint16_t *data,
				uint8_t *msg);
void build_write_8_bit_message(uint16_t write_address, uint8_t *data,
			       uint8_t *msg);

uint8_t parse_status_messages(uint8_t *msg, uint8_t *ids, int32_t *data);

#endif //_DYNAMIXEL_MESSAGE_H
