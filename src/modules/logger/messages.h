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

enum class ULogMessageType : uint8_t {
	FORMAT = 'F',
	DATA = 'D',
	INFO = 'I',
	PARAMETER = 'P',
	ADD_LOGGED_MSG = 'A',
	REMOVE_LOGGED_MSG = 'R',
	SYNC = 'S',
	DROPOUT = 'O',
	LOGGING = 'L',
};


/* declare message data structs with byte alignment (no padding) */
#pragma pack(push, 1)

/** first bytes of the file */
struct ulog_file_header_s {
	uint8_t magic[8];
	uint64_t timestamp;
};

#define ULOG_MSG_HEADER_LEN 3 //accounts for msg_size and msg_type
struct ulog_message_header_s {
	uint16_t msg_size;
	uint8_t msg_type;
};

struct ulog_message_format_s {
	uint16_t msg_size; //size of message - ULOG_MSG_HEADER_LEN
	uint8_t msg_type = static_cast<uint8_t>(ULogMessageType::FORMAT);

	char format[2096];
};

struct ulog_message_add_logged_s {
	uint16_t msg_size; //size of message - ULOG_MSG_HEADER_LEN
	uint8_t msg_type = static_cast<uint8_t>(ULogMessageType::ADD_LOGGED_MSG);

	uint8_t multi_id;
	uint16_t msg_id;
	char message_name[255];
};

struct ulog_message_remove_logged_s {
	uint16_t msg_size; //size of message - ULOG_MSG_HEADER_LEN
	uint8_t msg_type = static_cast<uint8_t>(ULogMessageType::REMOVE_LOGGED_MSG);

	uint16_t msg_id;
};

struct ulog_message_sync_s {
	uint16_t msg_size; //size of message - ULOG_MSG_HEADER_LEN
	uint8_t msg_type = static_cast<uint8_t>(ULogMessageType::SYNC);

	uint8_t sync_magic[8];
};

struct ulog_message_dropout_s {
	uint16_t msg_size = sizeof(uint16_t); //size of message - ULOG_MSG_HEADER_LEN
	uint8_t msg_type = static_cast<uint8_t>(ULogMessageType::DROPOUT);

	uint16_t duration; //in ms
};

struct ulog_message_data_header_s {
	uint16_t msg_size; //size of message - ULOG_MSG_HEADER_LEN
	uint8_t msg_type = static_cast<uint8_t>(ULogMessageType::DATA);

	uint16_t msg_id;
};

struct ulog_message_info_header_s {
	uint16_t msg_size; //size of message - ULOG_MSG_HEADER_LEN
	uint8_t msg_type = static_cast<uint8_t>(ULogMessageType::INFO);

	uint8_t key_len;
	char key[255];
};

struct ulog_message_logging_s {
	uint16_t msg_size; //size of message - ULOG_MSG_HEADER_LEN
	uint8_t msg_type = static_cast<uint8_t>(ULogMessageType::LOGGING);

	uint8_t log_level; //same levels as in the linux kernel
	uint64_t timestamp;
	char message[128]; //defines the maximum length of a logged message string
};

struct ulog_message_parameter_header_s {
	uint16_t msg_size;
	uint8_t msg_type = static_cast<uint8_t>(ULogMessageType::PARAMETER);

	uint8_t key_len;
	char key[255];
};
#pragma pack(pop)
