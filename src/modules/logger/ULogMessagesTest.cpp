/****************************************************************************
 *
 *   Copyright (C) 2025 PX4 Development Team. All rights reserved.
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

#include <gtest/gtest.h>
#include <cstdio>
#include <cstring>
#include "messages.h"

// ----------------------------------------------------------------------------
// Header & constants
// ----------------------------------------------------------------------------

TEST(ULogMessages, HeaderLen)
{
	EXPECT_EQ(ULOG_MSG_HEADER_LEN, 3u);
}

TEST(ULogMessages, MessageHeaderSize)
{
	EXPECT_EQ(sizeof(ulog_message_header_s), 3u);
}

TEST(ULogMessages, MessageHeaderFieldOffsets)
{
	ulog_message_header_s msg = {};
	uint8_t *base = reinterpret_cast<uint8_t *>(&msg);

	EXPECT_EQ(reinterpret_cast<uint8_t *>(&msg.msg_size) - base, 0);
	EXPECT_EQ(reinterpret_cast<uint8_t *>(&msg.msg_type) - base, 2);
}

// ----------------------------------------------------------------------------
// File header (16 bytes: 8 magic + 8 timestamp)
// ----------------------------------------------------------------------------

TEST(ULogMessages, FileHeaderSize)
{
	EXPECT_EQ(sizeof(ulog_file_header_s), 16u);
}

TEST(ULogMessages, FileHeaderMagic)
{
	ulog_file_header_s hdr = {};
	const uint8_t expected_magic[] = {'U', 'L', 'o', 'g', 0x01, 0x12, 0x35, 0x01};

	memcpy(hdr.magic, expected_magic, sizeof(expected_magic));
	hdr.timestamp = 1000000ULL;

	EXPECT_EQ(memcmp(hdr.magic, expected_magic, 7), 0);
	EXPECT_EQ(hdr.magic[7], 0x01);
	EXPECT_EQ(hdr.timestamp, 1000000ULL);
}

TEST(ULogMessages, FileHeaderFieldOffsets)
{
	ulog_file_header_s hdr = {};
	uint8_t *base = reinterpret_cast<uint8_t *>(&hdr);

	EXPECT_EQ(reinterpret_cast<uint8_t *>(&hdr.magic) - base, 0);
	EXPECT_EQ(reinterpret_cast<uint8_t *>(&hdr.timestamp) - base, 8);
}

// ----------------------------------------------------------------------------
// Flag Bits ('B') — must be first message after header
// ----------------------------------------------------------------------------

TEST(ULogMessages, FlagBitsSize)
{
	// payload: 8 compat + 8 incompat + 24 appended_offsets = 40
	EXPECT_EQ(sizeof(ulog_message_flag_bits_s), ULOG_MSG_HEADER_LEN + 40u);
}

TEST(ULogMessages, FlagBitsTypeCode)
{
	ulog_message_flag_bits_s msg = {};
	EXPECT_EQ(msg.msg_type, static_cast<uint8_t>('B'));
}

TEST(ULogMessages, FlagBitsFieldOffsets)
{
	ulog_message_flag_bits_s msg = {};
	uint8_t *base = reinterpret_cast<uint8_t *>(&msg);

	EXPECT_EQ(reinterpret_cast<uint8_t *>(&msg.compat_flags) - base, ULOG_MSG_HEADER_LEN);
	EXPECT_EQ(reinterpret_cast<uint8_t *>(&msg.incompat_flags) - base, ULOG_MSG_HEADER_LEN + 8);
	EXPECT_EQ(reinterpret_cast<uint8_t *>(&msg.appended_offsets) - base, ULOG_MSG_HEADER_LEN + 16);
}

TEST(ULogMessages, FlagBitsDataAppendedMask)
{
	EXPECT_EQ(ULOG_INCOMPAT_FLAG0_DATA_APPENDED_MASK, 1 << 0);
	EXPECT_EQ(ULOG_COMPAT_FLAG0_DEFAULT_PARAMETERS_MASK, 1 << 0);
}

// ----------------------------------------------------------------------------
// Format Definition ('F')
// ----------------------------------------------------------------------------

TEST(ULogMessages, FormatSize)
{
	EXPECT_EQ(sizeof(ulog_message_format_s), ULOG_MSG_HEADER_LEN + 1600u);
}

TEST(ULogMessages, FormatTypeCode)
{
	ulog_message_format_s msg = {};
	EXPECT_EQ(msg.msg_type, static_cast<uint8_t>('F'));
}

TEST(ULogMessages, FormatSerialization)
{
	ulog_message_format_s msg = {};
	const char *format_str = "test_topic:uint64_t timestamp;float value;";
	size_t len = strlen(format_str);

	strncpy(msg.format, format_str, sizeof(msg.format));
	msg.msg_size = len;

	EXPECT_EQ(msg.msg_size, len);
	EXPECT_EQ(memcmp(msg.format, format_str, len), 0);
}

// ----------------------------------------------------------------------------
// Information ('I')
// ----------------------------------------------------------------------------

TEST(ULogMessages, InfoSize)
{
	// header(3) + key_len(1) + key_value_str(255) = 259
	EXPECT_EQ(sizeof(ulog_message_info_s), ULOG_MSG_HEADER_LEN + 1u + 255u);
}

TEST(ULogMessages, InfoTypeCode)
{
	ulog_message_info_s msg = {};
	EXPECT_EQ(msg.msg_type, static_cast<uint8_t>('I'));
}

TEST(ULogMessages, InfoStringLayout)
{
	const char *name  = "hello";
	const char *value = "world";
	const size_t vlen = strlen(value);

	ulog_message_info_s msg = {};
	uint8_t *buffer = reinterpret_cast<uint8_t *>(&msg);
	msg.msg_type = static_cast<uint8_t>(ULogMessageType::INFO);

	msg.key_len = snprintf(msg.key_value_str, sizeof(msg.key_value_str),
			       "char[%zu] %s", vlen, name);
	size_t msg_size = sizeof(msg) - sizeof(msg.key_value_str) + msg.key_len;

	ASSERT_LT(vlen, sizeof(msg) - msg_size);
	memcpy(&buffer[msg_size], value, vlen);
	msg_size += vlen;

	msg.msg_size = msg_size - ULOG_MSG_HEADER_LEN;

	// msg_size == key_len_field(1) + key_len + vlen
	EXPECT_EQ(msg.msg_size, msg.key_len + vlen + 1);

	const char expected_key[] = "char[5] hello";
	EXPECT_EQ(msg.key_len, strlen(expected_key));

	const size_t header_size = sizeof(msg) - sizeof(msg.key_value_str);
	const uint8_t *value_start = &buffer[header_size + msg.key_len];
	EXPECT_EQ(memcmp(value_start, "world", vlen), 0);

	// no null terminator in msg_size
	EXPECT_EQ(msg_size, header_size + msg.key_len + vlen);
}

TEST(ULogMessages, InfoInt32Layout)
{
	const char *name = "sys_param";
	int32_t value = 42;

	ulog_message_info_s msg = {};
	uint8_t *buffer = reinterpret_cast<uint8_t *>(&msg);
	msg.msg_type = static_cast<uint8_t>(ULogMessageType::INFO);

	msg.key_len = snprintf(msg.key_value_str, sizeof(msg.key_value_str),
			       "int32_t %s", name);
	size_t msg_size = sizeof(msg) - sizeof(msg.key_value_str) + msg.key_len;

	ASSERT_LE(sizeof(value), sizeof(msg) - msg_size);
	memcpy(&buffer[msg_size], &value, sizeof(value));
	msg_size += sizeof(value);

	msg.msg_size = msg_size - ULOG_MSG_HEADER_LEN;

	EXPECT_EQ(msg.msg_size, msg.key_len + sizeof(int32_t) + 1);

	const char expected_key[] = "int32_t sys_param";
	EXPECT_EQ(msg.key_len, strlen(expected_key));

	const size_t header_size = sizeof(msg) - sizeof(msg.key_value_str);
	int32_t read_back = 0;
	memcpy(&read_back, &buffer[header_size + msg.key_len], sizeof(int32_t));
	EXPECT_EQ(read_back, 42);
}

TEST(ULogMessages, InfoFloatLayout)
{
	const char *name = "cal_offset";
	float value = 3.14f;

	ulog_message_info_s msg = {};
	uint8_t *buffer = reinterpret_cast<uint8_t *>(&msg);
	msg.msg_type = static_cast<uint8_t>(ULogMessageType::INFO);

	msg.key_len = snprintf(msg.key_value_str, sizeof(msg.key_value_str),
			       "float %s", name);
	size_t msg_size = sizeof(msg) - sizeof(msg.key_value_str) + msg.key_len;

	ASSERT_LE(sizeof(value), sizeof(msg) - msg_size);
	memcpy(&buffer[msg_size], &value, sizeof(value));
	msg_size += sizeof(value);

	msg.msg_size = msg_size - ULOG_MSG_HEADER_LEN;

	EXPECT_EQ(msg.msg_size, msg.key_len + sizeof(float) + 1);

	const size_t header_size = sizeof(msg) - sizeof(msg.key_value_str);
	float read_back = 0.f;
	memcpy(&read_back, &buffer[header_size + msg.key_len], sizeof(float));
	EXPECT_FLOAT_EQ(read_back, 3.14f);
}

// ----------------------------------------------------------------------------
// Information Multiple ('M')
// ----------------------------------------------------------------------------

TEST(ULogMessages, InfoMultipleSize)
{
	// header(3) + is_continued(1) + key_len(1) + key_value_str(1200) = 1205
	EXPECT_EQ(sizeof(ulog_message_info_multiple_s), ULOG_MSG_HEADER_LEN + 1u + 1u + 1200u);
}

TEST(ULogMessages, InfoMultipleTypeCode)
{
	ulog_message_info_multiple_s msg = {};
	EXPECT_EQ(msg.msg_type, static_cast<uint8_t>('M'));
}

TEST(ULogMessages, InfoMultipleStringLayout)
{
	const char *name  = "ver";
	const char *value = "1.0";
	const size_t vlen = strlen(value);

	ulog_message_info_multiple_s msg = {};
	uint8_t *buffer = reinterpret_cast<uint8_t *>(&msg);
	msg.msg_type = static_cast<uint8_t>(ULogMessageType::INFO_MULTIPLE);
	msg.is_continued = false;

	msg.key_len = snprintf(msg.key_value_str, sizeof(msg.key_value_str),
			       "char[%zu] %s", vlen, name);
	size_t msg_size = sizeof(msg) - sizeof(msg.key_value_str) + msg.key_len;

	ASSERT_LT(vlen, sizeof(msg) - msg_size);
	memcpy(&buffer[msg_size], value, vlen);
	msg_size += vlen;

	msg.msg_size = msg_size - ULOG_MSG_HEADER_LEN;

	const size_t header_size = sizeof(msg) - sizeof(msg.key_value_str);
	EXPECT_EQ(msg.msg_size, msg_size - ULOG_MSG_HEADER_LEN);
	EXPECT_EQ(msg.is_continued, 0);

	const char expected_key[] = "char[3] ver";
	EXPECT_EQ(msg.key_len, strlen(expected_key));

	const uint8_t *value_start = &buffer[header_size + msg.key_len];
	EXPECT_EQ(memcmp(value_start, "1.0", vlen), 0);

	EXPECT_EQ(msg_size, header_size + msg.key_len + vlen);
}

TEST(ULogMessages, InfoMultipleContinuation)
{
	ulog_message_info_multiple_s msg = {};
	msg.is_continued = 1;

	EXPECT_EQ(msg.is_continued, 1);
	EXPECT_EQ(msg.msg_type, static_cast<uint8_t>('M'));
}

TEST(ULogMessages, InfoMultipleFieldOffsets)
{
	ulog_message_info_multiple_s msg = {};
	uint8_t *base = reinterpret_cast<uint8_t *>(&msg);

	EXPECT_EQ(reinterpret_cast<uint8_t *>(&msg.is_continued) - base, ULOG_MSG_HEADER_LEN);
	EXPECT_EQ(reinterpret_cast<uint8_t *>(&msg.key_len) - base, ULOG_MSG_HEADER_LEN + 1);
	EXPECT_EQ(reinterpret_cast<uint8_t *>(&msg.key_value_str) - base, ULOG_MSG_HEADER_LEN + 2);
}

// ----------------------------------------------------------------------------
// Parameter ('P')
// ----------------------------------------------------------------------------

TEST(ULogMessages, ParameterSize)
{
	EXPECT_EQ(sizeof(ulog_message_parameter_s), ULOG_MSG_HEADER_LEN + 1u + 255u);
}

TEST(ULogMessages, ParameterTypeCode)
{
	ulog_message_parameter_s msg = {};
	EXPECT_EQ(msg.msg_type, static_cast<uint8_t>('P'));
}

TEST(ULogMessages, ParameterInt32Layout)
{
	const char *name = "SYS_AUTOSTART";
	int32_t value = 4001;

	ulog_message_parameter_s msg = {};
	uint8_t *buffer = reinterpret_cast<uint8_t *>(&msg);

	msg.key_len = snprintf(msg.key_value_str, sizeof(msg.key_value_str),
			       "int32_t %s", name);
	size_t msg_size = sizeof(msg) - sizeof(msg.key_value_str) + msg.key_len;

	memcpy(&buffer[msg_size], &value, sizeof(value));
	msg_size += sizeof(value);

	msg.msg_size = msg_size - ULOG_MSG_HEADER_LEN;

	EXPECT_EQ(msg.msg_size, msg.key_len + sizeof(int32_t) + 1);

	const size_t header_size = sizeof(msg) - sizeof(msg.key_value_str);
	int32_t read_back = 0;
	memcpy(&read_back, &buffer[header_size + msg.key_len], sizeof(int32_t));
	EXPECT_EQ(read_back, 4001);
}

TEST(ULogMessages, ParameterFloatLayout)
{
	const char *name = "MC_PITCHRATE_P";
	float value = 0.15f;

	ulog_message_parameter_s msg = {};
	uint8_t *buffer = reinterpret_cast<uint8_t *>(&msg);

	msg.key_len = snprintf(msg.key_value_str, sizeof(msg.key_value_str),
			       "float %s", name);
	size_t msg_size = sizeof(msg) - sizeof(msg.key_value_str) + msg.key_len;

	memcpy(&buffer[msg_size], &value, sizeof(value));
	msg_size += sizeof(value);

	msg.msg_size = msg_size - ULOG_MSG_HEADER_LEN;

	const size_t header_size = sizeof(msg) - sizeof(msg.key_value_str);
	float read_back = 0.f;
	memcpy(&read_back, &buffer[header_size + msg.key_len], sizeof(float));
	EXPECT_FLOAT_EQ(read_back, 0.15f);
}

// ----------------------------------------------------------------------------
// Default Parameter ('Q')
// ----------------------------------------------------------------------------

TEST(ULogMessages, ParameterDefaultSize)
{
	// header(3) + default_types(1) + key_len(1) + key_value_str(255) = 260
	EXPECT_EQ(sizeof(ulog_message_parameter_default_s), ULOG_MSG_HEADER_LEN + 1u + 1u + 255u);
}

TEST(ULogMessages, ParameterDefaultTypeCode)
{
	ulog_message_parameter_default_s msg = {};
	EXPECT_EQ(msg.msg_type, static_cast<uint8_t>('Q'));
}

TEST(ULogMessages, ParameterDefaultTypeBits)
{
	EXPECT_EQ(static_cast<uint8_t>(ulog_parameter_default_type_t::system), 1 << 0);
	EXPECT_EQ(static_cast<uint8_t>(ulog_parameter_default_type_t::current_setup), 1 << 1);

	auto combined = ulog_parameter_default_type_t::system | ulog_parameter_default_type_t::current_setup;
	EXPECT_EQ(static_cast<uint8_t>(combined), 0x03);
}

TEST(ULogMessages, ParameterDefaultFieldOffsets)
{
	ulog_message_parameter_default_s msg = {};
	uint8_t *base = reinterpret_cast<uint8_t *>(&msg);

	EXPECT_EQ(reinterpret_cast<uint8_t *>(&msg.default_types) - base, ULOG_MSG_HEADER_LEN);
	EXPECT_EQ(reinterpret_cast<uint8_t *>(&msg.key_len) - base, ULOG_MSG_HEADER_LEN + 1);
	EXPECT_EQ(reinterpret_cast<uint8_t *>(&msg.key_value_str) - base, ULOG_MSG_HEADER_LEN + 2);
}

// ----------------------------------------------------------------------------
// Subscription ('A')
// ----------------------------------------------------------------------------

TEST(ULogMessages, AddLoggedSize)
{
	// header(3) + multi_id(1) + msg_id(2) + message_name(255) = 261
	EXPECT_EQ(sizeof(ulog_message_add_logged_s), ULOG_MSG_HEADER_LEN + 1u + 2u + 255u);
}

TEST(ULogMessages, AddLoggedTypeCode)
{
	ulog_message_add_logged_s msg = {};
	EXPECT_EQ(msg.msg_type, static_cast<uint8_t>('A'));
}

TEST(ULogMessages, AddLoggedFieldOffsets)
{
	ulog_message_add_logged_s msg = {};
	uint8_t *base = reinterpret_cast<uint8_t *>(&msg);

	EXPECT_EQ(reinterpret_cast<uint8_t *>(&msg.multi_id) - base, ULOG_MSG_HEADER_LEN);
	EXPECT_EQ(reinterpret_cast<uint8_t *>(&msg.msg_id) - base, ULOG_MSG_HEADER_LEN + 1);
	EXPECT_EQ(reinterpret_cast<uint8_t *>(&msg.message_name) - base, ULOG_MSG_HEADER_LEN + 3);
}

TEST(ULogMessages, AddLoggedSerialization)
{
	ulog_message_add_logged_s msg = {};
	msg.multi_id = 0;
	msg.msg_id = 7;
	const char *topic = "sensor_accel";
	size_t len = strlen(topic);

	strncpy(msg.message_name, topic, sizeof(msg.message_name));
	msg.msg_size = sizeof(msg.multi_id) + sizeof(msg.msg_id) + len;

	EXPECT_EQ(msg.multi_id, 0);
	EXPECT_EQ(msg.msg_id, 7);
	EXPECT_EQ(memcmp(msg.message_name, "sensor_accel", len), 0);
}

// ----------------------------------------------------------------------------
// Unsubscription ('R')
// ----------------------------------------------------------------------------

TEST(ULogMessages, RemoveLoggedSize)
{
	// header(3) + msg_id(2) = 5
	EXPECT_EQ(sizeof(ulog_message_remove_logged_s), ULOG_MSG_HEADER_LEN + 2u);
}

TEST(ULogMessages, RemoveLoggedTypeCode)
{
	ulog_message_remove_logged_s msg = {};
	EXPECT_EQ(msg.msg_type, static_cast<uint8_t>('R'));
}

TEST(ULogMessages, RemoveLoggedSerialization)
{
	ulog_message_remove_logged_s msg = {};
	msg.msg_id = 42;
	msg.msg_size = sizeof(msg.msg_id);

	EXPECT_EQ(msg.msg_id, 42);
	EXPECT_EQ(msg.msg_size, 2);
}

// ----------------------------------------------------------------------------
// Logged Data ('D')
// ----------------------------------------------------------------------------

TEST(ULogMessages, DataSize)
{
	// header(3) + msg_id(2) = 5 (variable-length payload follows)
	EXPECT_EQ(sizeof(ulog_message_data_s), ULOG_MSG_HEADER_LEN + 2u);
}

TEST(ULogMessages, DataTypeCode)
{
	ulog_message_data_s msg = {};
	EXPECT_EQ(msg.msg_type, static_cast<uint8_t>('D'));
}

TEST(ULogMessages, DataFieldOffset)
{
	ulog_message_data_s msg = {};
	uint8_t *base = reinterpret_cast<uint8_t *>(&msg);

	EXPECT_EQ(reinterpret_cast<uint8_t *>(&msg.msg_id) - base, ULOG_MSG_HEADER_LEN);
}

// ----------------------------------------------------------------------------
// Logging ('L')
// ----------------------------------------------------------------------------

TEST(ULogMessages, LoggingSize)
{
	// header(3) + log_level(1) + timestamp(8) + message(128) = 140
	EXPECT_EQ(sizeof(ulog_message_logging_s), ULOG_MSG_HEADER_LEN + 1u + 8u + 128u);
}

TEST(ULogMessages, LoggingTypeCode)
{
	ulog_message_logging_s msg = {};
	EXPECT_EQ(msg.msg_type, static_cast<uint8_t>('L'));
}

TEST(ULogMessages, LoggingFieldOffsets)
{
	ulog_message_logging_s msg = {};
	uint8_t *base = reinterpret_cast<uint8_t *>(&msg);

	EXPECT_EQ(reinterpret_cast<uint8_t *>(&msg.log_level) - base, ULOG_MSG_HEADER_LEN);
	EXPECT_EQ(reinterpret_cast<uint8_t *>(&msg.timestamp) - base, ULOG_MSG_HEADER_LEN + 1);
	EXPECT_EQ(reinterpret_cast<uint8_t *>(&msg.message) - base, ULOG_MSG_HEADER_LEN + 9);
}

TEST(ULogMessages, LoggingSerialization)
{
	ulog_message_logging_s msg = {};
	msg.log_level = 6;
	msg.timestamp = 5000000ULL;
	const char *text = "test message";
	size_t len = strlen(text);

	strncpy(msg.message, text, sizeof(msg.message));
	msg.msg_size = sizeof(msg.log_level) + sizeof(msg.timestamp) + len;

	EXPECT_EQ(msg.log_level, 6);
	EXPECT_EQ(msg.timestamp, 5000000ULL);
	EXPECT_EQ(memcmp(msg.message, "test message", len), 0);
}

// ----------------------------------------------------------------------------
// Logging Tagged ('C')
// ----------------------------------------------------------------------------

TEST(ULogMessages, LoggingTaggedSize)
{
	// header(3) + log_level(1) + tag(2) + timestamp(8) + message(128) = 142
	EXPECT_EQ(sizeof(ulog_message_logging_tagged_s), ULOG_MSG_HEADER_LEN + 1u + 2u + 8u + 128u);
}

TEST(ULogMessages, LoggingTaggedTypeCode)
{
	ulog_message_logging_tagged_s msg = {};
	EXPECT_EQ(msg.msg_type, static_cast<uint8_t>('C'));
}

TEST(ULogMessages, LoggingTaggedFieldOffsets)
{
	ulog_message_logging_tagged_s msg = {};
	uint8_t *base = reinterpret_cast<uint8_t *>(&msg);

	EXPECT_EQ(reinterpret_cast<uint8_t *>(&msg.log_level) - base, ULOG_MSG_HEADER_LEN);
	EXPECT_EQ(reinterpret_cast<uint8_t *>(&msg.tag) - base, ULOG_MSG_HEADER_LEN + 1);
	EXPECT_EQ(reinterpret_cast<uint8_t *>(&msg.timestamp) - base, ULOG_MSG_HEADER_LEN + 3);
	EXPECT_EQ(reinterpret_cast<uint8_t *>(&msg.message) - base, ULOG_MSG_HEADER_LEN + 11);
}

// ----------------------------------------------------------------------------
// Sync ('S')
// ----------------------------------------------------------------------------

TEST(ULogMessages, SyncSize)
{
	// header(3) + sync_magic(8) = 11
	EXPECT_EQ(sizeof(ulog_message_sync_s), ULOG_MSG_HEADER_LEN + 8u);
}

TEST(ULogMessages, SyncTypeCode)
{
	ulog_message_sync_s msg = {};
	EXPECT_EQ(msg.msg_type, static_cast<uint8_t>('S'));
}

TEST(ULogMessages, SyncMagicBytes)
{
	ulog_message_sync_s msg = {};
	const uint8_t expected_magic[] = {0x2F, 0x73, 0x13, 0x20, 0x25, 0x0C, 0xBB, 0x12};

	memcpy(msg.sync_magic, expected_magic, sizeof(expected_magic));
	msg.msg_size = sizeof(msg.sync_magic);

	EXPECT_EQ(memcmp(msg.sync_magic, expected_magic, 8), 0);
	EXPECT_EQ(msg.msg_size, 8);
}

// ----------------------------------------------------------------------------
// Dropout ('O')
// ----------------------------------------------------------------------------

TEST(ULogMessages, DropoutSize)
{
	// header(3) + duration(2) = 5
	EXPECT_EQ(sizeof(ulog_message_dropout_s), ULOG_MSG_HEADER_LEN + 2u);
}

TEST(ULogMessages, DropoutTypeCode)
{
	ulog_message_dropout_s msg = {};
	EXPECT_EQ(msg.msg_type, static_cast<uint8_t>('O'));
}

TEST(ULogMessages, DropoutDefaultMsgSize)
{
	ulog_message_dropout_s msg = {};
	EXPECT_EQ(msg.msg_size, sizeof(uint16_t));
}

TEST(ULogMessages, DropoutSerialization)
{
	ulog_message_dropout_s msg = {};
	msg.duration = 150;

	EXPECT_EQ(msg.duration, 150);
	EXPECT_EQ(msg.msg_size, sizeof(uint16_t));
}

// ----------------------------------------------------------------------------
// ULogMessageType enum values
// ----------------------------------------------------------------------------

TEST(ULogMessages, TypeEnumValues)
{
	EXPECT_EQ(static_cast<uint8_t>(ULogMessageType::FORMAT), 'F');
	EXPECT_EQ(static_cast<uint8_t>(ULogMessageType::DATA), 'D');
	EXPECT_EQ(static_cast<uint8_t>(ULogMessageType::INFO), 'I');
	EXPECT_EQ(static_cast<uint8_t>(ULogMessageType::INFO_MULTIPLE), 'M');
	EXPECT_EQ(static_cast<uint8_t>(ULogMessageType::PARAMETER), 'P');
	EXPECT_EQ(static_cast<uint8_t>(ULogMessageType::PARAMETER_DEFAULT), 'Q');
	EXPECT_EQ(static_cast<uint8_t>(ULogMessageType::ADD_LOGGED_MSG), 'A');
	EXPECT_EQ(static_cast<uint8_t>(ULogMessageType::REMOVE_LOGGED_MSG), 'R');
	EXPECT_EQ(static_cast<uint8_t>(ULogMessageType::SYNC), 'S');
	EXPECT_EQ(static_cast<uint8_t>(ULogMessageType::DROPOUT), 'O');
	EXPECT_EQ(static_cast<uint8_t>(ULogMessageType::LOGGING), 'L');
	EXPECT_EQ(static_cast<uint8_t>(ULogMessageType::LOGGING_TAGGED), 'C');
	EXPECT_EQ(static_cast<uint8_t>(ULogMessageType::FLAG_BITS), 'B');
}
