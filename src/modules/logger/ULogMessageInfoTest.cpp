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

TEST(ULogMessageInfo, InfoMessageLayout)
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

	// msg_size == key_len + vlen + 1 (the +1 is the key_len field itself)
	EXPECT_EQ(msg.msg_size, msg.key_len + vlen + 1);
	EXPECT_EQ(msg.msg_type, static_cast<uint8_t>('I'));

	const char expected_key[] = "char[5] hello";
	EXPECT_EQ(msg.key_len, strlen(expected_key));

	const size_t header_size = sizeof(msg) - sizeof(msg.key_value_str);
	const uint8_t *value_start = &buffer[header_size + msg.key_len];
	EXPECT_EQ(memcmp(value_start, "world", vlen), 0);

	// msg_size must not account for a null terminator
	EXPECT_EQ(msg_size, header_size + msg.key_len + vlen);
}

TEST(ULogMessageInfo, InfoMultipleMessageLayout)
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
	EXPECT_EQ(msg.msg_type, static_cast<uint8_t>('M'));

	const char expected_key[] = "char[3] ver";
	EXPECT_EQ(msg.key_len, strlen(expected_key));

	const uint8_t *value_start = &buffer[header_size + msg.key_len];
	EXPECT_EQ(memcmp(value_start, "1.0", vlen), 0);

	EXPECT_EQ(msg_size, header_size + msg.key_len + vlen);
}
