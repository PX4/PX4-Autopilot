/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "MavlinkStatustextHandler.hpp"
#include <gtest/gtest.h>

static constexpr uint64_t some_time = 12345678;
static constexpr uint64_t some_other_time = 99999999;
static constexpr auto some_text = "Some short text";
static constexpr auto some_other_text = "Some more short text";
static constexpr uint8_t some_severity = MAV_SEVERITY_CRITICAL;
static constexpr uint8_t some_other_severity = MAV_SEVERITY_INFO;

TEST(MavlinkStatustextHandler, Singles)
{
	MavlinkStatustextHandler handler;

	mavlink_statustext_t statustext1 {};
	statustext1.severity = some_severity;
	strncpy(statustext1.text, some_text, sizeof(statustext1.text));

	EXPECT_FALSE(handler.should_publish_previous(statustext1));
	EXPECT_TRUE(handler.should_publish_current(statustext1, some_time));
	EXPECT_STREQ(handler.log_message().text, some_text);
	EXPECT_EQ(handler.log_message().severity, some_severity);
	EXPECT_EQ(handler.log_message().timestamp, some_time);

	mavlink_statustext_t statustext2 {};
	statustext2.severity = some_other_severity;
	strncpy(statustext2.text, some_other_text, sizeof(statustext2.text));

	EXPECT_FALSE(handler.should_publish_previous(statustext2));
	EXPECT_TRUE(handler.should_publish_current(statustext2, some_other_time));
	EXPECT_STREQ(handler.log_message().text, some_other_text);
	EXPECT_EQ(handler.log_message().severity, some_other_severity);
	EXPECT_EQ(handler.log_message().timestamp, some_other_time);
}

TEST(MavlinkStatustextHandler, Multiple)
{
	MavlinkStatustextHandler handler;

	mavlink_statustext_t statustext1 {};
	mavlink_statustext_t statustext2 {};

	statustext1.severity = some_severity;
	statustext2.severity = some_severity;

	statustext1.id = 1;
	statustext2.id = 1;

	statustext1.chunk_seq = 0;
	statustext2.chunk_seq = 1;

	memset(statustext1.text, 'a', 50);
	memset(statustext2.text, 'b', 25);

	// The first message is just stored, we don't have to publish yet.
	EXPECT_FALSE(handler.should_publish_previous(statustext1));
	EXPECT_FALSE(handler.should_publish_current(statustext1, some_time));
	// Now we should be able to publish.
	EXPECT_FALSE(handler.should_publish_previous(statustext2));
	EXPECT_TRUE(handler.should_publish_current(statustext2, some_other_time));

	// Check overall text length.
	EXPECT_EQ(strlen(handler.log_message().text), 75);

	// And a few samples.
	EXPECT_EQ(handler.log_message().text[30], 'a');
	EXPECT_EQ(handler.log_message().text[60], 'b');

	EXPECT_EQ(handler.log_message().severity, some_severity);
	EXPECT_EQ(handler.log_message().timestamp, some_time);
}

TEST(MavlinkStatustextHandler, TooMany)
{
	// If we receive too many, we need to cap it.
	MavlinkStatustextHandler handler;

	mavlink_statustext_t statustext1 {};
	mavlink_statustext_t statustext2 {};
	mavlink_statustext_t statustext3 {};

	statustext1.id = 1;
	statustext2.id = 1;
	statustext3.id = 1;

	statustext1.chunk_seq = 0;
	statustext2.chunk_seq = 1;
	statustext3.chunk_seq = 2;

	memset(statustext1.text, 'a', 50);
	memset(statustext2.text, 'b', 50);
	memset(statustext3.text, 'c', 49);

	// The first messages are just stored, we don't have to publish yet.
	EXPECT_FALSE(handler.should_publish_previous(statustext1));
	EXPECT_FALSE(handler.should_publish_current(statustext1, some_time));
	EXPECT_FALSE(handler.should_publish_previous(statustext2));
	EXPECT_FALSE(handler.should_publish_current(statustext2, some_time + 1));
	EXPECT_FALSE(handler.should_publish_previous(statustext3));
	EXPECT_TRUE(handler.should_publish_current(statustext3, some_time + 2));

	EXPECT_EQ(strlen(handler.log_message().text), sizeof(log_message_s::text) - 1);
}

TEST(MavlinkStatustextHandler, LastMissing)
{
	// Here we fail to send the last multi-chunk but we still want to
	// publish the incomplete message.

	MavlinkStatustextHandler handler;

	mavlink_statustext_t statustext1 {};
	statustext1.id = 1;
	statustext1.chunk_seq = 0;

	memset(statustext1.text, 'a', 50);

	// The first message is just stored, we don't have to publish yet.
	EXPECT_FALSE(handler.should_publish_previous(statustext1));
	EXPECT_FALSE(handler.should_publish_current(statustext1, some_time));

	// Now we send a single and we expect the previous one to be published.
	mavlink_statustext_t statustext2 {};
	memset(statustext2.text, 'b', 10);
	statustext2.id = 0;

	// We expect 50 a and then a missing bracket.
	EXPECT_TRUE(handler.should_publish_previous(statustext2));
	EXPECT_EQ(strlen(handler.log_message().text), 50 + 15);
	EXPECT_EQ(handler.log_message().text[25], 'a');

	EXPECT_TRUE(handler.should_publish_previous(statustext2));
	EXPECT_TRUE(handler.should_publish_current(statustext2, some_other_time));
	// And we expect the single message to get through as well.
	EXPECT_EQ(strlen(handler.log_message().text), 10);
	EXPECT_EQ(handler.log_message().text[5], 'b');
}

TEST(MavlinkStatustextHandler, FirstMissing)
{
	// Here we fail to send the first multi-chunk but we still want to
	// publish the incomplete message.

	MavlinkStatustextHandler handler;

	mavlink_statustext_t statustext2 {};
	statustext2.id = 1;
	statustext2.chunk_seq = 1;

	// By chosing 35 we might exactly trigger an overflow if we don't pay attention.
	memset(statustext2.text, 'b', 35);

	// The first message is just stored, we don't have to publish yet.
	EXPECT_FALSE(handler.should_publish_previous(statustext2));
	EXPECT_TRUE(handler.should_publish_current(statustext2, some_time));

	// We expect a missing bracket and our 10 b.
	EXPECT_EQ(strlen(handler.log_message().text), 15 + 35);
	EXPECT_EQ(handler.log_message().text[20], 'b');
}

TEST(MavlinkStatustextHandler, MiddleMissing)
{
	// This time one message in the middle goes missing.
	MavlinkStatustextHandler handler;

	mavlink_statustext_t statustext1 {};
	mavlink_statustext_t statustext3 {};

	statustext1.id = 1;
	statustext3.id = 1;

	statustext1.chunk_seq = 0;
	statustext3.chunk_seq = 2;

	memset(statustext1.text, 'a', 50);
	memset(statustext3.text, 'c', 25);

	// The first messages are just stored, we don't have to publish yet.
	EXPECT_FALSE(handler.should_publish_previous(statustext1));
	EXPECT_FALSE(handler.should_publish_current(statustext1, some_time));
	EXPECT_FALSE(handler.should_publish_previous(statustext3));
	EXPECT_TRUE(handler.should_publish_current(statustext3, some_time + 2));

	// The two texts plus the missing bracket.
	EXPECT_EQ(strlen(handler.log_message().text), 50 + 25 + 15);
	EXPECT_EQ(handler.log_message().text[20], 'a');
	EXPECT_EQ(handler.log_message().text[70], 'c');
}
