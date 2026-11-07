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

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include <mathlib/mathlib.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/mavlink_log.h>
#include <mavlink.h>

#include "mavlink_stream.h"

using namespace time_literals;

// Keep the transport connected and uncongested, and capture the sender's
// outgoing structs without starting a MAVLink task or opening a socket.
class Mavlink
{
public:
	bool is_gcs_connected() const { return true; }
	unsigned get_free_tx_buf() const { return 4096; }
	mavlink_channel_t get_channel() const { return MAVLINK_COMM_0; }
};

MavlinkStream::MavlinkStream(Mavlink *mavlink) : _mavlink(mavlink) {}

static std::vector<mavlink_statustext_t> sent_chunks;

static void mavlink_msg_statustext_send_struct(mavlink_channel_t, const mavlink_statustext_t *msg)
{
	sent_chunks.push_back(*msg);
}

#include "streams/STATUSTEXT.hpp"

static constexpr size_t chunk_size = sizeof(mavlink_statustext_t::text);

class MavlinkStatustextStreamTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		// A new subscription also sees the messages earlier tests left queued: send those first.
		while (_stream->request_message()) {}

		sent_chunks.clear();
	}

	// Publish one log message and let the stream send it.
	void send(const std::string &text)
	{
		mavlink_log_s log{};
		log.timestamp = hrt_absolute_time();
		log.severity = MAV_SEVERITY_INFO;
		ASSERT_LT(text.size(), sizeof(log.text));
		memcpy(log.text, text.data(), text.size());
		ASSERT_TRUE(_publication.publish(log));
		ASSERT_TRUE(_stream->request_message());
	}

	// Every chunk of one message: same nonzero id, consecutive chunk_seq, NUL only in the last one.
	void expect_chunks_of_one_message(size_t count)
	{
		ASSERT_EQ(sent_chunks.size(), count);

		for (size_t i = 0; i < count; ++i) {
			EXPECT_NE(sent_chunks[i].id, 0) << "the sender never uses id 0, which marks a STATUSTEXT that is not chunked";
			EXPECT_EQ(sent_chunks[i].id, sent_chunks.front().id);
			EXPECT_EQ(sent_chunks[i].chunk_seq, i);
			const bool has_nul = memchr(sent_chunks[i].text, '\0', chunk_size) != nullptr;
			EXPECT_EQ(has_nul, i == count - 1) << "chunk " << i;
		}
	}

	std::string text_of_chunk(size_t i) const { return std::string(sent_chunks[i].text, strnlen(sent_chunks[i].text, chunk_size)); }

	Mavlink _transport;
	std::unique_ptr<MavlinkStream> _stream{MavlinkStreamStatustext::new_instance(&_transport)};
	uORB::Publication<mavlink_log_s> _publication{ORB_ID(mavlink_log)};
};

TEST_F(MavlinkStatustextStreamTest, ShortTextIsOneChunk)
{
	send("Short text");
	expect_chunks_of_one_message(1);
	EXPECT_EQ(text_of_chunk(0), "Short text");
}

TEST_F(MavlinkStatustextStreamTest, EmptyTextSendsNothing)
{
	send("");
	EXPECT_TRUE(sent_chunks.empty());
}

TEST_F(MavlinkStatustextStreamTest, FullChunkIsFollowedByEmptyChunk)
{
	send(std::string(chunk_size, 'a'));
	expect_chunks_of_one_message(2);
	EXPECT_EQ(text_of_chunk(0), std::string(chunk_size, 'a'));
	EXPECT_EQ(text_of_chunk(1), "");
}

TEST_F(MavlinkStatustextStreamTest, FirstTextOfTwoFullChunksIsTerminated)
{
	// The first message used to get id 0, and two full chunks ended without a NUL.
	send(std::string(2 * chunk_size, 'a'));
	expect_chunks_of_one_message(3);
	EXPECT_EQ(text_of_chunk(0) + text_of_chunk(1), std::string(2 * chunk_size, 'a'));
	EXPECT_EQ(text_of_chunk(2), "");
}

TEST_F(MavlinkStatustextStreamTest, PartialLastChunkNeedsNoEmptyChunk)
{
	const std::string text = std::string(2 * chunk_size, 'a') + "tail";
	send(text);
	expect_chunks_of_one_message(3);
	EXPECT_EQ(text_of_chunk(0) + text_of_chunk(1) + text_of_chunk(2), text);
}

TEST_F(MavlinkStatustextStreamTest, IdSkipsZeroWhenItWraps)
{
	// One full cycle of the 16-bit id and one more message.
	for (uint32_t i = 0; i <= UINT16_MAX + 1; ++i) {
		send("x");
	}

	ASSERT_EQ(sent_chunks.size(), UINT16_MAX + 2u);
	bool wrapped = false;

	for (size_t i = 1; i < sent_chunks.size(); ++i) {
		const uint16_t previous = sent_chunks[i - 1].id;
		wrapped |= previous == UINT16_MAX;
		ASSERT_EQ(sent_chunks[i].id, previous == UINT16_MAX ? 1 : previous + 1) << "message " << i;
	}

	EXPECT_TRUE(wrapped);
}
