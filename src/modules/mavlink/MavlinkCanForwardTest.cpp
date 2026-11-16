/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
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

#include "mavlink_can_forward.h"

// the send path of the module is not part of this test
mavlink_system_t mavlink_system{};
mavlink_status_t *mavlink_get_channel_status(uint8_t chan) { static mavlink_status_t status{}; return &status; }
mavlink_message_t *mavlink_get_channel_buffer(uint8_t chan) { static mavlink_message_t buffer{}; return &buffer; }
void mavlink_start_uart_send(mavlink_channel_t chan, int length) {}
void mavlink_send_uart_bytes(mavlink_channel_t chan, const uint8_t *ch, int length) {}
void mavlink_end_uart_send(mavlink_channel_t chan, int length) {}

static constexpr float Start = 1.f;
static constexpr float Stop = 0.f;
static constexpr hrt_abstime Timeout = MavlinkCanForward::OwnerTimeout;

// The owner and its keepalive are shared by all instances: every test starts after the previous session has timed out
class MavlinkCanForwardTest : public ::testing::Test
{
protected:
	void SetUp() override { _epoch += 10 * Timeout; }

	static hrt_abstime _epoch;
	MavlinkCanForward _link1;
	MavlinkCanForward _link2;
};

hrt_abstime MavlinkCanForwardTest::_epoch = 0;

TEST_F(MavlinkCanForwardTest, FirstClientOwnsSession)
{
	EXPECT_FALSE(_link1.isOwner(_epoch));
	EXPECT_TRUE(_link1.handleCommand(255, 190, Start, _epoch));
	EXPECT_TRUE(_link1.isOwner(_epoch));
	EXPECT_FALSE(_link2.isOwner(_epoch));
}

TEST_F(MavlinkCanForwardTest, OtherLinkIsDenied)
{
	EXPECT_TRUE(_link1.handleCommand(255, 190, Start, _epoch));

	// the same system and component ID on another link is another client
	EXPECT_FALSE(_link2.handleCommand(255, 190, Start, _epoch + 1));
	EXPECT_FALSE(_link2.handleCommand(255, 190, Stop, _epoch + 2));
	EXPECT_TRUE(_link1.isOwner(_epoch + 3));
	EXPECT_FALSE(_link2.isOwner(_epoch + 3));
}

TEST_F(MavlinkCanForwardTest, OtherClientOnSameLinkIsDenied)
{
	EXPECT_TRUE(_link1.handleCommand(255, 190, Start, _epoch));
	EXPECT_FALSE(_link1.handleCommand(255, 191, Start, _epoch + 1));
	EXPECT_FALSE(_link1.handleCommand(254, 190, Stop, _epoch + 2));
	EXPECT_TRUE(_link1.isOwner(_epoch + 3));
}

TEST_F(MavlinkCanForwardTest, DeniedCommandDoesNotExtendSession)
{
	EXPECT_TRUE(_link1.handleCommand(255, 190, Start, _epoch));
	EXPECT_FALSE(_link2.handleCommand(255, 190, Start, _epoch + Timeout - 1));
	EXPECT_FALSE(_link1.isOwner(_epoch + Timeout));
}

TEST_F(MavlinkCanForwardTest, KeepaliveExtendsSession)
{
	EXPECT_TRUE(_link1.handleCommand(255, 190, Start, _epoch));
	EXPECT_TRUE(_link1.handleCommand(255, 190, Start, _epoch + Timeout - 1));
	EXPECT_TRUE(_link1.isOwner(_epoch + Timeout));
	EXPECT_FALSE(_link2.handleCommand(255, 190, Start, _epoch + Timeout));
}

TEST_F(MavlinkCanForwardTest, SessionTimesOut)
{
	EXPECT_TRUE(_link1.handleCommand(255, 190, Start, _epoch));
	EXPECT_TRUE(_link1.isOwner(_epoch + Timeout - 1));
	EXPECT_FALSE(_link1.isOwner(_epoch + Timeout));

	EXPECT_TRUE(_link2.handleCommand(255, 190, Start, _epoch + Timeout));
	EXPECT_TRUE(_link2.isOwner(_epoch + Timeout));
	EXPECT_FALSE(_link1.isOwner(_epoch + Timeout));

	// the previous owner is now the other client
	EXPECT_FALSE(_link1.handleCommand(255, 190, Start, _epoch + Timeout + 1));
}

TEST_F(MavlinkCanForwardTest, OwnerStopsSession)
{
	EXPECT_TRUE(_link1.handleCommand(255, 190, Start, _epoch));
	EXPECT_TRUE(_link1.handleCommand(255, 190, Stop, _epoch + 1));
	EXPECT_FALSE(_link1.isOwner(_epoch + 2));
	EXPECT_TRUE(_link2.handleCommand(1, 1, Start, _epoch + 3));
	EXPECT_TRUE(_link2.isOwner(_epoch + 3));
}

TEST_F(MavlinkCanForwardTest, InvalidBusKeepsSession)
{
	EXPECT_TRUE(_link1.handleCommand(255, 190, Start, _epoch));
	EXPECT_TRUE(_link1.handleCommand(255, 190, NAN, _epoch + 1));
	EXPECT_TRUE(_link1.isOwner(_epoch + 2));
}

TEST_F(MavlinkCanForwardTest, StopWithoutSessionReachesDriver)
{
	EXPECT_TRUE(_link1.handleCommand(255, 190, Stop, _epoch));
	EXPECT_FALSE(_link1.isOwner(_epoch));
}

TEST_F(MavlinkCanForwardTest, DestructorReleasesSession)
{
	{
		MavlinkCanForward link;
		EXPECT_TRUE(link.handleCommand(255, 190, Start, _epoch));
	}

	EXPECT_TRUE(_link2.handleCommand(255, 190, Start, _epoch + 1));
}
