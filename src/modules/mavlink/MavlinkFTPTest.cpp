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

#include "mavlink_ftp_internal.h"

TEST(MavlinkFTP, CachedReplyResendZeroFillsUnusedPayloadBytes)
{
	EXPECT_EQ(kMavlinkFTPCachedReplyLength, 19u);

	mavlink_file_transfer_protocol_t reply{};
	memset(&reply, 0xA5, sizeof(reply));

	reply.target_system = 42;
	reply.target_component = 7;

	auto *payload = reinterpret_cast<MavlinkFTP::PayloadHeader *>(&reply.payload[0]);
	payload->seq_number = 512;
	payload->opcode = MavlinkFTP::kRspNak;
	payload->req_opcode = MavlinkFTP::kCmdResetSessions;
	payload->size = 1;
	payload->data[0] = MavlinkFTP::kErrFail;
	payload->data[1] = 0x11;
	payload->data[2] = 0x22;
	payload->data[3] = 0x33;

	mavlink_ftp_trim_reply_payload(reply);

	uint8_t cached_reply[kMavlinkFTPCachedReplyLength];
	memcpy(cached_reply, &reply, sizeof(cached_reply));

	mavlink_file_transfer_protocol_t resent_reply{};
	mavlink_ftp_expand_cached_reply(cached_reply, resent_reply);

	auto *resent_payload = reinterpret_cast<MavlinkFTP::PayloadHeader *>(&resent_reply.payload[0]);

	EXPECT_EQ(resent_reply.target_system, reply.target_system);
	EXPECT_EQ(resent_reply.target_component, reply.target_component);
	EXPECT_EQ(resent_payload->seq_number, payload->seq_number);
	EXPECT_EQ(resent_payload->opcode, payload->opcode);
	EXPECT_EQ(resent_payload->req_opcode, payload->req_opcode);
	EXPECT_EQ(resent_payload->size, payload->size);
	EXPECT_EQ(resent_payload->data[0], payload->data[0]);
	EXPECT_EQ(resent_payload->data[1], 0);
	EXPECT_EQ(resent_payload->data[2], 0);
	EXPECT_EQ(resent_payload->data[3], 0);
	EXPECT_EQ(resent_reply.payload[250], 0);
}
