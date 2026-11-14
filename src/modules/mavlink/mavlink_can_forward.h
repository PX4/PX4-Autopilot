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

/**
 * @file mavlink_can_forward.h
 *
 * MAVLink side of DroneCAN over MAVLink. The instance that received the latest
 * MAV_CMD_CAN_FORWARD exchanges frames with the requesting client.
 */

#pragma once

#include "mavlink_bridge_header.h"

#include <px4_platform_common/atomic.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/can_forward_filter.h>
#include <uORB/topics/can_forward_frames.h>

class MavlinkCanForward
{
public:
	MavlinkCanForward() = default;
	~MavlinkCanForward();

	MavlinkCanForward(const MavlinkCanForward &) = delete;
	MavlinkCanForward &operator=(const MavlinkCanForward &) = delete;

	// receiver thread
	void handleCommand(uint32_t sysid, uint8_t compid, float param1);

	void handleMessage(const mavlink_message_t &msg);

	void flushToBus();

	bool isOwner() const { return _owner.load() == this; }

	// main thread
	void dropFrames();

	// main thread, send lock held
	void sendToGcs(mavlink_channel_t channel, unsigned free_tx_buf);

private:
	static uint64_t client(uint32_t sysid, uint8_t compid) { return (uint64_t(sysid) << 8) | compid; }
	void queueFrame(uint8_t bus, uint32_t id, uint8_t len, const uint8_t *data);

	static px4::atomic<MavlinkCanForward *> _owner;
	px4::atomic<uint64_t> _client{0}; ///< sysid << 8 | compid

	// receiver thread only
	can_forward_frames_s _tx_batch{};
	uORB::Publication<can_forward_frames_s> _tx_pub{ORB_ID(can_forward_tx)};
	uORB::Publication<can_forward_filter_s> _filter_pub{ORB_ID(can_forward_filter)};

	// main thread only
	uORB::Subscription _rx_sub{ORB_ID(can_forward_rx)};
	can_forward_frames_s _rx_batch{};
	uint8_t _rx_index{0};
	bool _rx_pending{false};
};
