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
 * MAVLink side of DroneCAN over MAVLink. The instance that received the request
 * for the session exchanges frames with its client.
 */

#pragma once

#include "mavlink_bridge_header.h"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/atomic.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/can_forward_filter.h>
#include <uORB/topics/can_forward_frames.h>
#include <uORB/topics/can_forward_status.h>

/// Routes CAN frames between a GCS and the uavcan driver, which owns the session (can_forward_status)
class MavlinkCanForward
{
public:
	MavlinkCanForward() = default;
	~MavlinkCanForward();

	MavlinkCanForward(const MavlinkCanForward &) = delete;
	MavlinkCanForward &operator=(const MavlinkCanForward &) = delete;

	// receiver thread
	/// @return false if another link carries the session: the command must not reach the driver
	bool handleCommand(float param1, uint8_t system, uint8_t component);

	void handleMessage(const mavlink_message_t &msg);

	void flushToBus();

	// main thread
	/// @return true if this link carries the session
	bool update();

	uint32_t lostBatches() const { return _rx_lost; }

	// main thread, send lock held
	/// @param budget bytes that may be sent, reduced by what was sent
	void sendToGcs(mavlink_channel_t channel, unsigned message_size, unsigned &budget);

private:
	static constexpr hrt_abstime ClaimTimeout = 100'000; ///< the driver answers a request within a few ms

	bool claimLink(); ///< receiver thread
	bool isClient(uint8_t system, uint8_t component) const; ///< receiver thread
	void queueFrame(uint8_t bus, uint32_t id, uint8_t len, const uint8_t *data);
	bool nextRxBatch(); ///< main thread

	static px4::atomic<MavlinkCanForward *> _link; ///< the link that carries the session, so frames use one link only

	// receiver thread only
	uORB::Subscription _receiver_status_sub{ORB_ID(can_forward_status)};
	can_forward_status_s _receiver_status{};
	can_forward_frames_s _tx_batch{};
	uORB::Publication<can_forward_frames_s> _tx_pub{ORB_ID(can_forward_tx)};
	uORB::Publication<can_forward_filter_s> _filter_pub{ORB_ID(can_forward_filter)};

	// main thread only
	uORB::Subscription _status_sub{ORB_ID(can_forward_status)};
	can_forward_status_s _status{};
	hrt_abstime _claimed_since{0}; ///< this link holds _link without a session
	bool _carried{false};
	uORB::Subscription _rx_sub{ORB_ID(can_forward_rx)};
	can_forward_frames_s _rx_batch{};
	uint8_t _rx_index{0};
	bool _rx_pending{false};
	bool _rx_seen{false};
	uint32_t _rx_lost{0}; ///< batches overwritten in the uORB queue, e.g. on a saturated link
};
