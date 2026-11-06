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
 * MAVLink side of CAN frame forwarding (DroneCAN over MAVLink).
 *
 * The uavcan driver owns the forwarding session (MAV_CMD_CAN_FORWARD, timeout, arming gate).
 * This class binds the session to the MAVLink instance the request came in on, like ulog
 * streaming does:
 *
 *  - receiver thread: a MAV_CMD_CAN_FORWARD is recorded as pending; CAN_FRAME, CANFD_FRAME and
 *    CAN_FILTER_MODIFY from the bound client are checked and published to can_forward_tx /
 *    can_forward_filter
 *  - main thread: an ACCEPTED ack for the pending request binds the instance, can_forward_status
 *    unbinds it; frames from can_forward_rx are sent to the bound client as CAN_FRAME
 *
 * One instance per Mavlink instance, created on the first MAV_CMD_CAN_FORWARD.
 * Frames only ever go to the instance that made the latest request.
 */

#pragma once

#include "mavlink_bridge_header.h"

#include <lib/can_forward/CanFrameCodec.hpp>

#include <pthread.h>
#include <px4_platform_common/atomic.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/can_forward_filter.h>
#include <uORB/topics/can_forward_frames.h>
#include <uORB/topics/can_forward_status.h>
#include <uORB/topics/vehicle_command_ack.h>

class MavlinkCanForward
{
public:
	/// Output of CAN_FRAME messages, implemented by the Mavlink instance
	class Sender
	{
	public:
		virtual ~Sender() = default;

		/// free space in the transmit buffer [bytes]
		virtual unsigned freeTxBuf() = 0;

		virtual void sendCanFrame(mavlink_can_frame_t &msg, uint32_t target_system) = 0;
	};

	struct Counters {
		uint32_t frames_to_gcs{0};
		uint32_t frames_from_gcs{0};
		uint32_t dropped_not_owner{0};   ///< from a client that does not own the session, or none is active
		uint32_t dropped_invalid{0};     ///< malformed frame, wrong bus or CAN FD frame
		uint32_t dropped_to_gcs{0};      ///< batches lost to a can_forward_rx queue overrun
	};

	static constexpr unsigned CanFrameMessageSize = MAVLINK_MSG_ID_CAN_FRAME_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;

	MavlinkCanForward();
	~MavlinkCanForward();

	MavlinkCanForward(const MavlinkCanForward &) = delete;
	MavlinkCanForward &operator=(const MavlinkCanForward &) = delete;

	// receiver thread

	/// MAV_CMD_CAN_FORWARD received on this instance, before it is published to the driver
	void handleCommand(uint32_t sysid, uint8_t compid, float param1);

	/// CAN_FRAME, CANFD_FRAME or CAN_FILTER_MODIFY addressed to this vehicle (target already checked)
	void handleMessage(const mavlink_message_t &msg);

	/// publish the frames collected so far, call when a receive buffer has been parsed
	void flushToBus(hrt_abstime now);

	// main thread

	/// a vehicle_command_ack for MAV_CMD_CAN_FORWARD
	void handleAck(const vehicle_command_ack_s &ack);

	/// follow can_forward_status and send frames to the bound client
	void update(Sender &sender);

	bool bound() const;
	Counters counters() const;

private:
	void handleCanFrame(uint32_t sysid, uint8_t compid, uint8_t bus, uint32_t id, uint8_t len, const uint8_t *data,
			    uint8_t data_size);
	void handleFilter(uint32_t sysid, uint8_t compid, const mavlink_can_filter_modify_t &filter);
	bool isOwner(uint32_t sysid, uint8_t compid) const;
	void unbind();

	mutable pthread_mutex_t _mutex;

	// binding, under _mutex
	bool _bound{false};
	uint32_t _sysid{0};
	uint8_t _compid{0};
	uint8_t _bus{0};
	hrt_abstime _bound_since{0};

	// latest MAV_CMD_CAN_FORWARD on this instance, under _mutex
	bool _pending{false};
	uint32_t _pending_sysid{0};
	uint8_t _pending_compid{0};
	uint8_t _pending_bus{0};
	bool _pending_stop{false};
	uint32_t _pending_sequence{0};

	/// increments with every MAV_CMD_CAN_FORWARD on any instance
	static px4::atomic<uint32_t> _request_sequence;

	Counters _counters{}; // under _mutex

	// receiver thread only
	can_forward_frames_s _tx_batch{};
	uORB::Publication<can_forward_frames_s> _tx_pub{ORB_ID(can_forward_tx)};
	uORB::Publication<can_forward_filter_s> _filter_pub{ORB_ID(can_forward_filter)};

	// main thread only
	uORB::Subscription _rx_sub{ORB_ID(can_forward_rx)};
	uORB::Subscription _status_sub{ORB_ID(can_forward_status)};
	can_forward_frames_s _rx_batch{};
	uint8_t _rx_index{0};
	bool _rx_pending{false};
};
