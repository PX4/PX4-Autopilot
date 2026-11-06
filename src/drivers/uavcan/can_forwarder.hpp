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
 * @file can_forwarder.hpp
 *
 * DroneCAN over MAVLink: CAN frame forwarding between a CAN bus and a MAVLink GCS
 * (MAV_CMD_CAN_FORWARD, CAN_FRAME, CAN_FILTER_MODIFY), as used by dronecan_gui_tool and
 * Mission Planner, mainly to update node firmware.
 *
 * The uavcan driver owns the forwarding session: it handles MAV_CMD_CAN_FORWARD from
 * vehicle_command, acks it, applies the keepalive timeout and the arming gate and publishes
 * can_forward_status. The mavlink module only moves frames between MAVLink and the
 * can_forward_rx / can_forward_tx topics.
 *
 * All methods run on the uavcan work queue with the node mutex held.
 */

#pragma once

#include <lib/can_forward/ForwardingCanDriver.hpp>
#include <lib/can_forward/CanForwardSession.hpp>

#include <drivers/drv_hrt.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/can_forward_filter.h>
#include <uORB/topics/can_forward_frames.h>
#include <uORB/topics/can_forward_status.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>

class CanForwarder : public can_forward::IFrameSink
{
public:
	/// how long a frame from the GCS may wait for transmission before it is dropped
	static constexpr hrt_abstime TxDeadline = 100 * 1000; // [us]

	/// frames from the GCS that waited longer than this in uORB are dropped
	static constexpr hrt_abstime MaxBatchAge = 1000 * 1000; // [us]

	explicit CanForwarder(px4::WorkItem *work_item) : _tx_sub{work_item, ORB_ID(can_forward_tx)} {}

	~CanForwarder() override { _tx_sub.unregisterCallback(); }

	/// Start handling MAV_CMD_CAN_FORWARD. driver is the decorator the node was created with.
	void init(can_forward::ForwardingCanDriver *driver);

	/// Stop forwarding and the topic callback. Must be called before the work item stops.
	void deinit();

	/**
	 * Handle commands, filter changes, the session timeout and frames from the GCS.
	 * Call before the node spins, so injected frames are processed in the same cycle.
	 *
	 * @param blocked vehicle armed or prearmed
	 */
	void update(hrt_abstime now, bool blocked, uint8_t local_node_id, uavcan::CanIOManager &canio);

	/// Publish frames collected during the node spin and the status. Call after the node spins.
	void publish(hrt_abstime now);

	bool sessionActive() const { return _session.active(); }

	void printStatus() const;

	void onFrameToGcs(uint8_t iface_index, const uavcan::CanFrame &frame) override;

private:
	void handleCommand(const vehicle_command_s &cmd, hrt_abstime now, bool blocked);
	void pushFramesToBus(hrt_abstime now, uavcan::CanIOManager &canio);
	void configureDriver(uint8_t local_node_id);
	void flushRx(hrt_abstime now);

	can_forward::ForwardingCanDriver *_driver{nullptr};
	can_forward::CanForwardSession _session;
	can_forward::CanForwardFilter _filter;

	uORB::Subscription _vcmd_sub{ORB_ID(vehicle_command)};
	uORB::Subscription _filter_sub{ORB_ID(can_forward_filter)};
	uORB::SubscriptionCallbackWorkItem _tx_sub;

	uORB::Publication<can_forward_frames_s> _rx_pub{ORB_ID(can_forward_rx)};
	uORB::Publication<can_forward_status_s> _status_pub{ORB_ID(can_forward_status)};
	uORB::Publication<vehicle_command_ack_s> _ack_pub{ORB_ID(vehicle_command_ack)};

	can_forward_frames_s _rx_batch{};

	can_forward_frames_s _tx_batch{};
	uint8_t _tx_index{0};
	bool _tx_pending{false};
	bool _tx_received_any{false};

	uint8_t _num_buses{0};
	bool _blocked{false};
	bool _status_changed{true};
	hrt_abstime _last_status_pub{0};

	uint32_t _frames_from_gcs{0};
	uint32_t _dropped_blocked{0};
	uint32_t _dropped_stale{0};
	uint32_t _dropped_queue{0};
};
