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
 * DroneCAN over MAVLink (MAV_CMD_CAN_FORWARD, CAN_FRAME, CAN_FILTER_MODIFY).
 */

#pragma once

#include <lib/can_forward/CanForward.hpp>

#include <drivers/drv_hrt.h>
#include <lib/parameters/param.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uavcan/transport/dispatcher.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/can_forward_filter.h>
#include <uORB/topics/can_forward_frames.h>
#include <uORB/topics/can_forward_status.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>

/// Owns the session and moves frames between the bus and the mavlink module. Listens on the bus only during a session
class CanForwarder : private uavcan::IRxFrameListener, private uavcan::ITxFrameListener
{
public:
	static constexpr hrt_abstime TxDeadline = 100'000;
	static constexpr hrt_abstime MaxBatchAge = 1'000'000;
	static constexpr hrt_abstime RxBatchAge = 10'000;
	static constexpr uint8_t MaxFramesPerUpdate = 8;

	explicit CanForwarder(px4::WorkItem *work_item) : _tx_sub{work_item, ORB_ID(can_forward_tx)} {}
	~CanForwarder() { deinit(); }

	void init(uavcan::Dispatcher &dispatcher);
	void deinit();

	/// call before the node spins
	void update(hrt_abstime now);

	/// call after the node spins
	void publish(hrt_abstime now);

	void printStatus() const;

private:
	void handleRxFrame(const uavcan::CanRxFrame &frame, uavcan::CanIOFlags flags) override;
	void handleTxFrame(const uavcan::CanFrame &frame, uint8_t iface_mask) override;

	void handleCommand(const vehicle_command_s &cmd, hrt_abstime now);
	void applySession(hrt_abstime now);
	void mirror(const uavcan::CanFrame &frame);
	void pushFramesToBus(hrt_abstime now);
	bool nextTxBatch();
	void flushRx(hrt_abstime now);

	uavcan::Dispatcher *_dispatcher{nullptr};
	param_t _param_can_fwd{PARAM_INVALID};

	can_forward::Session _session;
	can_forward_status_s _status{};
	bool _config_only{true};
	can_forward::Filter _filter;

	uORB::Subscription _armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription _vcmd_sub{ORB_ID(vehicle_command)};
	uORB::Subscription _filter_sub{ORB_ID(can_forward_filter)};
	uORB::SubscriptionCallbackWorkItem _tx_sub;
	uORB::Publication<can_forward_frames_s> _rx_pub{ORB_ID(can_forward_rx)};
	uORB::Publication<can_forward_status_s> _status_pub{ORB_ID(can_forward_status)};
	uORB::Publication<vehicle_command_ack_s> _ack_pub{ORB_ID(vehicle_command_ack)};

	orb_advert_t _mavlink_log_pub{nullptr};

	bool _armed{false};

	can_forward_frames_s _rx_batch{};
	can_forward_frames_s _tx_batch{};
	uint8_t _tx_index{0};
	bool _tx_pending{false};
	bool _tx_seen{false};

	uint32_t _frames_to_gcs{0};
	uint32_t _frames_from_gcs{0};
	uint32_t _dropped_from_gcs{0};
	uint32_t _lost_from_gcs{0}; ///< batches overwritten in the uORB queue
};
