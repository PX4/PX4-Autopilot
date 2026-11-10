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
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uavcan/driver/can.hpp>
#include <uavcan/transport/can_io.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/can_forward_filter.h>
#include <uORB/topics/can_forward_frames.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>

class CanForwarder;

/// Mirrors bus and own frames to the forwarder and sends frames from the GCS
class ForwardingCanDriver : public uavcan::ICanDriver
{
public:
	static constexpr uavcan::CanIOFlags CanIOFlagForwarded = 1U << 15;

	explicit ForwardingCanDriver(uavcan::ICanDriver &inner);

	void setForwarder(CanForwarder *forwarder) { _forwarder = forwarder; }

	uavcan::ICanIface *getIface(uint8_t iface_index) override;
	uint8_t getNumIfaces() const override { return _num_ifaces; }

	int16_t select(uavcan::CanSelectMasks &inout_masks, const uavcan::CanFrame * (&pending_tx)[uavcan::MaxCanIfaces],
		       uavcan::MonotonicTime blocking_deadline) override
	{
		return _inner.select(inout_masks, pending_tx, blocking_deadline);
	}

private:
	class Iface : public uavcan::ICanIface
	{
	public:
		int16_t send(const uavcan::CanFrame &frame, uavcan::MonotonicTime tx_deadline, uavcan::CanIOFlags flags) override;
		int16_t receive(uavcan::CanFrame &out_frame, uavcan::MonotonicTime &out_ts_monotonic, uavcan::UtcTime &out_ts_utc,
				uavcan::CanIOFlags &out_flags) override;
		int16_t configureFilters(const uavcan::CanFilterConfig *filter_configs, uint16_t num_configs) override
		{
			return inner->configureFilters(filter_configs, num_configs);
		}
		uint16_t getNumFilters() const override { return inner->getNumFilters(); }
		uint64_t getErrorCount() const override { return inner->getErrorCount(); }

		ForwardingCanDriver *owner{nullptr};
		uavcan::ICanIface *inner{nullptr};
		uint8_t index{0};
	};

	uavcan::ICanDriver &_inner;
	Iface _ifaces[uavcan::MaxCanIfaces];
	uint8_t _num_ifaces{0};
	CanForwarder *_forwarder{nullptr};
};

/// Session handling and frame transport between the bus and the mavlink module
class CanForwarder
{
public:
	static constexpr hrt_abstime KeepaliveTimeout = 5'000'000;
	static constexpr hrt_abstime TxDeadline = 100'000;
	static constexpr hrt_abstime MaxBatchAge = 1'000'000;
	static constexpr hrt_abstime RxBatchAge = 10'000;
	static constexpr uint8_t MaxFramesPerUpdate = 8;

	explicit CanForwarder(px4::WorkItem *work_item) : _tx_sub{work_item, ORB_ID(can_forward_tx)} {}
	~CanForwarder() { deinit(); }

	void init(ForwardingCanDriver *driver);
	void deinit();

	/// call before the node spins
	void update(hrt_abstime now, uavcan::CanIOManager &canio);

	/// call after the node spins
	void publish(hrt_abstime now);

	bool active() const { return _bus >= 0; }
	void printStatus() const;

	bool allowInject(uint8_t bus) const { return _bus == bus && !_armed; }
	void mirror(uint8_t bus, const uavcan::CanFrame &frame);

private:
	void handleCommand(const vehicle_command_s &cmd, hrt_abstime now);
	void pushFramesToBus(hrt_abstime now, uavcan::CanIOManager &canio);
	void flushRx(hrt_abstime now);

	ForwardingCanDriver *_driver{nullptr};

	int8_t _bus{-1};
	hrt_abstime _last_keepalive{0};
	bool _armed{false};
	can_forward::Filter _filter;

	uORB::Subscription _armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription _vcmd_sub{ORB_ID(vehicle_command)};
	uORB::Subscription _filter_sub{ORB_ID(can_forward_filter)};
	uORB::SubscriptionCallbackWorkItem _tx_sub;
	uORB::Publication<can_forward_frames_s> _rx_pub{ORB_ID(can_forward_rx)};
	uORB::Publication<vehicle_command_ack_s> _ack_pub{ORB_ID(vehicle_command_ack)};

	can_forward_frames_s _rx_batch{};
	can_forward_frames_s _tx_batch{};
	uint8_t _tx_index{0};
	bool _tx_pending{false};

	uint32_t _frames_to_gcs{0};
	uint32_t _frames_from_gcs{0};
	uint32_t _dropped_from_gcs{0};
};
