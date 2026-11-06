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
 * @file ForwardingCanDriver.hpp
 *
 * uavcan::ICanDriver decorator that implements the data paths of MAVLink CAN forwarding:
 *
 *  1. frames received from the bus are mirrored to the GCS
 *  2. frames the autopilot sends itself are mirrored to the GCS
 *  3. frames from the GCS go out on the bus (sent by the caller through CanIOManager with
 *     CanIOFlagForwarded) and, when addressed to the autopilot's node, are also fed into the
 *     local receive path, so the GCS can talk to the autopilot's own DroneCAN node.
 *
 * Forwarded frames are never mirrored back to the GCS. Loopback frames are not mirrored on
 * receive, they were already mirrored when they were sent.
 *
 * Everything here runs in the context of the libuavcan node (spinOnce() and the node's own
 * send path), so it needs no locking of its own.
 *
 * Assumption: the node only calls select() non-blocking (spinOnce()). When injected frames are
 * pending the inner select() is called non-blocking as well, so they are picked up immediately.
 */

#pragma once

#include "CanForwardFilter.hpp"
#include "CanFrameCodec.hpp"

#include <uavcan/driver/can.hpp>
#include <uavcan/driver/system_clock.hpp>
#include <uavcan/transport/can_io.hpp>

namespace can_forward
{

/// Private flag on frames from the GCS. Never reaches the platform driver.
static constexpr uavcan::CanIOFlags CanIOFlagForwarded = 1U << 15;

/// Receives frames to forward to the GCS
class IFrameSink
{
public:
	virtual ~IFrameSink() = default;
	virtual void onFrameToGcs(uint8_t iface_index, const uavcan::CanFrame &frame) = 0;
};

/**
 * true if a frame from the GCS has to be fed to the local node: broadcast and anonymous messages,
 * and service frames addressed to the local node. Service frames for other nodes (e.g. the file
 * read responses of a firmware update) are not.
 */
static inline bool isForLocalNode(const uavcan::CanFrame &frame, uint8_t local_node_id)
{
	if (!frame.isExtended()) {
		return false;
	}

	const uint32_t id = frame.id & uavcan::CanFrame::MaskExtID;

	if (id & 0x80U) {
		const uint8_t destination = (id >> 8) & 0x7FU;
		return local_node_id != 0 && destination == local_node_id;
	}

	return true;
}

class ForwardingCanDriver : public uavcan::ICanDriver
{
public:
	static constexpr uint8_t InjectQueueLength = 16;

	struct Counters {
		uint32_t frames_to_gcs{0};
		uint32_t frames_to_bus{0};
		uint32_t dropped_blocked{0};
		uint32_t dropped_inject_queue{0};
	};

	ForwardingCanDriver(uavcan::ICanDriver &inner, uavcan::ISystemClock &clock) :
		_inner(inner), _clock(clock)
	{
		_num_ifaces = inner.getNumIfaces();

		if (_num_ifaces > uavcan::MaxCanIfaces) {
			_num_ifaces = uavcan::MaxCanIfaces;
		}

		for (uint8_t i = 0; i < _num_ifaces; i++) {
			_ifaces[i].init(this, i);
		}
	}

	/**
	 * Configure forwarding. Called from the node context.
	 *
	 * @param forward_bus  bus mirrored to the GCS, negative for none
	 * @param inject_allowed frames with CanIOFlagForwarded may go on the bus (not armed or prearmed)
	 */
	void configure(int8_t forward_bus, bool inject_allowed, const CanForwardFilter *filter, IFrameSink *sink,
		       uint8_t local_node_id)
	{
		_forward_bus = forward_bus;
		_inject_allowed = inject_allowed;
		_filter = filter;
		_sink = sink;
		_local_node_id = local_node_id;

		if (forward_bus < 0) {
			for (uint8_t i = 0; i < _num_ifaces; i++) {
				_ifaces[i].clearInjected();
			}
		}
	}

	const Counters &counters() const { return _counters; }

	uavcan::ICanIface *getIface(uint8_t iface_index) override
	{
		if (iface_index >= _num_ifaces) {
			return nullptr;
		}

		// resolved on each call: some platform drivers only create their interfaces in init()
		uavcan::ICanIface *inner = _inner.getIface(iface_index);
		_ifaces[iface_index].setInner(inner);
		return (inner != nullptr) ? &_ifaces[iface_index] : nullptr;
	}

	uint8_t getNumIfaces() const override { return _num_ifaces; }

	int16_t select(uavcan::CanSelectMasks &inout_masks, const uavcan::CanFrame * (&pending_tx)[uavcan::MaxCanIfaces],
		       uavcan::MonotonicTime blocking_deadline) override
	{
		const uint8_t requested_read = inout_masks.read;
		uint8_t injected_ready = 0;

		for (uint8_t i = 0; i < _num_ifaces; i++) {
			if ((requested_read & (1U << i)) && _ifaces[i].hasInjected()) {
				injected_ready |= 1U << i;
			}
		}

		const int16_t res = _inner.select(inout_masks, pending_tx,
						  injected_ready ? uavcan::MonotonicTime() : blocking_deadline);

		if (res < 0) {
			return res;
		}

		// only report what was asked for, CanIOManager::send() asserts that read stays 0
		inout_masks.read |= injected_ready;

		int16_t ready = 0;

		for (uint8_t i = 0; i < _num_ifaces; i++) {
			if ((inout_masks.read | inout_masks.write) & (1U << i)) {
				ready++;
			}
		}

		return ready;
	}

private:
	class Iface : public uavcan::ICanIface
	{
	public:
		void init(ForwardingCanDriver *owner, uint8_t index)
		{
			_owner = owner;
			_index = index;
		}

		void setInner(uavcan::ICanIface *inner) { _inner = inner; }
		bool hasInjected() const { return _count > 0; }
		void clearInjected() { _head = 0; _count = 0; }

		bool pushInjected(const uavcan::CanFrame &frame, uavcan::MonotonicTime ts)
		{
			if (_count >= InjectQueueLength) {
				return false;
			}

			Entry &e = _queue[(_head + _count) % InjectQueueLength];
			e.frame = frame;
			e.ts = ts;
			_count++;
			return true;
		}

		int16_t send(const uavcan::CanFrame &frame, uavcan::MonotonicTime tx_deadline, uavcan::CanIOFlags flags) override
		{
			ForwardingCanDriver &o = *_owner;

			if (flags & CanIOFlagForwarded) {
				// checked at transmit time, so a frame queued before arming never reaches the bus after
				if (!o._inject_allowed || o._forward_bus != _index) {
					o._counters.dropped_blocked++;
					return 1; // consumed
				}

				const int16_t res = _inner->send(frame, tx_deadline, flags & ~CanIOFlagForwarded);

				if (res > 0) {
					o._counters.frames_to_bus++;

					if (isForLocalNode(frame, o._local_node_id)) {
						if (!pushInjected(frame, o._clock.getMonotonic())) {
							o._counters.dropped_inject_queue++;
						}
					}
				}

				return res;
			}

			const int16_t res = _inner->send(frame, tx_deadline, flags);

			if (res > 0) {
				o.mirror(_index, frame);
			}

			return res;
		}

		int16_t receive(uavcan::CanFrame &out_frame, uavcan::MonotonicTime &out_ts_monotonic, uavcan::UtcTime &out_ts_utc,
				uavcan::CanIOFlags &out_flags) override
		{
			if (_count > 0) {
				const Entry &e = _queue[_head];
				out_frame = e.frame;
				out_ts_monotonic = e.ts;
				out_ts_utc = uavcan::UtcTime();
				out_flags = 0;
				_head = (_head + 1) % InjectQueueLength;
				_count--;
				return 1; // a GCS frame, not mirrored back
			}

			const int16_t res = _inner->receive(out_frame, out_ts_monotonic, out_ts_utc, out_flags);

			if (res > 0 && !(out_flags & uavcan::CanIOFlagLoopback)) {
				_owner->mirror(_index, out_frame);
			}

			return res;
		}

		int16_t configureFilters(const uavcan::CanFilterConfig *filter_configs, uint16_t num_configs) override
		{
			return _inner->configureFilters(filter_configs, num_configs);
		}

		uint16_t getNumFilters() const override { return _inner->getNumFilters(); }
		uint64_t getErrorCount() const override { return _inner->getErrorCount(); }

	private:
		struct Entry {
			uavcan::CanFrame frame;
			uavcan::MonotonicTime ts;
		};

		ForwardingCanDriver *_owner{nullptr};
		uavcan::ICanIface *_inner{nullptr};
		uint8_t _index{0};
		Entry _queue[InjectQueueLength];
		uint8_t _head{0};
		uint8_t _count{0};
	};

	void mirror(uint8_t index, const uavcan::CanFrame &frame)
	{
		if (_sink != nullptr && _forward_bus == index && (_filter == nullptr || _filter->matches(frame.id))) {
			_counters.frames_to_gcs++;
			_sink->onFrameToGcs(index, frame);
		}
	}

	uavcan::ICanDriver &_inner;
	uavcan::ISystemClock &_clock;
	Iface _ifaces[uavcan::MaxCanIfaces];
	uint8_t _num_ifaces{0};

	int8_t _forward_bus{-1};
	bool _inject_allowed{false};
	const CanForwardFilter *_filter{nullptr};
	IFrameSink *_sink{nullptr};
	uint8_t _local_node_id{0};

	Counters _counters;
};

/**
 * Hand frames from the GCS to libuavcan for transmission on one bus, in order.
 *
 * A frame is only handed over while libuavcan's TX queue for that bus is empty, so at most one
 * forwarded frame waits in the queue at a time: the frames of a multi-frame transfer keep their
 * order, the autopilot's own traffic is not starved and the node's memory pool is not filled by a
 * burst. The rest stays with the caller until the next call.
 *
 * @return number of frames handed over
 */
static inline uint8_t pumpToBus(uavcan::CanIOManager &canio, uint8_t bus, const Frame *frames, uint8_t count,
				uavcan::MonotonicTime tx_deadline)
{
	uint8_t sent = 0;

	while (sent < count) {
		if (canio.makePendingTxMask() & (1U << bus)) {
			break;
		}

		const Frame &f = frames[sent];
		const uavcan::CanFrame frame(f.id, f.data, f.len);
		// non-blocking: goes straight to a free mailbox or into the (empty) TX queue
		(void)canio.send(frame, tx_deadline, uavcan::MonotonicTime(), static_cast<uint8_t>(1U << bus),
				 uavcan::CanTxQueue::Persistent, CanIOFlagForwarded);
		sent++;
	}

	return sent;
}

} // namespace can_forward
