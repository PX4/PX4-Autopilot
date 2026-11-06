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
 * @file FakeCan.hpp
 *
 * In-memory CAN bus, driver and clock for the can_forward tests. The driver has a configurable
 * number of free TX mailboxes so tests can make the hardware "full".
 */

#pragma once

#include <uavcan/driver/can.hpp>
#include <uavcan/driver/system_clock.hpp>

#include <climits>
#include <deque>
#include <vector>

namespace can_forward_test
{

class FakeClock : public uavcan::ISystemClock
{
public:
	uavcan::MonotonicTime getMonotonic() const override { return uavcan::MonotonicTime::fromUSec(usec); }
	uavcan::UtcTime getUtc() const override { return uavcan::UtcTime(); }
	void adjustUtc(uavcan::UtcDuration) override {}

	void advance(uint64_t us) { usec += us; }

	uint64_t usec{1'000'000};
};

class FakeIface;

/// Every frame one member transmits is received by all other members
class FakeBus
{
public:
	void attach(FakeIface *iface) { _members.push_back(iface); }
	void transmit(FakeIface *from, const uavcan::CanFrame &frame);

	std::vector<uavcan::CanFrame> wire; ///< everything that was transmitted, in order

private:
	std::vector<FakeIface *> _members;
};

class FakeIface : public uavcan::ICanIface
{
public:
	struct RxEntry {
		uavcan::CanFrame frame;
		uavcan::CanIOFlags flags;
	};

	struct TxEntry {
		uavcan::CanFrame frame;
		uavcan::CanIOFlags flags;
	};

	explicit FakeIface(FakeClock &clock) : _clock(clock) {}

	void setBus(FakeBus *bus)
	{
		_bus = bus;

		if (bus) { bus->attach(this); }
	}

	int16_t send(const uavcan::CanFrame &frame, uavcan::MonotonicTime tx_deadline, uavcan::CanIOFlags flags) override
	{
		if (tx_deadline < _clock.getMonotonic()) {
			return 0;
		}

		if (mailboxes_free <= 0) {
			return 0;
		}

		mailboxes_free--;
		sent.push_back({frame, flags});

		if (_bus) { _bus->transmit(this, frame); }

		if (flags & uavcan::CanIOFlagLoopback) {
			rx.push_back({frame, uavcan::CanIOFlagLoopback});
		}

		return 1;
	}

	int16_t receive(uavcan::CanFrame &out_frame, uavcan::MonotonicTime &out_ts_monotonic, uavcan::UtcTime &out_ts_utc,
			uavcan::CanIOFlags &out_flags) override
	{
		if (rx.empty()) {
			return 0;
		}

		out_frame = rx.front().frame;
		out_flags = rx.front().flags;
		out_ts_monotonic = _clock.getMonotonic();
		out_ts_utc = uavcan::UtcTime();
		rx.pop_front();
		return 1;
	}

	int16_t configureFilters(const uavcan::CanFilterConfig *, uint16_t) override { return 0; }
	uint16_t getNumFilters() const override { return 0; }
	uint64_t getErrorCount() const override { return 0; }

	void receiveFromBus(const uavcan::CanFrame &frame) { rx.push_back({frame, 0}); }

	int mailboxes_free{INT_MAX};
	std::deque<RxEntry> rx;
	std::vector<TxEntry> sent; ///< as seen by the "hardware", with flags

private:
	FakeClock &_clock;
	FakeBus *_bus{nullptr};
};

inline void FakeBus::transmit(FakeIface *from, const uavcan::CanFrame &frame)
{
	wire.push_back(frame);

	for (FakeIface *member : _members) {
		if (member != from) {
			member->receiveFromBus(frame);
		}
	}
}

class FakeCanDriver : public uavcan::ICanDriver
{
public:
	static constexpr uint8_t MaxIfaces = 2;

	FakeCanDriver(FakeClock &clock, uint8_t num_ifaces = 1) : _num_ifaces(num_ifaces), _ifaces{FakeIface(clock), FakeIface(clock)} {}

	FakeIface &iface(uint8_t i) { return _ifaces[i]; }

	uavcan::ICanIface *getIface(uint8_t iface_index) override
	{
		return iface_index < _num_ifaces ? &_ifaces[iface_index] : nullptr;
	}

	uint8_t getNumIfaces() const override { return _num_ifaces; }

	int16_t select(uavcan::CanSelectMasks &inout_masks, const uavcan::CanFrame * (&)[uavcan::MaxCanIfaces],
		       uavcan::MonotonicTime blocking_deadline) override
	{
		last_blocking_deadline = blocking_deadline;
		uint8_t read = 0;
		uint8_t write = 0;

		for (uint8_t i = 0; i < _num_ifaces; i++) {
			if ((inout_masks.read & (1U << i)) && !_ifaces[i].rx.empty()) {
				read |= 1U << i;
			}

			if ((inout_masks.write & (1U << i)) && _ifaces[i].mailboxes_free > 0) {
				write |= 1U << i;
			}
		}

		inout_masks.read = read;
		inout_masks.write = write;
		return __builtin_popcount(read | write);
	}

	uavcan::MonotonicTime last_blocking_deadline;

private:
	uint8_t _num_ifaces;
	FakeIface _ifaces[MaxIfaces];
};

} // namespace can_forward_test
