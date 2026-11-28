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

/**
 * @file ImuPerf.hpp
 * @brief Per-instance RAII ownership of native PX4 performance counters.
 */

#pragma once

#include <cstdint>
#include <lib/perf/perf_counter.h>

namespace imu
{

template<perf_counter_type type>
class PerfCounter
{
public:
	explicit PerfCounter(const char *name = nullptr) : _handle(name ? perf_alloc(type, name) : nullptr)
	{
		static_assert(type == PC_COUNT || type == PC_INTERVAL || type == PC_ELAPSED, "Invalid counter type");
	}

	~PerfCounter() { perf_free(_handle); }

	PerfCounter(const PerfCounter &) = delete;
	PerfCounter &operator=(const PerfCounter &) = delete;
	PerfCounter(PerfCounter &&) = delete;
	PerfCounter &operator=(PerfCounter &&) = delete;

	bool valid() const { return _handle != nullptr; }

	__attribute__((always_inline)) void count() const
	{
		static_assert(type != PC_ELAPSED, "Elapsed counters require begin/end");
		perf_count(_handle);
	}

	__attribute__((always_inline)) void begin() const
	{
		static_assert(type == PC_ELAPSED, "Only elapsed counters support begin");
		perf_begin(_handle);
	}

	__attribute__((always_inline)) void end() const
	{
		static_assert(type == PC_ELAPSED, "Only elapsed counters support end");
		perf_end(_handle);
	}

	__attribute__((always_inline)) void cancel() const
	{
		static_assert(type == PC_ELAPSED, "Only elapsed counters support cancel");
		perf_cancel(_handle);
	}

	__attribute__((always_inline)) uint64_t eventCount() const { return perf_event_count(_handle); }

	void print() const { perf_print_counter(_handle); }

private:
	perf_counter_t _handle;
};

struct TransferPerfCounters {
	TransferPerfCounters(const char *bad_register_name, const char *bad_transfer_name) :
		bad_register(bad_register_name), bad_transfer(bad_transfer_name) {}

	void print() const
	{
		bad_register.print();
		bad_transfer.print();
	}

	PerfCounter<PC_COUNT> bad_register;
	PerfCounter<PC_COUNT> bad_transfer;
};

struct FifoPerfCounters {
	FifoPerfCounters(const char *empty_name, const char *overflow_name, const char *reset_name) :
		empty(empty_name), overflow(overflow_name), reset(reset_name) {}

	void print() const
	{
		empty.print();
		overflow.print();
		reset.print();
	}

	PerfCounter<PC_COUNT> empty;
	PerfCounter<PC_COUNT> overflow;
	PerfCounter<PC_COUNT> reset;
};

} // namespace imu
