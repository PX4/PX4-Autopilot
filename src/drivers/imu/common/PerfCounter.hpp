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
 * @file
 * @brief Per-instance ownership of native PX4 performance counters.
 */

#pragma once

#include <cstdint>
#include <lib/perf/perf_counter.h>

namespace imu
{

/**
 * @brief Own exactly one native counter, without shared instances or hot-path allocation.
 * @tparam type Native PC_COUNT, PC_INTERVAL or PC_ELAPSED measurement type.
 * @note Labels must outlive the counter; pass string literals. A null label disables
 * allocation. Failed allocation is also safe: native perf operations accept null handles.
 * Hot-path forwarding methods are forced inline to avoid an extra branch under -Os.
 */
template<perf_counter_type type>
class PerfCounter
{
public:
	/**
	 * @brief Allocate an independent counter if requested.
	 * @param[in] name Persistent label, or nullptr for an unused optional diagnostic.
	 */
	explicit PerfCounter(const char *name = nullptr) : _handle(name ? perf_alloc(type, name) : nullptr)
	{
		static_assert(
			type == PC_COUNT
			|| type == PC_INTERVAL
			|| type == PC_ELAPSED,
			"Invalid counter type");
	}

	~PerfCounter() { perf_free(_handle); }

	PerfCounter(const PerfCounter &) = delete;
	PerfCounter &operator=(const PerfCounter &) = delete;
	PerfCounter(PerfCounter &&) = delete;
	PerfCounter &operator=(PerfCounter &&) = delete;

	/** @return True if allocation succeeded, false for a disabled or unavailable counter. */
	bool valid() const { return _handle != nullptr; }

	/** Record a count/interval event; unavailable for elapsed counters. */
	__attribute__((always_inline)) void count() const
	{
		static_assert(type != PC_ELAPSED, "Elapsed counters require begin/end");
		perf_count(_handle);
	}

	/** Begin an elapsed measurement; unavailable for count/interval counters. */
	__attribute__((always_inline)) void begin() const
	{
		static_assert(type == PC_ELAPSED, "Only elapsed counters support begin");
		perf_begin(_handle);
	}

	/** Complete an elapsed measurement. */
	__attribute__((always_inline)) void end() const
	{
		static_assert(type == PC_ELAPSED, "Only elapsed counters support end");
		perf_end(_handle);
	}

	/** Cancel an elapsed measurement without recording an event. */
	__attribute__((always_inline)) void cancel() const
	{
		static_assert(type == PC_ELAPSED, "Only elapsed counters support cancel");
		perf_cancel(_handle);
	}

	/** @return Native event count; zero for a disabled or unavailable counter. */
	__attribute__((always_inline)) uint64_t eventCount() const { return perf_event_count(_handle); }

	/** Print the native counter, if allocated. */
	void print() const { perf_print_counter(_handle); }

private:
	const perf_counter_t _handle;
};

/** Optional shared transfer diagnostics; error aggregation remains the caller's policy. */
struct TransferPerfCounters {
	/**
	 * @param[in] bad_register_name Persistent register-validation label.
	 * @param[in] bad_transfer_name Persistent bus/wire-error label.
	 */
	TransferPerfCounters(const char *bad_register_name, const char *bad_transfer_name) :
		bad_register(bad_register_name), bad_transfer(bad_transfer_name) {}

	/** Print both counters without combining their event counts. */
	void print() const
	{
		bad_register.print();
		bad_transfer.print();
	}

	PerfCounter<PC_COUNT> bad_register; ///< Register validation failures.
	PerfCounter<PC_COUNT> bad_transfer; ///< Bus or wire validation failures.
};

} // namespace imu
