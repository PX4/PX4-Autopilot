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
 * @brief FIFO-only diagnostics composed from independent native counters.
 */

#pragma once

#include "PerfCounter.hpp"

namespace imu
{

/** FIFO events only; transfer failures, elapsed reads and burst checks are separate capabilities. */
struct FifoPerfCounters {
	/**
	 * @param[in] empty_name Persistent FIFO-empty label.
	 * @param[in] overflow_name Persistent FIFO-overflow label.
	 * @param[in] reset_name Persistent FIFO-reset label.
	 * @note Labels may be null to omit individual counters; reset policy remains in the driver.
	 */
	FifoPerfCounters(const char *empty_name, const char *overflow_name, const char *reset_name) :
		empty(empty_name), overflow(overflow_name), reset(reset_name) {}

	/** Print FIFO events without treating every event as a sensor error. */
	void print() const
	{
		empty.print();
		overflow.print();
		reset.print();
	}

	PerfCounter<PC_COUNT> empty; ///< Empty FIFO observations.
	PerfCounter<PC_COUNT> overflow; ///< FIFO overflow observations.
	PerfCounter<PC_COUNT> reset; ///< FIFO resets, including intentional resets.
};

} // namespace imu
