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
 * @file VteTask.h
 * @brief Abstract interface for tasks that drive the Vision Target Estimator.
 *
 * A task owns all state that only exists for the duration of a particular VTE use-case
 * (subscriptions, setpoint caches, external status flags). VisionTargetEst keeps a
 * priority-ordered registry and drives every task through this interface, which
 * replaces the parallel if/else-if branches that used to dispatch on the task mask.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#pragma once

#include <cstdint>

namespace vision_target_estimator
{

class VTEPosition;

// Single source of truth for VTE_TASK_MASK bit values.
namespace task_bits
{
static constexpr uint8_t kPrecLand = 1u << 0;
static constexpr uint8_t kDebug    = 1u << 1;
} // namespace task_bits

class VteTask
{
public:
	virtual ~VteTask() = default;

	/// VTE_TASK_MASK bit that enables this task (e.g. task_bits::kPrecLand).
	virtual uint8_t maskBit() const = 0;

	/// Short, stable name for log messages. Must be a stable string literal.
	virtual const char *name() const = 0;

	/// Poll external status topics. Called every cycle while the task's mask bit is set.
	virtual void pollStatus() {}

	/// True once the task's external preconditions are met and it can become active.
	virtual bool isReady() const = 0;

	/// True when the task has reached its end state by its own signals (e.g. landed).
	/// Mask-bit deselection is handled by the registry, not here.
	virtual bool isComplete() { return false; }

	/// Called on the inactive → active transition, before the estimator is (re)started.
	virtual void onActivate() {}

	/// Called right after VTEPosition::resetFilter() on start; lets the task seed the filter.
	virtual void onPosEstStart(VTEPosition & /*pos*/) {}
};

/**
 * Debug task: runs as soon as the mask bit is set and never self-terminates.
 * Useful for forcing the estimator to run on the bench without any precland.
 */
class DebugTask final : public VteTask
{
public:
	uint8_t maskBit() const override { return task_bits::kDebug; }
	const char *name() const override { return "debug"; }
	bool isReady() const override { return true; }
};

} // namespace vision_target_estimator
