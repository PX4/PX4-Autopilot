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

#pragma once

#include <stdint.h>

// Failsafe action-mode enums (values must match the parameter metadata
// in commander_params.yaml), one namespace per failsafe. Kept in this
// dependency-free header so the enum and its severity predicates can be
// shared between the failsafe state machine and the HealthAndArmingChecks
// without the latter pulling in the failsafe framework.

namespace traffic_avoidance
{

// COM_TRAFF_AVOID parameter values.
enum class FailsafeMode : int32_t {
	Disabled = 0,
	Warning = 1, // arming allowed, in-flight warning
	Error = 2, // arming blocked, in-flight warning
	Return = 3, // arming blocked, in-flight RTL
	Land = 4, // arming blocked, in-flight Land
};

// Increasing order of severity in enum is assumed.
constexpr bool isEnabled(FailsafeMode mode) { return mode >= FailsafeMode::Warning; }
constexpr bool blocksArming(FailsafeMode mode) { return mode >= FailsafeMode::Error; }

} // namespace traffic_avoidance
