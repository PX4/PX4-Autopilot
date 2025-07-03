/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * @file Safety.hpp
 */

#pragma once

#include <lib/parameters/param.h>
#include <px4_platform_common/defines.h>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/button_event.h>
#include <uORB/topics/safety_switch.h>

class Safety
{
public:
	Safety();
	~Safety() = default;

	bool safetySwitchHandler();
	void activateSafety();
	bool isSafetySwitchAvailable() const { return _safety_switch_available; }
	bool isSafetyOff() const { return _safety_off; }
	bool isSafetyDisabled() const { return _safety_disabled; }

private:
	enum class SafetyMode : int32_t {
		SAFETY_BUTTON = 0,
		LEVEL_HIGH = 1,
		LEVEL_LOW = 2
	};

	void handleModeButton();
	void handleModeLevel();

	uORB::Subscription _safety_button_sub{ORB_ID::safety_button};
	uORB::Subscription _safety_switch_sub{ORB_ID::safety_switch};

	bool _safety_switch_available{false};///< Set to true if a safety switch is connected
	bool _safety_off{false}; ///< Set to true if safety is off
	bool _previous_safety_off{false}; ///< Previous safety value
	bool _safety_disabled{false}; ///< Set to true if safety is disabled
	SafetyMode _safety_switch_mode{SafetyMode::SAFETY_BUTTON}; //< Value of the safety switch mode parameter
};
