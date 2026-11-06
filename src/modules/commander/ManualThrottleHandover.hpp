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

#include <uORB/topics/vehicle_status.h>

namespace commander
{

constexpr bool isDirectManualThrustMode(uint8_t nav_state)
{
	return nav_state == vehicle_status_s::NAVIGATION_STATE_MANUAL
	       || nav_state == vehicle_status_s::NAVIGATION_STATE_MANUAL_PARKING
	       || nav_state == vehicle_status_s::NAVIGATION_STATE_ACRO
	       || nav_state == vehicle_status_s::NAVIGATION_STATE_STAB;
}

constexpr bool rejectManualThrottleHandover(float minimum_throttle, float manual_throttle,
		bool manual_control_available, bool armed, bool landed, bool rotary_wing, uint8_t current_nav_state,
		uint8_t requested_nav_state)
{
	return (minimum_throttle > -1.f)
	       && armed
	       && !landed
	       && rotary_wing
	       && !isDirectManualThrustMode(current_nav_state)
	       && isDirectManualThrustMode(requested_nav_state)
	       && (!manual_control_available || (manual_throttle < minimum_throttle));
}

} // namespace commander
