/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * @file AutogyroTakeoff.cpp
 * Autogyro takeoff handling for autogyro UAVs with steerable wheels.
 *
 * @author Roman Dvorak, ThunderFly s.r.o. <dvorakroman@thunderfly.cz>
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "AutogyroTakeoff.h"
#include <systemlib/mavlink_log.h>
#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>

#include <uORB/Publication.hpp>

using matrix::Vector2f;
using namespace time_literals;

namespace autogrolanding
{


void AutogyroLanding::init()
//void AutogyroTakeoff::init(const hrt_abstime &time_now, const float initial_yaw,
//			   const matrix::Vector2d &start_pos_global)
{
	// initial_yaw_ = initial_yaw;
	// initialized_ = true;
	// state_ = AutogyroTakeoffState::PRE_TAKEOFF_PREROTATE_START;
	// state_last_ = AutogyroTakeoffState::PRE_TAKEOFF_PREROTATE_START;
	// time_initialized_ = time_now;
	// //_time_in_state_= time_now;
	// last_sent_release_status_ = time_now;
	// climbout_ = true; // this is true until climbout is finished

	// takeoff_wp_ = start_pos_global;
	// initial_wp_ = start_pos_global;
}




} // namespace autogyrolanding
