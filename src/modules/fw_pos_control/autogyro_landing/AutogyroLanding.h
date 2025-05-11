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
 * @file AutogyroLanding.h
 * Autogyro automated landing, header files
 *
 * @author Roman Dvorak, ThunderFly s.r.o. <dvorakroman@thunderfly.cz>
 */


#pragma once


#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include <drivers/drv_hrt.h>
#include <px4_platform_common/module_params.h>
#include <systemlib/mavlink_log.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/takeoff_status.h>
#include <uORB/topics/debug_value.h>
#include <uORB/topics/tune_control.h>
#include <uORB/topics/actuator_armed.h>


namespace autogyrolanding
{

enum AutogryLandingStates {
	AGL_STATE_NONE = 0,
	AGL_STATE_INIT,
	AGL_STATE_WAITING_FOR_TAKEOFF,
	AGL_STATE_TAKEOFF,
	AGL_STATE_FLYING,
	AGL_STATE_LANDING,
	AGL_STATE_LANDED,
	AGL_STATE_FINISHED,
	AGL_STATE_ABORTED,
	AGL_STATE_MAX
};


class __EXPORT AutogyroLanding : public ModuleParams
{
public:
	AutogyroLaning(ModuleParams *parent) : ModuleParams(parent) {}
	~AutogyroLanding() = default;


	/**
	 * @brief Initializes the state machine.
	 *
	 * @param time_now Absolute time since system boot [us]
	 * @param initial_yaw Vehicle yaw angle at time of initialization [us]
	 * @param start_pos_global Vehicle global (lat, lon) position at time of initialization [deg]
	 */
	void init();








} // namespace autogyrolanding
