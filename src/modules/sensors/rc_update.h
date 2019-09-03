/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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

/**
 * @file rc_update.h
 *
 * @author Beat Kueng <beat-kueng@gmx.net>
 */

#include "parameters.h"

#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/rc_parameter_map.h>

namespace sensors
{

/**
 ** class RCUpdate
 *
 * Handling of RC updates
 */
class RCUpdate
{
public:
	/**
	 * @param parameters parameter values. These do not have to be initialized when constructing this object.
	 * Only when calling init(), they have to be initialized.
	 */
	RCUpdate(const Parameters &parameters);

	/**
	 * Check for changes in rc_parameter_map
	 */
	void rc_parameter_map_poll(ParameterHandles &parameter_handles, bool forced = false);

	/**
	 * update the RC functions. Call this when the parameters change.
	 */
	void update_rc_functions();

	/**
	 * Gather and publish RC input data.
	 */
	void		rc_poll(const ParameterHandles &parameter_handles);

private:

	/**
	 * Get and limit value for specified RC function. Returns NAN if not mapped.
	 */
	float		get_rc_value(uint8_t func, float min_value, float max_value);

	/**
	 * Get switch position for specified function.
	 */
	switch_pos_t	get_rc_sw3pos_position(uint8_t func, float on_th, bool on_inv, float mid_th, bool mid_inv);
	switch_pos_t	get_rc_sw2pos_position(uint8_t func, float on_th, bool on_inv);

	/**
	 * Update parameters from RC channels if the functionality is activated and the
	 * input has changed since the last update
	 *
	 * @param
	 */
	void set_params_from_rc(const ParameterHandles &parameter_handles);


	uORB::Subscription	_rc_sub{ORB_ID(input_rc)};				/**< raw rc channels data subscription */
	uORB::Subscription	_rc_parameter_map_sub{ORB_ID(rc_parameter_map)};	/**< rc parameter map subscription */

	uORB::Publication<rc_channels_s>	_rc_pub{ORB_ID(rc_channels)};				/**< raw r/c control topic */
	uORB::Publication<actuator_controls_s>	_actuator_group_3_pub{ORB_ID(actuator_controls_3)};	/**< manual control as actuator topic */

	uORB::PublicationMulti<manual_control_setpoint_s>	_manual_control_pub{ORB_ID(manual_control_setpoint), ORB_PRIO_HIGH};	/**< manual control signal topic */

	rc_channels_s _rc {};			/**< r/c channel data */

	rc_parameter_map_s _rc_parameter_map {};
	float _param_rc_values[rc_parameter_map_s::RC_PARAM_MAP_NCHAN] {};	/**< parameter values for RC control */

	const Parameters &_parameters;

	hrt_abstime _last_rc_to_param_map_time = 0;

	math::LowPassFilter2p _filter_roll; /**< filters for the main 4 stick inputs */
	math::LowPassFilter2p _filter_pitch; /** we want smooth setpoints as inputs to the controllers */
	math::LowPassFilter2p _filter_yaw;
	math::LowPassFilter2p _filter_throttle;

};



} /* namespace sensors */
