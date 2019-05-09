/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @file MulticopterLandDetector.h
 * Land detection implementation for multicopters.
 *
 * @author Johan Jansen <jnsn.johan@gmail.com>
 * @author Morten Lysgaard <morten@lysgaard.no>
 * @author Julian Oes <julian@oes.ch>
 */

#pragma once

#include "LandDetector.h"

#include <parameters/param.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_bias.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_local_position.h>

using namespace time_literals;

namespace land_detector
{

class MulticopterLandDetector : public LandDetector
{
public:
	MulticopterLandDetector();

protected:
	void _initialize_topics() override;
	void _update_params() override;
	void _update_topics() override;

	bool _get_landed_state() override;
	bool _get_ground_contact_state() override;
	bool _get_maybe_landed_state() override;
	bool _get_freefall_state() override;
	bool _get_ground_effect_state() override;

	float _get_max_altitude() override;
private:

	/** Time in us that landing conditions have to hold before triggering a land. */
	static constexpr hrt_abstime LAND_DETECTOR_TRIGGER_TIME_US = 300_ms;

	/** Time in us that almost landing conditions have to hold before triggering almost landed . */
	static constexpr hrt_abstime MAYBE_LAND_DETECTOR_TRIGGER_TIME_US = 250_ms;

	/** Time in us that ground contact condition have to hold before triggering contact ground */
	static constexpr hrt_abstime GROUND_CONTACT_TRIGGER_TIME_US = 350_ms;

	/** Time interval in us in which wider acceptance thresholds are used after landed. */
	static constexpr hrt_abstime LAND_DETECTOR_LAND_PHASE_TIME_US = 2_s;

	/**
	* @brief Handles for interesting parameters
	**/
	struct {
		param_t maxClimbRate;
		param_t maxVelocity;
		param_t maxRotation;
		param_t minThrottle;
		param_t hoverThrottle;
		param_t minManThrottle;
		param_t freefall_acc_threshold;
		param_t freefall_trigger_time;
		param_t altitude_max;
		param_t landSpeed;
		param_t low_thrust_threshold;
	} _paramHandle{};

	struct {
		float maxClimbRate;
		float maxVelocity;
		float maxRotation_rad_s;
		float minThrottle;
		float hoverThrottle;
		float minManThrottle;
		float freefall_acc_threshold;
		float freefall_trigger_time;
		float altitude_max;
		float landSpeed;
		float low_thrust_threshold;
	} _params{};

	int _vehicleLocalPositionSub{ -1};
	int _vehicleLocalPositionSetpointSub{ -1};
	int _actuatorsSub{ -1};
	int _attitudeSub{ -1};
	int _sensor_bias_sub{ -1};
	int _vehicle_control_mode_sub{ -1};
	int _battery_sub{ -1};

	vehicle_local_position_s				_vehicleLocalPosition {};
	vehicle_local_position_setpoint_s	_vehicleLocalPositionSetpoint {};
	actuator_controls_s					_actuators {};
	vehicle_attitude_s					_vehicleAttitude {};
	sensor_bias_s					_sensors {};
	vehicle_control_mode_s				_control_mode {};
	battery_status_s						_battery {};

	hrt_abstime _min_trust_start{0};		///< timestamp when minimum trust was applied first
	hrt_abstime _landed_time{0};

	bool _in_descend{false};	///< vehicle is desending
	bool _horizontal_movement{false};	///< vehicle is moving horizontally

	/* get control mode dependent pilot throttle threshold with which we should quit landed state and take off */
	float _get_takeoff_throttle();
	bool _has_altitude_lock();
	bool _has_position_lock();
	bool _has_minimal_thrust();
	bool _has_low_thrust();
	bool _is_climb_rate_enabled();
};


} // namespace land_detector
