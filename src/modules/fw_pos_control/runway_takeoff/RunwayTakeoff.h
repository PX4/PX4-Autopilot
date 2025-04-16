/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
 * @file RunwayTakeoff.h
 * Runway takeoff handling for fixed-wing UAVs with steerable wheels.
 *
 * @author Roman Bapst <roman@px4.io>
 * @author Andreas Antener <andreas@uaventure.com>
 */

#ifndef RUNWAYTAKEOFF_H
#define RUNWAYTAKEOFF_H

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include <drivers/drv_hrt.h>
#include <px4_platform_common/module_params.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>

namespace runwaytakeoff
{

enum RunwayTakeoffState {
	THROTTLE_RAMP = 0, // ramping up throttle
	CLAMPED_TO_RUNWAY, // clamped to runway, controlling yaw directly (wheel or rudder)
	CLIMBOUT, // climbout to safe height before navigation
	FLY // navigate freely
};

class __EXPORT RunwayTakeoff : public ModuleParams
{
public:
	RunwayTakeoff(ModuleParams *parent) : ModuleParams(parent) {}
	~RunwayTakeoff() = default;

	/**
	 * @brief Initializes the state machine.
	 *
	 * @param time_now Absolute time since system boot [us]
	 */
	void init(const hrt_abstime &time_now);

	/**
	 * @brief Updates the state machine based on the current vehicle condition.
	 *
	 * @param time_now Absolute time since system boot [us]
	 * @param takeoff_airspeed Calibrated airspeed setpoint for the takeoff climbout [m/s]
	 * @param calibrated_airspeed Vehicle calibrated airspeed [m/s]
	 * @param vehicle_altitude Vehicle altitude (AGL) [m]
	 * @param clearance_altitude Altitude (AGL) above which we have cleared all occlusions in the runway path [m]
	 */
	void update(const hrt_abstime &time_now, const float takeoff_airspeed, const float calibrated_airspeed,
		    const float vehicle_altitude, const float clearance_altitude);

	/**
	 * @return Current takeoff state
	 */
	RunwayTakeoffState getState() { return takeoff_state_; }

	/**
	 * @return The state machine is initialized
	 */
	bool isInitialized() { return initialized_; }

	/**
	 * @return Runway takeoff is enabled
	 */
	bool runwayTakeoffEnabled() { return param_rwto_tkoff_.get(); }

	/**
	 * @return Pitch angle setpoint (limited while plane is on runway) [rad]
	 */
	float getPitch();

	/**
	 * @return Roll angle setpoint [rad]
	 */
	float getRoll();

	/**
	 * @brief Returns the throttle setpoint.
	 *
	 * Ramps up over RWTO_RAMP_TIME to RWTO_MAX_THR until the aircraft lifts off the runway and then
	 * ramps from RWTO_MAX_THR to the externally defined throttle setting over the takeoff rotation time
	 *
	 * @param idle_throttle normalized [0,1]
	 * @return Throttle setpoint, normalized [0,1]
	 */
	float getThrottle(const float idle_throttle) const;

	/**
	 * @param min_pitch_in_climbout Minimum pitch angle during climbout [rad]
	 * @param min_pitch Externally commanded minimum pitch angle [rad]
	 * @return Minimum pitch angle [rad]
	 */
	float getMinPitch(const float min_pitch_in_climbout, const float min_pitch) const;

	/**
	 * @param max_pitch Externally commanded maximum pitch angle [rad]
	 * @return Maximum pitch angle [rad]
	 */
	float getMaxPitch(const float max_pitch) const;

	// NOTE: this is only to be used for mistaken mode transitions to takeoff while already in air
	void forceSetFlyState() { takeoff_state_ = RunwayTakeoffState::FLY; }

	/**
	 * @brief Reset the state machine.
	 */
	void reset();

	/**
	 * @brief Linearly interpolates between a start and end value over an absolute time span.
	 *
	 * @param start_value
	 * @param end_value
	 * @param start_time Absolute start time [us]
	 * @param interpolation_time The time span to interpolate over [s]
	 * @return interpolated value
	 */
	float interpolateValuesOverAbsoluteTime(const float start_value, const float end_value, const hrt_abstime &start_time,
						const float interpolation_time) const;

private:
	/**
	 * Current state of runway takeoff procedure
	 */
	RunwayTakeoffState takeoff_state_{THROTTLE_RAMP};

	/**
	 * True if the runway state machine is initialized
	 */
	bool initialized_{false};

	/**
	 * The absolute time since system boot at which the state machine was intialized [us]
	 */
	hrt_abstime time_initialized_{0};

	/**
	 * The absolute time the takeoff state transitions from CLAMPED_TO_RUNWAY -> CLIMBOUT [us]
	 */
	hrt_abstime takeoff_time_{0};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::RWTO_TKOFF>) param_rwto_tkoff_,
		(ParamFloat<px4::params::RWTO_MAX_THR>) param_rwto_max_thr_,
		(ParamFloat<px4::params::RWTO_PSP>) param_rwto_psp_,
		(ParamFloat<px4::params::RWTO_RAMP_TIME>) param_rwto_ramp_time_,
		(ParamFloat<px4::params::RWTO_ROT_AIRSPD>) param_rwto_rot_airspd_,
		(ParamFloat<px4::params::RWTO_ROT_TIME>) param_rwto_rot_time_
	)
};

} // namespace runwaytakeoff

#endif // RUNWAYTAKEOFF_H
