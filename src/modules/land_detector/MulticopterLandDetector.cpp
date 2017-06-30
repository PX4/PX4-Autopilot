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
 * @file MulticopterLandDetector.cpp
 *
 *The MC land-detector goes through 3 states before it will detect landed:
 *
 *State 1 (=ground_contact):
 *ground_contact is detected once the vehicle is not moving along the NED-z direction and has
 *a thrust value below 0.3 of the thrust_range (thrust_max - thrust_min). The condition has to be true
 *for GROUND_CONTACT_TRIGGER_TIME_US in order to detect ground_contact
 *
 *State 2 (=maybe_landed):
 *maybe_landed can only occur if the internal ground_contact hysteresis state is true. maybe_landed criteria requires to have no motion in x and y,
 *no rotation and a thrust below 0.1 of the thrust_range (thrust_max - thrust_min). In addition, the mc_pos_control turns off the thrust_sp in
 *body frame along x and y. The criteria for maybe_landed needs to be true for MAYBE_LAND_DETECTOR_TRIGGER_TIME_US.
 *
 *State 3 (=landed)
 *landed can only be detected if maybe_landed is true for LAND_DETECTOR_TRIGGER_TIME_US. No farther criteria is tested, but the mc_pos_control goes into
 *idle (thrust_sp = 0). By doing this the thrust_criteria of State 2 will always be met, however the remaining criteria of no rotation and no motion still
 *has to be valid.

 *It is to note that if one criteria is not met, then vehicle exits the state directly without blocking.
 *
 *If the land-detector does not detect ground_contact, then the vehicle is either flying or falling, where free fall detection heavily relies
 *on the acceleration. TODO: verify that free fall is reliable
 *
 * @author Johan Jansen <jnsn.johan@gmail.com>
 * @author Morten Lysgaard <morten@lysgaard.no>
 * @author Julian Oes <julian@oes.ch>
 */

#include <cmath>
#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>

#include "MulticopterLandDetector.h"


namespace land_detector
{

MulticopterLandDetector::MulticopterLandDetector() :
	_paramHandle(),
	_params(),
	_vehicleLocalPositionSub(-1),
	_vehicleLocalPositionSetpointSub(-1),
	_actuatorsSub(-1),
	_armingSub(-1),
	_attitudeSub(-1),
	_manualSub(-1),
	_ctrl_state_sub(-1),
	_vehicle_control_mode_sub(-1),
	_battery_sub(-1),
	_vehicleLocalPosition{},
	_vehicleLocalPositionSetpoint{},
	_actuators{},
	_arming{},
	_vehicleAttitude{},
	_manual{},
	_ctrl_state{},
	_control_mode{},
	_battery{},
	_min_trust_start(0),
	_arming_time(0)
{
	_paramHandle.maxRotation = param_find("LNDMC_ROT_MAX");
	_paramHandle.maxVelocity = param_find("LNDMC_XY_VEL_MAX");
	_paramHandle.maxClimbRate = param_find("LNDMC_Z_VEL_MAX");
	_paramHandle.throttleRange = param_find("LNDMC_THR_RANGE");
	_paramHandle.minThrottle = param_find("MPC_THR_MIN");
	_paramHandle.hoverThrottle = param_find("MPC_THR_HOVER");
	_paramHandle.minManThrottle = param_find("MPC_MANTHR_MIN");
	_paramHandle.freefall_acc_threshold = param_find("LNDMC_FFALL_THR");
	_paramHandle.freefall_trigger_time = param_find("LNDMC_FFALL_TTRI");
	_paramHandle.manual_stick_down_threshold = param_find("LNDMC_MAN_DWNTHR");
	_paramHandle.altitude_max = param_find("LNDMC_ALT_MAX");
	_paramHandle.manual_stick_up_position_takeoff_threshold = param_find("LNDMC_POS_UPTHR");
	_paramHandle.landSpeed = param_find("MPC_LAND_SPEED");
}

void MulticopterLandDetector::_initialize_topics()
{
	// subscribe to position, attitude, arming and velocity changes
	_vehicleLocalPositionSub = orb_subscribe(ORB_ID(vehicle_local_position));
	_vehicleLocalPositionSetpointSub = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));
	_attitudeSub = orb_subscribe(ORB_ID(vehicle_attitude));
	_actuatorsSub = orb_subscribe(ORB_ID(actuator_controls_0));
	_armingSub = orb_subscribe(ORB_ID(actuator_armed));
	_parameterSub = orb_subscribe(ORB_ID(parameter_update));
	_manualSub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
	_vehicle_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_battery_sub = orb_subscribe(ORB_ID(battery_status));
}

void MulticopterLandDetector::_update_topics()
{
	_orb_update(ORB_ID(vehicle_local_position), _vehicleLocalPositionSub, &_vehicleLocalPosition);
	_orb_update(ORB_ID(vehicle_local_position_setpoint), _vehicleLocalPositionSetpointSub, &_vehicleLocalPositionSetpoint);
	_orb_update(ORB_ID(vehicle_attitude), _attitudeSub, &_vehicleAttitude);
	_orb_update(ORB_ID(actuator_controls_0), _actuatorsSub, &_actuators);
	_orb_update(ORB_ID(actuator_armed), _armingSub, &_arming);
	_orb_update(ORB_ID(manual_control_setpoint), _manualSub, &_manual);
	_orb_update(ORB_ID(control_state), _ctrl_state_sub, &_ctrl_state);
	_orb_update(ORB_ID(vehicle_control_mode), _vehicle_control_mode_sub, &_control_mode);
	_orb_update(ORB_ID(battery_status), _battery_sub, &_battery);
}

void MulticopterLandDetector::_update_params()
{
	param_get(_paramHandle.maxClimbRate, &_params.maxClimbRate);
	param_get(_paramHandle.maxVelocity, &_params.maxVelocity);
	param_get(_paramHandle.maxRotation, &_params.maxRotation_rad_s);
	_params.maxRotation_rad_s = math::radians(_params.maxRotation_rad_s);
	param_get(_paramHandle.minThrottle, &_params.minThrottle);
	param_get(_paramHandle.hoverThrottle, &_params.hoverThrottle);
	param_get(_paramHandle.throttleRange, &_params.throttleRange);
	param_get(_paramHandle.minManThrottle, &_params.minManThrottle);
	param_get(_paramHandle.freefall_acc_threshold, &_params.freefall_acc_threshold);
	param_get(_paramHandle.freefall_trigger_time, &_params.freefall_trigger_time);
	_freefall_hysteresis.set_hysteresis_time_from(false, (hrt_abstime)(1e6f * _params.freefall_trigger_time));
	param_get(_paramHandle.manual_stick_down_threshold, &_params.manual_stick_down_threshold);
	param_get(_paramHandle.altitude_max, &_params.altitude_max);
	param_get(_paramHandle.manual_stick_up_position_takeoff_threshold, &_params.manual_stick_up_position_takeoff_threshold);
	param_get(_paramHandle.landSpeed, &_params.landSpeed);
}


bool MulticopterLandDetector::_get_freefall_state()
{
	if (_params.freefall_acc_threshold < 0.1f
	    || _params.freefall_acc_threshold > 10.0f) {	//if parameter is set to zero or invalid, disable free-fall detection.
		return false;
	}

	if (_ctrl_state.timestamp == 0) {
		// _ctrl_state is not valid yet, we have to assume we're not falling.
		return false;
	}

	float acc_norm = _ctrl_state.x_acc * _ctrl_state.x_acc
			 + _ctrl_state.y_acc * _ctrl_state.y_acc
			 + _ctrl_state.z_acc * _ctrl_state.z_acc;
	acc_norm = sqrtf(acc_norm);	//norm of specific force. Should be close to 9.8 m/s^2 when landed.

	return (acc_norm < _params.freefall_acc_threshold);	//true if we are currently falling
}

bool MulticopterLandDetector::_get_ground_contact_state()
{
	// Time base for this function
	const uint64_t now = hrt_absolute_time();

	// only trigger flight conditions if we are armed
	if (!_arming.armed) {
		_arming_time = 0;
		return true;

	} else if (_arming_time == 0) {
		_arming_time = now;
	}

	// If in manual flight mode never report landed if the user has more than idle throttle
	// Check if user commands throttle and if so, report no ground contact based on
	// the user intent to take off (even if the system might physically still have
	// ground contact at this point).
	const bool manual_control_idle = (_has_manual_control_present() && _manual.z < _params.manual_stick_down_threshold);
	const bool manual_control_idle_or_auto = manual_control_idle || !_control_mode.flag_control_manual_enabled;

	// Widen acceptance thresholds for landed state right after arming
	// so that motor spool-up and other effects do not trigger false negatives.
	float armThresholdFactor = 1.0f;

	if (hrt_elapsed_time(&_arming_time) < LAND_DETECTOR_ARM_PHASE_TIME_US) {
		armThresholdFactor = 2.5f;
	}

	// Check if we are moving vertically - this might see a spike after arming due to
	// throttle-up vibration. If accelerating fast the throttle thresholds will still give
	// an accurate in-air indication.
	bool verticalMovement = fabsf(_vehicleLocalPosition.vz) > _params.maxClimbRate * armThresholdFactor;

	// if we have a valid velocity setpoint and the vehicle is demanded to go down but no vertical movement present,
	// we then can assume that the vehicle hit ground
	bool in_descend = _is_velocity_control_active() && (_vehicleLocalPositionSetpoint.vz >= 0.9f * _params.landSpeed);
	bool hit_ground = in_descend && !verticalMovement;

	// If pilots commands down or in auto mode and we are already below minimal thrust and we do not move down we assume ground contact
	// TODO: we need an accelerometer based check for vertical movement for flying without GPS
	if (manual_control_idle_or_auto && (_has_low_thrust() || hit_ground) &&
	    (!verticalMovement || !_has_altitude_lock())) {
		return true;
	}

	return false;
}

bool MulticopterLandDetector::_get_maybe_landed_state()
{
	// Time base for this function
	const uint64_t now = hrt_absolute_time();

	// only trigger flight conditions if we are armed
	if (!_arming.armed) {
		return true;
	}

	// If we control manually and are still landed, we want to stay idle until the pilot rises the throttle for takeoff
	if (_state == LandDetectionState::LANDED && _has_manual_control_present()) {
		if (_manual.z < _get_takeoff_throttle()) {
			return true;

		} else {
			// Pilot wants to take off, assume no groundcontact anymore and therefore allow thrust
			_ground_contact_hysteresis.set_state_and_update(false);
			return false;
		}
	}

	if (_has_minimal_thrust()) {
		if (_min_trust_start == 0) {
			_min_trust_start = now;
		}

	} else {
		_min_trust_start = 0;
	}

	// Return status based on armed state and throttle if no position lock is available.
	if (!_has_altitude_lock()) {
		// The system has minimum trust set (manual or in failsafe)
		// if this persists for 8 seconds AND the drone is not
		// falling consider it to be landed. This should even sustain
		// quite acrobatic flight.
		return (_min_trust_start > 0) && (hrt_elapsed_time(&_min_trust_start) > 8000000);
	}

	float armThresholdFactor = 1.0f;

	// Widen acceptance thresholds for landed state right after arming
	// so that motor spool-up and other effects do not trigger false negatives.
	if (hrt_elapsed_time(&_arming_time) < LAND_DETECTOR_ARM_PHASE_TIME_US) {
		armThresholdFactor = 2.5f;
	}

	// Check if we are moving horizontally.
	bool horizontalMovement = sqrtf(_vehicleLocalPosition.vx * _vehicleLocalPosition.vx
					+ _vehicleLocalPosition.vy * _vehicleLocalPosition.vy) > _params.maxVelocity;

	// Next look if all rotation angles are not moving.
	float maxRotationScaled = _params.maxRotation_rad_s * armThresholdFactor;

	bool rotating = (fabsf(_vehicleAttitude.rollspeed)  > maxRotationScaled) ||
			(fabsf(_vehicleAttitude.pitchspeed) > maxRotationScaled) ||
			(fabsf(_vehicleAttitude.yawspeed) > maxRotationScaled);

	if (_ground_contact_hysteresis.get_state() && _has_minimal_thrust() && !rotating &&
	    (!horizontalMovement || !_has_position_lock())) {
		// Ground contact, no thrust and no movement -> landed
		return true;
	}

	return false;
}

bool MulticopterLandDetector::_get_landed_state()
{
	// if we have maybe_landed, the mc_pos_control goes into idle (thrust_sp = 0.0)
	// therefore check if all other condition of the landed state remain true
	return _maybe_landed_hysteresis.get_state();

}

float MulticopterLandDetector::_get_takeoff_throttle()
{
	/* Position mode */
	if (_control_mode.flag_control_manual_enabled && _control_mode.flag_control_altitude_enabled) {
		/* Should be above 0.5 because below that we do not gain altitude and won't take off.
		 * Also it should be quite high such that we don't accidentally take off when using
		 * a spring loaded throttle and have a useful vertical speed to start with. */
		return _params.manual_stick_up_position_takeoff_threshold;
	}

	/* Manual/attitude mode */
	if (_control_mode.flag_control_manual_enabled && _control_mode.flag_control_attitude_enabled) {
		/* Should be quite low and certainly below hover throttle because pilot controls throttle manually. */
		return 0.15f;
	}

	/* As default for example in acro mode we do not want to stay landed. */
	return 0.0f;
}

float MulticopterLandDetector::_get_max_altitude()
{
	/* ToDo: add a meaningful altitude */
	float valid_altitude_max = _params.altitude_max;

	if (_battery.warning == battery_status_s::BATTERY_WARNING_LOW) {
		valid_altitude_max = _params.altitude_max * 0.75f;
	}

	if (_battery.warning == battery_status_s::BATTERY_WARNING_CRITICAL) {
		valid_altitude_max = _params.altitude_max * 0.5f;
	}

	if (_battery.warning == battery_status_s::BATTERY_WARNING_EMERGENCY) {
		valid_altitude_max = _params.altitude_max * 0.25f;
	}

	return valid_altitude_max;
}

bool MulticopterLandDetector::_has_altitude_lock()
{
	return _vehicleLocalPosition.timestamp != 0 &&
	       hrt_elapsed_time(&_vehicleLocalPosition.timestamp) < 500000 &&
	       _vehicleLocalPosition.z_valid;
}

bool MulticopterLandDetector::_has_position_lock()
{
	return _has_altitude_lock() && _vehicleLocalPosition.xy_valid;
}

bool MulticopterLandDetector::_has_manual_control_present()
{
	return _control_mode.flag_control_manual_enabled && _manual.timestamp > 0;
}

bool MulticopterLandDetector::_is_velocity_control_active()
{
	bool is_finite = PX4_ISFINITE(_vehicleLocalPositionSetpoint.vx) && PX4_ISFINITE(_vehicleLocalPositionSetpoint.vy)
			 && PX4_ISFINITE(_vehicleLocalPositionSetpoint.vz);

	return (_vehicleLocalPositionSetpoint.timestamp != 0) &&
	       (hrt_elapsed_time(&_vehicleLocalPositionSetpoint.timestamp) < 500000) && is_finite;
}

bool MulticopterLandDetector::_has_low_thrust()
{
	// 30% of throttle range between min and hover
	float sys_min_throttle = _params.minThrottle + (_params.hoverThrottle - _params.minThrottle) * 0.3f;

	// Check if thrust output is less than the minimum auto throttle param.
	return _actuators.control[3] <= sys_min_throttle;
}

bool MulticopterLandDetector::_has_minimal_thrust()
{
	// 10% of throttle range between min and hover once we entered ground contact
	float sys_min_throttle = _params.minThrottle + (_params.hoverThrottle - _params.minThrottle) * _params.throttleRange;

	// Determine the system min throttle based on flight mode
	if (!_control_mode.flag_control_altitude_enabled) {
		sys_min_throttle = (_params.minManThrottle + 0.01f);
	}

	// Check if thrust output is less than the minimum auto throttle param.
	return _actuators.control[3] <= sys_min_throttle;
}

}
