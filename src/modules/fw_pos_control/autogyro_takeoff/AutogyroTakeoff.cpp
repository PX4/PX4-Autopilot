/****************************************************************************
 *
 *   Copyright (c) 2021-2023 PX4 Development Team. All rights reserved.
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

namespace autogyrotakeoff
{



//void AutogyroTakeoff::init(const hrt_abstime &now, float yaw, const matrix::Vector2d &start_pos_global)
void AutogyroTakeoff::init(const hrt_abstime &time_now, const float initial_yaw,
			   const matrix::Vector2d &start_pos_global)
{
	initial_yaw_ = initial_yaw;
	initialized_ = true;
	state_ = AutogyroTakeoffState::PRE_TAKEOFF_PREROTATE_START;
	state_last_ = AutogyroTakeoffState::PRE_TAKEOFF_PREROTATE_START;
	time_initialized_ = time_now;
	//_time_in_state_= time_now;
	last_sent_release_status_ = time_now;
	climbout_ = true; // this is true until climbout is finished

	takeoff_wp_ = start_pos_global;
	initial_wp_ = start_pos_global;
}

void AutogyroTakeoff::update(const hrt_abstime &time_now, const float takeoff_airspeed, const float calibrated_airspeed,
			     const float vehicle_altitude, const float clearance_altitude, float rotor_rpm, orb_advert_t *mavlink_log_pub,
				 const matrix::Vector2d &current_pos_global)
{
	climbout_ = true;
	takeoff_status_s takeoff_status = {};

	actuator_armed_s actuator_armed;
	_actuator_armed_sub.update(&actuator_armed);

	//PX4_INFO("Takeoff: state %i, altitude: %f to %f", state_, (double)vehicle_altitude, (double)clearance_altitude);

	// TODO Zachrana pri uzemeni po startu.
	//	if (actuator_armed.manual_lockdown && state_<= AutogyroTakeoffState::PRE_TAKEOFF_RAMPUP) {
	//		state_= AutogyroTakeoffState::TAKEOFF_ERROR;
	//	}

	switch (state_) {
	/*
	    Hangling error states of takeoff mode. Should lead in allerting operator and/or
	    abrod takeoff process

	    IN: error state
	*/
	case AutogyroTakeoffState::TAKEOFF_ERROR: {
			if (state_ != state_last_) {
				PX4_INFO("ERR STATE");
				mavlink_log_info(mavlink_log_pub, "#Takeoff: Error state");
			}
		}
		break;

	/*
	    Initial state of regulator, wait for manual prerotate of rotor.

	    IN: initial, reset
	    OUT: Minimal rotor RPM
	*/
	case AutogyroTakeoffState::PRE_TAKEOFF_PREROTATE_START:

		if (rotor_rpm > param_ag_prerotator_minimal_rpm_.get()) {

			// Eletrical prerotator, controlled from autopilot
			if (param_ag_prerotator_type_.get() == AutogyroTakeoffType::ELPREROT_PLATFORM
			    || param_ag_prerotator_type_.get() == AutogyroTakeoffType::ELPREROT_RUNWAY) {
				if (doPrerotate()) {
					play_next_tone();
					setState(AutogyroTakeoffState::PRE_TAKEOFF_PREROTATE, time_now);
					// state_= AutogyroTakeoffState::PRE_TAKEOFF_PREROTATE;
					// time_in_state_ = time_now;
				}

			} else { // manually controlled prerotator or prerotation by forward movement
				play_next_tone();
				setState(AutogyroTakeoffState::PRE_TAKEOFF_PREROTATE, time_now);
				// state_= AutogyroTakeoffState::PRE_TAKEOFF_PREROTATE;
				// time_in_state_ = time_now;
			}

			mavlink_log_info(mavlink_log_pub, "Takeoff: minimal RPM for prerotator reached");
			PX4_INFO("Takeoff: minimal RPM for prerotator reached");
		}

		break;


	/*
	    Reach minimal RPM of rotor. It can be ensured with several ways. For autogyro
	    with electronic prerotator, it can be controlled from autopilot or this is command
	    for get some groundspeed for airflow and for prerotation of rotor

	    IN: rotor with minimal RPM,
	    PROCESS: increase rotor RPM to flight RPM
	    OUT: rotor in flight state
	*/
	case AutogyroTakeoffState::PRE_TAKEOFF_PREROTATE:  // 1

		if (rotor_rpm > param_ag_prerotator_target_rpm_.get()) {
			setState(AutogyroTakeoffState::PRE_TAKEOFF_DONE, time_now);
			// state_= AutogyroTakeoffState::PRE_TAKEOFF_DONE;
			// time_in_state_ = time_now;
			play_next_tone();
			mavlink_log_info(mavlink_log_pub, "Takeoff, prerotator RPM reached");
			PX4_INFO("Takeoff: prerotator RPM reached");
		}

		break;

	/*
	    All required takeoff conditions are satisfied. Now it is prepared to
	    try to start main motor.

	    IN: rotor prepared;
	    OUT: rotor prepared; minimal airspeed; motor max power,
	*/
	case AutogyroTakeoffState::PRE_TAKEOFF_DONE: {     // 2
			bool ready_for_release = true;

			if (rotor_rpm < param_ag_rotor_flight_rpm_.get()) {
				ready_for_release = false;
				PX4_INFO("Takeofff, waiting for flight rpm.");
				// Some histesis needs to be applied for the start interrupt procedure.
				// Currently, this does not allow the start to be interrupted.
				//_state = AutogyroTakeoffState::PRE_TAKEOFF_PREROTATE;
				//time_in_state_ = time_now;
			}

			// check minimal airspeed
			if (takeoff_airspeed < (param_fw_airspd_min_.get() * param_rwto_airspd_scl_.get())) {
				ready_for_release = false;
				PX4_INFO("Takeofff, waiting for min airspeed.");
			}

			if (ready_for_release) {
				initial_yaw_ = get_bearing_to_next_waypoint(initial_wp_(0), initial_wp_(1), current_pos_global(0), current_pos_global(1));

				setState(AutogyroTakeoffState::PRE_TAKEOFF_RAMPUP, time_now);
				// state_= AutogyroTakeoffState::PRE_TAKEOFF_RAMPUP;
				// time_in_state_ = time_now;
				mavlink_log_info(mavlink_log_pub, "Ready to start motor");
				//PX4_INFO("Takeoff, Please release.");
				play_next_tone();
			}

		}
		break;

	/*
	    Slowly rampup motor. Keep trying to check other flight parameters.
	    If I can still fly
	*/
	case AutogyroTakeoffState::PRE_TAKEOFF_RAMPUP: {
			bool ready_for_release = true;

			if (rotor_rpm < param_ag_rotor_flight_rpm_.get()) {
				ready_for_release = false;
				//PX4_INFO("Takeofff, waiting for flight rpm.");
				// Some histesis needs to be applied for the start interrupt procedure.
				// Currently, this does not allow the start to be interrupted.
				//_state = AutogyroTakeoffState::PRE_TAKEOFF_PREROTATE;
				//time_in_state_ = time_now;
			}

			// check minimal airspeed
			if (takeoff_airspeed < (param_fw_airspd_min_.get() * param_rwto_airspd_scl_.get())) {
				ready_for_release = false;
				//PX4_INFO("Takeofff, waiting for min airspeed.");
			}

			// ramp-up time elapsed
			if (hrt_elapsed_time(&time_in_state_) < (param_rwto_ramp_time_.get() * 1_s * 2.0f)) {
				ready_for_release = false;
			}

			// Check if motor/esc power (properller RPM) is suffiscient
			// TODO
			if (false) {
				ready_for_release = false;
			}

			if (ready_for_release) {

				// state_= AutogyroTakeoffState::TAKEOFF_RELEASE;
				// time_in_state_ = time_now;
				setState(AutogyroTakeoffState::TAKEOFF_RELEASE, time_now);
				doRelease(true);
				PX4_INFO("Takeoff, Please release.");
				play_next_tone();
			}
		}
		break;

	/*
	    Command for release. Sound signal for release from hand or release from
	    some takeoff platform with mavlink command. This step ends on release ACK.
	    In the case of hand release it is done inmedietly. If it is not ACKed
	    it fall in error state


	    IN: autogyro is prepared for takeoff
	    OUT: Command for release
	*/
	case AutogyroTakeoffState::TAKEOFF_RELEASE: {

			play_release_tone();

			if (vehicle_altitude > clearance_altitude*0.8f) {
				mavlink_log_info(mavlink_log_pub, "Climbout");
				PX4_INFO("Takeoff: Climbout.");
				setState(AutogyroTakeoffState::TAKEOFF_CLIMBOUT, time_now);
				//state_= AutogyroTakeoffState::TAKEOFF_CLIMBOUT;
				play_next_tone();
				//time_in_state_ = time_now;

				// set current position as center of loiter
				// TODO
				// if (param_rwto_hdg_.get() == 0) {
				// 	takeoff_wp_(0) = current_lat;
				// 	takeoff_wp_(1) = current_lon;
				// }
			}
		}
		break;

	/*
	    Reach minimal altitude and then fly!

	    IN: Released
	    OUT: Mission continue
	*/
	case AutogyroTakeoffState::TAKEOFF_CLIMBOUT:
		if (vehicle_altitude > clearance_altitude) {
			climbout_ = false;
			setState(AutogyroTakeoffState::FLY, time_now);
			//#state_= AutogyroTakeoffState::FLY;
			//time_in_state_ = time_now;
			PX4_INFO("Takeoff:FLY.");
		}

		//climbout_ = false;
		break;

	case AutogyroTakeoffState::FLY:
		climbout_ = false;
		break;

	default:
		break;
	}



	takeoff_status.time_in_state = hrt_elapsed_time(&time_in_state_);
	takeoff_status.takeoff_state = (int) state_;
	_takeoff_status_pub.publish(takeoff_status);

	if (hrt_elapsed_time(&last_sent_release_status_) > 1_s / 4 || state_ != state_last_) {
		last_sent_release_status_ = time_now;
		debug_value_s takeoff_information{};
		takeoff_information.timestamp = time_now;
		// templorary sollution, should be changed by custom mavlink message
		takeoff_information.value = state_;
		_takeoff_informations_pub.publish(takeoff_information);
	}

	state_last_ = state_;
}

/*
 * Send command for release from hand or from some platform
 */
bool AutogyroTakeoff::doRelease(bool release)
{
	// vehicle_command_s vcmd = {};

	// vcmd.command = vehicle_command_s::VEHICLE_CMD_DO_WINCH;
	// vcmd.param1 = 1;
	// vcmd.param2 = (int)release; // Action - 0: Hold, 1: Release
	// vcmd.param3 = 0;  // Length -
	// vcmd.param4 = 0;  // Rate - Required speed
	// vcmd.param5 = 0;
	// vcmd.param6 = 0;
	// vcmd.param7 = 0;

	return release;
}


/*
 * Send command for start prerotation or for obtain formard movement
 */
bool AutogyroTakeoff::doPrerotate()
{
	return true;
}


void AutogyroTakeoff::setState(AutogyroTakeoffState state, const hrt_abstime &time_now)
{
	state_ = state;
	time_state_start_ = time_now;
	time_in_state_ = time_now;
	PX4_INFO("State changed: %i", state);
}

// float AutogyroTakeoff::getRequestedAirspeed()
// {
// 	switch (_state) {
// 	case AutogyroTakeoffState::PRE_TAKEOFF_PREROTATE_START:
// 	case AutogyroTakeoffState::PRE_TAKEOFF_PREROTATE:
// 	case AutogyroTakeoffState::PRE_TAKEOFF_DONE:
// 	case AutogyroTakeoffState::PRE_TAKEOFF_RAMPUP:
// 		return param_fw_airspd_min_.get() * param_rwto_airspd_scl_.get();

// 	default:
// 		return param_fw_airspd_trim_.get();
// 	}
// }

/*
 * Returns true as long as we're below navigation altitude
 */
bool AutogyroTakeoff::controlYaw()
{
	return false;
	// keep controlling yaw directly until we start navigation
	return state_ < AutogyroTakeoffState::TAKEOFF_CLIMBOUT;
}

/*
 * Returns pitch setpoint to use.
 *
 * Limited (parameter) as long as the plane is on runway. Otherwise
 * use the one from TECS
 */
float AutogyroTakeoff::getPitch(float external_pitch_setpoint)
{
	switch (state_) {

	case AutogyroTakeoffState::TAKEOFF_ERROR:
		return 0;

	case AutogyroTakeoffState::PRE_TAKEOFF_PREROTATE_START: // 0 Null pitch
	case AutogyroTakeoffState::PRE_TAKEOFF_PREROTATE:   // 1 maximal pitch
	case AutogyroTakeoffState::PRE_TAKEOFF_DONE: // 2
	case AutogyroTakeoffState::PRE_TAKEOFF_RAMPUP:
		//return math::radians(param_rwto_max_pitch_.get());
		return math::radians(param_rwto_psp_.get());

	// FLy
	default:
		return external_pitch_setpoint;
	}
}

/*
 * Returns the roll setpoint to use.
 */
float AutogyroTakeoff::getRoll(float external_roll_setpoint)
{
	// until we have enough ground clearance, set roll to 0
	if (state_ < AutogyroTakeoffState::TAKEOFF_RELEASE) {
		return 0.0f;
	}

	// allow some limited roll during RELEASE and CLIMBOUT
	else if (state_ < AutogyroTakeoffState::FLY) {
		return math::constrain(external_roll_setpoint,
				       math::radians(-param_ag_tko_max_roll_.get()),
				       math::radians(param_ag_tko_max_roll_.get()));
	}

	return external_roll_setpoint;
}

/*
 * Returns the yaw setpoint to use.
 *
 * In heading hold mode (_heading_mode == 0), it returns initial yaw as long as it's on the
 * runway. When it has enough ground clearance we start navigation towards WP.
 */
float AutogyroTakeoff::getYaw(float external_yaw_setpoint)
{
	//return external_yaw_setpoint;

	if (param_rwto_hdg_.get() == 0 && state_ < AutogyroTakeoffState::TAKEOFF_CLIMBOUT) {
		PX4_INFO("initial yaw: %f", (double)initial_yaw_);
		return initial_yaw_;

	} else {
		return external_yaw_setpoint;
	}
}

/*
 * Returns the throttle setpoint to use.
 *
 * Ramps up in the beginning, until it lifts off the runway it is set to
 * parameter value, then it returns the TECS throttle.
 */
float AutogyroTakeoff::getThrottle(const float idle_throttle, const float external_throttle_setpoint) const
{

	//float idle = (double)param_fw_thr_idle_.get();
	float idle = (double)idle_throttle;

	//PX4_INFO("GET THROTTLE %f, state: %f, time: %f", (double)idle, (double)state_, (double)(now - time_in_state_));

	switch (state_) {

	case AutogyroTakeoffState::TAKEOFF_ERROR:
	case AutogyroTakeoffState::PRE_TAKEOFF_PREROTATE_START:
		return 0;

	case AutogyroTakeoffState::PRE_TAKEOFF_PREROTATE: {
			if (param_ag_prerotator_type_.get() == AutogyroTakeoffType::WOPREROT_RUNWAY) {
			// Without prerotator, forward movement obtained by throttle
				float throttle = (hrt_elapsed_time(&time_in_state_) / (param_rwto_ramp_time_.get() * 1_s)) * param_rwto_max_thr_.get();
				return math::min(throttle, param_rwto_max_thr_.get());
			} else {
				return idle;
			}
		}

	case AutogyroTakeoffState::PRE_TAKEOFF_DONE: {
			if (param_ag_prerotator_type_.get() == AutogyroTakeoffType::WOPREROT_RUNWAY) {
			// Without prerotator, forward movement obtained by throttle
				return param_rwto_max_thr_.get();
			} else {
				return idle;
			}
		}

	case AutogyroTakeoffState::PRE_TAKEOFF_RAMPUP: {
			float throttle = idle;

			if (param_ag_prerotator_type_.get() == AutogyroTakeoffType::WOPREROT_RUNWAY) {
			// Without prerotator, forward movement obtained by throttle
				throttle = param_rwto_max_thr_.get();

			} else if (param_ag_prerotator_type_.get() == AutogyroTakeoffType::WOPREROT_PLATFORM) {
			// without prerotator, forward movement by platform (car, winch) speed
				throttle = (hrt_elapsed_time(&time_in_state_) / (param_rwto_ramp_time_.get() * 1_s)) * param_rwto_max_thr_.get();
				throttle = math::min(throttle, param_rwto_max_thr_.get());
			}
			return throttle;
		}


	case AutogyroTakeoffState::TAKEOFF_RELEASE: {
			return math::max(external_throttle_setpoint, param_rwto_max_thr_.get());
		}

	// TAKEOFF_CLIMBOUT a FLY
	default:
		return external_throttle_setpoint;
	}
}


/*
 * Returns the minimum pitch for TECS to use.
 *
 * In climbout we either want what was set on the waypoint (sp_min) but at least
 * the climbtout minimum pitch (parameter).
 * Otherwise use the minimum that is enforced generally (parameter).
 */
float AutogyroTakeoff::getMinPitch(float min_pitch_in_climbout, float min_pitch) const
{
	if (state_ < AutogyroTakeoffState::FLY) {
		return min_pitch_in_climbout;
	}

	else {
		return min_pitch;
	}
}

/*
 * Returns the maximum pitch for TECS to use.
 *
 * Limited by parameter (if set) until climbout is done.
 */
float AutogyroTakeoff::getMaxPitch(float max_pitch) const
{
	// use max pitch from parameter if set (> 0.1)
	if (state_ < AutogyroTakeoffState::FLY && param_rwto_max_pitch_.get() > 0.1f) {
		return param_rwto_max_pitch_.get();
	}

	else {
		return max_pitch;
	}
}


bool AutogyroTakeoff::resetIntegrators()
{
	// reset integrators if we're still on runway
	return state_ < AutogyroTakeoffState::TAKEOFF_RELEASE;
}


// bool AutogyroTakeoff::resetAltTakeoff()
// {
// 	return state_< AutogyroTakeoffState::TAKEOFF_RELEASE;
// }

void AutogyroTakeoff::reset()
{
	initialized_ = false;
	state_ = AutogyroTakeoffState::PRE_TAKEOFF_PREROTATE_START;
	takeoff_time_ = 0;
}


float AutogyroTakeoff::interpolateValuesOverAbsoluteTime(const float start_value, const float end_value,
		const hrt_abstime &start_time, const float interpolation_time) const
{
	const float seconds_since_start = hrt_elapsed_time(&start_time) * 1.e-6f;
	const float interpolator = math::constrain(seconds_since_start / interpolation_time, 0.0f, 1.0f);

	return interpolator * end_value + (1.0f - interpolator) * start_value;
}

void AutogyroTakeoff::play_next_tone()
{
	tune_control_s tune_control{};
	tune_control.tune_id = tune_control_s::TUNE_ID_TAKEOFF_NEXT_STEP;
	tune_control.volume = tune_control_s::VOLUME_LEVEL_MAX;
	tune_control.timestamp = hrt_absolute_time();
	_tune_control.publish(tune_control);
}

void AutogyroTakeoff::play_error_tone()
{
	tune_control_s tune_control{};
	tune_control.tune_id = tune_control_s::TUNE_ID_TAKEOFF_ERROR;
	tune_control.volume = tune_control_s::VOLUME_LEVEL_MAX;
	tune_control.timestamp = hrt_absolute_time();
	_tune_control.publish(tune_control);
}

void AutogyroTakeoff::play_release_tone()
{
	tune_control_s tune_control{};
	tune_control.tune_id = tune_control_s::TUNE_ID_TAKEOFF_RELEASE;
	tune_control.volume = tune_control_s::VOLUME_LEVEL_MAX;
	tune_control.timestamp = hrt_absolute_time();
	_tune_control.publish(tune_control);
}

} // end of namespace autogyrotakeoff
