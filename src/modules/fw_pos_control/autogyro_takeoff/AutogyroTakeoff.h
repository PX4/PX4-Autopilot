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
 * @file AutogyroTakeoff.h
 * Autogyro automated takeoff, header files
 *
 * @author Roman Dvorak, ThunderFly s.r.o. <dvorakroman@thunderfly.cz>
 */

#ifndef AUTOGYROTAKEOFF_H
#define AUTOGYROTAKEOFF_H

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

#include "../runway_takeoff/RunwayTakeoff.h"

namespace autogyrotakeoff
{


enum AutogyroTakeoffState {
	TAKEOFF_ERROR = -1,
	PRE_TAKEOFF_PREROTATE_START = 0, /**< Wait for manual rotor prerotation or for some other trigger */
	PRE_TAKEOFF_PREROTATE = 1, /**< Start prerotation of rotor controlled from AP or prerotation with some movement */
	PRE_TAKEOFF_DONE = 2, /**< autogyro conditions are OK for takeoff*/
	PRE_TAKEOFF_RAMPUP = 3, /**< Get ready to takeoff, rampup motor */
	TAKEOFF_RELEASE = 4, /**< command for release */
	TAKEOFF_CLIMBOUT = 5, /**< Climbout for minimal altitude */
	FLY /**< fly to next waypoint */
};


enum AutogyroTakeoffType {
	WOPREROT_PLATFORM = 0, // platform without prerotator
	WOPREROT_RUNWAY = 1,   // Without prerotator on runway
	ELPREROT_PLATFORM = 2, // Moving platform wint powered prerotator
	ELPREROT_RUNWAY = 3,   // From runway with powered prerotator
	FG_SITL = 10           // FlightGear SITL
};

class __EXPORT AutogyroTakeoff : public ModuleParams
{
public:
	AutogyroTakeoff(ModuleParams *parent) : ModuleParams(parent) {}
	~AutogyroTakeoff() = default;


	/**
	 * @brief Initializes the state machine.
	 *
	 * @param time_now Absolute time since system boot [us]
	 * @param initial_yaw Vehicle yaw angle at time of initialization [us]
	 * @param start_pos_global Vehicle global (lat, lon) position at time of initialization [deg]
	 */
	void init(const hrt_abstime &time_now, const float initial_yaw, const matrix::Vector2d &start_pos_global);

// 	void init(const hrt_abstime &now, float yaw, const matrix::Vector2d &start_pos_global);

	/**
	 * @brief Updates the state machine based on the current vehicle condition.
	 *
	 * @param time_now Absolute time since system boot [us]
	 * @param takeoff_airspeed Calibrated airspeed setpoint for the takeoff climbout [m/s]
	 * @param calibrated_airspeed Vehicle calibrated airspeed [m/s]
	 * @param vehicle_altitude Vehicle altitude (AGL) [m]
	 * @param clearance_altitude Altitude (AGL) above which we have cleared all occlusions in the runway path [m]
	 * @param rotor_rpm Current rotor RPM [rpm]
	 * @param mavlink_log_pub Mavlink log uORB handle
	 * @param actual_pos_global Vehicle global (lat, lon) position [deg]
	 */
	void update(const hrt_abstime &time_now, const float takeoff_airspeed, const float calibrated_airspeed,
		    const float vehicle_altitude, const float clearance_altitude, const float rotor_rpm, orb_advert_t *mavlink_log_pub,
		    const matrix::Vector2d &actual_pos_global);

	bool doRelease(bool release);
	bool doPrerotate();

	/**
	 * @brief Set new takeoff state and save the time when the state was entered.
	 *
	 * @param state
	 * @param time_now
	 * @return ** void
	 */
	void setState(AutogyroTakeoffState state, const hrt_abstime &time_now);

// 	AutogyroTakeoffState getState() { return state_; }
// 	float getMinAirspeedScaling() { return _param_rwto_airspd_scl.get(); }
// 	bool isInitialized() { return _initialized; }

	/**
	 * @return Current takeoff state
	 */
	AutogyroTakeoffState getState() { return takeoff_state_; }

	/**
	 * @return The state machine is initialized
	 */
	bool isInitialized() { return initialized_; }

	/**
	 * @return Runway takeoff is enabled
	 */
	bool TakeoffEnabled() { return param_ag_tkoff_.get();}


	/**
	 * @return Initial vehicle yaw angle [rad]
	 */
	float getInitYaw() { return initial_yaw_; }

	/**
	 * @return The vehicle should control yaw via rudder or nose gear
	 */
	bool controlYaw();

	/**
	 * @param external_pitch_setpoint Externally commanded pitch angle setpoint (usually from TECS) [rad]
	 * @return Pitch angle setpoint (limited while plane is on runway) [rad]
	 */
	float getPitch(float external_pitch_setpoint);

	/**
	 * @param external_roll_setpoint Externally commanded roll angle setpoint (usually from path navigation) [rad]
	 * @return Roll angle setpoint [rad]
	 */
	float getRoll(float external_roll_setpoint);

	/**
	 * @brief Returns the appropriate yaw angle setpoint.
	 *
	 * In heading hold mode (_heading_mode == 0), it returns initial yaw as long as it's on the runway.
	 * When it has enough ground clearance we start navigation towards WP.
	 *
	 * @param external_yaw_setpoint Externally commanded yaw angle setpoint [rad]
	 * @return Yaw angle setpoint [rad]
	 */
	float getYaw(float external_yaw_setpoint);

	/**
	 * @brief Returns the throttle setpoint.
	 *
	 * Ramps up over RWTO_RAMP_TIME to RWTO_MAX_THR until the aircraft lifts off the runway and then
	 * ramps from RWTO_MAX_THR to the externally defined throttle setting over the takeoff rotation time
	 *
	 * @param idle_throttle normalized [0,1]
	 * @param external_throttle_setpoint Externally commanded throttle setpoint (usually from TECS), normalized [0,1]
	 * @return Throttle setpoint, normalized [0,1]
	 */
	float getThrottle(const float idle_throttle, const float external_throttle_setpoint) const;

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

	/**
	 * @return Runway takeoff starting position in global frame (lat, lon) [deg]
	 */
	const matrix::Vector2d &getStartPosition() const { return start_pos_global_; };

	// NOTE: this is only to be used for mistaken mode transitions to takeoff while already in air
	void forceSetFlyState() { takeoff_state_ = AutogyroTakeoffState::FLY; }

	/**
	 * @return If the attitude / rate control integrators should be continually reset.
	 * This is the case during ground roll.
	 */
	bool resetIntegrators();

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


// 	float getRequestedAirspeed();
// 	float getInitYaw() { return _init_yaw; }

// 	bool controlYaw();
	bool climbout() { return climbout_; }
// 	float getPitch(float tecsPitch);
// 	float getRoll(float navigatorRoll);
// 	float getYaw(float navigatorYaw);
// 	float getThrottle(const hrt_abstime &now, float tecsThrottle);
// 	bool resetIntegrators();
// 	bool resetAltTakeoff();
// 	float getMinPitch(float climbout_min, float min);
// 	float getMaxPitch(float max);
// 	// bool setState(int new_state);
// 	const matrix::Vector2d &getStartWP() const { return _takeoff_wp; };

// 	void reset();

	uORB::Subscription _actuator_armed_sub{ORB_ID(actuator_armed)};

	void play_next_tone();
	void play_release_tone();
	void play_error_tone();

private:
	/** state variables **/
	AutogyroTakeoffState state_{PRE_TAKEOFF_PREROTATE_START};
	AutogyroTakeoffState state_last_{PRE_TAKEOFF_PREROTATE_START};

	/**
	 * Current state of runway takeoff procedure
	 */
	AutogyroTakeoffState takeoff_state_{PRE_TAKEOFF_PREROTATE_START};

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

	/**
	 * Initial yaw of the vehicle on first pass through the runway takeoff state machine.
	 * used for heading hold mode. [rad]
	 */
	float initial_yaw_{0.f};

	/**
	 * The global (lat, lon) position of the vehicle on first pass through the runway takeoff state machine. The
	 * takeoff path emanates from this point to correct for any GNSS uncertainty from the planned takeoff point. The
	 * vehicle should accordingly be set on the center of the runway before engaging the mission. [deg]
	 */
	matrix::Vector2d start_pos_global_{};
	hrt_abstime time_in_state_{0};
	hrt_abstime time_state_start_{0};
	hrt_abstime last_sent_release_status_{0};
	float init_yaw_{0.f};
	bool climbout_{false};

	matrix::Vector2d initial_wp_;
	matrix::Vector2d takeoff_wp_;

	uORB::Publication<tune_control_s> _tune_control{ORB_ID(tune_control)};
	uORB::Publication<takeoff_status_s> _takeoff_status_pub{ORB_ID(takeoff_status)};

// 	// TODO: templorary sollution. Should be replaced with autogyro takeoff status with
// 	// translation into custom mavlink message.  Used to inform launch platform to
// 	// release drone from lock
	uORB::Publication<debug_value_s> _takeoff_informations_pub{ORB_ID(debug_value)};

	DEFINE_PARAMETERS(

		(ParamBool<px4::params::AG_TKOFF>) param_ag_tkoff_,
		(ParamBool<px4::params::RWTO_TKOFF>) param_rwto_tkoff_,
		(ParamInt<px4::params::RWTO_HDG>) param_rwto_hdg_,
		(ParamFloat<px4::params::RWTO_MAX_THR>) param_rwto_max_thr_,
		(ParamFloat<px4::params::RWTO_PSP>) param_rwto_psp_,
		(ParamFloat<px4::params::RWTO_MAX_PITCH>) param_rwto_max_pitch_,
		(ParamFloat<px4::params::RWTO_AIRSPD_SCL>) param_rwto_airspd_scl_,
		(ParamFloat<px4::params::RWTO_RAMP_TIME>) param_rwto_ramp_time_,

		(ParamFloat<px4::params::FW_CLMBOUT_DIFF>) param_fw_clmbout_diff_,
		(ParamFloat<px4::params::FW_AIRSPD_MAX>) param_fw_airspd_max_,
		(ParamFloat<px4::params::FW_AIRSPD_MIN>) param_fw_airspd_min_,
		(ParamFloat<px4::params::FW_AIRSPD_TRIM>) param_fw_airspd_trim_,
		(ParamFloat<px4::params::FW_THR_IDLE>) param_fw_thr_idle_,

		(ParamFloat<px4::params::AG_PROT_MIN_RPM>) param_ag_prerotator_minimal_rpm_,
		(ParamFloat<px4::params::AG_PROT_TRG_RPM>) param_ag_prerotator_target_rpm_,
		(ParamFloat<px4::params::AG_ROTOR_RPM>) param_ag_rotor_flight_rpm_,
		(ParamInt<px4::params::AG_PROT_TYPE>) param_ag_prerotator_type_,

		(ParamFloat<px4::params::AG_NAV_ALT>) param_ag_nav_alt_,
		(ParamFloat<px4::params::AG_TKO_MAX_ROLL>)  param_ag_tko_max_roll_


	)



};

}

#endif // AUTOGYROTAKEOFF_H
