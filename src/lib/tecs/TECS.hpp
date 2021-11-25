/****************************************************************************
 *
 *   Copyright (c) 2017-2020 PX4 Development Team. All rights reserved.
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
 * @file tecs.cpp
 *
 * @author Paul Riseborough
 */

#pragma once

#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>

#include <uORB/Publication.hpp>
#include <uORB/topics/tecs_status.h>
#include <uORB/uORB.h>
#include <motion_planning/VelocitySmoothing.hpp>
#include <motion_planning/ManualVelocitySmoothingZ.hpp>

#include <uORB/Publication.hpp>
#include <uORB/topics/tecs_status.h>
#include <uORB/uORB.h>

class TECS
{
public:
	TECS();
	~TECS() = default;

	// no copy, assignment, move, move assignment
	TECS(const TECS &) = delete;
	TECS &operator=(const TECS &) = delete;
	TECS(TECS &&) = delete;
	TECS &operator=(TECS &&) = delete;

	/**
	 * Get the current airspeed status
	 *
	 * @return true if airspeed is enabled for control
	 */
	bool airspeed_sensor_enabled() { return _airspeed_enabled; }

	/**
	 * Set the airspeed enable state
	 */
	void enable_airspeed(bool enabled) { _airspeed_enabled = enabled; }

	/**
	 * Updates the following vehicle kineamtic state estimates:
	 * Vertical position, velocity and acceleration.
	 * Speed derivative
	 * Must be called prior to udating tecs control loops
	 * Must be called at 50Hz or greater
	 */
	void update_vehicle_state_estimates(float equivalent_airspeed, const float speed_deriv_forward, bool altitude_lock,
					    bool in_air,
					    float altitude, float vz);

	/**
	 * Update the control loop calculations
	 */
	void update_pitch_throttle(float pitch, float baro_altitude, float hgt_setpoint,
				   float EAS_setpoint, float equivalent_airspeed, float eas_to_tas, bool climb_out_setpoint, float pitch_min_climbout,
				   float throttle_min, float throttle_setpoint_max, float throttle_cruise,
				   float pitch_limit_min, float pitch_limit_max, float hgt_rate_sp = NAN,
				   bool eco_mode_enabled = false);

	void reset_state() { _states_initialized = false; }

	enum ECL_TECS_MODE {
		ECL_TECS_MODE_NORMAL = 0,
		ECL_TECS_MODE_UNDERSPEED,
		ECL_TECS_MODE_BAD_DESCENT,
		ECL_TECS_MODE_CLIMBOUT,
		ECL_TECS_MODE_ECO
	};

	void set_detect_underspeed_enabled(bool enabled) { _detect_underspeed_enabled = enabled; }

	// setters for controller parameters
	void set_integrator_gain_throttle(float gain) { _integrator_gain_throttle = gain; }
	void set_integrator_gain_pitch(float gain) { _integrator_gain_pitch = gain; }

	void set_min_sink_rate(float rate) { _min_sink_rate = rate; }
	void set_max_sink_rate(float sink_rate) { _max_sink_rate = sink_rate; }
	void set_max_climb_rate(float climb_rate) { _max_climb_rate = climb_rate; }

	void set_heightrate_ff(float heightrate_ff) { _height_setpoint_gain_ff = heightrate_ff; }
	void set_height_error_time_constant(float time_const) { _height_error_gain = 1.0f / math::max(time_const, 0.1f); }

	void set_equivalent_airspeed_max(float airspeed) { _equivalent_airspeed_max = airspeed; }
	void set_equivalent_airspeed_min(float airspeed) { _equivalent_airspeed_min = airspeed; }
	void set_equivalent_airspeed_cruise(float airspeed) { _equivalent_airspeed_cruise = airspeed; }

	void set_pitch_damping(float damping) { _pitch_damping_gain = damping; }
	void set_vertical_accel_limit(float limit) { _vert_accel_limit = limit; }

	void set_speed_comp_filter_omega(float omega) { _tas_estimate_freq = omega; }
	void set_speed_weight(float weight) { _pitch_speed_weight = weight; }
	void set_airspeed_error_time_constant(float time_const) { _airspeed_error_gain = 1.0f / math::max(time_const, 0.1f); }

	void set_throttle_damp(float throttle_damp) { _throttle_damping_gain = throttle_damp; }
	void set_throttle_slewrate(float slewrate) { _throttle_slewrate = slewrate; }

	void set_roll_throttle_compensation(float compensation) { _load_factor_correction = compensation; }
	void set_load_factor(float load_factor) { _load_factor = load_factor; }

	void set_ste_rate_time_const(float time_const) { _STE_rate_time_const = time_const; }
	void set_speed_derivative_time_constant(float time_const) { _speed_derivative_time_const = time_const; }

	void set_seb_rate_ff_gain(float ff_gain) { _SEB_rate_ff = ff_gain; }

	void set_target_climb_rate(float target_climb_rate) { _target_climb_rate = target_climb_rate; }
	void set_target_sink_rate(float target_sink_rate) { _target_sink_rate = target_sink_rate; }

	// eco mode settings
	void set_speed_weight_eco(float weight_eco) { _pitch_speed_weight_eco = weight_eco; }
	void set_height_error_time_constant_eco(float time_const_eco) { _height_error_gain_eco = 1.0f / math::max(time_const_eco, 0.1f); }
	void set_target_climb_rate_eco(float target_climb_rate_eco) { _target_climb_rate_eco = target_climb_rate_eco; }

	// getters
	float get_throttle_setpoint() { return _last_throttle_setpoint; }
	float get_pitch_setpoint() { return _last_pitch_setpoint; }

	float get_hgt_setpoint() { return _hgt_setpoint; }
	float get_hgt_rate_setpoint() { return _hgt_rate_setpoint; }
	float get_STE() { return _SPE_estimate + _SKE_estimate; }
	float get_STE_setpoint() { return _SPE_setpoint + _SKE_setpoint; }
	float get_STE_rate() { return _SPE_rate + _SKE_rate; }
	float get_STE_rate_setpoint() { return _SPE_rate_setpoint + _SKE_rate_setpoint; }
	float get_SEB() { return _SPE_estimate * _SPE_weighting - _SKE_estimate * _SKE_weighting; }
	float get_SEB_setpoint() { return _SPE_setpoint * _SPE_weighting - _SKE_setpoint * _SKE_weighting; }
	float get_SEB_rate() { return _SPE_rate * _SPE_weighting - _SKE_rate * _SKE_weighting; }
	float get_SEB_rate_setpoint() { return _SPE_rate_setpoint * _SPE_weighting - _SKE_rate_setpoint * _SKE_weighting; }
	int get_flight_phase() { return _flight_phase; }

	void tecs_status_publish(const hrt_abstime &now);

	/**
	 * Handle the altitude reset
	 *
	 * If the estimation system resets the height in one discrete step this
	 * will gracefully even out the reset over time.
	 */
	void handle_alt_step(float delta_alt, float altitude)
	{
		_hgt_setpoint += delta_alt;

		// reset height states
		_vert_pos_state = altitude;
		_vert_vel_state = 0.0f;
	}

private:

	static constexpr float _jerk_max =
		1000.0f;	// maximum jerk for creating height rate trajectories, we want infinite jerk so set a high value

	enum ECL_TECS_MODE _tecs_mode {ECL_TECS_MODE_NORMAL};

	// timestamps
	uint64_t _state_update_timestamp{0};				///< last timestamp of the 50 Hz function call
	uint64_t _speed_update_timestamp{0};				///< last timestamp of the speed function call
	uint64_t _pitch_update_timestamp{0};				///< last timestamp of the pitch function call

	// controller parameters
	float _tas_estimate_freq{0.0f};					///< cross-over frequency of the true airspeed complementary filter (rad/sec)
	float _max_climb_rate{2.0f};					///< climb rate produced by max allowed throttle (m/sec)
	float _min_sink_rate{1.0f};					///< sink rate produced by min allowed throttle (m/sec)
	float _max_sink_rate{2.0f};					///< maximum safe sink rate (m/sec)
	float _pitch_damping_gain{0.0f};				///< damping gain of the pitch demand calculation (sec)
	float _throttle_damping_gain{0.0f};				///< damping gain of the throttle demand calculation (sec)
	float _integrator_gain_throttle{0.0f};				///< integrator gain used by the throttle demand calculation
	float _integrator_gain_pitch{0.0f};				///< integrator gain used by the pitch demand calculation
	float _vert_accel_limit{0.0f};					///< magnitude of the maximum vertical acceleration allowed (m/sec**2)
	float _load_factor{0.0f};					///< additional normal load factor
	float _load_factor_correction{0.0f};				///< gain from normal load factor increase to total energy rate demand (m**2/sec**3)
	float _pitch_speed_weight{1.0f};				///< speed control weighting used by pitch demand calculation
	float _height_error_gain{0.2f};					///< height error inverse time constant [1/s]
	float _height_setpoint_gain_ff{0.0f};				///< gain from height demand derivative to demanded climb rate
	float _airspeed_error_gain{0.1f};				///< airspeed error inverse time constant [1/s]
	float _equivalent_airspeed_min{3.0f};				///< equivalent airspeed demand lower limit (m/sec)
	float _equivalent_airspeed_max{30.0f};				///< equivalent airspeed demand upper limit (m/sec)
	float _equivalent_airspeed_cruise{15.0f};			///< equivalent cruise airspeed for airspeed less mode (m/sec)
	float _throttle_slewrate{0.0f};					///< throttle demand slew rate limit (1/sec)
	float _STE_rate_time_const{0.1f};				///< filter time constant for specific total energy rate (damping path) (s)
	float _speed_derivative_time_const{0.01f};			///< speed derivative filter time constant (s)
	float _SEB_rate_ff{1.0f};

	float _height_error_gain_eco{0.2f};				///< in eco mode: height error inverse time constant [1/s]
	float _pitch_speed_weight_eco{1.0f};				///< in eco mode: speed control weighting used by pitch demand calculation
	float _target_climb_rate_eco{2.f};

	// complimentary filter states
	float _vert_vel_state{0.0f};					///< complimentary filter state - height rate (m/sec)
	float _vert_pos_state{0.0f};					///< complimentary filter state - height (m)
	float _tas_rate_state{0.0f};					///< complimentary filter state - true airspeed first derivative (m/sec**2)
	float _tas_state{0.0f};						///< complimentary filter state - true airspeed (m/sec)
	float _tas_innov{0.0f};						///< complimentary filter true airspeed innovation (m/sec)

	// controller states
	float _throttle_integ_state{0.0f};				///< throttle integrator state
	float _pitch_integ_state{0.0f};					///< pitch integrator state (rad)
	float _last_throttle_setpoint{0.0f};				///< throttle demand rate limiter state (1/sec)
	float _last_pitch_setpoint{0.0f};				///< pitch demand rate limiter state (rad/sec)
	float _tas_rate_filtered{0.0f};					///< low pass filtered rate of change of speed along X axis (m/sec**2)

	// speed demand calculations
	float _EAS{0.0f};						///< equivalent airspeed (m/sec)
	float _TAS_max{30.0f};						///< true airpeed demand upper limit (m/sec)
	float _TAS_min{3.0f};						///< true airpeed demand lower limit (m/sec)
	float _TAS_setpoint{0.0f};					///< current airpeed demand (m/sec)
	float _TAS_setpoint_last{0.0f};					///< previous true airpeed demand (m/sec)
	float _EAS_setpoint{0.0f};					///< Equivalent airspeed demand (m/sec)
	float _TAS_setpoint_adj{0.0f};					///< true airspeed demand tracked by the TECS algorithm (m/sec)
	float _TAS_rate_setpoint{0.0f};					///< true airspeed rate demand tracked by the TECS algorithm (m/sec**2)
	float _tas_rate_raw{0.0f};					///< true airspeed rate, calculated as inertial acceleration in body X direction

	// height demand calculations
	float _hgt_setpoint{0.0f};					///< demanded height tracked by the TECS algorithm (m)
	float _hgt_rate_setpoint{0.0f};					///< demanded climb rate tracked by the TECS algorithm

	// vehicle physical limits
	float _pitch_setpoint_unc{0.0f};				///< pitch demand before limiting (rad)
	float _STE_rate_max{0.0f};					///< specific total energy rate upper limit achieved when throttle is at _throttle_setpoint_max (m**2/sec**3)
	float _STE_rate_min{0.0f};					///< specific total energy rate lower limit acheived when throttle is at _throttle_setpoint_min (m**2/sec**3)
	float _throttle_setpoint_max{0.0f};				///< normalised throttle upper limit
	float _throttle_setpoint_min{0.0f};				///< normalised throttle lower limit
	float _pitch_setpoint_max{0.5f};				///< pitch demand upper limit (rad)
	float _pitch_setpoint_min{-0.5f};				///< pitch demand lower limit (rad)

	// specific energy quantities
	float _SPE_setpoint{0.0f};					///< specific potential energy demand (m**2/sec**2)
	float _SKE_setpoint{0.0f};					///< specific kinetic energy demand (m**2/sec**2)
	float _SPE_rate_setpoint{0.0f};					///< specific potential energy rate demand (m**2/sec**3)
	float _SKE_rate_setpoint{0.0f};					///< specific kinetic energy rate demand (m**2/sec**3)
	float _SPE_estimate{0.0f};					///< specific potential energy estimate (m**2/sec**2)
	float _SKE_estimate{0.0f};					///< specific kinetic energy estimate (m**2/sec**2)
	float _SPE_rate{0.0f};						///< specific potential energy rate estimate (m**2/sec**3)
	float _SKE_rate{0.0f};						///< specific kinetic energy rate estimate (m**2/sec**3)

	// specific energy error quantities
	float _STE_error{0.0f};						///< specific total energy error (m**2/sec**2)
	float _STE_rate_error{0.0f};					///< specific total energy rate error (m**2/sec**3)
	float _SEB_error{0.0f};						///< specific energy balance error (m**2/sec**2)
	float _SEB_rate_error{0.0f};					///< specific energy balance rate error (m**2/sec**3)

	// speed height weighting
	float _SPE_weighting{1.0f};
	float _SKE_weighting{1.0f};

	float _target_climb_rate{3.f};
	float _target_sink_rate{2.f};

	// time steps (non-fixed)
	float _dt{DT_DEFAULT};						///< Time since last update of main TECS loop (sec)
	static constexpr float DT_DEFAULT = 0.02f;			///< default value for _dt (sec)

	// controller mode logic
	bool _underspeed_detected{false};				///< true when an underspeed condition has been detected
	bool _detect_underspeed_enabled{true};				///< true when underspeed detection is enabled
	bool _uncommanded_descent_recovery{false};			///< true when a continuous descent caused by an unachievable airspeed demand has been detected
	bool _climbout_mode_active{false};				///< true when in climbout mode
	bool _airspeed_enabled{false};					///< true when airspeed use has been enabled
	bool _states_initialized{false};					///< true when TECS states have been iniitalized
	bool _in_air{false};						///< true when the vehicle is flying
	bool _eco_mode_enabled{false};

	// flight phase
	int _flight_phase{tecs_status_s::TECS_FLIGHT_PHASE_LEVEL};

	/**
	 * Update the airspeed internal state using a second order complementary filter
	 */
	void _update_speed_states(float airspeed_setpoint, float equivalent_airspeed, float cas_to_tas);

	/**
	 * Update the desired airspeed
	 */
	void _update_speed_setpoint();

	/**
	 * Calculate desired height rate from altitude demand
	 */
	void runAltitudeControllerSmoothVelocity(float alt_sp_amsl_m, float target_climbrate_m_s, float target_sinkrate_m_s,
			float alt_amsl);

	/**
	 * Detect if the system is not capable of maintaining airspeed
	 */
	void _detect_underspeed();

	/**
	 * Update specific energy
	 */
	void _update_energy_estimates();

	/**
	 * Update throttle setpoint
	 */
	void _update_throttle_setpoint(float throttle_cruise);

	/**
	 * Detect an uncommanded descent
	 */
	void _detect_uncommanded_descent();

	/**
	 * Update the pitch setpoint
	 */
	void _update_pitch_setpoint();

	void _updateTrajectoryGenerationConstraints();

	void _updateFlightPhase(float altitude_sp_amsl, float height_rate_setpoint);

	void _calculateHeightRateSetpoint(float altitude_sp_amsl, float height_rate_sp, float target_climbrate,
					  float target_sinkrate, float altitude_amsl);

	/**
	 * Initialize the controller
	 */
	void _initialize_states(float pitch, float throttle_cruise, float baro_altitude, float pitch_min_climbout,
				float eas_to_tas);

	/**
	 * Calculate specific total energy rate limits
	 */
	void _update_STE_rate_lim();

	void _update_speed_height_weights();

	AlphaFilter<float> _STE_rate_error_filter;

	AlphaFilter<float> _TAS_rate_filter;

	VelocitySmoothing
	_alt_control_traj_generator;	// generates height rate and altitude setpoint trajectory when altitude is commanded
	ManualVelocitySmoothingZ
	_velocity_control_traj_generator;	// generates height rate and altitude setpoint trajectory when height rate is commanded

	uORB::Publication<tecs_status_s>	_tecs_status_pub{ORB_ID(tecs_status)};

};
