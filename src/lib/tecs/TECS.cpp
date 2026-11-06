/****************************************************************************
 *
 *   Copyright (c) 2017-2023 PX4 Development Team. All rights reserved.
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
 * @file TECS.cpp
 *
 * @author Paul Riseborough
 */

#include "TECS.hpp"

#include <lib/geo/geo.h>

#include <px4_platform_common/defines.h>

using math::constrain;
using math::max;
using math::min;

static constexpr bool isValidTimeStep(float dt) { return (PX4_ISFINITE(dt) && dt > FLT_EPSILON);}
static constexpr bool fastDescendFullyEngaged(float fast_descend) { return 1.f - fast_descend < FLT_EPSILON; }

// AlphaFilter keeps a single non-finite sample in its state for good, so the controller would not recover from a
// single bad frame (e.g. a non-finite setpoint when exiting offboard velocity mode). Only take finite samples and hold
// the state otherwise.
static void updateIfFinite(AlphaFilter<float> &filter, float dt, float time_const, float sample)
{
	if (PX4_ISFINITE(sample)) {
		filter.setParameters(dt, time_const);
		filter.update(sample);
	}
}

static void resetIfFinite(AlphaFilter<float> &filter, float sample, float fallback)
{
	filter.reset(PX4_ISFINITE(sample) ? sample : fallback);
}

void TECSAirspeedFilter::initialize(const float equivalent_airspeed, const float equivalent_airspeed_trim,
				    const bool airspeed_sensor_available)
{
	if (airspeed_sensor_available && PX4_ISFINITE(equivalent_airspeed)) {
		_airspeed_state.speed = equivalent_airspeed;

	} else {
		_airspeed_state.speed = equivalent_airspeed_trim;
	}

	_airspeed_state.speed_rate = 0.f;
}

void TECSAirspeedFilter::update(const float dt, const Input &input, const Param &param,
				const bool airspeed_sensor_available)
{
	// Input checking
	if (!isValidTimeStep(dt)) {
		// Do not update the states.
		return;
	}

	float airspeed;

	if (PX4_ISFINITE(input.equivalent_airspeed) && airspeed_sensor_available) {
		airspeed = input.equivalent_airspeed;

	} else {
		airspeed = param.equivalent_airspeed_trim;
	}

	float airspeed_derivative;

	if (PX4_ISFINITE(input.equivalent_airspeed_rate) && airspeed_sensor_available) {
		airspeed_derivative = input.equivalent_airspeed_rate;

	} else {
		airspeed_derivative = 0.0f;
	}

	/* Filter airspeed and rate using a constant airspeed rate model in a steady state Kalman Filter.
	 We use the gains of the continuous Kalman filter Kc and approximate the discrete version Kalman gain Kd =dt*Kc,
	 since the continuous algebraic Riccatti equation is easier to solve.
	*/

	matrix::Vector2f new_state_predicted;

	new_state_predicted(0) = _airspeed_state.speed + dt * _airspeed_state.speed_rate;
	new_state_predicted(1) = _airspeed_state.speed_rate;

	const matrix::Vector2f innovation{(airspeed - new_state_predicted(0)), (airspeed_derivative - new_state_predicted(1))};

	matrix::Vector2f new_state;
	new_state(0) = new_state_predicted(0) + dt * (kKg00 * innovation(0) + kKg01 * innovation(1));
	new_state(1) = new_state_predicted(1) + dt * (kKg10 * innovation(0) + kKg11 * innovation(1));

	// Clip airspeed at zero
	if (new_state(0) < FLT_EPSILON) {
		new_state(0) = 0.0f;
		// calculate input that would result in zero speed.
		const float desired_airspeed_innovation = (-new_state_predicted(0) / dt - kKg01 * innovation(1)) / kKg00;
		new_state(1) = new_state_predicted(1) + dt * (kKg10 * desired_airspeed_innovation + kKg11 * innovation(1));
	}

	// Update states
	_airspeed_state.speed = new_state(0);
	_airspeed_state.speed_rate = new_state(1);
}

TECSAirspeedFilter::AirspeedFilterState TECSAirspeedFilter::getState() const
{
	return _airspeed_state;
}

void TECSAltitudeReferenceModel::update(const float dt, const AltitudeReferenceState &setpoint, float altitude,
					float height_rate, const Param &param)
{
	// Input checks
	if (!isValidTimeStep(dt)) {
		// Do not update the states.
		return;
	}

	const float current_alt = PX4_ISFINITE(altitude) ? altitude : 0.f;

	_velocity_control_traj_generator.setMaxJerk(JERK_MAX);
	_velocity_control_traj_generator.setMaxAccelUp(param.vert_accel_limit);
	_velocity_control_traj_generator.setMaxAccelDown(param.vert_accel_limit);
	_velocity_control_traj_generator.setMaxVelUp(param.max_sink_rate); // different convention for FW than for MC
	_velocity_control_traj_generator.setMaxVelDown(param.max_climb_rate); // different convention for FW than for MC

	// Altitude setpoint reference
	_alt_control_traj_generator.setMaxJerk(JERK_MAX);
	_alt_control_traj_generator.setMaxAccel(param.vert_accel_limit);
	_alt_control_traj_generator.setMaxVel(fmaxf(param.max_climb_rate, param.max_sink_rate));

	// XXX: this is a bit risky.. .alt_rate here could be NAN (by interface design) - and is only ok to input to the
	// setVelSpFeedback() method because it calls the reset in the logic below when it is NAN.
	// TODO: stop it with the NAN interfaces, make sure to take care of this when refactoring and separating altitude
	// and height rate control loops.
	_velocity_control_traj_generator.setVelSpFeedback(setpoint.alt_rate);

	bool control_altitude = true;
	float altitude_setpoint = setpoint.alt;

	if (!PX4_ISFINITE(setpoint.alt) && !PX4_ISFINITE(setpoint.alt_rate)) {
		// neither altitude nor altitude rate is set - reset to current altitude
		_velocity_control_traj_generator.reset(0.f, 0, current_alt);
		altitude_setpoint = current_alt;

	} else if (PX4_ISFINITE(setpoint.alt_rate)) {
		// input is height rate (not altitude)
		_velocity_control_traj_generator.setCurrentPositionEstimate(current_alt);
		_velocity_control_traj_generator.update(dt, setpoint.alt_rate);
		altitude_setpoint = _velocity_control_traj_generator.getCurrentPosition();
		control_altitude = PX4_ISFINITE(altitude_setpoint); // returns true if altitude is locked

	} else {
		// input is altitude
		_velocity_control_traj_generator.reset(0, height_rate, altitude_setpoint);
	}

	if (control_altitude) {
		const float target_climbrate_m_s = math::min(param.target_climbrate, param.max_climb_rate);
		const float target_sinkrate_m_s = math::min(param.target_sinkrate, param.max_sink_rate);

		const float delta_trajectory_to_target_m = altitude_setpoint - _alt_control_traj_generator.getCurrentPosition();

		float height_rate_target = math::signNoZero<float>(delta_trajectory_to_target_m) *
					   math::trajectory::computeMaxSpeedFromDistance(
						   JERK_MAX, param.vert_accel_limit, fabsf(delta_trajectory_to_target_m), 0.f);

		height_rate_target = math::constrain(height_rate_target, -target_sinkrate_m_s, target_climbrate_m_s);

		_alt_control_traj_generator.updateDurations(height_rate_target);
		_alt_control_traj_generator.updateTraj(dt);
		_height_rate_setpoint_direct = NAN;

	} else {
		_alt_control_traj_generator.setCurrentVelocity(_velocity_control_traj_generator.getCurrentVelocity());
		_alt_control_traj_generator.setCurrentPosition(current_alt);
		_height_rate_setpoint_direct = _velocity_control_traj_generator.getCurrentVelocity();
	}
}

TECSAltitudeReferenceModel::AltitudeReferenceState TECSAltitudeReferenceModel::getAltitudeReference() const
{
	TECSAltitudeReferenceModel::AltitudeReferenceState ref{
		.alt = _alt_control_traj_generator.getCurrentPosition(),
		.alt_rate = _alt_control_traj_generator.getCurrentVelocity(),
	};

	return ref;
}

void TECSAltitudeReferenceModel::initialize(const AltitudeReferenceState &state)
{
	const float init_state_alt = PX4_ISFINITE(state.alt) ? state.alt : 0.f;
	const float init_state_alt_rate = PX4_ISFINITE(state.alt_rate) ? state.alt_rate : 0.f;

	_alt_control_traj_generator.reset(0.0f, init_state_alt_rate, init_state_alt);
	_velocity_control_traj_generator.reset(0.f, init_state_alt_rate, init_state_alt);
}

void TECSControl::initialize(const Setpoint &setpoint, const Input &input, const Param &param, const Flag &flag)
{
	resetIntegrals();

	// Initialization always goes through the altitude loop, even if a direct height rate setpoint is given
	Setpoint altitude_loop_setpoint{setpoint};
	altitude_loop_setpoint.altitude_rate_setpoint_direct = NAN;

	const STERateLimit limit{_calculateTotalEnergyRateLimit(param)};
	const SpecificEnergyRates specific_energy_rate{_updateOuterLoops(altitude_loop_setpoint, input, limit, param, flag)};

	ControlValues ste_rate{_calcThrottleControlSteRate(limit, specific_energy_rate, param)};
	resetIfFinite(_ste_rate_error_filter, _getControlError(ste_rate), 0.f);

	_throttle_setpoint = _calcThrottleControlOutput(limit, ste_rate, param, flag);

	if (!PX4_ISFINITE(_throttle_setpoint)) {
		_throttle_setpoint = constrain(param.throttle_trim, param.throttle_min, param.throttle_max);
	}

	// The achieved total energy rate starts from the measured one
	resetIfFinite(_ste_rate_estimate_filter, ste_rate.estimate, 0.f);
	resetIfFinite(_ste_rate_model_error_filter, ste_rate.estimate - _steRateModel(_throttle_setpoint, limit, param), 0.f);
	_throttle_shortfall_filter.reset(0.f);

	const PitchControlValues pitch_values{_calcPitchControlSebRateValues(altitude_loop_setpoint, input, limit,
					      specific_energy_rate, param, flag)};
	const ControlValues &seb_rate{pitch_values.seb_rate};

	_pitch_setpoint = _calcPitchControlOutput(input, seb_rate, pitch_values.ste_rate, param);

	// Debug output
	_debug_output.total_energy_rate_estimate = ste_rate.estimate;
	_debug_output.total_energy_rate_sp = ste_rate.setpoint;
	_debug_output.throttle_integrator = _throttle_integ_state;
	_debug_output.energy_balance_rate_estimate = seb_rate.estimate;
	_debug_output.energy_balance_rate_sp = seb_rate.setpoint;
	_debug_output.pitch_integrator = _pitch_integ_state;
}

void TECSControl::update(const float dt, const Setpoint &setpoint, const Input &input, const Param &param,
			 const Flag &flag)
{
	// Input checking
	if (!isValidTimeStep(dt)) {
		// Do not update the states and output.
		return;
	}

	const STERateLimit limit{_calculateTotalEnergyRateLimit(param)};
	const SpecificEnergyRates specific_energy_rate{_updateOuterLoops(setpoint, input, limit, param, flag)};

	_calcPitchControl(dt, setpoint, input, limit, specific_energy_rate, param, flag);

	_calcThrottleControl(dt, limit, specific_energy_rate, param, flag);
}

TECSControl::SpecificEnergyRates TECSControl::_updateOuterLoops(const Setpoint &setpoint, const Input &input,
		const STERateLimit &limit, const Param &param, const Flag &flag)
{
	// First, as the airspeed demand limits depend on it
	_detectUnderspeed(input, param, flag);

	const SpecificEnergyRates specific_energy_rate{_calcEnergyModeRates(setpoint, input, limit, param, flag)};

	// The altitude and airspeed rates the energy rate setpoints demand
	_debug_output.altitude_rate_control = specific_energy_rate.spe_rate.setpoint / CONSTANTS_ONE_G;
	_debug_output.true_airspeed_derivative_control = specific_energy_rate.ske_rate.setpoint / math::max(input.tas,
			FLT_EPSILON);

	return specific_energy_rate;
}

TECSControl::STERateLimit TECSControl::_calculateTotalEnergyRateLimit(const Param &param) const
{
	TECSControl::STERateLimit limit;
	// Calculate the specific total energy rate limits from the max throttle limits
	limit.STE_rate_max = math::max(param.max_climb_rate, FLT_EPSILON) * CONSTANTS_ONE_G;
	limit.STE_rate_min = - math::max(param.min_sink_rate, FLT_EPSILON) * CONSTANTS_ONE_G;

	return limit;
}

float TECSControl::_calcAltitudeControlOutput(const Setpoint &setpoint, const Input &input, const Param &param) const
{
	float altitude_rate_output;
	altitude_rate_output = (setpoint.altitude_reference.alt - input.altitude) * param.altitude_error_gain
			       + param.altitude_setpoint_gain_ff * setpoint.altitude_reference.alt_rate;

	altitude_rate_output = math::constrain(altitude_rate_output, -param.max_sink_rate, param.max_climb_rate);

	return altitude_rate_output;
}

TECSControl::SpecificEnergyRates TECSControl::_calcEnergyModeRates(const Setpoint &setpoint, const Input &input,
		const STERateLimit &limit, const Param &param, const Flag &flag) const
{
	// Define the ideal, first order energy dynamics in the eigenbasis of the
	// closed loop, with the two eigenvectors (the columns of P) being:
	//  - SPE (1, 0): a pure potential energy error (don't want airspeed errors when correcting an altitude error)
	//  - SEB (1, -1): a pure exchange between kinetic and potential energy (don't want to change the total energy
	//    when it is already correct)
	//
	// z is the energy error (e_SPE, e_SKE) expressed in these two directions:
	//
	//   [ e_SPE ] = P z       P = inv(P) = [ 1   1 ]
	//   [ e_SKE ]                          [ 0  -1 ]
	//
	//   z = inv(P) [ e_SPE ] = [ e_SPE + e_SKE ] = [  e_STE ]
	//              [ e_SKE ]   [    -e_SKE     ]   [ -e_SKE ]
	//
	// z_1 is the total energy error, z_2 the kinetic energy surplus to exchange into potential energy.
	//
	// In this basis the control law is diagonal, each component is demanded at its error over its own time constant
	//   z' = diag(1/tau_h, 1/tau_v) z
	// so the total energy error decays with tau_h and the kinetic energy error with tau_v.
	//
	// In the original basis (SPE, SKE) the equivalent controller is
	//   [ SPE' ] = P diag(1/tau_h, 1/tau_v) inv(P) [ e_SPE ] = [ 1/tau_h   1/tau_h - 1/tau_v ] [ e_SPE ]
	//   [ SKE' ]                                   [ e_SKE ]   [    0          1/tau_v        ] [ e_SKE ]
	//
	// Below, the coupling term (1/tau_h - 1/tau_v) e_SKE is written as (tau_v/tau_h - 1) e_SKE/tau_v.
	//
	// The two error terms below carry what the outer loops add on top: the altitude loop's reference rate
	// feedforward, the direct altitude rate setpoint, and the clamps of both loops.

	// e_SPE / tau_h: altitude error over tau_h plus the reference rate feedforward, clamped to the climb and sink
	// rate limits; or the direct altitude rate setpoint, which has no altitude error
	const float altitude_rate_setpoint = PX4_ISFINITE(setpoint.altitude_rate_setpoint_direct)
					     ? setpoint.altitude_rate_setpoint_direct
					     : _calcAltitudeControlOutput(setpoint, input, param);
	const float e_spe_over_tau_h = CONSTANTS_ONE_G * altitude_rate_setpoint;

	// e_SKE / tau_v. tau_v is clamped to at most tau_h: a slower airspeed than total energy loop would mean pitching
	// up when slow to hold back the throttle's correction.
	const float airspeed_error_gain = math::max(param.airspeed_error_gain, param.altitude_error_gain);
	const float tau_v_over_tau_h = param.altitude_error_gain / airspeed_error_gain;

	float e_ske_over_tau_v = 0.f;

	if (flag.airspeed_enabled) {
		const float tas = math::max(input.tas, FLT_EPSILON);
		// V (V_sp - V) is the kinetic energy error times 2V / (V_sp + V), exact at the setpoint
		const float e_ske = kAirspeedLinearKineticError
				    ? tas * (setpoint.tas_setpoint - tas)
				    : 0.5f * (setpoint.tas_setpoint * setpoint.tas_setpoint - tas * tas);
		const float limit_share = _airspeedShareOfEnergyRateLimits(param);
		e_ske_over_tau_v = constrain(e_ske * airspeed_error_gain, limit_share * limit.STE_rate_min,
					     limit_share * limit.STE_rate_max);
	}

	// The controller, row by row. Summed this is the total energy error over tau_h, the throttle's setpoint.
	SpecificEnergyRates specific_energy_rates;
	specific_energy_rates.spe_rate.setpoint = e_spe_over_tau_h + (tau_v_over_tau_h - 1.f) * e_ske_over_tau_v;
	specific_energy_rates.ske_rate.setpoint = e_ske_over_tau_v;

	specific_energy_rates.spe_rate.estimate = input.altitude_rate * CONSTANTS_ONE_G;
	specific_energy_rates.ske_rate.estimate = input.tas * input.tas_rate;

	return specific_energy_rates;
}

float TECSControl::_airspeedShareOfEnergyRateLimits(const Param &param) const
{
	// Normally half of the total energy rate limits, leaving the other half to altitude. Fast descend and underspeed
	// recovery give airspeed priority: the airspeed may then use all of it.
	return 0.5f + 0.5f * constrain(max(param.fast_descend, _ratio_undersped), 0.f, 1.f);
}

void TECSControl::_detectUnderspeed(const Input &input, const Param &param, const Flag &flag)
{
	if (!flag.detect_underspeed_enabled || !flag.airspeed_enabled) {
		_ratio_undersped = 0.0f;
		return;
	}

	// Underspeed starts at 1/3 stall + 2/3 min
	const float tas_underspeed_starting = (param.tas_stall + 2.f * param.tas_min) / 3.f;

	// Underspeed complete halfway between stall and start, but limit ramp width to 2 m/s
	const float tas_underspeed_complete = fmaxf(0.5f * (param.tas_stall + tas_underspeed_starting),
					      tas_underspeed_starting - 2.0f);

	_ratio_undersped = math::interpolate(input.tas, tas_underspeed_complete, tas_underspeed_starting, 1.0f, 0.0f);
}

void TECSControl::_calcPitchControl(float dt, const Setpoint &setpoint, const Input &input, const STERateLimit &limit,
				    const SpecificEnergyRates &specific_energy_rates, const Param &param, const Flag &flag)
{
	// The achieved total energy rate the pitch uses when the throttle does not deliver its setpoint: the slowly filtered
	// measured total energy rate, lowered at once by the throttle model when the throttle is reduced (see
	// kSteRateAchievedTimeConst). Uses the last throttle output, the pitch runs before the throttle.
	const float ste_rate_estimate = specific_energy_rates.spe_rate.estimate + specific_energy_rates.ske_rate.estimate;
	updateIfFinite(_ste_rate_estimate_filter, dt, kSteRateAchievedTimeConst, ste_rate_estimate);
	updateIfFinite(_ste_rate_model_error_filter, dt, kSteRateAchievedTimeConst,
		       ste_rate_estimate - _steRateModel(_throttle_setpoint, limit, param));

	const PitchControlValues pitch_values{_calcPitchControlSebRateValues(setpoint, input, limit, specific_energy_rates,
					      param, flag)};
	const ControlValues &seb_rate{pitch_values.seb_rate};

	_calcPitchControlUpdate(dt, input, seb_rate, param);
	const float pitch_setpoint{_calcPitchControlOutput(input, seb_rate, pitch_values.ste_rate, param)};

	// Comply with the specified vertical acceleration limit by applying a pitch rate limit
	// NOTE: at zero airspeed, the pitch increment is unbounded
	const float pitch_increment = dt * param.vert_accel_limit / math::max(input.tas, FLT_EPSILON);
	_pitch_setpoint = constrain(pitch_setpoint, _pitch_setpoint - pitch_increment,
				    _pitch_setpoint + pitch_increment);
	_pitch_setpoint = constrain(_pitch_setpoint, param.pitch_min, param.pitch_max);

	//Debug Output
	_debug_output.energy_balance_rate_estimate = seb_rate.estimate;
	_debug_output.energy_balance_rate_sp = seb_rate.setpoint;
	_debug_output.pitch_integrator = _pitch_integ_state;
}

TECSControl::PitchControlValues TECSControl::_calcPitchControlSebRateValues(const Setpoint &setpoint,
		const Input &input, const STERateLimit &limit, const SpecificEnergyRates &specific_energy_rates, const Param &param,
		const Flag &flag)
{
	const float ste_rate_setpoint = specific_energy_rates.spe_rate.setpoint + specific_energy_rates.ske_rate.setpoint;

	PitchControlValues pitch_values;

	const KineticPriority kinetic_priority{_calcKineticPriority(param, flag)};

	// Trading altitude for airspeed with the pitch is only wanted up to trim airspeed, above it accelerating is
	// left to the throttle. The pitch's kinetic error therefore uses the airspeed setpoint capped at
	// max(trim, current airspeed). Between trim and the setpoint the pitch holds altitude; if the throttle saturates
	// there, the vehicle settles at a lower airspeed instead of below the altitude setpoint.
	// When the throttle is overridden or has no authority (fast descend, underspeed, no throttle), the pitch is the
	// only way to reach the airspeed setpoint, and the cap is faded out.
	Setpoint pitch_setpoint{setpoint};
	const float tas_setpoint_capped = min(setpoint.tas_setpoint, max(param.tas_trim, input.tas));
	pitch_setpoint.tas_setpoint = lerp(tas_setpoint_capped, setpoint.tas_setpoint, kinetic_priority.forced);
	const SpecificEnergyRates pitch_rates{_calcEnergyModeRates(pitch_setpoint, input, limit, param, flag)};

	// Kinetic priority: If throttle saturation were not to happen, we could just formulate the energy balance as
	// potential energy - kinetic energy. However, when throttle saturates or in general is not able to provide the
	// desired energy rate setpoint, we want to avoid airspeed tracking to deteriorate as a result of that. Therefore, we
	// formulate the energy balance in terms of potential energy and total energy so that we can plug in achieved values,
	// rather than unachieved setpoint values

	// the total energy rate achieved is the minimum of what we measure and what we predict from the current throttle
	const float ste_rate_achieved = min(_ste_rate_estimate_filter.getState(),
					    _steRateModel(_throttle_setpoint, limit, param) + _ste_rate_model_error_filter.getState());

	const float ste_rate_pitch = lerp(ste_rate_setpoint, ste_rate_achieved, kinetic_priority.total);

	// the potential energy rate setpoint follows from the total energy rate achieved and the kinetic energy rate setpoint
	const float spe_rate_pitch = lerp(pitch_rates.spe_rate.setpoint, ste_rate_pitch - pitch_rates.ske_rate.setpoint,
					  kinetic_priority.total);

	// SEB = SPE - SKE = 2 SPE - STE
	pitch_values.seb_rate.setpoint = 2 * spe_rate_pitch - ste_rate_pitch;
	pitch_values.ste_rate = ste_rate_pitch;

	// Without airspeed the pitch loop knows nothing about the kinetic energy and controls altitude only
	pitch_values.seb_rate.estimate = specific_energy_rates.spe_rate.estimate
					 - (flag.airspeed_enabled ? specific_energy_rates.ske_rate.estimate : 0.f);

	return pitch_values;
}

float TECSControl::_steRateModel(float throttle, const STERateLimit &limit, const Param &param)
{
	if (throttle <= param.throttle_min + FLT_EPSILON) {
		return limit.STE_rate_min;
	}

	if (throttle >= param.throttle_max - FLT_EPSILON) {
		return limit.STE_rate_max;
	}

	const float throttle_trim = constrain(param.throttle_trim, param.throttle_min, param.throttle_max);

	if (throttle >= throttle_trim) {
		return (throttle - throttle_trim) / (param.throttle_max - throttle_trim) * limit.STE_rate_max;
	}

	return (throttle_trim - throttle) / (throttle_trim - param.throttle_min) * limit.STE_rate_min;
}

TECSControl::KineticPriority TECSControl::_calcKineticPriority(const Param &param, const Flag &flag) const
{
	KineticPriority kinetic_priority{0.f, 0.f};

	// Without airspeed there is no kinetic demand to give priority to
	if (!flag.airspeed_enabled) {
		return kinetic_priority;
	}

	// The throttle is overridden (fast descend, underspeed) or cannot move at all (gliders)
	const bool no_throttle_range = param.throttle_max - param.throttle_min < FLT_EPSILON;
	kinetic_priority.forced = no_throttle_range ? 1.f : constrain(max(param.fast_descend, _ratio_undersped), 0.f, 1.f);

	// Or it does not deliver its energy rate setpoint, e.g. climbing at full throttle
	kinetic_priority.total = max(kinetic_priority.forced, constrain(_throttle_shortfall_filter.getState(), 0.f, 1.f));

	return kinetic_priority;
}

void TECSControl::_calcPitchControlUpdate(float dt, const Input &input, const ControlValues &seb_rate,
		const Param &param)
{
	if (param.integrator_gain_pitch > FLT_EPSILON) {

		// Calculate pitch integrator input term: the balance rate error is a balance rate increment at the current throttle
		float pitch_integ_input = _sebRateToClimbAngle(_getControlError(seb_rate) * param.integrator_gain_pitch, 0.f, input,
					  param);

		// Guard against NaN integrator input - discard bad input sample (e.g., exiting offboard velocity mode)
		if (!PX4_ISFINITE(pitch_integ_input)) {
			pitch_integ_input = 0.f;
		}

		// Prevent the integrator changing in a direction that will increase pitch demand saturation
		if (_pitch_setpoint >= param.pitch_max) {
			pitch_integ_input = min(pitch_integ_input, 0.f);

		} else if (_pitch_setpoint <= param.pitch_min) {
			pitch_integ_input = max(pitch_integ_input, 0.f);
		}

		// Update the pitch integrator state
		_pitch_integ_state = _pitch_integ_state + pitch_integ_input * dt;

		// Safety check: reset integrator if state became corrupted
		if (!PX4_ISFINITE(_pitch_integ_state)) {
			_pitch_integ_state = 0.f;
		}

	} else {
		_pitch_integ_state = 0.0f;
	}
}

float TECSControl::_sebRateToClimbAngle(float seb_rate, float ste_rate, const Input &input, const Param &param)
{
	// The balance rate depends not only on the climb angle but also on total energy rate.
	// Working from definitions, with climb angle γ (sin linearised as usual here):
	//     STE' = total energy rate from the throttle (ideally the STE rate setpoint)
	//          = SKE' + SPE' = v v' + g V γ
	//     SPE' = g V γ
	//     SKE' = STE' - SPE'
	//     SEB' = SPE' - SKE' = 2 SPE' - STE' = 2 g V γ - STE'
	//                                        = 2 g V γ - (v v' + g V γ)
	//                                        = 2 g V γ -  v v' - g V γ)
	//                                        = g V γ   -  v v'
	//                                        = g V γ   -  SKE'
	// maybe neater to take second form to avoid factor of 2?

	// Solving for γ:
	//     γ = (SEB' + STE') / (2 g V)
	// But today
	//     γ = SEB' / (g V)

	// max() returns its second argument for a NaN first one, so a NaN airspeed falls back to the minimum airspeed
	const float tas = max(input.tas, param.tas_min, FLT_EPSILON);

	return (seb_rate + ste_rate) / (2.f * tas * CONSTANTS_ONE_G);
}

float TECSControl::_calcPitchControlOutput(const Input &input, const ControlValues &seb_rate, float ste_rate_setpoint,
		const Param &param) const
{
	// Calculate a specific energy correction that doesn't include the integrator contribution
	float SEB_rate_correction = _getControlError(seb_rate) * param.pitch_damping_gain +
				    param.seb_rate_ff *
				    seb_rate.setpoint;

	// Convert the balance rate correction to a climb angle and add the level-flight pitch, the angle of attack at
	// zero climb angle, to get the absolute pitch setpoint. This calculation assumes:
	// a) The climb angle follows pitch angle with a lag that is small enough not to destabilise the control loop.
	// b) The offset between climb angle and pitch angle (angle of attack) is constant and equal to the level-flight
	// pitch, excluding the effect of pitch transients due to control action or turbulence.
	float pitch_setpoint_unc = _sebRateToClimbAngle(SEB_rate_correction, ste_rate_setpoint, input, param)
				   + _pitch_integ_state
				   + param.level_flight_pitch;

	// Guard against a non-finite feedforward/damping contribution (e.g. seb_rate going NaN on the energy/speed
	// side when exiting offboard velocity mode). constrain() does not reject NaN, so fall back to the integrator
	// state, which is kept finite by _calcPitchControlUpdate, to avoid latching a NaN pitch demand.
	if (!PX4_ISFINITE(pitch_setpoint_unc)) {
		pitch_setpoint_unc = _pitch_integ_state + param.level_flight_pitch;
	}

	return constrain(pitch_setpoint_unc, param.pitch_min, param.pitch_max);
}

void TECSControl::_calcThrottleControl(float dt, const STERateLimit &limit,
				       const SpecificEnergyRates &specific_energy_rates, const Param &param, const Flag &flag)
{
	ControlValues ste_rate{_calcThrottleControlSteRate(limit, specific_energy_rates, param)};

	// Low-pass the energy rate error for the feedback; the feedforward acts on the unfiltered setpoint. Filtering
	// the error rather than the estimate keeps a setpoint change from appearing as a lagged phantom error.
	updateIfFinite(_ste_rate_error_filter, dt, param.ste_rate_time_const, _getControlError(ste_rate));

	// The throttle does not deliver its energy rate setpoint when it is held at a limit (it sits there and its energy
	// rate error pushes it further in), or when the energy rate error is large although it is not, as with a throttle
	// model that overestimates the climb performance. The pitch then gives the kinetic demand priority (see
	// _calcKineticPriority()). Uses the last throttle output, the pitch runs before the throttle.
	const float ste_rate_error = _ste_rate_error_filter.getState();
	const bool held_at_max = (_throttle_setpoint >= param.throttle_max - FLT_EPSILON) && (ste_rate_error > 0.f);
	const bool held_at_min = (_throttle_setpoint <= param.throttle_min + FLT_EPSILON) && (ste_rate_error < 0.f);
	const float shortfall = interpolate(fabsf(ste_rate_error) * _steRateToThrottle(limit), kThrottleShortfallStart,
					    kThrottleShortfallFull, 0.f, 1.f);
	updateIfFinite(_throttle_shortfall_filter, dt, kThrottleShortfallTimeConst, (held_at_max || held_at_min) ? 1.f : shortfall);

	// During fast descend, we control airspeed over the pitch control loop. Give minimal thrust as soon as we are descending
	float throttle_setpoint{param.throttle_min};

	if (!fastDescendFullyEngaged(param.fast_descend)) {
		_calcThrottleControlUpdate(dt, limit, ste_rate, param, flag);
		throttle_setpoint = (1.f - param.fast_descend) * _calcThrottleControlOutput(limit, ste_rate, param,
				    flag) + param.fast_descend * param.throttle_min;
	}

	// Hold the last throttle on a non-finite demand, e.g. from a non-finite setpoint for a single frame. Otherwise it
	// would also reach the throttle model of the pitch loop (see _calcPitchControl()).
	if (!PX4_ISFINITE(throttle_setpoint)) {
		throttle_setpoint = _throttle_setpoint;
	}

	// Rate limit the throttle demand
	if (fabsf(param.throttle_slewrate) > FLT_EPSILON) {
		const float throttle_increment_limit = dt * (param.throttle_max - param.throttle_min) * param.throttle_slewrate;
		throttle_setpoint = constrain(throttle_setpoint, _throttle_setpoint - throttle_increment_limit,
					      _throttle_setpoint + throttle_increment_limit);
	}

	_throttle_setpoint = constrain(throttle_setpoint, param.throttle_min, param.throttle_max);

	// Debug output
	_debug_output.total_energy_rate_estimate = ste_rate.estimate;
	_debug_output.total_energy_rate_sp = ste_rate.setpoint;
	_debug_output.throttle_integrator = _throttle_integ_state;
}

TECSControl::ControlValues TECSControl::_calcThrottleControlSteRate(const STERateLimit &limit,
		const SpecificEnergyRates &specific_energy_rates,
		const Param &param) const
{
	// Output ste rate values
	ControlValues ste_rate;
	ste_rate.setpoint = specific_energy_rates.spe_rate.setpoint + specific_energy_rates.ske_rate.setpoint;

	// Adjust the demanded total energy rate to compensate for induced drag rise in turns.
	// Assume induced drag scales linearly with normal load factor.
	// The additional normal load factor is given by (1/cos(bank angle) - 1)
	ste_rate.setpoint += param.load_factor_correction * (param.load_factor - 1.f);

	ste_rate.setpoint = constrain(ste_rate.setpoint, limit.STE_rate_min, limit.STE_rate_max);
	ste_rate.estimate = specific_energy_rates.spe_rate.estimate + specific_energy_rates.ske_rate.estimate;

	return ste_rate;
}

void TECSControl::_calcThrottleControlUpdate(float dt, const STERateLimit &limit, const ControlValues &ste_rate,
		const Param &param, const Flag &flag)
{
	const float STE_rate_to_throttle = _steRateToThrottle(limit);

	// Integral handling
	if (flag.airspeed_enabled) {
		if (param.integrator_gain_throttle > FLT_EPSILON) {
			// underspeed conditions zero out integration
			float throttle_integ_input = (_ste_rate_error_filter.getState() * param.integrator_gain_throttle) * dt *
						     STE_rate_to_throttle * (1.0f - _ratio_undersped);

			// only allow integrator propagation into direction which unsaturates throttle
			if (_throttle_setpoint >= param.throttle_max) {
				throttle_integ_input = math::min(0.f, throttle_integ_input);

			}

			if (_throttle_setpoint <= param.throttle_min) {
				throttle_integ_input = math::max(0.f, throttle_integ_input);
			}

			// Calculate a throttle demand from the integrated total energy rate error
			// This will be added to the total throttle demand to compensate for steady state errors
			_throttle_integ_state = PX4_ISFINITE(throttle_integ_input) ? _throttle_integ_state + throttle_integ_input : 0.f;

		} else {
			_throttle_integ_state = 0.0f;
		}

	}
}

float TECSControl::_calcThrottleControlOutput(const STERateLimit &limit, const ControlValues &ste_rate,
		const Param &param,
		const Flag &flag) const
{
	const float STE_rate_to_throttle = _steRateToThrottle(limit);

	// Calculate a predicted throttle from the demanded rate of change of energy, using the cruise throttle
	// as the starting point. Assume:
	// Specific total energy rate = _STE_rate_max is achieved when throttle is set to _throttle_setpoint_max
	// Specific total energy rate = 0 at cruise throttle
	// Specific total energy rate = _STE_rate_min is achieved when throttle is set to _throttle_setpoint_min

	// assume airspeed and density-independent delta_throttle to sink/climb rate mapping
	// TODO: include air density for thrust mappings
	const float throttle_above_trim_per_ste_rate = (param.throttle_max - param.throttle_trim) / limit.STE_rate_max;
	const float throttle_below_trim_per_ste_rate = (param.throttle_trim - param.throttle_min) / limit.STE_rate_min;

	float throttle_predicted = 0.0f;

	if (ste_rate.setpoint >= FLT_EPSILON) {
		// throttle is between trim and maximum
		throttle_predicted = param.throttle_trim + ste_rate.setpoint * throttle_above_trim_per_ste_rate;

	} else {
		// throttle is between trim and minimum
		throttle_predicted = param.throttle_trim - ste_rate.setpoint * throttle_below_trim_per_ste_rate;

	}

	// Add proportional control feedback to the predicted throttle and constrain to throttle limits
	float throttle_setpoint = (_ste_rate_error_filter.getState() * param.throttle_damping_gain) * STE_rate_to_throttle +
				  throttle_predicted;

	if (flag.airspeed_enabled) {
		// Add the integrator feedback during closed loop operation with an airspeed sensor
		throttle_setpoint += _throttle_integ_state;

	} else {
		/* We want to avoid reducing the throttle output when switching from airspeed enabled mode into airspeedless mode.
		  Thus, if the throttle integrator has a positive value, add it still to the throttle setpoint. */
		throttle_setpoint += math::max(0.0f, _throttle_integ_state);
	}


	// ramp in max throttle setting with underspeediness value
	throttle_setpoint = _ratio_undersped * param.throttle_max + (1.0f - _ratio_undersped) * throttle_setpoint;

	return constrain(throttle_setpoint, param.throttle_min, param.throttle_max);
}

void TECSControl::resetIntegrals()
{
	_pitch_integ_state = 0.0f;
	_throttle_integ_state = 0.0f;
}

void TECS::initControlParams(const Input &input)
{
	// Update parameters from input
	// Reference model
	_reference_param.target_climbrate = input.target_climbrate;
	_reference_param.target_sinkrate = input.target_sinkrate;
	// Control
	_control_param.tas_min = input.eas_to_tas * _equivalent_airspeed_min;
	_control_param.tas_stall = input.eas_to_tas * _equivalent_airspeed_stall;
	_control_param.tas_max = input.eas_to_tas * _equivalent_airspeed_max;
	_control_param.tas_trim = input.eas_to_tas * _control_param.equivalent_airspeed_trim;
	_control_param.pitch_max = input.pitch_max;
	_control_param.pitch_min = input.pitch_min;
	_control_param.throttle_trim = input.throttle_trim;
	_control_param.throttle_max = input.throttle_max;
	_control_param.throttle_min = input.throttle_min;
}

float TECS::calcTrueAirspeedSetpoint(float eas_to_tas, float eas_setpoint)
{
	return lerp(eas_to_tas * eas_setpoint, _control_param.tas_max, _fast_descend);
}

void TECS::initialize(const Input &input)
{
	// Init subclasses
	TECSAltitudeReferenceModel::AltitudeReferenceState current_state{.alt = input.altitude,
			.alt_rate = input.altitude_rate};
	_altitude_reference_model.initialize(current_state);
	_airspeed_filter.initialize(input.equivalent_airspeed, _airspeed_filter_param.equivalent_airspeed_trim,
				    _control_flag.airspeed_enabled);

	// Seed the controller from the filter, which falls back to trim airspeed without a valid measurement,
	// so the controller sees the same airspeed on reset as on every following update
	const float true_airspeed = input.eas_to_tas * _airspeed_filter.getState().speed;

	TECSControl::Setpoint control_setpoint;
	control_setpoint.altitude_reference = _altitude_reference_model.getAltitudeReference();
	control_setpoint.altitude_rate_setpoint_direct = NAN; // initialization always goes through the altitude loop
	control_setpoint.tas_setpoint = true_airspeed;

	const TECSControl::Input control_input{ .altitude = input.altitude,
						.altitude_rate = input.altitude_rate,
						.tas = true_airspeed,
						.tas_rate = 0.0f};

	_control.initialize(control_setpoint, control_input, _control_param, _control_flag);

	_fast_descend = 0.f;
	_enabled_fast_descend_timestamp = 0U;
}

void TECS::update(const Input &input)
{
	// Calculate the time since last update (seconds)
	const hrt_abstime now = hrt_absolute_time();
	const float dt = static_cast<float>(now - _update_timestamp) * 1e-6f;

	initControlParams(input);

	if (dt < DT_MIN) {
		// Update interval too small, do not update. Assume constant states/output in this case.
		return;
	}

	float true_airspeed_setpoint;

	if (dt > DT_MAX || _update_timestamp == 0UL) {
		// Update time interval too large, can't guarantee sanity of state updates anymore. reset the control loop.
		initialize(input);
		true_airspeed_setpoint = calcTrueAirspeedSetpoint(input.eas_to_tas, input.equivalent_airspeed_setpoint);

	} else {
		/* Check if we want to fast descend. On fast descend, we set the throttle to min, and use the altitude control
		loop to control the speed to the maximum airspeed. */
		_setFastDescend(input.altitude_setpoint, input.altitude);
		_control_param.fast_descend = _fast_descend;

		// Update airspeedfilter submodule
		const TECSAirspeedFilter::Input airspeed_input{ .equivalent_airspeed = input.equivalent_airspeed,
				.equivalent_airspeed_rate = input.speed_deriv_forward / input.eas_to_tas};

		_airspeed_filter.update(dt, airspeed_input, _airspeed_filter_param, _control_flag.airspeed_enabled);

		// Update Reference model submodule
		if (fastDescendFullyEngaged(_fast_descend)) {
			// Reset the altitude reference model, while we are in fast descend.
			const TECSAltitudeReferenceModel::AltitudeReferenceState init_state{
				.alt = input.altitude,
				.alt_rate = input.altitude_rate};
			_altitude_reference_model.initialize(init_state);

		} else if (_control.getUnderspeedRatio() > 0.5f) {
			// Reset altitude reference during underspeed to prevent altitude error accumulation
			// that would cause aggressive pitch-up (and potential secondary stall) on recovery.
			const TECSAltitudeReferenceModel::AltitudeReferenceState init_state{
				.alt = input.altitude,
				.alt_rate = input.altitude_rate};
			_altitude_reference_model.initialize(init_state);

		} else {
			const TECSAltitudeReferenceModel::AltitudeReferenceState setpoint{ .alt = input.altitude_setpoint,
					.alt_rate = input.altitude_rate_setpoint};

			_altitude_reference_model.update(dt, setpoint, input.altitude, input.altitude_rate, _reference_param);
		}

		TECSControl::Setpoint control_setpoint;
		control_setpoint.altitude_reference = _altitude_reference_model.getAltitudeReference();
		control_setpoint.altitude_rate_setpoint_direct = _altitude_reference_model.getHeightRateSetpointDirect();
		true_airspeed_setpoint = calcTrueAirspeedSetpoint(input.eas_to_tas, input.equivalent_airspeed_setpoint);
		control_setpoint.tas_setpoint = true_airspeed_setpoint;

		const TECSAirspeedFilter::AirspeedFilterState airspeed_state{_airspeed_filter.getState()};
		const TECSControl::Input control_input{ .altitude = input.altitude,
							.altitude_rate = input.altitude_rate,
							.tas = input.eas_to_tas * airspeed_state.speed,
							.tas_rate = input.eas_to_tas * airspeed_state.speed_rate};

		_control.update(dt, control_setpoint, control_input, _control_param, _control_flag);
	}

	_debug_status.control = _control.getDebugOutput();
	_debug_status.true_airspeed_sp = true_airspeed_setpoint;
	const TECSAirspeedFilter::AirspeedFilterState airspeed_state{_airspeed_filter.getState()};
	const TECSAltitudeReferenceModel::AltitudeReferenceState altitude_reference{_altitude_reference_model.getAltitudeReference()};
	_debug_status.true_airspeed_filtered = input.eas_to_tas * airspeed_state.speed;
	_debug_status.true_airspeed_derivative = input.eas_to_tas * airspeed_state.speed_rate;
	_debug_status.altitude_reference = altitude_reference.alt;
	_debug_status.height_rate_reference = altitude_reference.alt_rate;
	_debug_status.height_rate_direct = _altitude_reference_model.getHeightRateSetpointDirect();
	_debug_status.fast_descend = _fast_descend;

	_update_timestamp = now;
}

void TECS::_setFastDescend(const float alt_setpoint, const float alt)
{
	// disable fast descend if we are close to the target altitude or the altitude setpoint is not finite

	if (PX4_ISFINITE(alt_setpoint) && _control_flag.airspeed_enabled && (_fast_descend_alt_err > FLT_EPSILON)
	    && ((alt_setpoint + _fast_descend_alt_err) < alt)) {
		auto now = hrt_absolute_time();

		if (_enabled_fast_descend_timestamp == 0U) {
			_enabled_fast_descend_timestamp = now;
		}

		_fast_descend = constrain(max(_fast_descend, static_cast<float>(now - _enabled_fast_descend_timestamp) /
					      static_cast<float>(FAST_DESCEND_RAMP_UP_TIME)), 0.f, 1.f);

	} else if (PX4_ISFINITE(alt_setpoint) && (_fast_descend > FLT_EPSILON) && (_fast_descend_alt_err > FLT_EPSILON)) {
		// We're in fast descend, scale it down until FAST_DESCEND_HEIGHT_MARGIN above target altitude
		_fast_descend = constrain((alt - alt_setpoint - FAST_DESCEND_HEIGHT_MARGIN) / _fast_descend_alt_err, 0.f, 1.f);
		_enabled_fast_descend_timestamp = 0U;

	} else {
		_fast_descend = 0.f;
		_enabled_fast_descend_timestamp = 0U;
	}
}
