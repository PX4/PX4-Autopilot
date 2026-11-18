/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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


#include "fixedWingPropulsionCheck.hpp"

#include <matrix/math.hpp>

using namespace time_literals;

void FixedWingPropulsionChecks::checkAndReport(const Context &context, Report &reporter)
{
	const hrt_abstime now = hrt_absolute_time();

	if (_param_com_ef_curr.get() < FLT_EPSILON) {
		// disabling the check in flight also clears a detected failure, e.g. a false positive
		_failure_detected = false;
		_low_current_hysteresis.set_state_and_update(false, now);
		_throttle_high_since = 0;

	} else {
		_low_current_hysteresis.set_hysteresis_time_from(false, (hrt_abstime)(1_s * math::max(_param_com_ef_time.get(), 0.f)));
		_low_current_hysteresis.set_state_and_update(isCurrentTooLow(context), now);
		_failure_detected = _failure_detected || _low_current_hysteresis.get_state();
	}

	// the failsafe flags persist between runs
	reporter.failsafeFlags().fd_fw_motor = _failure_detected;

	if (_failure_detected) {
		/* EVENT
		 * @description
		 * In fixed-wing flight, the battery current stayed below <param>COM_EF_CURR</param> for <param>COM_EF_TIME</param>,
		 * although the throttle was above <param>COM_EF_THROT</param>.
		 * Land and check the fixed-wing propulsion. A reboot clears it.
		 */
		reporter.healthFailure(NavModes::All, health_component_t::motors_escs, events::ID("check_fw_propulsion_failure"),
				       events::Log::Emergency, "Fixed-wing propulsion failure");

		if (reporter.mavlink_log_pub()) {
			mavlink_log_emergency(reporter.mavlink_log_pub(), "Fixed-wing propulsion failure\t");
		}
	}
}

bool FixedWingPropulsionChecks::isCurrentTooLow(const Context &context)
{
	vehicle_land_detected_s vehicle_land_detected;
	const bool landed = !_vehicle_land_detected_sub.copy(&vehicle_land_detected) || vehicle_land_detected.landed;

	vehicle_thrust_setpoint_s vehicle_thrust_setpoint;
	float throttle = NAN;

	if (_vehicle_thrust_setpoint_sub.copy(&vehicle_thrust_setpoint)
	    && hrt_elapsed_time(&vehicle_thrust_setpoint.timestamp) < 1_s) {
		// fixed-wing thrust is along x, on tiltrotors and tailsitters along -z, and a stopped motor reads NAN
		throttle = matrix::Vector3f(vehicle_thrust_setpoint.xyz).norm();
	}

	if (!context.isArmed() || landed || context.status().vehicle_type != vehicle_status_s::VEHICLE_TYPE_FIXED_WING
	    || !PX4_ISFINITE(throttle) || throttle < _param_com_ef_throt.get()) {
		_throttle_high_since = 0;
		return false;
	}

	if (_throttle_high_since == 0) {
		_throttle_high_since = hrt_absolute_time();
	}

	// The propulsion draws most of the current in fixed-wing flight, which any connected battery can feed: each needs
	// a current measurement taken at this throttle. Smart batteries report every few seconds.
	float current = 0.f;
	bool current_measured = false;

	for (auto &battery_status_sub : _battery_status_subs) {
		battery_status_s battery_status;

		if (!battery_status_sub.copy(&battery_status) || !battery_status.connected) {
			continue;
		}

		if (battery_status.current_a < 0.f || battery_status.timestamp < _throttle_high_since
		    || hrt_elapsed_time(&battery_status.timestamp) > 5_s) {
			return false;
		}

		current += battery_status.current_a;
		current_measured = true;
	}

	return current_measured && current < _param_com_ef_curr.get();
}
