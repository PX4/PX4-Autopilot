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

#include "TurtleControl.hpp"

#include <cmath>
#include <drivers/drv_hrt.h>
#include <matrix/matrix/math.hpp>

using namespace time_literals;

TurtleControl::TurtleControl(ModuleParams *parent) :
	ModuleParams(parent)
{
	ModuleParams::updateParams();
}

TurtleControl::~TurtleControl()
{
	stop();
}

void TurtleControl::stop()
{
	if (_selected) {
		turtle_control_s control{};
		control.timestamp = hrt_absolute_time();
		control.direction_request_id = _request_id;
		control.disarm_requested = _arm_state != ArmState::Disarmed;
		_control_pub.publish(control);
		_selected = false;
	}
}

bool TurtleControl::fresh(uint64_t timestamp, uint64_t now, uint64_t timeout)
{
	return timestamp != 0 && timestamp <= now && now - timestamp < timeout;
}

void TurtleControl::requestReverse(uint64_t authorizing_status_timestamp)
{
	_request_id = math::max(_request_id, _direction_sub.get().request_id) + 1;

	if (_request_id == 0) {
		++_request_id;
	}

	motor_direction_request_s request{};
	request.timestamp = hrt_absolute_time();
	request.timestamp_sample = authorizing_status_timestamp;
	request.request_id = _request_id;
	request.motor_mask = MOTOR_MASK;
	_direction_pub.publish(request);
}

bool TurtleControl::update(const vehicle_status_s &status, const manual_control_setpoint_s &manual,
			   const vehicle_attitude_s &attitude)
{
	const bool armed = status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;

	if (status.nav_state != vehicle_status_s::NAVIGATION_STATE_TURTLE) {
		stop();
		// An already-armed flight cannot become a recovery session by changing modes.
		_arm_state = armed ? ArmState::Aborted : ArmState::Disarmed;
		return false;
	}

	_selected = true;
	_allocation_sub.update();
	_direction_sub.update();
	const hrt_abstime now = hrt_absolute_time();
	const control_allocator_status_s &allocation = _allocation_sub.get();
	const motor_direction_status_s &direction = _direction_sub.get();
	const bool normal = direction.reversed_mask == 0 && direction.state != motor_direction_status_s::STATE_PENDING;

	if (_parameters_pending && !armed && normal) {
		ModuleParams::updateParams();
		_parameters_pending = false;
	}

	const bool configuration_valid = _param_enabled.get() && !status.is_vtol
					 && status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
					 && allocation.turtle_mode_active && allocation.turtle_geometry_valid
					 && allocation.timestamp >= status.nav_state_timestamp
					 && fresh(allocation.timestamp, now, 250_ms)
					 && PX4_ISFINITE(_param_maximum.get()) && _param_maximum.get() > 0.f
					 && _param_maximum.get() <= 1.f
					 && PX4_ISFINITE(_param_time.get()) && _param_time.get() >= 0.1f
					 && _param_time.get() <= 3.f;
	// ManualControl owns RC-loss detection and republishes valid input on its 200 ms backup schedule.
	// Watch its publication here so a stalled producer still stops recovery.
	const bool input_valid = manual.valid && manual.data_source == manual_control_setpoint_s::SOURCE_RC
				 && fresh(manual.timestamp, now, 300_ms)
				 && PX4_ISFINITE(manual.roll) && PX4_ISFINITE(manual.pitch)
				 && PX4_ISFINITE(manual.yaw) && PX4_ISFINITE(manual.throttle)
				 && fabsf(manual.roll) <= 1.f && fabsf(manual.pitch) <= 1.f
				 && fabsf(manual.yaw) <= 1.f && manual.throttle >= -1.f
				 && manual.throttle < THROTTLE_LOW;
	const bool centered = fabsf(manual.roll) <= STICK_DEADBAND && fabsf(manual.pitch) <= STICK_DEADBAND;
	const matrix::Quatf q(attitude.q);
	const bool attitude_valid = fresh(attitude.timestamp, now, 100_ms)
				    && q.isAllFinite() && fabsf(q.norm_squared() - 1.f) < 0.01f;
	const bool capable = fresh(direction.timestamp, now, 300_ms) && direction.supported_motor_mask == MOTOR_MASK;
	const bool recovery_valid = configuration_valid && input_valid && attitude_valid && capable && !status.failsafe;
	const bool entry_valid = recovery_valid && normal && centered && fabsf(manual.yaw) <= STICK_DEADBAND;

	turtle_control_s control{};
	control.timestamp = now;
	control.timestamp_sample = manual.timestamp_sample;
	control.enabled = _param_enabled.get();
	control.ready_to_arm = !armed && entry_valid;

	if (armed) {
		if (_arm_state == ArmState::Disarmed) {
			_session.reset();
			_arm_state = entry_valid ? ArmState::Active : ArmState::Aborted;

			if (entry_valid) {
				requestReverse(status.timestamp);
			}
		}

		const bool matching_request = direction.request_id == _request_id;

		if (!recovery_valid || (matching_request && direction.state == motor_direction_status_s::STATE_REJECTED)) {
			_arm_state = ArmState::Aborted;
		}

		control.disarm_requested = _arm_state == ArmState::Aborted;

		if (_arm_state == ArmState::Active && matching_request && direction.state == motor_direction_status_s::STATE_COMPLETE
		    && direction.reversed_mask == MOTOR_MASK
		    && _session.update(now, centered, (uint64_t)(_param_time.get() * 1e6f))) {
			control.roll = manual.roll;
			control.pitch = manual.pitch;
			control.thrust = _param_maximum.get();
			control.driving = true;
		}

	} else {
		_arm_state = ArmState::Disarmed;
	}

	control.direction_request_id = _request_id;
	_control_pub.publish(control);
	return true;
}
