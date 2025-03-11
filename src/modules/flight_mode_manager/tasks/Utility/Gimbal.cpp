/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#include "Gimbal.hpp"
#include <px4_platform_common/events.h>
#include <math.h>

using namespace time_literals;
using namespace matrix;

// -------------------------------------------------------------------------------------------------
// Helper: compute a roll‑free yaw that stays continuous through 90° pitch
// and automatically corrects the 180° flip that occurs once the camera
// tilts past straight down (or straight up).  The correction is done by
// detecting when the denominator of the fused‑yaw formula changes sign.
// -------------------------------------------------------------------------------------------------
static float pan_yaw_from_cam_quat(const Quatf &q)
{
	Quatf q_norm = q.normalized();

	float w = q_norm(0);
	float x = q_norm(1);
	float y = q_norm(2);
	float z = q_norm(3);

	/*  Sign-lock:  q and –q encode the same attitude.
	*  Forcing w ≥ 0 picks a single representation, so
	*  headings derived from the quaternion never jump
	*  by ±π when w crosses zero.                       */
	if (w < 0.0f) {
		w = -w;
		x = -x;
		y = -y;
		z = -z;
	}


	/*  Fused-yaw forms
	*  ---------------
	*  long  = atan2( 2(wz+xy) , w²+x²−y²−z² )
	*          · roll-invariant, but blows up when |tilt| → 90°
	*
	*  short = 2*atan2(z , w) (same as atan2(2wz , w²−z²))
	*          · always stable, but drifts with roll
	*
	*  Select long form except when the long denominator
	*  is too close to zero (≈ vertical nose).
	*/
	const float num_full = 2.f * (w * z + x * y);                    // 2(w z + x y)
	const float denom_full = w * w + x * x - y * y - z * z;  // w²+x²−y²−z²

	float hdg = 0.f;

	if (fabsf(denom_full) > 1e-3f) { /* far from ±90°  →  long form */
		hdg = atan2f(num_full, denom_full);

	} else {                          /* near ±90°      →  short form */
		hdg = 2.f * atan2f(z, w); /* 2·atan2(z,w)   (stable)     */
	}

	/* π-offset correction when body-Z points mostly down */
	/*  include deadband to protect agauinst noise  */
	const float hemi = w * w + z * z; /* w² + z²               */

	if (hemi < 0.5f - 1e-3f) {             /* only if >90° */
		hdg += (hdg >= 0.f) ? -M_PI_F : M_PI_F;
	}


	/* wrap to (–π, π] */
	if (hdg <= -M_PI_F) { hdg += 2.f * M_2_PI_F; }

	if (hdg > M_PI_F) { hdg -= 2.f * M_2_PI_F; }

	return hdg;
}

Gimbal::Gimbal(ModuleParams *parent) :
	ModuleParams(parent)
{}

Gimbal::~Gimbal()
{
	releaseGimbalControlIfNeeded();
}

bool Gimbal::checkForTelemetry(const hrt_abstime now)
{
	if (_gimbal_device_attitude_status_sub.updated()) {
		gimbal_device_attitude_status_s gimbal_device_attitude_status{};

		if (_gimbal_device_attitude_status_sub.copy(&gimbal_device_attitude_status)) {
			_telemtry_timestamp = gimbal_device_attitude_status.timestamp;
			_telemetry_flags    = gimbal_device_attitude_status.device_flags;

			// ----------------------------------------------------------------------------
			// Robust yaw extraction – follow the PAN joint only.
			// ----------------------------------------------------------------------------
			const Quatf q(gimbal_device_attitude_status.q);
			_telemetry_yaw = pan_yaw_from_cam_quat(q);
		}
	}

	return now < _telemtry_timestamp + 2_s;
}

void Gimbal::acquireGimbalControlIfNeeded()
{
	gimbal_manager_status_s gimbal_manager_status;

	if (_gimbal_manager_status_sub.updated()) {
		_gimbal_manager_status_sub.copy(&gimbal_manager_status);

		if (gimbal_manager_status.primary_control_compid != _param_mav_comp_id.get() ||
		    gimbal_manager_status.primary_control_sysid != _param_mav_sys_id.get()) {
			_tried_to_have_gimbal_control = true;
			vehicle_command_s vehicle_command{};
			vehicle_command.command = vehicle_command_s::VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE;
			vehicle_command.param1 = _param_mav_sys_id.get();
			vehicle_command.param2 = _param_mav_comp_id.get();
			vehicle_command.param3 = -1.0f; // Leave unchanged.
			vehicle_command.param4 = -1.0f; // Leave unchanged.
			vehicle_command.timestamp = hrt_absolute_time();
			vehicle_command.source_system = _param_mav_sys_id.get();
			vehicle_command.source_component = _param_mav_comp_id.get();
			vehicle_command.target_system = _param_mav_sys_id.get();
			vehicle_command.target_component = _param_mav_sys_id.get();
			vehicle_command.from_external = false;
			_vehicle_command_pub.publish(vehicle_command);
		}
	}
}

void Gimbal::releaseGimbalControlIfNeeded()
{
	if (_tried_to_have_gimbal_control) {
		_tried_to_have_gimbal_control = false;

		// Restore default flags, setting rate setpoints to NAN lead to unexpected behavior
		publishGimbalManagerSetAttitude(FLAGS_ROLL_PITCH_LOCKED,
						Quatf(NAN, NAN, NAN, NAN),
						Vector3f(NAN, 0.f, 0.f));

		// Release gimbal
		vehicle_command_s vehicle_command{};
		vehicle_command.command = vehicle_command_s::VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE;
		vehicle_command.param1 = -3.0f; // Remove control if it had it.
		vehicle_command.param2 = -3.0f; // Remove control if it had it.
		vehicle_command.param3 = -1.0f; // Leave unchanged.
		vehicle_command.param4 = -1.0f; // Leave unchanged.
		vehicle_command.timestamp = hrt_absolute_time();
		vehicle_command.from_external = false;
		vehicle_command.source_system = _param_mav_sys_id.get();
		vehicle_command.source_component = _param_mav_comp_id.get();
		vehicle_command.target_system = _param_mav_sys_id.get();
		vehicle_command.target_component = _param_mav_comp_id.get();
		_vehicle_command_pub.publish(vehicle_command);
	}
}

void Gimbal::publishGimbalManagerSetAttitude(const uint16_t gimbal_flags,
		const matrix::Quatf &q_gimbal_setpoint,
		const matrix::Vector3f &gimbal_rates)
{
	gimbal_manager_set_attitude_s gimbal_setpoint{};
	gimbal_setpoint.origin_sysid = _param_mav_sys_id.get();
	gimbal_setpoint.origin_compid =  _param_mav_comp_id.get();
	gimbal_setpoint.flags = gimbal_flags;
	q_gimbal_setpoint.copyTo(gimbal_setpoint.q);
	gimbal_setpoint.angular_velocity_x =  gimbal_rates(0);
	gimbal_setpoint.angular_velocity_y = gimbal_rates(1);
	gimbal_setpoint.angular_velocity_z = gimbal_rates(2);
	gimbal_setpoint.timestamp = hrt_absolute_time();
	_gimbal_manager_set_attitude_pub.publish(gimbal_setpoint);
}

float Gimbal::getPitch(const hrt_abstime now)
{
	if (_gimbal_manager_set_manual_control_sub.updated()) {
		gimbal_manager_set_manual_control_s gimbal_manager_set_manual_control{};

		if (_gimbal_manager_set_manual_control_sub.copy(&gimbal_manager_set_manual_control)
		    && gimbal_manager_set_manual_control.origin_compid != _param_mav_comp_id.get()
		    && gimbal_manager_set_manual_control.origin_sysid != _param_mav_sys_id.get()) {
			_set_manual_control_timestamp = gimbal_manager_set_manual_control.timestamp;
			_set_manual_control_pitchrate = gimbal_manager_set_manual_control.pitch_rate;
		}
	}

	if (now > _set_manual_control_timestamp + 2_s) {
		_set_manual_control_pitchrate = 0.f;
	}

	return _set_manual_control_pitchrate;
}
