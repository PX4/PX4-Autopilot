/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/log.h>
#include <px4_platform_common/events.h>

#include "ActuatorEffectivenessControlSurfaces.hpp"

using namespace matrix;

ActuatorEffectivenessControlSurfaces::ActuatorEffectivenessControlSurfaces(ModuleParams *parent)
	: ModuleParams(parent)
{
	for (int i = 0; i < MAX_COUNT; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_SV_CS%u_TYPE", i);
		_param_handles[i].type = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_SV_CS%u_TRQ_R", i);
		_param_handles[i].torque[0] = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_SV_CS%u_TRQ_P", i);
		_param_handles[i].torque[1] = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_SV_CS%u_TRQ_Y", i);
		_param_handles[i].torque[2] = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_SV_CS%u_TRIM", i);
		_param_handles[i].trim = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_SV_CS%u_FLAP", i);
		_param_handles[i].scale_flap = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_SV_CS%u_SPOIL", i);
		_param_handles[i].scale_spoiler = param_find(buffer);
	}

	_flaps_setpoint_with_slewrate.setSlewRate(_param_ca_flap_slew.get());
	_spoilers_setpoint_with_slewrate.setSlewRate(kSpoilersSlewRate);

	_count_handle = param_find("CA_SV_CS_COUNT");
	_param_handle_ca_cs_laun_lk = param_find("CA_CS_LAUN_LK");
	updateParams();
}

void ActuatorEffectivenessControlSurfaces::updateParams()
{
	ModuleParams::updateParams();

	if (param_get(_count_handle, &_count) != PX4_OK) {
		PX4_ERR("param_get failed");
		return;
	}

	// Update flap slewrates
	_flaps_setpoint_with_slewrate.setSlewRate(_param_ca_flap_slew.get());

	// Helper to check if a PWM center parameter is enabled, and clamp it to valid range
	auto check_pwm_center = [](const char *prefix, int channel) -> bool {
		char param_name[17];
		snprintf(param_name, sizeof(param_name), "%s_CENT%d", prefix, channel);
		param_t param = param_find(param_name);

		if (param != PARAM_INVALID)
		{
			int32_t value;

			if (param_get(param, &value) == PX4_OK && value != -1) {
				// Clamp PWM center to valid range [800, 2200]
				if (value < 800 || value > 2200) {
					int32_t clamped = (value < 800) ? 800 : 2200;
					PX4_WARN("%s_CENT%d (%d) out of range, clamping to %d", prefix, channel, (int)value, (int)clamped);
					param_set(param, &clamped);
				}

				return true;
			}
		}

		return false;
	};

	// Check if any PWM_MAIN or PWM_AUX center is configured
	bool pwm_center_set = false;

	for (int i = 1; i <= 8; i++) {
		if (check_pwm_center("PWM_MAIN", i) || check_pwm_center("PWM_AUX", i)) {
			pwm_center_set = true;
		}
	}

	param_get(_param_handle_ca_cs_laun_lk, &_param_ca_cs_laun_lk);

	for (int i = 0; i < _count; i++) {
		param_get(_param_handles[i].type, (int32_t *)&_params[i].type);

		Vector3f &torque = _params[i].torque;

		for (int n = 0; n < 3; ++n) {
			param_get(_param_handles[i].torque[n], &torque(n));
		}

		param_get(_param_handles[i].trim, &_params[i].trim);

		// If PWM center is set and CA_SV_CS trim is non-zero, warn and reset to 0
		if (pwm_center_set && fabsf(_params[i].trim) > FLT_EPSILON) {
			/* EVENT
			* @description Display warning in GCS when TRIM settings were present and now CENTER are set.
			*/
			events::send<uint8_t, float>(events::ID("control_surfaces_reset_trim"), events::Log::Warning,
						     "CA_SV_CS{1}_TRIM ({2}) is reset to 0 as PWM CENTER is used", i, _params[i].trim);

			_params[i].trim = 0.0f;
			// Update the parameter storage
			param_set(_param_handles[i].trim, &_params[i].trim);
		}

		param_get(_param_handles[i].scale_flap, &_params[i].scale_flap);
		param_get(_param_handles[i].scale_spoiler, &_params[i].scale_spoiler);

		// TODO: enforce limits (note that tailsitter uses different limits)?
		switch (_params[i].type) {

		case Type::LeftAileron:
			break;

		case Type::RightAileron:
			break;

		case Type::Elevator:
			break;

		case Type::Rudder:
			break;

		case Type::LeftElevon:
			break;

		case Type::RightElevon:
			break;

		case Type::LeftVTail:
			break;

		case Type::RightVTail:
			break;

		case Type::LeftFlap:
		case Type::RightFlap:
			torque.setZero();
			break;

		case Type::Airbrake:
			torque.setZero();
			break;

		case Type::Custom:
			break;

		case Type::LeftATail:
			break;

		case Type::RightATail:
			break;

		case Type::SingleChannelAileron:
			break;

		case Type::SteeringWheel:
			torque.setZero();
			break;

		case Type::LeftSpoiler:
		case Type::RightSpoiler:
			torque.setZero();
			break;
		}
	}
}

bool ActuatorEffectivenessControlSurfaces::addActuators(Configuration &configuration)
{
	for (int i = 0; i < _count; i++) {
		int actuator_idx = configuration.addActuator(ActuatorType::SERVOS, _params[i].torque, Vector3f{});

		if (actuator_idx >= 0) {
			configuration.trim[configuration.selected_matrix](actuator_idx) = _params[i].trim;
		}
	}

	return true;
}

void ActuatorEffectivenessControlSurfaces::applyFlaps(float flaps_control, int first_actuator_idx, float dt,
		ActuatorVector &actuator_sp)
{
	_flaps_setpoint_with_slewrate.update(flaps_control, dt);

	for (int i = 0; i < _count; ++i) {
		if (_params[i].type == Type::LeftFlap || _params[i].type == Type::RightFlap) {
			// map [0, 1] to [-1, 1] for pure flaps
			actuator_sp(i + first_actuator_idx) += (_flaps_setpoint_with_slewrate.getState() * 2.f - 1.f) * _params[i].scale_flap;

		} else {
			// map [0, 1] to [0, 1] for flaperons (ailerons deflected down)
			actuator_sp(i + first_actuator_idx) += _flaps_setpoint_with_slewrate.getState() * _params[i].scale_flap;
		}
	}
}

void ActuatorEffectivenessControlSurfaces::applySpoilers(float spoilers_control, int first_actuator_idx, float dt,
		ActuatorVector &actuator_sp)
{
	_spoilers_setpoint_with_slewrate.update(spoilers_control, dt);

	for (int i = 0; i < _count; ++i) {
		if (_params[i].type == Type::LeftSpoiler || _params[i].type == Type::RightSpoiler) {
			// map [0, 1] to [-1, 1] for pure spoilers
			actuator_sp(i + first_actuator_idx) += (_spoilers_setpoint_with_slewrate.getState() * 2.f - 1.f) * _params[i].scale_spoiler;

		} else {
			// map [0, 1] to [0, 1] for spoilerons (ailerons deflected up)
			actuator_sp(i + first_actuator_idx) += _spoilers_setpoint_with_slewrate.getState() * _params[i].scale_spoiler;
		}
	}
}

void ActuatorEffectivenessControlSurfaces::applyLaunchLock(int first_actuator_idx,
		ActuatorVector &actuator_sp)
{
	for (int i = 0; i < _count; ++i) {

		if (_param_ca_cs_laun_lk & (1u << i)) {
			actuator_sp(i + first_actuator_idx) = NAN;
		}
	}
}
