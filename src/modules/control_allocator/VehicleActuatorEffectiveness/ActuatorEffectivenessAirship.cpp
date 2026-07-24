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

#include "ActuatorEffectivenessAirship.hpp"

#include <float.h>

bool
ActuatorEffectivenessAirship::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (!ActuatorEffectivenessCustom::getEffectivenessMatrix(configuration, external_update)) {
		return false;
	}

	EffectivenessMatrix &matrix = configuration.effectiveness_matrices[configuration.selected_matrix];

	for (int i = 0; i < NUM_ACTUATORS; ++i) {
		matrix(ControlAxis::THRUST_Y, i) = 0.f;
	}

	float yaw_scale = 0.f;

	for (int i = 0; i < NUM_ACTUATORS; ++i) {
		yaw_scale = fmaxf(yaw_scale, fabsf(matrix(ControlAxis::YAW, i)));
	}

	if (yaw_scale > FLT_EPSILON) {
		for (int i = 0; i < NUM_ACTUATORS; ++i) {
			matrix(ControlAxis::YAW, i) /= yaw_scale;
		}
	}

	return true;
}
