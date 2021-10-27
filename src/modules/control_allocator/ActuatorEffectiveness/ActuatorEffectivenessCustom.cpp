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

#include "ActuatorEffectivenessCustom.hpp"

ActuatorEffectivenessCustom::ActuatorEffectivenessCustom(ModuleParams *parent):
	ModuleParams(parent)
{
}

bool ActuatorEffectivenessCustom::getEffectivenessMatrix(matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &matrix,
		bool force)
{
	if (_updated || force) {
		_updated = false;

		int num_actuators = 0;

		for (int n = 0; n < NUM_ACTUATORS; n++) {
			// CA_ACTn_TRQ_R
			// CA_ACTn_TRQ_P
			// CA_ACTn_TRQ_Y
			char torque_str[3][17];
			sprintf(torque_str[0], "CA_ACT%u_TRQ_R", n);
			sprintf(torque_str[1], "CA_ACT%u_TRQ_P", n);
			sprintf(torque_str[2], "CA_ACT%u_TRQ_Y", n);

			// CA_ACTn_THR_X
			// CA_ACTn_THR_Y
			// CA_ACTn_THR_Z
			char thrust_str[3][17];
			sprintf(thrust_str[0], "CA_ACT%u_THR_X", n);
			sprintf(thrust_str[1], "CA_ACT%u_THR_Y", n);
			sprintf(thrust_str[2], "CA_ACT%u_THR_Z", n);

			for (int i = 0; i < 3; i++) {
				// CA_ACTn_TRQ_{R,P,Y}
				param_get(param_find(torque_str[i]), &_matrix(n, i));

				// CA_ACTn_THR_{X,Y,Z}
				param_get(param_find(thrust_str[i]), &_matrix(n, i + 3));
			}

			if (_matrix.row(n).longerThan(0.f)) {
				num_actuators++;
			}
		}

		_num_actuators = num_actuators;
		matrix = _matrix;
		return true;
	}

	return false;
}
