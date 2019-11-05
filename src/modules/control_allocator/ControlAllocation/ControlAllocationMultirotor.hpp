/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file ControlAllocationMultirotor.hpp
 *
 * Control Allocation Algorithm for multirotors
 *
 * It computes the pseudo-inverse of the effectiveness matrix
 * Actuator saturation is handled by ad-hoc logic adapted
 * from the multirotor mixer (src/lib/mixer/mixer_multirotor.cpp)
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#pragma once

#include "ControlAllocation.hpp"

class ControlAllocationMultirotor: public ControlAllocation
{
public:

	ControlAllocationMultirotor() = default;
	virtual ~ControlAllocationMultirotor() = default;

	void allocate() override;
	void setEffectivenessMatrix(const matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &B) override;

private:
	matrix::Matrix<float, NUM_ACTUATORS, NUM_AXES> _A;

	ControlAxis _axis_prio_increasing [NUM_AXES] = {ControlAxis::YAW, ControlAxis::THRUST_Y, ControlAxis::THRUST_X, ControlAxis::THRUST_Z, ControlAxis::PITCH, ControlAxis::ROLL};

	ActuatorVector desaturateActuators(ActuatorVector actuator_sp, ControlAxis axis);

	ActuatorVector getDesaturationVector(ControlAxis axis);

	float computeDesaturationGain(ActuatorVector desaturation_vector, ActuatorVector actuator_sp);
};
