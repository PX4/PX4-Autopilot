/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @file ActuatorEffectivenessStandardVTOL.hpp
 *
 * Actuator effectiveness for standard VTOL
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#pragma once

#include "ActuatorEffectiveness.hpp"
#include "ActuatorEffectivenessRotors.hpp"
#include "ActuatorEffectivenessControlSurfaces.hpp"

#include <uORB/topics/actuator_controls.h>

class ActuatorEffectivenessStandardVTOL : public ModuleParams, public ActuatorEffectiveness
{
public:
	ActuatorEffectivenessStandardVTOL(ModuleParams *parent);
	virtual ~ActuatorEffectivenessStandardVTOL() = default;

	bool getEffectivenessMatrix(Configuration &configuration, bool force) override;

	const char *name() const override { return "Standard VTOL"; }

	int numMatrices() const override { return 2; }

	void getDesiredAllocationMethod(AllocationMethod allocation_method_out[MAX_NUM_MATRICES]) const override
	{
		static_assert(MAX_NUM_MATRICES >= 2, "expecting at least 2 matrices");
		allocation_method_out[0] = AllocationMethod::SEQUENTIAL_DESATURATION;
		allocation_method_out[1] = AllocationMethod::PSEUDO_INVERSE;
	}

	void getNormalizeRPY(bool normalize[MAX_NUM_MATRICES]) const override
	{
		normalize[0] = true;
		normalize[1] = false;
	}

	void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index,
			    ActuatorVector &actuator_sp) override;

	void setFlightPhase(const FlightPhase &flight_phase) override;

	uint32_t getStoppedMotors() const override { return _stopped_motors; }

private:
	ActuatorEffectivenessRotors _mc_rotors;
	ActuatorEffectivenessControlSurfaces _control_surfaces;

	uint32_t _mc_motors_mask{}; ///< mc motors (stopped during forward flight)
	uint32_t _stopped_motors{}; ///< currently stopped motors

	int _first_control_surface_idx{0}; ///< applies to matrix 1

	uORB::Subscription _actuator_controls_1_sub{ORB_ID(actuator_controls_0)};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::CA_STDVTOL_N_P>) _param_ca_stdvtol_n_p ///< number of pushers
	)

};
