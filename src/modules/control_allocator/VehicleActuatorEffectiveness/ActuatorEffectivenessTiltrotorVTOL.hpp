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
 * @file ActuatorEffectivenessTiltrotorVTOL.hpp
 *
 * Actuator effectiveness for tiltrotor VTOL
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#pragma once

#include "control_allocation/actuator_effectiveness/ActuatorEffectiveness.hpp"
#include "ActuatorEffectivenessRotors.hpp"
#include "ActuatorEffectivenessControlSurfaces.hpp"
#include "ActuatorEffectivenessTilts.hpp"

#include <px4_platform_common/module_params.h>

#include <uORB/topics/normalized_unsigned_setpoint.h>
#include <uORB/topics/tiltrotor_extra_controls.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/Subscription.hpp>

class ActuatorEffectivenessTiltrotorVTOL : public ModuleParams, public ActuatorEffectiveness
{
public:
	ActuatorEffectivenessTiltrotorVTOL(ModuleParams *parent);
	virtual ~ActuatorEffectivenessTiltrotorVTOL() = default;

	bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) override;

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

	void setFlightPhase(const FlightPhase &flight_phase) override;

	void allocateAuxilaryControls(const float dt, int matrix_index, ActuatorVector &actuator_sp) override;

	void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index,
			    ActuatorVector &actuator_sp, const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
			    const matrix::Vector<float, NUM_ACTUATORS> &actuator_max) override;

	const char *name() const override { return "VTOL Tiltrotor"; }

	void getUnallocatedControl(int matrix_index, control_allocator_status_s &status) override;

protected:
	bool _collective_tilt_updated{true};
	ActuatorEffectivenessRotors _mc_rotors;
	ActuatorEffectivenessControlSurfaces _control_surfaces;
	ActuatorEffectivenessTilts _tilts;

	uint32_t _motors{};
	uint32_t _untiltable_motors{};

	int _first_control_surface_idx{0}; ///< applies to matrix 1
	int _first_tilt_idx{0}; ///< applies to matrix 0

	float _last_collective_tilt_control{NAN};

	uORB::Subscription _flaps_setpoint_sub{ORB_ID(flaps_setpoint)};
	uORB::Subscription _spoilers_setpoint_sub{ORB_ID(spoilers_setpoint)};

	struct YawTiltSaturationFlags {
		bool tilt_yaw_pos;
		bool tilt_yaw_neg;
	};

	YawTiltSaturationFlags _yaw_tilt_saturation_flags{};

	uORB::Subscription _tiltrotor_extra_controls_sub{ORB_ID(tiltrotor_extra_controls)};

private:

	void updateParams() override;

	struct ParamHandles {
		param_t com_spoolup_time;
	};

	ParamHandles _param_handles{};

	float _param_spoolup_time{1.f};

	// Tilt handling during motor spoolup: leave the tilts in their disarmed position unitil 1s after arming
	bool throttleSpoolupFinished();
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	bool _armed{false};
	uint64_t _armed_time{0};
};
