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

#pragma once

#include "ActuatorEffectiveness.hpp"
#include "ActuatorEffectivenessRotors.hpp"
#include "ActuatorEffectivenessControlSurfaces.hpp"
#include "ActuatorEffectivenessTilts.hpp"

#include <px4_platform_common/module_params.h>

class ActuatorEffectivenessCustomTiltrotor : public ModuleParams, public ActuatorEffectiveness
{
public:
    ActuatorEffectivenessCustomTiltrotor(ModuleParams *parent);
    ~ActuatorEffectivenessCustomTiltrotor() = default;

    bool getEffectivenessMatrix(Configuration &configuration,
                                 EffectivenessUpdateReason external_update) override;

    void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
                         int matrix_index,
                         ActuatorVector &actuator_sp,
                         const ActuatorVector &actuator_min,
                         const ActuatorVector &actuator_max) override;

    const char *name() const override { return "Custom Tiltrotor"; }

private:
    ActuatorEffectivenessRotors _mc_rotors;
    ActuatorEffectivenessControlSurfaces _control_surfaces;
    ActuatorEffectivenessTilts _tilts;

    int _first_tilt_idx{0};
    int _first_control_surface_idx{0};

    float _lambda{0.f};
    float _current_tilt_control{-1.f}; // start at full hover

    static constexpr float WASH_ONSET_LAMBDA = 0.15f;
    static constexpr float WASH_FULL_LAMBDA  = 0.5f;

    float propwashGain(float lambda) const;
};