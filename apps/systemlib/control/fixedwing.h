/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file fixedwing.h
 *
 * Controller library code
 */

#pragma once

#include "control.h"

namespace control
{

template<class PID_CLASS>
class FixedWingStabilization :
    public Named
{
private:
    PID_CLASS _pid_roll2Ail;
    PID_CLASS _pid_pitch2Elv;
    PID_CLASS _pid_yawR2Rdr;
    float _aileronCmd;
    float _elevatorCmd;
    float _rudderCmd;
public:
    FixedWingStabilization(const char * name) :
        Named(name),
        _pid_roll2Ail(prependName("Roll_Ail")),
        _pid_pitch2Elv(prependName("Pitch_Elv")),
        _pid_yawR2Rdr(prependName("YawR_Rdr")),
        _aileronCmd(0),
        _elevatorCmd(0),
        _rudderCmd(0)
    {
    }
    void update(
            float rollCmd,
            float pitchCmd,
            float yawRCmd,
            float roll,
            float pitch,
            float yawR,
            uint16_t dt)
    {
        _aileronCmd = _pid_roll2Ail.update(rollCmd - roll, dt);
        _elevatorCmd = _pid_pitch2Elv.update(pitchCmd - pitch, dt);
        _rudderCmd = _pid_yawR2Rdr.update(yawRCmd - yawR, dt);
    }
    void updateParams()
    {
        _pid_roll2Ail.updateParams();
        _pid_pitch2Elv.updateParams();
        _pid_yawR2Rdr.updateParams();
    }
    float getAileronCmd() {return _aileronCmd;}
    float getElevatorCmd() {return _elevatorCmd;}
    float getRudderCmd() {return _rudderCmd;}
};

} // namespace control

