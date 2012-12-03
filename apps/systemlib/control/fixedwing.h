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

template<class BLOCK_LOWPASS, class BLOCK_HIGHPASS, class BLOCK_P>
class BlockFixedWingStabilization :
    public Block
{
private:
    BLOCK_LOWPASS _pLowPass;
    BLOCK_LOWPASS _qLowPass;
    BLOCK_LOWPASS _rLowPass;
    BLOCK_HIGHPASS _rWashout;
    BLOCK_P _p2Ail;
    BLOCK_P _q2Elv;
    BLOCK_P _r2Rdr;
    float _aileronCmd;
    float _elevatorCmd;
    float _rudderCmd;
public:
    BlockFixedWingStabilization(const char * name, Block * parent) :
        Block(name, parent),
        _pLowPass("P_LP", this),
        _qLowPass("Q_LP", this),
        _rLowPass("R_LP", this),
        _rWashout("R_HP", this),
        _p2Ail("P2AIL", this),
        _q2Elv("Q2ELV", this),
        _r2Rdr("R2RDR", this),
        _aileronCmd(0),
        _elevatorCmd(0),
        _rudderCmd(0)
    {
    }
    virtual ~BlockFixedWingStabilization() {};
    void update(
            float pCmd,
            float qCmd,
            float rCmd,
            float p,
            float q,
            float r,
            uint16_t dt)
    {
        _aileronCmd = _p2Ail.update(pCmd - _pLowPass.update(p,dt), dt);
        _elevatorCmd = _q2Elv.update(qCmd - _qLowPass.update(q,dt), dt);
        _rudderCmd = _r2Rdr.update(rCmd -
                _rWashout.update(_rLowPass.update(r,dt),dt), dt);
    }
    float getAileronCmd() {return _aileronCmd;}
    float getElevatorCmd() {return _elevatorCmd;}
    float getRudderCmd() {return _rudderCmd;}
};

} // namespace control

