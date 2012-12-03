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

/**
 * Stability augmentation system.
 * Aircraft Control and Simulation, Stevens and Lewis, pg. 292, 299
 */
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
    void update(float pCmd, float qCmd, float rCmd,
            float p, float q, float r)
    {
        _aileronCmd = _p2Ail.update(pCmd - _pLowPass.update(p));
        _elevatorCmd = _q2Elv.update(qCmd - _qLowPass.update(q));
        _rudderCmd = _r2Rdr.update(rCmd -
                _rWashout.update(_rLowPass.update(r)));
    }
    float getAileronCmd() {return _aileronCmd;}
    float getElevatorCmd() {return _elevatorCmd;}
    float getRudderCmd() {return _rudderCmd;}
};

/**
 * Heading hold autopilot block.
 * Aircraft Control and Simulation, Stevens and Lewis, pg. 348
 */
template<class BLOCK_P, class BLOCK_LIMIT>
class BlockFixedWingHeadingHold :
    public Block
{
private:
    BLOCK_P _psi2Phi;
    BLOCK_P _p2Phi;
    BLOCK_P _phi2Ail;
    BLOCK_LIMIT _phiLimit;
    float _aileronCmd;
public:
    BlockFixedWingHeadingHold(const char * name, Block * parent) :
        Block(name, parent),
        _psi2Phi("PSI2PHI",this),
        _p2Phi("P2PHI",this),
        _phi2Ail("PHI2AIL",this),
        _phiLimit("PHI_LIM",this),
        _aileronCmd(0)
    {
    }
    virtual ~BlockFixedWingHeadingHold() {};
    void update(float psiCmd, float phi, float psi, float p)
    {
        float psiError = psiCmd - psi;
        float phiCmd = _phiLimit.update(_psi2Phi.update(psiError));
        float phiError = phiCmd - phi;
        _aileronCmd = _phi2Ail.update(phiError - _p2Phi.update(p));
    }
    float getAileronCmd() {return _aileronCmd;}
};

/**
 * Backside velocity hold autopilot block.
 */
template<class BLOCK_PID>
class BlockFixedWingVelocityHoldBackside :
    public Block
{
private:
    BLOCK_PID _v2Elv;
    float _elevatorCmd;
public:
    BlockFixedWingVelocityHoldBackside(const char * name, Block * parent) :
        Block(name, parent),
        _v2Elv("V2ELV",this),
        _elevatorCmd(0)
    {
    }
    virtual ~BlockFixedWingVelocityHoldBackside() {};
    void update(float vCmd, float v)
    {
        _elevatorCmd = _v2Elv.update(vCmd - v);
    }
    float getElevatorCmd() {return _elevatorCmd;}
};

/**
 * Frontside velocity hold autopilot block.
 */
template<class BLOCK_PID>
class BlockFixedWingVelocityHoldFrontside :
    public Block
{
private:
    BLOCK_PID _v2Thr;
    float _thrCmd;
public:
    BlockFixedWingVelocityHoldFrontside(const char * name, Block * parent) :
        Block(name, parent),
        _v2Thr("V2THR",this),
        _thrCmd(0)
    {
    }
    virtual ~BlockFixedWingVelocityHoldFrontside() {};
    void update(float vCmd, float v)
    {
        _thrCmd = _v2Thr.update(vCmd - v);
    }
    float getThrCmd() {return _thrCmd;}
};





} // namespace control

