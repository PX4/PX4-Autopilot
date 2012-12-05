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

namespace fixedwing
{

/**
 * Yaw damper
 */
template<class BLOCK_LOWPASS, class BLOCK_HIGHPASS, class BLOCK_P>
class BlockYawDamper :
    public Block
{
private:
    BLOCK_LOWPASS _rLowPass;
    BLOCK_HIGHPASS _rWashout;
    BLOCK_P _r2Rdr;
public:
    BlockYawDamper(const char * name, Block * parent) :
        Block(name, parent),
        _rLowPass("LP", this),
        _rWashout("HP", this),
        _r2Rdr("2RDR", this)
    {
    }
    virtual ~BlockYawDamper() {};
    float update(float rCmd, float r)
    {
        return _r2Rdr.update(rCmd -
                _rWashout.update(_rLowPass.update(r)));
    }
};

/**
 * Stability augmentation system.
 * Aircraft Control and Simulation, Stevens and Lewis, pg. 292, 299
 */
template<class BLOCK_LOWPASS, class BLOCK_HIGHPASS, class BLOCK_P>
class BlockStabilization :
    public Block
{
private:
    BlockYawDamper<BLOCK_LOWPASS, BLOCK_HIGHPASS, BLOCK_P> _yawDamper;
    BLOCK_LOWPASS _pLowPass;
    BLOCK_LOWPASS _qLowPass;
    BLOCK_P _p2Ail;
    BLOCK_P _q2Elv;
    float _aileronCmd;
    float _elevatorCmd;
    float _rudderCmd;
public:
    BlockStabilization(const char * name, Block * parent) :
        Block(name, parent),
        _yawDamper("R", this),
        _pLowPass("P_LP", this),
        _qLowPass("Q_LP", this),
        _p2Ail("P_2AIL", this),
        _q2Elv("Q_2ELV", this),
        _aileronCmd(0),
        _elevatorCmd(0),
        _rudderCmd(0)
    {
    }
    virtual ~BlockStabilization() {};
    void update(float pCmd, float qCmd, float rCmd,
            float p, float q, float r)
    {
        _aileronCmd = _p2Ail.update(pCmd - _pLowPass.update(p));
        _elevatorCmd = _q2Elv.update(qCmd - _qLowPass.update(q));
        _rudderCmd = _yawDamper.update(rCmd,r);
    }
    float getAileronCmd() {return _aileronCmd;}
    float getElevatorCmd() {return _elevatorCmd;}
    float getRudderCmd() {return _rudderCmd;}
};

/**
 * Heading hold autopilot block.
 * Aircraft Control and Simulation, Stevens and Lewis
 * Heading hold, pg. 348
 */
template<class BLOCK_P, class BLOCK_LIMIT>
class BlockHeadingHold :
    public Block
{
private:
    BLOCK_P _psi2Phi;
    BLOCK_P _p2Phi;
    BLOCK_P _phi2Ail;
    BLOCK_LIMIT _phiLimit;
    float _aileronCmd;
public:
    BlockHeadingHold(const char * name, Block * parent) :
        Block(name, parent),
        _psi2Phi("PSI_2PHI",this),
        _p2Phi("P_2PHI",this),
        _phi2Ail("PHI_2AIL",this),
        _phiLimit("PHI_LIM",this),
        _aileronCmd(0)
    {
    }
    virtual ~BlockHeadingHold() {};
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
 * Frontside/ Backside Control Systems
 *
 * Frontside : 
 *   velocity error -> throttle
 *   altitude error -> elevator
 *
 * Backside :
 *   velocity error -> elevator
 *   altitude error -> throttle
 *
 * Backside control systems are more resilient at 
 * slow speeds on the back-side of the power 
 * required curve/ landing etc. Less performance
 * than frontside at high speeds.
 */

/**
 * Backside velocity hold autopilot block.
 * v -> theta -> q -> elevator
 */
template<class BLOCK_PID>
class BlockVelocityHoldBackside :
    public Block
{
private:
    BLOCK_PID _v2Theta;
    BLOCK_PID _theta2Q;
    float _elevatorCmd;
public:
    BlockVelocityHoldBackside(const char * name, Block * parent) :
        Block(name, parent),
        _v2Theta("V_2THETA",this),
        _theta2Q("THETA_2Q",this),
        _elevatorCmd(0)
    {
    }
    virtual ~BlockVelocityHoldBackside() {};
    void update(float vCmd, float v, float theta, float q)
    {
        float thetaCmd = _v2Theta.update(vCmd - v);
        float qCmd = _theta2Q.update(thetaCmd - theta);
        _elevatorCmd = qCmd - q;
    }
    float getElevatorCmd() {return _elevatorCmd;}
};

/**
 * Frontside velocity hold autopilot block.
 * v -> throttle
 */
template<class BLOCK_PID>
class BlockVelocityHoldFrontside :
    public Block
{
private:
    BLOCK_PID _v2Thr;
    float _thrCmd;
public:
    BlockVelocityHoldFrontside(const char * name, Block * parent) :
        Block(name, parent),
        _v2Thr("V_2THR",this),
        _thrCmd(0)
    {
    }
    virtual ~BlockVelocityHoldFrontside() {};
    void update(float vCmd, float v)
    {
        _thrCmd = _v2Thr.update(vCmd - v);
    }
    float getThrCmd() {return _thrCmd;}
};

/**
 * Backside altitude hold autopilot block.
 * h -> throttle
 */
template<class BLOCK_PID>
class BlockAltitudeHoldBackside :
    public Block
{
private:
    BLOCK_PID _h2Thr;
    float _thrCmd;
public:
    BlockAltitudeHoldBackside(const char * name, Block * parent) :
        Block(name, parent),
        _h2Thr("H_2THR",this),
        _thrCmd(0)
    {
    }
    virtual ~BlockAltitudeHoldBackside() {};
    void update(float hCmd, float h)
    {
        _thrCmd = _h2Thr.update(hCmd - h);
    }
    float getThrCmd() {return _thrCmd;}
};

/**
 * Frontside altitude hold autopilot block.
 * h -> theta > q -> elevator
 */
template<class BLOCK_PID>
class BlockAltitudeHoldFrontside :
    public Block
{
private:
    BLOCK_PID _h2Theta;
    BLOCK_PID _theta2Q;
    float _elevatorCmd;
public:
    BlockAltitudeHoldFrontside(const char * name, Block * parent) :
        Block(name, parent),
        _h2Theta("H_2THETA",this),
        _theta2Q("THETA_2Q",this),
        _elevatorCmd(0)
    {
    }
    virtual ~BlockAltitudeHoldFrontside() {};
    void update(float hCmd, float h, float theta, float q)
    {
        float thetaCmd = _h2Theta.update(hCmd - h);
        float qCmd = _theta2Q(thetaCmd - theta);
        _elevatorCmd = qCmd - q;
    }
    float getElevatorCmd() {return _elevatorCmd;}
};

/**
 * Backside autopilot
 */
class BacksideAutopilot
{
public:
private:
};

} // namespace fixedwing

} // namespace control

