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

#include "blocks.h"

namespace control
{

namespace fixedwing
{

/**
 * Yaw damper
 */
class BlockYawDamper :
    public Block
{
private:
    BlockLowPass _rLowPass;
    BlockHighPass _rWashout;
    BlockP _r2Rdr;
public:
    BlockYawDamper(Block * parent, const char * name) :
        Block(parent, name),
        _rLowPass(this, "LP"),
        _rWashout(this, "HP"),
        _r2Rdr(this, "2RDR")
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
class BlockStabilization :
    public Block
{
private:
    BlockYawDamper _yawDamper;
    BlockLowPass _pLowPass;
    BlockLowPass _qLowPass;
    BlockP _p2Ail;
    BlockP _q2Elv;
    float _aileronCmd;
    float _elevatorCmd;
    float _rudderCmd;
public:
    BlockStabilization(Block * parent, const char * name) :
        Block(parent, name),
        _yawDamper(this, "R"),
        _pLowPass(this, "P_LP"),
        _qLowPass(this, "Q_LP"),
        _p2Ail(this, "P_2AIL"),
        _q2Elv(this, "Q_2ELV"),
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
class BlockHeadingHold :
    public Block
{
private:
    BlockP _psi2Phi;
    BlockP _p2Phi;
    BlockP _phi2Ail;
    BlockLimit _phiLimit;
    float _aileronCmd;
public:
    BlockHeadingHold(Block * parent, const char * name) :
        Block(parent, name),
        _psi2Phi(this, "PSI_2PHI"),
        _p2Phi(this, "P_2PHI"),
        _phi2Ail(this, "PHI_2AIL"),
        _phiLimit(this, "PHI_LIM"),
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
class BlockVelocityHoldBackside :
    public Block
{
private:
    BlockPID _v2Theta;
    BlockPID _theta2Q;
    float _elevatorCmd;
public:
    BlockVelocityHoldBackside(Block * parent, const char * name) :
        Block(parent, name),
        _v2Theta(this,"V_2THETA"),
        _theta2Q(this,"THETA_2Q"),
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
class BlockVelocityHoldFrontside :
    public Block
{
private:
    BlockPID _v2Thr;
    float _thrCmd;
public:
    BlockVelocityHoldFrontside(Block * parent, const char * name) :
        Block(parent, name),
        _v2Thr(this,"V_2THR"),
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
class BlockAltitudeHoldBackside :
    public Block
{
private:
    BlockPID _h2Thr;
    float _thrCmd;
public:
    BlockAltitudeHoldBackside(Block * parent, const char * name) :
        Block(parent, name),
        _h2Thr(this, "H_2THR"),
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
class BlockAltitudeHoldFrontside :
    public Block
{
private:
    BlockPID _h2Theta;
    BlockPID _theta2Q;
    float _elevatorCmd;
public:
    BlockAltitudeHoldFrontside(Block * parent, const char * name) :
        Block(parent, name),
        _h2Theta(this, "H_2THETA"),
        _theta2Q(this, "THETA_2Q"),
        _elevatorCmd(0)
    {
    }
    virtual ~BlockAltitudeHoldFrontside() {};
    void update(float hCmd, float h, float theta, float q)
    {
        float thetaCmd = _h2Theta.update(hCmd - h);
        float qCmd = _theta2Q.update(thetaCmd - theta);
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

