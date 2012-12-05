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
    BlockOutput _rudder;
public:
    BlockYawDamper(Block * parent, const char * name) :
        Block(parent, name),
        _rLowPass(this, "LP"),
        _rWashout(this, "HP"),
        _r2Rdr(this, "2RDR"),
        _rudder(this,"RDR")
    {
    }
    virtual ~BlockYawDamper() {};
    void update(float rCmd, float r)
    {
        _rudder.update(_r2Rdr.update(rCmd -
                _rWashout.update(_rLowPass.update(r))));
    }
    float getRudder() { return _rudder.get(); }
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
    BlockOutput _aileron;
    BlockOutput _elevator;
public:
    BlockStabilization(Block * parent, const char * name) :
        Block(parent, name),
        _yawDamper(this, "R"),
        _pLowPass(this, "P_LP"),
        _qLowPass(this, "Q_LP"),
        _p2Ail(this, "P_2AIL"),
        _q2Elv(this, "Q_2ELV"),
        _aileron(this,"AIL"),
        _elevator(this,"ELV")
    {
    }
    virtual ~BlockStabilization() {};
    void update(float pCmd, float qCmd, float rCmd,
            float p, float q, float r)
    {
        _aileron.update(_p2Ail.update(
                    pCmd - _pLowPass.update(p)));
        _elevator.update(_q2Elv.update(
                    qCmd - _qLowPass.update(q)));
        _yawDamper.update(rCmd, r);
    }
    float getAileron() {return _aileron.get();}
    float getElevator() {return _elevator.get();}
    float getRudder() {return _yawDamper.getRudder();}
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
    BlockOutput _aileron;
public:
    BlockHeadingHold(Block * parent, const char * name) :
        Block(parent, name),
        _psi2Phi(this, "PSI_2PHI"),
        _p2Phi(this, "P_2PHI"),
        _phi2Ail(this, "PHI_2AIL"),
        _phiLimit(this, "PHI_LIM"),
        _aileron(this, "AIL")
    {
    }
    virtual ~BlockHeadingHold() {};
    void update(float psiCmd, float phi, float psi, float p)
    {
        float psiError = psiCmd - psi;
        float phiCmd = _phiLimit.update(_psi2Phi.update(psiError));
        float phiError = phiCmd - phi;
        _aileron.update(_phi2Ail.update(phiError - _p2Phi.update(p)));
    }
    float getAileron() {return _aileron.get();}
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
    BlockOutput _elevator;
public:
    BlockVelocityHoldBackside(Block * parent, const char * name) :
        Block(parent, name),
        _v2Theta(this,"V_2THETA"),
        _theta2Q(this,"THETA_2Q"),
        _elevator(this,"ELV")
    {
    }
    virtual ~BlockVelocityHoldBackside() {};
    void update(float vCmd, float v, float theta, float q)
    {
        float thetaCmd = _v2Theta.update(vCmd - v);
        float qCmd = _theta2Q.update(thetaCmd - theta);
        _elevator.update(qCmd - q);
    }
    float getElevator() {return _elevator.get();}
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
    BlockOutput _throttle;
public:
    BlockVelocityHoldFrontside(Block * parent, const char * name) :
        Block(parent, name),
        _v2Thr(this,"V_2THR"),
        _throttle(this,"THR")
    {
    }
    virtual ~BlockVelocityHoldFrontside() {};
    void update(float vCmd, float v)
    {
        _throttle.update(_v2Thr.update(vCmd - v));
    }
    float getThrottle() {return _throttle.get();}
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
    BlockOutput _throttle;
public:
    BlockAltitudeHoldBackside(Block * parent, const char * name) :
        Block(parent, name),
        _h2Thr(this, "H_2THR"),
        _throttle(this, "THR")
    {
    }
    virtual ~BlockAltitudeHoldBackside() {};
    void update(float hCmd, float h)
    {
        _throttle.update(_h2Thr.update(hCmd - h));
    }
    float getThrottle() {return _throttle.get();}
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
    BlockOutput _elevator;
public:
    BlockAltitudeHoldFrontside(Block * parent, const char * name) :
        Block(parent, name),
        _h2Theta(this, "H_2THETA"),
        _theta2Q(this, "THETA_2Q"),
        _elevator(this, "ELV")
    {
    }
    virtual ~BlockAltitudeHoldFrontside() {};
    void update(float hCmd, float h, float theta, float q)
    {
        float thetaCmd = _h2Theta.update(hCmd - h);
        float qCmd = _theta2Q.update(thetaCmd - theta);
        _elevator.update(qCmd - q);
    }
    float getElevatorCmd() {return _elevator.get();}
};

/**
 * Backside autopilot
 */
class BlockBacksideAutopilot : public Block
{
private:
    BlockYawDamper _yawDamper;
    BlockHeadingHold _headingHold;
    BlockVelocityHoldBackside _velocityHold;
    BlockAltitudeHoldBackside _altitudeHold;
public:
    BlockBacksideAutopilot(Block * parent, const char * name) :
        Block(parent, name),
        _yawDamper(this,"R"),
        _headingHold(this,"PSI"),
        _velocityHold(this,"V"),
        _altitudeHold(this,"H")
    {
    }
    virtual ~BlockBacksideAutopilot() {};
    void update(float hCmd, float vCmd, float rCmd, float psiCmd,
            float h, float v,
            float phi, float theta, float psi,
            float p, float q, float r)
    {
        _yawDamper.update(rCmd, r);
        _headingHold.update(psiCmd, phi, psi, p);
        _velocityHold.update(vCmd, v, theta, q);
        _altitudeHold.update(hCmd, h);
    };
    float getRudder() {return _yawDamper.getRudder();}
    float getAileron() {return _headingHold.getAileron();}
    float getElevator() {return _velocityHold.getElevator();}
    float getThrottle() {return _altitudeHold.getThrottle();}
};

} // namespace fixedwing

} // namespace control

