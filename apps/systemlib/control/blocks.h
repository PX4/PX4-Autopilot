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
 * @file control.h
 *
 * Controller library code
 */

#pragma once

#include "block/Block.h"
#include "block/BlockParam.h"

namespace control
{

/**
 * A limiter/ saturation.
 * The output of update is the input, bounded
 * by min/max.
 */
class __EXPORT BlockLimit : public Block
{
public:
// methods
    BlockLimit(Block * parent, const char * name) : 
        Block(parent, name),
        _min(this,"MIN"),
        _max(this,"MAX")
    {};
    virtual ~BlockLimit() {};
    float update(float input);
// accessors
    float getMin() { return _min.get(); }
    float getMax() { return _max.get(); }
protected:
// accessors
    void setMin(float min) { _min.set(min); }
    void setMax(float max) { _max.set(max); }
// attributes
    BlockParam<float> _min;
    BlockParam<float> _max;
};

/**
 * A low pass filter as described here: 
 * http://en.wikipedia.org/wiki/Low-pass_filter.
 */
class __EXPORT BlockLowPass : public Block
{
public:
// methods
    BlockLowPass(Block * parent, const char * name) :
        Block(parent, name),
        _state(0),
        _fCut(this,"FCUT")
    {};
    virtual ~BlockLowPass() {};
    float update(float input);
// accessors
    float getState() { return _state; }
    float getFCut() { return _fCut.get(); }
    void setState(float state) { _state = state; }
protected:
// accessors
    void setFCut(float fCut) { _fCut.set(fCut); }
// attributes
    float _state;
    BlockParam<float> _fCut;
};

/**
 * A high pass filter as described here: 
 * http://en.wikipedia.org/wiki/High-pass_filter.
 */
class __EXPORT BlockHighPass : public Block
{
public:
// methods
    BlockHighPass(Block * parent, const char * name) :
        Block(parent, name),
        _state(0),
        _fCut(this,"FCUT")
    {};
    virtual ~BlockHighPass() {};
    float update(float input, float dt);
// accessors
    float getState() {return _state;}
    float getFCut() {return _fCut.get();}
    void setState(float state) {_state = state;}
protected:
// accessors
    void setFCut(float fCut) { _fCut.set(fCut);}
// attributes
    float _state; /**< previous output */
    BlockParam<float> _fCut; /**< cut-off frequency, Hz */
};

/**
 * A trapezoidal integrator.
 * http://en.wikipedia.org/wiki/Trapezoidal_rule
 * A limiter is built into the class to bound the
 * integral's internal state. This is important
 * for windup protection.
 * @see Limit
 */
class __EXPORT BlockIntegral : public Block
{
public: 
// methods
   BlockIntegral(Block * parent, const char * name) :
        Block(parent, name),
        _state(0),
        _limit(this, "") {};
    virtual ~BlockIntegral() {};
    float update(float input);
// accessors
    float getState() {return _state;}
    float getMin() {return _limit.getMin();}
    float getMax() {return _limit.getMax();}
    void setState(float state) {_state = state;}
protected:
// attributes
    float _state; /**< previous output */
    BlockLimit _limit; /**< limiter */
};

/**
 * A simple derivative approximation.
 * This uses the previous and current input.
 * This has a built in low pass filter.
 * @see LowPass
 */
class __EXPORT BlockDerivative : public Block
{
public:
// methods
   BlockDerivative(Block * parent, const char * name) :
        Block(parent, name),
        _state(0),
        _lowPass(this, "")
    {};
    virtual ~BlockDerivative() {};
    float update(float input);
// accessors
    void setState(float state) {_state = state;}
    float getState() {return _state;}
    float getFCut() {return _lowPass.getFCut();}
protected:
// attributes
    float _state; /**< previous input */
    BlockLowPass _lowPass; /**< low pass filter */
};

/**
 * A proportional controller.
 * @link http://en.wikipedia.org/wiki/PID_controller
 */
class __EXPORT BlockP: public Block
{
public:
// methods
    BlockP(Block * parent, const char * name) :
        Block(parent, name),
        _kP(this,"P")
    {};
    virtual ~BlockP() {};
    float update(float input)
    {
        return getKP()*input;
    }
// accessors
    float getKP() { return _kP.get(); }
protected:
    void setKP(float kP) { _kP.set(kP); }
    BlockParam<float> _kP;
};

/**
 * A proportional-integral controller.
 * @link http://en.wikipedia.org/wiki/PID_controller
 */
class __EXPORT BlockPI: public Block
{
public:
// methods
    BlockPI(Block * parent, const char * name) :
        Block(parent, name),
        _integral(this,"I"),
        _kP(this,"P"),
        _kI(this,"I")
    {};
    virtual ~BlockPI() {};
    float update(float input)
    {
        return getKP()*input + 
            getKI()*getIntegral().update(input);
    }
// accessors
    float getKP() { return _kP.get(); }
    float getKI() { return _kI.get(); }
    BlockIntegral & getIntegral() { return _integral; }
private:
    void setKP(float kP) { _kP.set(kP); }
    void setKI(float kI) { _kI.set(kI); }
    BlockIntegral _integral;
    BlockParam<float> _kP;
    BlockParam<float> _kI;
};

/**
 * A proportional-derivative controller.
 * @link http://en.wikipedia.org/wiki/PID_controller
 */
class __EXPORT BlockPD: public Block
{
public:
// methods
    BlockPD(Block * parent, const char * name) :
        Block(parent, name),
        _derivative(this,"D"),
        _kP(this,"P"),
        _kD(this,"D")
    {};
    virtual ~BlockPD() {};
    float update(float input)
    {
        return getKP()*input + 
            getKD()*getDerivative().update(input);
    }
// accessors
    float getKP() { return _kP.get(); }
    float getKD() { return _kD.get(); }
    BlockDerivative & getDerivative() { return _derivative; }
private:
    void setKP(float kP) { _kP.set(kP); }
    void setKD(float kD) { _kD.set(kD); }
    BlockDerivative _derivative;
    BlockParam<float> _kP;
    BlockParam<float> _kD;
};

/**
 * A proportional-integral-derivative controller.
 * @link http://en.wikipedia.org/wiki/PID_controller
 */
class __EXPORT BlockPID: public Block
{
public:
// methods
    BlockPID(Block * parent, const char * name) :
        Block(parent, name),
        _integral(this,"I"),
        _derivative(this,"D"),
        _kP(this,"P"),
        _kI(this,"I"),
        _kD(this,"D")
    {};
    virtual ~BlockPID() {};
    float update(float input)
    {
        return getKP()*input + 
            getKI()*getIntegral().update(input) +
            getKD()*getDerivative().update(input);
    }
// accessors
    float getKP() { return _kP.get(); }
    float getKI() { return _kI.get(); }
    float getKD() { return _kD.get(); }
    BlockIntegral & getIntegral() { return _integral; }
    BlockDerivative & getDerivative() { return _derivative; }
private:
// accessors
    void setKP(float kP) { _kP.set(kP); }
    void setKI(float kI) { _kI.set(kI); }
    void setKD(float kD) { _kD.set(kD); }
// attributes
    BlockIntegral _integral;
    BlockDerivative _derivative;
    BlockParam<float> _kP;
    BlockParam<float> _kI;
    BlockParam<float> _kD;
};

} // namespace control
