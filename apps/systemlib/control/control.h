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

namespace control
{

/**
 * A limiter/ saturation.
 * The output of update is the input, bounded
 * by min/max.
 */
class Limit
{
public:
    Limit();
    virtual ~Limit();
    float update(float input, uint16_t dt);
    void setMin(float min) {_min=min;}
    float getMin() {return _min;}
    void setMax(float max) {_max=max;}
    float getMax() {return _max;}
private:
    float _min; /**< output minimum */
    float _max; /**< output maximum */
};

/**
 * A low pass filter as described here: 
 * http://en.wikipedia.org/wiki/Low-pass_filter.
 */
class LowPass
{
public:
// methods
    LowPass();
    virtual ~LowPass();
    float update(float input, uint16_t dt);
// accessors
    void setState(float state) {_state = state;}
    float getState() {return _state;}
    void setCutFreq(float cutFreq) {_cutFreq=cutFreq;}
    float getCutFreq() {return _cutFreq;}
private:
// attributes
    float _state; /**< previous output */
    float _cutFreq; /**< cut-off frequency, Hz */
};

/**
 * A trapezoidal integrator.
 * http://en.wikipedia.org/wiki/Trapezoidal_rule
 * A limiter is built into the class to bound the
 * integral's internal state. This is important
 * for windup protection.
 * @see Limit
 */
class Integral
{
public: 
// methods
    Integral();
    virtual ~Integral();
    float update(float input, uint16_t dt);
// accessors
    void setState(float state) {_state = state;}
    float getState() {return _state;}
    void setMin(float min) {_limit.setMin(min);}
    float getMin() {return _limit.getMin();}
    void setMax(float max) {_limit.setMax(max);}
    float getMax() {return _limit.getMax();}
private:
// attributes
    float _state; /**< previous output */
    Limit _limit; /**< limiter */
};

/**
 * A simple derivative approximation.
 * This uses the previous and current input.
 * This has a built in low pass filter.
 * @see LowPass
 */
class Derivative
{
public:
// methods
    Derivative();
    virtual ~Derivative();
    float update(float input, uint16_t dt);
// accessors
    void setState(float state) {_state = state;}
    float getState() {return _state;}
    void setCutFreq(float cutFreq) {_lowPass.setCutFreq(cutFreq);}
    float getCutFreq() {return _lowPass.getCutFreq();}
private:
// attributes
    float _state; /**< previous input */
    LowPass _lowPass; /**< low pass filter */
};

/**
 * A proportional-integral-derivative controller.
 * http://en.wikipedia.org/wiki/PID_controller
 * This is built upon the Integral 
 * and Derivative class.
 * @see Integral, Derivative
 */
class PID
{
public:
// methods
    PID();
    virtual ~PID();
    float update(float input, uint16_t dt);
public:
// accessors
    void setKP(float kP) {_kP = kP;}
    float getKP() {return _kP;}
    void setKI(float kI) {_kI = kI;}
    float getKI() {return _kI;}
    void setKD(float kD) {_kD = kD;}
    float getKD() {return _kD;}
    void setIMin(float min) {getIntegral().setMin(min);}
    float getIMin() {return getIntegral().getMin();}
    void setIMax(float max) {getIntegral().setMax(max);}
    float getIMax() {return getIntegral().getMax();}
private:
// accessors
    Integral & getIntegral() {return _integral;}
    Derivative & getDerivative() {return _derivative;}
// attributes
    float _kP; /**< proportional gain */
    float _kI; /**< integral gain */
    float _kD; /**< derivative gain */
    Integral _integral; /**< integral calculator */
    Derivative _derivative; /**< derivative calculator */
};

} // namespace control
