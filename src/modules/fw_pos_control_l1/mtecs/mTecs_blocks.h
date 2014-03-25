/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: 	@author Thomas Gubler <thomasgubler@gmail.com>
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
 * @file mTecs_blocks.h
 *
 * Custom blocks for the mTecs
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */

#pragma once

#include <controllib/blocks.hpp>
#include <systemlib/err.h>

namespace fwPosctrl
{

using namespace control;

/* Integrator without limit */
class BlockIntegralNoLimit: public SuperBlock
{
public:
// methods
	BlockIntegralNoLimit(SuperBlock *parent, const char *name) :
		SuperBlock(parent, name),
		_y(0) {};
	virtual ~BlockIntegralNoLimit() {};
	float update(float input) {
		setY(getY() + input * getDt());
		return getY();
	};
// accessors
	float getY() {return _y;}
	void setY(float y) {_y = y;}
protected:
// attributes
	float _y; /**< previous output */
};

/* An block which can be used to limit the output */
class BlockOutputLimiter: public SuperBlock
{
public:
// methods
	BlockOutputLimiter(SuperBlock *parent, const char *name, bool isAngularLimit = false) :
		SuperBlock(parent, name),
		_isAngularLimit(isAngularLimit),
		_min(this, "MIN"),
		_max(this, "MAX")
	{};
	virtual ~BlockOutputLimiter() {};
	bool limit(float& value, float& difference) {
		float minimum = isAngularLimit() ? getMin() * M_DEG_TO_RAD_F : getMin();
		float maximum = isAngularLimit() ? getMax() * M_DEG_TO_RAD_F : getMax();
		char name[blockNameLengthMax];
		getName(name, blockNameLengthMax);
//		warnx("%s minimum %.2f maximum %.2f, getMin() %.2f, getMax() %.2f, isAngularLimit() %u",
//				 name,(double)minimum,(double)maximum,  (double)getMin(), (double)getMax(), isAngularLimit());
		if (value < minimum) {
			difference = value - minimum;
			value = minimum;
			return true;
		} else if (value > maximum) {
			difference = value - maximum;
			value = maximum;
			return true;
		}
		return false;
	}
//accessor:
	bool isAngularLimit() {return _isAngularLimit ;}
	float getMin() { return _min.get(); }
	float getMax() { return _max.get(); }
protected:
//attributes
	bool _isAngularLimit;
	control::BlockParamFloat _min;
	control::BlockParamFloat _max;
};

typedef

/* A combination of feed forward, P and I gain  using the output limiter*/
class BlockFFPILimited: public SuperBlock
{
public:
// methods
	BlockFFPILimited(SuperBlock *parent, const char *name, bool isAngularLimit = false) :
		SuperBlock(parent, name),
		_integral(this, "I"),
		_kFF(this, "FF"),
		_kP(this, "P"),
		_kI(this, "I"),
		_offset(this, "OFF"),
		_outputLimiter(this, "", isAngularLimit)
	{};
	virtual ~BlockFFPILimited() {};
	float update(float inputValue, float inputError) {
		float difference = 0.0f;
		float integralYPrevious = _integral.getY();
		float output = getOffset() + getKFF() * inputValue + getKP() * inputError + getKI() * getIntegral().update(inputError);
		char name[blockNameLengthMax];
		getName(name, blockNameLengthMax);
//		warnx("%s output %.2f getKFF() %.2f, inputValue  %.2f, getKP() %.2f, getKI() %.2f, getIntegral().getY() %.2f, inputError %.2f getIntegral().getDt() %.2f", name,
//				(double)output, (double)getKFF(), (double)inputValue, (double)getKP(), (double)getKI(), (double)getIntegral().getY(), (double)inputError, (double)getIntegral().getDt());
		if(!getOutputLimiter().limit(output, difference) &&
			(((difference < 0) && (getKI() * getIntegral().update(inputError) < 0)) ||
			((difference > 0) && (getKI() * getIntegral().update(inputError) > 0)))) {
				getIntegral().setY(integralYPrevious);
		}
//		warnx("%s output limited %.2f",
//				 name,(double)output);
		return output;
	}
// accessors
	BlockIntegralNoLimit &getIntegral() { return _integral; }
	float getKFF() { return _kFF.get(); }
	float getKP() { return _kP.get(); }
	float getKI() { return _kI.get(); }
	float getOffset() { return _offset.get(); }
	BlockOutputLimiter &getOutputLimiter() { return _outputLimiter; };
private:
	BlockIntegralNoLimit _integral;
	BlockParamFloat _kFF;
	BlockParamFloat _kP;
	BlockParamFloat _kI;
	BlockParamFloat _offset;
	BlockOutputLimiter _outputLimiter;
};

/* A combination of P gain and output limiter */
class BlockPLimited: public SuperBlock
{
public:
// methods
	BlockPLimited(SuperBlock *parent, const char *name, bool isAngularLimit = false) :
		SuperBlock(parent, name),
		_kP(this, "P"),
		_outputLimiter(this, "", isAngularLimit)
	{};
	virtual ~BlockPLimited() {};
	float update(float input) {
		float difference = 0.0f;
		float output = getKP() * input;
		char name[blockNameLengthMax];
		getName(name, blockNameLengthMax);
//		warnx("%s output %.2f _kP.get() %.2f, input",
//				 name,(double)output, (double)_kP.get(), (double)input);
		getOutputLimiter().limit(output, difference);
//		warnx("%s output limited %.2f",
//				 name,(double)output);
		return output;
	}
// accessors
	BlockOutputLimiter &getOutputLimiter() { return _outputLimiter; };
	float getKP() { return _kP.get(); }
private:
	control::BlockParamFloat _kP;
	BlockOutputLimiter _outputLimiter;
};

}

