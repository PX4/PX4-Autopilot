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

/* An block which can be used to limit the output */
class BlockOutputLimiter: public SuperBlock
{
public:
// methods
	BlockOutputLimiter(SuperBlock *parent, const char *name, bool fAngularLimit = false) :
		SuperBlock(parent, name),
		_isAngularLimit(fAngularLimit),
		_min(this, "MIN"),
		_max(this, "MAX")
	{};
	virtual ~BlockOutputLimiter() {};
	/*
	 * Imposes the limits given by _min and _max on value
	 *
	 * @param value is changed to be on the interval _min to _max
	 * @param difference if the value is changed this corresponds to the change of value * (-1)
     * otherwise unchanged
	 * @return: true if the limit is applied, false otherwise
	 */
	bool limit(float& value, float& difference) {
		float minimum = getIsAngularLimit() ? getMin() * M_DEG_TO_RAD_F : getMin();
		float maximum = getIsAngularLimit() ? getMax() * M_DEG_TO_RAD_F : getMax();
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
	bool getIsAngularLimit() {return _isAngularLimit ;}
	float getMin() { return _min.get(); }
	float getMax() { return _max.get(); }
	void setMin(float value) { _min.set(value); }
	void setMax(float value) { _max.set(value); }
protected:
//attributes
	bool _isAngularLimit;
	control::BlockParamFloat _min;
	control::BlockParamFloat _max;
};


/* A combination of feed forward, P and I gain using the output limiter*/
class BlockFFPILimited: public SuperBlock
{
public:
// methods
	BlockFFPILimited(SuperBlock *parent, const char *name, bool isAngularLimit = false) :
		SuperBlock(parent, name),
		_outputLimiter(this, "", isAngularLimit),
		_integral(this, "I"),
		_kFF(this, "FF"),
		_kP(this, "P"),
		_kI(this, "I"),
		_offset(this, "OFF")
	{};
	virtual ~BlockFFPILimited() {};
	float update(float inputValue, float inputError) { return calcLimitedOutput(inputValue, inputError, _outputLimiter); }
// accessors
	BlockIntegral &getIntegral() { return _integral; }
	float getKFF() { return _kFF.get(); }
	float getKP() { return _kP.get(); }
	float getKI() { return _kI.get(); }
	float getOffset() { return _offset.get(); }
	BlockOutputLimiter &getOutputLimiter() { return _outputLimiter; };
protected:
	BlockOutputLimiter _outputLimiter;

	float calcUnlimitedOutput(float inputValue, float inputError) {return getOffset() + getKFF() * inputValue + getKP() * inputError + getKI() * getIntegral().update(inputError);}
	float calcLimitedOutput(float inputValue, float inputError, BlockOutputLimiter &outputLimiter) {
		float difference = 0.0f;
		float integralYPrevious = _integral.getY();
		float output = calcUnlimitedOutput(inputValue, inputError);
		if(outputLimiter.limit(output, difference) &&
			(((difference < 0) && (getKI() * getIntegral().getY() < 0)) ||
			((difference > 0) && (getKI() * getIntegral().getY() > 0)))) {
				getIntegral().setY(integralYPrevious);
		}
		return output;
	}
private:
	BlockIntegral _integral;
	BlockParamFloat _kFF;
	BlockParamFloat _kP;
	BlockParamFloat _kI;
	BlockParamFloat _offset;
};

/* A combination of feed forward, P and I gain using the output limiter with the option to provide a special output limiter (for example for takeoff)*/
class BlockFFPILimitedCustom: public BlockFFPILimited
{
public:
// methods
	BlockFFPILimitedCustom(SuperBlock *parent, const char *name, bool isAngularLimit = false) :
		BlockFFPILimited(parent, name, isAngularLimit)
		{};
	virtual ~BlockFFPILimitedCustom() {};
	float update(float inputValue, float inputError, BlockOutputLimiter *outputLimiter = NULL) {
		return calcLimitedOutput(inputValue, inputError, outputLimiter == NULL ? _outputLimiter : *outputLimiter);
	}
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
		getOutputLimiter().limit(output, difference);
		return output;
	}
// accessors
	BlockOutputLimiter &getOutputLimiter() { return _outputLimiter; };
	float getKP() { return _kP.get(); }
private:
	control::BlockParamFloat _kP;
	BlockOutputLimiter _outputLimiter;
};

/* A combination of P, D gains and output limiter */
class BlockPDLimited: public SuperBlock
{
public:
// methods
	BlockPDLimited(SuperBlock *parent, const char *name, bool isAngularLimit = false) :
		SuperBlock(parent, name),
		_kP(this, "P"),
		_kD(this, "D"),
		_derivative(this, "D"),
		_outputLimiter(this, "", isAngularLimit)
	{};
	virtual ~BlockPDLimited() {};
	float update(float input) {
		float difference = 0.0f;
		float output = getKP() * input + (getDerivative().getDt() > 0.0f ? getKD() * getDerivative().update(input) : 0.0f);
		getOutputLimiter().limit(output, difference);

		return output;
	}
// accessors
	float getKP() { return _kP.get(); }
	float getKD() { return _kD.get(); }
	BlockDerivative &getDerivative() { return _derivative; }
	BlockOutputLimiter &getOutputLimiter() { return _outputLimiter; };
private:
	control::BlockParamFloat _kP;
	control::BlockParamFloat _kD;
	BlockDerivative _derivative;
	BlockOutputLimiter _outputLimiter;
};

}

