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
 * @file blocks.h
 *
 * Controller library code
 */

#pragma once

#include <px4_defines.h>
#include <assert.h>
#include <time.h>
#include <stdlib.h>
#include <math.h>
#include <mathlib/math/test/test.hpp>
#include <mathlib/math/filter/LowPassFilter2p.hpp>

#include "block/Block.hpp"
#include "block/BlockParam.hpp"

#include "matrix/math.hpp"

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
	BlockLimit(SuperBlock *parent, const char *name) :
		Block(parent, name),
		_min(this, "MIN"),
		_max(this, "MAX")
	{};
	virtual ~BlockLimit() {};
	float update(float input);
// accessors
	float getMin() { return _min.get(); }
	float getMax() { return _max.get(); }
protected:
// attributes
	control::BlockParamFloat _min;
	control::BlockParamFloat _max;
};

/**
 * A symmetric limiter/ saturation.
 * Same as limiter but with only a max, is used for
 * upper limit of +max, and lower limit of -max
 */
class __EXPORT BlockLimitSym : public Block
{
public:
// methods
	BlockLimitSym(SuperBlock *parent, const char *name) :
		Block(parent, name),
		_max(this, "MAX")
	{};
	virtual ~BlockLimitSym() {};
	float update(float input);
// accessors
	float getMax() { return _max.get(); }
protected:
// attributes
	control::BlockParamFloat _max;
};

/**
 * A low pass filter as described here:
 * http://en.wikipedia.org/wiki/Low-pass_filter.
 */
class __EXPORT BlockLowPass : public Block
{
public:
// methods
	BlockLowPass(SuperBlock *parent, const char *name) :
		Block(parent, name),
		_state(0.0f / 0.0f /* initialize to invalid val, force into is_finite() check on first call */),
		_fCut(this, "") // only one parameter, no need to name
	{};
	virtual ~BlockLowPass() {};
	float update(float input);
// accessors
	float getState() { return _state; }
	float getFCut() { return _fCut.get(); }
	void setState(float state) { _state = state; }
protected:
// attributes
	float _state;
	control::BlockParamFloat _fCut;
};

template<class Type, size_t M>
class __EXPORT BlockLowPassVector: public Block
{
public:
// methods
	BlockLowPassVector(SuperBlock *parent,
			   const char *name) :
		Block(parent, name),
		_state(),
		_fCut(this, "") // only one parameter, no need to name
	{
		for (int i = 0; i < M; i++) {
			_state(i) = 0.0f / 0.0f;
		}
	};
	virtual ~BlockLowPassVector() {};
	matrix::Vector<Type, M> update(const matrix::Matrix<Type, M, 1> &input)
	{
		for (int i = 0; i < M; i++) {
			if (!PX4_ISFINITE(getState()(i))) {
				setState(input);
			}
		}

		float b = 2 * float(M_PI) * getFCut() * getDt();
		float a = b / (1 + b);
		setState(input * a + getState() * (1 - a));
		return getState();
	}
// accessors
	matrix::Vector<Type, M> getState() { return _state; }
	float getFCut() { return _fCut.get(); }
	void setState(const matrix::Vector<Type, M> &state) { _state = state; }
private:
// attributes
	matrix::Vector<Type, M> _state;
	control::BlockParamFloat _fCut;
};

/**
 * A high pass filter as described here:
 * http://en.wikipedia.org/wiki/High-pass_filter.
 */
class __EXPORT BlockHighPass : public Block
{
public:
// methods
	BlockHighPass(SuperBlock *parent, const char *name) :
		Block(parent, name),
		_u(0),
		_y(0),
		_fCut(this, "") // only one parameter, no need to name
	{};
	virtual ~BlockHighPass() {};
	float update(float input);
// accessors
	float getU() {return _u;}
	float getY() {return _y;}
	float getFCut() {return _fCut.get();}
	void setU(float u) {_u = u;}
	void setY(float y) {_y = y;}
protected:
// attributes
	float _u; /**< previous input */
	float _y; /**< previous output */
	control::BlockParamFloat _fCut; /**< cut-off frequency, Hz */
};

/**
 * A 2nd order low pass filter block which uses the default px4 2nd order low pass filter
 */
class __EXPORT BlockLowPass2 : public Block
{
public:
// methods
	BlockLowPass2(SuperBlock *parent, const char *name, float sample_freq) :
		Block(parent, name),
		_state(0.0 / 0.0 /* initialize to invalid val, force into is_finite() check on first call */),
		_fCut(this, ""), // only one parameter, no need to name
		_fs(sample_freq),
		_lp(_fs, _fCut.get())
	{};
	virtual ~BlockLowPass2() {};
	float update(float input);
// accessors
	float getState() { return _state; }
	float getFCutParam() { return _fCut.get(); }
	void setState(float state) { _state = _lp.reset(state); }
protected:
// attributes
	float _state;
	control::BlockParamFloat _fCut;
	float _fs;
	math::LowPassFilter2p _lp;
};

/**
 * A rectangular integrator.
 * A limiter is built into the class to bound the
 * integral's internal state. This is important
 * for windup protection.
 * @see Limit
 */
class __EXPORT BlockIntegral: public SuperBlock
{
public:
// methods
	BlockIntegral(SuperBlock *parent, const char *name) :
		SuperBlock(parent, name),
		_y(0),
		_limit(this, "") {};
	virtual ~BlockIntegral() {};
	float update(float input);
// accessors
	float getY() {return _y;}
	float getMax() {return _limit.getMax();}
	void setY(float y) {_y = y;}
protected:
// attributes
	float _y; /**< previous output */
	BlockLimitSym _limit; /**< limiter */
};

/**
 * A trapezoidal integrator.
 * http://en.wikipedia.org/wiki/Trapezoidal_rule
 * A limiter is built into the class to bound the
 * integral's internal state. This is important
 * for windup protection.
 * @see Limit
 */
class __EXPORT BlockIntegralTrap : public SuperBlock
{
public:
// methods
	BlockIntegralTrap(SuperBlock *parent, const char *name) :
		SuperBlock(parent, name),
		_u(0),
		_y(0),
		_limit(this, "") {};
	virtual ~BlockIntegralTrap() {};
	float update(float input);
// accessors
	float getU() {return _u;}
	float getY() {return _y;}
	float getMax() {return _limit.getMax();}
	void setU(float u) {_u = u;}
	void setY(float y) {_y = y;}
protected:
// attributes
	float _u; /**< previous input */
	float _y; /**< previous output */
	BlockLimitSym _limit; /**< limiter */
};

/**
 * A simple derivative approximation.
 * This uses the previous and current input.
 * This has a built in low pass filter.
 * @see LowPass
 */
class __EXPORT BlockDerivative : public SuperBlock
{
public:
// methods
	BlockDerivative(SuperBlock *parent, const char *name) :
		SuperBlock(parent, name),
		_u(0),
		_initialized(false),
		_lowPass(this, "LP")
	{};
	virtual ~BlockDerivative() {};

	/**
	 * Update the state and get current derivative
	 *
	 * This call updates the state and gets the current
	 * derivative. As the derivative is only valid
	 * on the second call to update, it will return
	 * no change (0) on the first. To get a closer
	 * estimate of the derivative on the first call,
	 * call setU() one time step before using the
	 * return value of update().
	 *
	 * @param input the variable to calculate the derivative of
	 * @return the current derivative
	 */
	float update(float input);
// accessors
	void setU(float u) {_u = u;}
	float getU() {return _u;}
	float getLP() {return _lowPass.getFCut();}
	float getO() { return _lowPass.getState(); }
protected:
// attributes
	float _u; /**< previous input */
	bool _initialized;
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
	BlockP(SuperBlock *parent, const char *name) :
		Block(parent, name),
		_kP(this, "") // only one param, no need to name
	{};
	virtual ~BlockP() {};
	float update(float input)
	{
		return getKP() * input;
	}
// accessors
	float getKP() { return _kP.get(); }
protected:
	control::BlockParamFloat _kP;
};

/**
 * A proportional-integral controller.
 * @link http://en.wikipedia.org/wiki/PID_controller
 */
class __EXPORT BlockPI: public SuperBlock
{
public:
// methods
	BlockPI(SuperBlock *parent, const char *name) :
		SuperBlock(parent, name),
		_integral(this, "I"),
		_kP(this, "P"),
		_kI(this, "I")
	{};
	virtual ~BlockPI() {};
	float update(float input)
	{
		return getKP() * input +
		       getKI() * getIntegral().update(input);
	}
// accessors
	float getKP() { return _kP.get(); }
	float getKI() { return _kI.get(); }
	BlockIntegral &getIntegral() { return _integral; }
private:
	BlockIntegral _integral;
	control::BlockParamFloat _kP;
	control::BlockParamFloat _kI;
};

/**
 * A proportional-derivative controller.
 * @link http://en.wikipedia.org/wiki/PID_controller
 */
class __EXPORT BlockPD: public SuperBlock
{
public:
// methods
	BlockPD(SuperBlock *parent, const char *name) :
		SuperBlock(parent, name),
		_derivative(this, "D"),
		_kP(this, "P"),
		_kD(this, "D")
	{};
	virtual ~BlockPD() {};
	float update(float input)
	{
		return getKP() * input +
		       getKD() * getDerivative().update(input);
	}
// accessors
	float getKP() { return _kP.get(); }
	float getKD() { return _kD.get(); }
	BlockDerivative &getDerivative() { return _derivative; }
private:
	BlockDerivative _derivative;
	control::BlockParamFloat _kP;
	control::BlockParamFloat _kD;
};

/**
 * A proportional-integral-derivative controller.
 * @link http://en.wikipedia.org/wiki/PID_controller
 */
class __EXPORT BlockPID: public SuperBlock
{
public:
// methods
	BlockPID(SuperBlock *parent, const char *name) :
		SuperBlock(parent, name),
		_integral(this, "I"),
		_derivative(this, "D"),
		_kP(this, "P"),
		_kI(this, "I"),
		_kD(this, "D")
	{};
	virtual ~BlockPID() {};
	float update(float input)
	{
		return getKP() * input +
		       getKI() * getIntegral().update(input) +
		       getKD() * getDerivative().update(input);
	}
// accessors
	float getKP() { return _kP.get(); }
	float getKI() { return _kI.get(); }
	float getKD() { return _kD.get(); }
	BlockIntegral &getIntegral() { return _integral; }
	BlockDerivative &getDerivative() { return _derivative; }
private:
// attributes
	BlockIntegral _integral;
	BlockDerivative _derivative;
	control::BlockParamFloat _kP;
	control::BlockParamFloat _kI;
	control::BlockParamFloat _kD;
};

/**
 * An output trim/ saturation block
 */
class __EXPORT BlockOutput: public SuperBlock
{
public:
// methods
	BlockOutput(SuperBlock *parent, const char *name) :
		SuperBlock(parent, name),
		_trim(this, "TRIM"),
		_limit(this, ""),
		_val(0)
	{
		update(0);
	};
	virtual ~BlockOutput() {};
	void update(float input)
	{
		_val = _limit.update(input + getTrim());
	}
// accessors
	float getMin() { return _limit.getMin(); }
	float getMax() { return _limit.getMax(); }
	float getTrim() { return _trim.get(); }
	float get() { return _val; }
private:
// attributes
	control::BlockParamFloat _trim;
	BlockLimit _limit;
	float _val;
};

/**
 * A uniform random number generator
 */
class __EXPORT BlockRandUniform: public Block
{
public:
// methods
	BlockRandUniform(SuperBlock *parent,
			 const char *name) :
		Block(parent, name),
		_min(this, "MIN"),
		_max(this, "MAX")
	{
		// seed should be initialized somewhere
		// in main program for all calls to rand
		// XXX currently in nuttx if you seed to 0, rand breaks
	};
	virtual ~BlockRandUniform() {};
	float update()
	{
		static float rand_max = MAX_RAND;
		float rand_val = rand();
		float bounds = getMax() - getMin();
		return getMin() + (rand_val * bounds) / rand_max;
	}
// accessors
	float getMin() { return _min.get(); }
	float getMax() { return _max.get(); }
private:
// attributes
	control::BlockParamFloat _min;
	control::BlockParamFloat _max;
};

class __EXPORT BlockRandGauss: public Block
{
public:
// methods
	BlockRandGauss(SuperBlock *parent,
		       const char *name) :
		Block(parent, name),
		_mean(this, "MEAN"),
		_stdDev(this, "DEV")
	{
		// seed should be initialized somewhere
		// in main program for all calls to rand
		// XXX currently in nuttx if you seed to 0, rand breaks
	};
	virtual ~BlockRandGauss() {};
	float update()
	{
		static float V1, V2, S;
		static int phase = 0;
		float X;

		if (phase == 0) {
			do {
				float U1 = (float)rand() / MAX_RAND;
				float U2 = (float)rand() / MAX_RAND;
				V1 = 2 * U1 - 1;
				V2 = 2 * U2 - 1;
				S = V1 * V1 + V2 * V2;
			} while (S >= 1 || fabsf(S) < 1e-8f);

			X = V1 * float(sqrt(-2 * float(log(S)) / S));

		} else {
			X = V2 * float(sqrt(-2 * float(log(S)) / S));
		}

		phase = 1 - phase;
		return X * getStdDev() + getMean();
	}
// accessors
	float getMean() { return _mean.get(); }
	float getStdDev() { return _stdDev.get(); }
private:
// attributes
	control::BlockParamFloat _mean;
	control::BlockParamFloat _stdDev;
};

template<class Type, size_t M>
class __EXPORT BlockStats: public Block
{
public:
// methods
	BlockStats(SuperBlock *parent,
		   const char *name) :
		Block(parent, name),
		_sum(),
		_sumSq(),
		_count(0)
	{
	};
	virtual ~BlockStats() {};
	void update(const matrix::Vector<Type, M> &u)
	{
		_sum += u;
		_sumSq += u.emult(u);
		_count += 1;
	}
	void reset()
	{
		_sum.setZero();
		_sumSq.setZero();
		_count = 0;
	}
// accessors
	size_t getCount() { return _count; }
	matrix::Vector<Type, M> getMean() { return _sum / _count; }
	matrix::Vector<Type, M> getVar()
	{
		return (_sumSq - _sum.emult(_sum) / _count) / _count;
	}
	matrix::Vector<Type, M> getStdDev()
	{
		return getVar().pow(0.5);
	}
private:
// attributes
	matrix::Vector<Type, M> _sum;
	matrix::Vector<Type, M> _sumSq;
	size_t _count;
};

template<class Type, size_t M, size_t N, size_t LEN>
class __EXPORT BlockDelay: public Block
{
public:
// methods
	BlockDelay(SuperBlock *parent,
		   const char *name) :
		Block(parent, name),
		_h(),
		_index(0),
		_delay(-1)
	{
	};
	virtual ~BlockDelay() {};
	matrix::Matrix<Type, M, N> update(const matrix::Matrix<Type, M, N> &u)
	{
		// store current value
		_h[_index] = u;

		// delay starts at zero, then increases to LEN
		_delay += 1;

		if (_delay > (LEN - 1)) {
			_delay = LEN - 1;
		}

		// compute position of delayed value
		int j = _index - _delay;

		if (j < 0) {
			j += LEN;
		}

		// increment storage position
		_index += 1;

		if (_index > (LEN - 1)) {
			_index  = 0;
		}

		// get delayed value
		return _h[j];
	}
	matrix::Matrix<Type, M, N> get(size_t delay)
	{
		int j = _index - delay;

		if (j < 0) { j += LEN; }

		return _h[j];
	}
private:
// attributes
	matrix::Matrix<Type, M, N> _h[LEN];
	size_t _index;
	int _delay;
};

} // namespace control
