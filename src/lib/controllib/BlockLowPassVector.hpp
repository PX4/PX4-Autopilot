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

#include <px4_platform_common/defines.h>
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
		for (size_t i = 0; i < M; i++) {
			_state(i) = 0.0f / 0.0f;
		}
	}
	virtual ~BlockLowPassVector() {}
	matrix::Vector<Type, M> update(const matrix::Matrix<Type, M, 1> &input)
	{
		for (size_t i = 0; i < M; i++) {
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

} // namespace control
