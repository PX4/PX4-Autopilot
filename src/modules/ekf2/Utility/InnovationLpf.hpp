/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * First order "alpha" IIR digital filter with input saturation
 */

#include <mathlib/mathlib.h>

class InnovationLpf final
{
public:
	InnovationLpf() = default;
	~InnovationLpf() = default;

	void reset(float val = 0.f) { _x = val; }

	/**
	 * Update the filter with a new value and returns the filtered state
	 * The new value is constained by the limit set in setSpikeLimit
	 * @param val new input
	 * @param alpha normalized weight of the new input
	 * @param spike_limit the amplitude of the saturation at the input of the filter
	 * @return filtered output
	 */
	float update(float val, float alpha, float spike_limit)
	{
		float val_constrained = math::constrain(val, -spike_limit, spike_limit);
		float beta = 1.f - alpha;

		_x = beta * _x + alpha * val_constrained;

		return _x;
	}

	/**
	 * Helper function to compute alpha from dt and the inverse of tau
	 * @param dt sampling time in seconds
	 * @param tau_inv inverse of the time constant of the filter
	 * @return alpha, the normalized weight of a new measurement
	 */
	static float computeAlphaFromDtAndTauInv(float dt, float tau_inv)
	{
		return math::constrain(dt * tau_inv, 0.f, 1.f);
	}

private:
	float _x{}; ///< current state of the filter
};
