/****************************************************************************
 *
 *   Copyright (c) 2015-2022 PX4 Development Team. All rights reserved.
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
 * @file Integrator.hpp
 *
 * A resettable integrator
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Julian Oes <julian@oes.ch>
 */

#pragma once

#include <mathlib/mathlib.h>
#include <matrix/math.hpp>

namespace sensors
{

class Integrator
{
public:
	Integrator() = default;
	~Integrator() = default;

	static constexpr float DT_MIN{1e-6f}; // 1 microsecond
	static constexpr float DT_MAX{static_cast<float>(UINT16_MAX) * 1e-6f};

	/**
	 * Put an item into the integral.
	 *
	 * @param timestamp	Timestamp of the current value.
	 * @param val		Item to put.
	 * @return		true if data was accepted and integrated.
	 */
	inline void put(const matrix::Vector3f &val, const float dt)
	{
		if ((dt > DT_MIN) && (_integral_dt + dt < DT_MAX)) {
			_alpha += integrate(val, dt);

		} else {
			reset();
			_last_val = val;
		}
	}

	/**
	 * Set reset interval during runtime. This won't reset the integrator.
	 *
	 * @param reset_interval	New reset time interval for the integrator in microseconds.
	 */
	void set_reset_interval(uint32_t reset_interval_us) { _reset_interval_min = reset_interval_us * 1e-6f; }

	/**
	 * Set required samples for reset. This won't reset the integrator.
	 *
	 * @param reset_samples	    	New reset time interval for the integrator.
	 */
	void set_reset_samples(uint8_t reset_samples) { _reset_samples_min = reset_samples; }
	uint8_t get_reset_samples() const { return _reset_samples_min; }

	/**
	 * Is the Integrator ready to reset?
	 *
	 * @return		true if integrator has sufficient data (minimum interval & samples satisfied) to reset.
	 */
	inline bool integral_ready() const { return (_integrated_samples >= _reset_samples_min) || (_integral_dt >= _reset_interval_min); }

	float integral_dt() const { return _integral_dt; }

	void reset()
	{
		_alpha.zero();
		_integral_dt = 0;
		_integrated_samples = 0;
	}

	/* Reset integrator and return current integral & integration time
	 *
	 * @param integral_dt	Get the dt in us of the current integration.
	 * @return		true if integral valid
	 */
	bool reset(matrix::Vector3f &integral, uint16_t &integral_dt)
	{
		if (integral_ready()) {
			integral = _alpha;
			integral_dt = roundf(_integral_dt * 1e6f); // seconds to microseconds

			reset();

			return true;
		}

		return false;
	}

protected:

	inline matrix::Vector3f integrate(const matrix::Vector3f &val, const float dt)
	{
		// Use trapezoidal integration to calculate the delta integral
		_integrated_samples++;
		_integral_dt += dt;
		const matrix::Vector3f delta_alpha{(val + _last_val) *dt * 0.5f};
		_last_val = val;

		return delta_alpha;
	}

	matrix::Vector3f _alpha{0.f, 0.f, 0.f};    /**< integrated value before coning corrections are applied */
	matrix::Vector3f _last_val{0.f, 0.f, 0.f}; /**< previous input */
	float _integral_dt{0};

	float _reset_interval_min{0.001f}; /**< the interval after which the content will be published and the integrator reset */
	uint8_t _reset_samples_min{1};

	uint8_t _integrated_samples{0};
};

class IntegratorConing : public Integrator
{
public:
	IntegratorConing() = default;
	~IntegratorConing() = default;

	/**
	 * Put an item into the integral.
	 *
	 * @param timestamp	Timestamp of the current value.
	 * @param val		Item to put.
	 * @return		true if data was accepted and integrated.
	 */
	inline void put(const matrix::Vector3f &val, const float dt)
	{
		if ((dt > DT_MIN) && (_integral_dt + dt < DT_MAX)) {
			// Use trapezoidal integration to calculate the delta integral
			const matrix::Vector3f delta_alpha{integrate(val, dt)};

			// Calculate coning corrections
			// Coning compensation derived by Paul Riseborough and Jonathan Challinger,
			// following:
			// Strapdown Inertial Navigation Integration Algorithm Design Part 1: Attitude Algorithms
			// Sourced: https://arc.aiaa.org/doi/pdf/10.2514/2.4228
			// Simulated: https://github.com/priseborough/InertialNav/blob/master/models/imu_error_modelling.m
			_beta += ((_last_alpha + _last_delta_alpha * (1.f / 6.f)) % delta_alpha) * 0.5f;
			_last_delta_alpha = delta_alpha;
			_last_alpha = _alpha;

			// accumulate delta integrals
			_alpha += delta_alpha;

		} else {
			reset();
			_last_val = val;
		}
	}

	void reset()
	{
		Integrator::reset();
		_beta.zero();
		_last_alpha.zero();
	}

	const matrix::Vector3f &accumulated_coning_corrections() const { return _beta; }

	/* Reset integrator and return current integral & integration time
	 *
	 * @param integral_dt	Get the dt in us of the current integration.
	 * @return		true if integral valid
	 */
	bool reset(matrix::Vector3f &integral, uint16_t &integral_dt)
	{
		if (Integrator::reset(integral, integral_dt)) {
			// apply coning corrections
			integral += _beta;
			_beta.zero();
			_last_alpha.zero();
			return true;
		}

		return false;
	}

private:
	matrix::Vector3f _beta{0.f, 0.f, 0.f};             /**< accumulated coning corrections */
	matrix::Vector3f _last_delta_alpha{0.f, 0.f, 0.f}; /**< integral from previous previous sampling interval */
	matrix::Vector3f _last_alpha{0.f, 0.f, 0.f};       /**< previous value of _alpha */

};

}; // namespace sensors
