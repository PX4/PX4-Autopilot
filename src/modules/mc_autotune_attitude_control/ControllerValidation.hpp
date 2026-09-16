/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#pragma once

#include <cmath>
#include <cstdint>
#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>

// Streaming, independent frequency-response verification of a controller change.
// This is a finite-band empirical check, not an all-frequency stability proof.
class ControllerValidation
{
public:
	// NuttX does not provide <complex>. Only the float operations used below are needed.
	struct Complex {
		float x{}, y{};
		Complex() = default;
		Complex(float real, float imag = 0.f) : x(real), y(imag) {}
		float real() const { return x; }
		float imag() const { return y; }
		friend Complex operator+(Complex a, Complex b) { return {a.x + b.x, a.y + b.y}; }
		friend Complex operator-(Complex a, Complex b) { return {a.x - b.x, a.y - b.y}; }
		friend Complex operator*(Complex a, Complex b) { return {a.x *b.x - a.y * b.y, a.x *b.y + a.y * b.x}; }
		friend Complex operator/(Complex a, Complex b)
		{
			const float denominator = norm(b);
			return {(a.x * b.x + a.y * b.y) / denominator, (a.y * b.x - a.x * b.y) / denominator};
		}
		Complex &operator+=(Complex b) { *this = *this + b; return *this; }
		Complex &operator-=(Complex b) { *this = *this - b; return *this; }
		Complex &operator*=(float scale) { x *= scale; y *= scale; return *this; }
		static float norm(Complex value) { return value.x * value.x + value.y * value.y; }
		static float abs(Complex value) { return sqrtf(norm(value)); }
		static Complex conj(Complex value) { return {value.x, -value.y}; }
		static Complex polar(float magnitude, float phase) { return {magnitude * cosf(phase), magnitude * sinf(phase)}; }
	};
	static constexpr int MaxFrequencies = 48;
	static constexpr int SettlingPeriods = 2;
	static constexpr int MeasurementPeriods = 4;
	static constexpr int GroupPeriods = SettlingPeriods + MeasurementPeriods;
	static constexpr int TotalPeriods = 2 * GroupPeriods;
	struct Gains { matrix::Vector3f p{}, i{}, d{}, attitude{}; };
	enum class Result { Pass, InsufficientBandwidth, Reject };
	enum class Error { None, Configuration, MissingPeriod, InvalidSample, IncompletePeriod, MissingExcitation, InvalidResponse };

	void configure(float period, float maximum_frequency)
	{
		_configuration_valid = std::isfinite(period) && period >= 4.f && period <= 128.f
				       && std::isfinite(maximum_frequency) && maximum_frequency >= 1.f / period
				       && period * maximum_frequency < 65535.f;

		if (!_configuration_valid) { return; }

		_period = period;
		_count = 1;
		_harmonic[0] = 1;
		const int maximum = int(period * maximum_frequency);

		while (_count < MaxFrequencies - 1) {
			const int next = math::max(int(_harmonic[_count - 1] * 1.25f + .5f), _harmonic[_count - 1] + 1);

			if (next > maximum) { break; }

			_harmonic[_count++] = next;
		}

		if (_harmonic[_count - 1] < maximum) { _harmonic[_count++] = maximum; }

		for (int axis = 0; axis < 3; ++axis) { clearAxis(axis); }
	}

	void beginAxis(int axis, uint64_t start, float amplitude)
	{
		_axis = axis;
		_start = start;
		_amplitude = amplitude;
		_last = 0;
		_block = 0;
		clearAxis(axis);
		clearBlock();
	}

	float elapsed(uint64_t now) const { return (now - _start) * 1e-6f; }
	bool trained(uint64_t now) const { return elapsed(now) >= SettlingPeriods * _period; }
	bool finished() const { return _block >= TotalPeriods; }
	bool validData() const { return _error[_axis] == Error::None; }
	Error error() const { return _error[_axis]; }
	float sampleInterval() const { return _sample_interval; }
	float period() const { return _period; }
	int frequencies() const { return _count; }
	float frequency(int bin) const { return _harmonic[bin] / _period; }

	float excitation(uint64_t now) const
	{
		const float t = elapsed(now);

		if (t >= TotalPeriods * _period) { return 0.f; }

		const float phase_offset = t >= GroupPeriods * _period ? .9f : 0.f;
		float value = 0.f;

		for (int k = 0; k < _count; ++k) {
			const float phase = M_PI_F * k * (k - 1) / _count + phase_offset * (k + 1);
			value += _amplitude * sinf(2.f * M_PI_F * _harmonic[k] * fmodf(t, _period) / _period + phase);
		}

		return value;
	}

	bool update(uint64_t timestamp, float controller_dt, const matrix::Vector3f &input,
		    const matrix::Vector3f &rate, const matrix::Vector3f &acceleration, float applied)
	{
		if (!validData() || timestamp <= _start || timestamp == _last || finished()) { return false; }

		const float t = elapsed(timestamp);
		const int block = int(t / _period);

		if (block != _block) {
			if (block != _block + 1) { _error[_axis] = Error::MissingPeriod; return false; }

			if (_block % GroupPeriods >= SettlingPeriods) { finishBlock(); }

			_block = block;
			clearBlock();
		}

		const float dt = _last ? (timestamp - _last) * 1e-6f : controller_dt;
		_sample_interval = dt;
		_last = timestamp;

		if (finished()) { return false; }

		if (!input.isAllFinite() || !rate.isAllFinite() || !acceleration.isAllFinite()
		    || !std::isfinite(applied) || !std::isfinite(dt) || dt <= 0.f || dt > .05f
		    || !std::isfinite(controller_dt) || controller_dt <= 0.f) {
			_error[_axis] = Error::InvalidSample;
			return false;
		}

		if (_block % GroupPeriods < SettlingPeriods) { return true; }

		const float fraction = t / _period - block;
		const float values[10] {applied, input(0), input(1), input(2), rate(0), rate(1), rate(2),
					acceleration(0), acceleration(1), acceleration(2)
				       };
		_duration += dt;
		_controller_dt += controller_dt;
		++_samples;

		for (int channel = 0; channel < 10; ++channel) {
			_trend[channel] += dt * (fraction - .5f) * values[channel];
		}

		for (int k = 0; k < _count; ++k) {
			const float phase = 2.f * M_PI_F * _harmonic[k] * fraction;
			const Complex reference(cosf(phase) * dt, -sinf(phase) * dt);

			for (int channel = 0; channel < 10; ++channel) {
				_sum[k][channel] += values[channel] * reference;
			}
		}

		return true;
	}

	Result check(const Gains &old, const Gains &candidate, float yaw_cutoff, float &minimum) const
	{
		minimum = INFINITY;

		if (!_configuration_valid) { return Result::Reject; }

		for (Error error : _error) { if (error != Error::None) { return Result::Reject; } }

		bool insufficient_low_frequency = false;

		for (int group = 0; group < 2; ++group) {
			for (int axis = 0; axis < 3; ++axis) {
				if (_periods[group][axis] != MeasurementPeriods) { return Result::Reject; }
			}

			float low = 0.f, high = 0.f;

			for (int row = 0; row < 3; ++row) {
				for (int col = 0; col < 3; ++col) {
					low += Complex::norm(_response[group][0][col][row][0].mean);
					high += Complex::norm(Complex(row == col ? 1.f : 0.f) - _response[group][_count - 1][col][row][0].mean);
				}
			}

			if (sqrtf(high) >= .2f) { return Result::Reject; }

			insufficient_low_frequency |= sqrtf(low) >= .2f;
		}

		if (insufficient_low_frequency) { return Result::InsufficientBandwidth; }

		float dt = 0.f;

		for (const auto &group : _dt_sum) { for (float value : group) { dt += value; } }

		dt /= 6.f * MeasurementPeriods;

		for (int axis = 0; axis < 3; ++axis) {
			if (!(old.p(axis) > 0.f && old.i(axis) > 0.f && candidate.p(axis) > 0.f && candidate.i(axis) > 0.f
			      && old.attitude(axis) > 0.f && candidate.attitude(axis) > 0.f)
			    || dt * old.i(axis) / old.p(axis) >= 2.f) { return Result::Reject; }
		}

		for (int bin = 0; bin < _count; ++bin) {
			const Complex q = Complex::polar(1.f, -2.f * M_PI_F * frequency(bin) * dt);
			const Complex integral = dt * q / (1.f - q);
			Complex R[2][3][3] {};
			float radius_squared[2] {};

			for (int row = 0; row < 3; ++row) {
				Complex filter(1.f);

				if (row == 2 && yaw_cutoff > 0.f) {
					const float alpha = dt / (dt + 1.f / (2.f * M_PI_F * yaw_cutoff));
					filter = alpha / (1.f - (1.f - alpha) * q);
				}

				const Complex cp0 = filter * (old.p(row) + old.i(row) * integral);
				const Complex cpn = filter * (candidate.p(row) + candidate.i(row) * integral);
				const Complex r = cpn / cp0 * (candidate.attitude(row) / old.attitude(row));
				const Complex weight[3] {1.f - r, cpn - r * cp0, filter *(candidate.d(row) - r * old.d(row))};

				for (int group = 0; group < 2; ++group) {
					for (int col = 0; col < 3; ++col) {
						R[group][row][col] = row == col ? r : Complex{};
						float radius = 0.f;

						for (int channel = 0; channel < 3; ++channel) {
							const Estimate &value = _response[group][bin][col][row][channel];
							R[group][row][col] += weight[channel] * value.mean;
							// Triangle bound retains correlation safety without a large covariance matrix.
							radius += Complex::abs(weight[channel]) * 5.841f * sqrtf(math::max(value.m2, 0.f) / 12.f);
						}

						radius_squared[group] += radius * radius;
					}
				}
			}

			float difference = 0.f;

			for (int row = 0; row < 3; ++row) {
				for (int col = 0; col < 3; ++col) { difference += Complex::norm(R[1][row][col] - R[0][row][col]); }
			}

			const float uncertainty = math::max(sqrtf(radius_squared[0]), sqrtf(difference) + sqrtf(radius_squared[1]));

			// Gershgorin gives a conservative lower bound on the smallest Hermitian eigenvalue.
			for (int row = 0; row < 3; ++row) {
				float bound = R[0][row][row].real() - uncertainty;

				for (int col = 0; col < 3; ++col) {
					if (row != col) { bound -= .5f * Complex::abs(R[0][row][col] + Complex::conj(R[0][col][row])); }
				}

				if (!std::isfinite(bound)) { return Result::Reject; }

				minimum = math::min(minimum, bound);
			}
		}

		return minimum > .2f ? Result::Pass : Result::Reject;
	}

private:
	struct Estimate { Complex mean{}; float m2{}; };
	// group, frequency, excited axis, output axis, S/H/A
	Estimate _response[2][MaxFrequencies][3][3][3] {};
	Complex _sum[MaxFrequencies][10] {};
	float _trend[10] {};
	int _periods[2][3] {};
	uint16_t _harmonic[MaxFrequencies] {};
	int _count{}, _axis{}, _block{}, _samples{};
	uint64_t _start{}, _last{};
	float _period{}, _amplitude{}, _duration{}, _controller_dt{}, _sample_interval{};
	float _dt_sum[2][3] {};
	bool _configuration_valid{};
	Error _error[3] {};

	void clearAxis(int axis)
	{
		_error[axis] = _configuration_valid ? Error::None : Error::Configuration;

		for (int group = 0; group < 2; ++group) {
			_periods[group][axis] = 0;
			_dt_sum[group][axis] = 0.f;

			for (int bin = 0; bin < MaxFrequencies; ++bin) {
				for (int row = 0; row < 3; ++row) {
					for (int channel = 0; channel < 3; ++channel) { _response[group][bin][axis][row][channel] = {}; }
				}
			}
		}
	}

	void clearBlock()
	{
		for (int k = 0; k < MaxFrequencies; ++k) {
			for (int channel = 0; channel < 10; ++channel) { _sum[k][channel] = {}; }
		}

		for (float &value : _trend) { value = 0.f; }

		_duration = _controller_dt = 0.f;
		_samples = 0;
	}

	void finishBlock()
	{
		if (_duration < .98f * _period || _duration > 1.02f * _period || _samples < 100) { _error[_axis] = Error::IncompletePeriod; return; }

		const int group = _block / GroupPeriods;
		const int n = ++_periods[group][_axis];
		// Average the controller interval, not the response subscriber interval.
		_dt_sum[group][_axis] += _controller_dt / _samples;
		// Remove a jointly fitted linear drift. Complete integer periods make
		// different tones orthogonal; their coupling to the ramp is analytic.
		float denominator = 1.f / 12.f;

		for (int k = 0; k < _count; ++k) {
			const float cross = 1.f / (2.f * M_PI_F * _harmonic[k]);
			denominator -= 2.f * cross * cross;
		}

		for (int channel = 0; channel < 10; ++channel) {
			float slope = _trend[channel] / _period;

			for (int k = 0; k < _count; ++k) {
				_sum[k][channel] *= 2.f / _period;
				slope -= _sum[k][channel].imag() / (2.f * M_PI_F * _harmonic[k]);
			}

			slope /= denominator;

			for (int k = 0; k < _count; ++k) {
				_sum[k][channel] -= Complex(0.f, slope / (M_PI_F * _harmonic[k]));
			}
		}

		for (int k = 0; k < _count; ++k) {
			const Complex e = _sum[k][0];

			if (!(Complex::abs(e) > .5f * _amplitude)) { _error[_axis] = Error::MissingExcitation; return; }

			for (int row = 0; row < 3; ++row) {
				for (int channel = 0; channel < 3; ++channel) {
					const Complex value = _sum[k][1 + channel * 3 + row] / e;

					if (!std::isfinite(value.real()) || !std::isfinite(value.imag())) { _error[_axis] = Error::InvalidResponse; return; }

					Estimate &estimate = _response[group][k][_axis][row][channel];
					const Complex delta = value - estimate.mean;
					estimate.mean += delta / float(n);
					estimate.m2 += (delta * Complex::conj(value - estimate.mean)).real();
				}
			}
		}
	}
};
