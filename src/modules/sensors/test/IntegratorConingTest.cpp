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


#include <gtest/gtest.h>
#include <modules/sensors/Integrator.hpp>

using matrix::AxisAngled;
using matrix::Quatd;
using matrix::Vector3d;
using matrix::Vector3f;

/**
 * Classical coning: the body is rotated by the cone angle about an axis that itself turns
 * in the body x-y plane. The attitude has a closed form, so every integration window can
 * be compared with the exact rotation.
 */
class ConingMotion
{
public:
	ConingMotion(double cone_angle, double frequency_hz) :
		_cone_angle(cone_angle),
		_omega(2.0 * M_PI * frequency_hz)
	{}

	Quatd attitude(double t) const
	{
		const double sin_half_angle = sin(0.5 * _cone_angle);
		return Quatd(cos(0.5 * _cone_angle), sin_half_angle * cos(_omega * t), sin_half_angle * sin(_omega * t), 0.0);
	}

	Vector3d bodyRate(double t) const
	{
		return Vector3d(-_omega * sin(_cone_angle) * sin(_omega * t),
				_omega * sin(_cone_angle) * cos(_omega * t),
				-_omega * (1.0 - cos(_cone_angle)));
	}

	Vector3d meanBodyRate(double t_begin, double t_end) const
	{
		const double dt = t_end - t_begin;
		return Vector3d(sin(_cone_angle) * (cos(_omega * t_end) - cos(_omega * t_begin)),
				sin(_cone_angle) * (sin(_omega * t_end) - sin(_omega * t_begin)),
				-_omega * (1.0 - cos(_cone_angle)) * dt) / dt;
	}

	// Rotation vector from the body frame at t_begin to the body frame at t_end
	Vector3d rotation(double t_begin, double t_end) const
	{
		return AxisAngled(attitude(t_begin).inversed() * attitude(t_end));
	}

private:
	double _cone_angle;
	double _omega;
};

enum class SampleModel {
	Point,     // instantaneous rate, as from drivers without a FIFO
	BlockMean, // mean rate since the previous sample, as from PX4Gyroscope::updateFIFO
};

// Magnitude of the mean attitude drift rate (rad/s)
struct ConingDrift {
	double compensated;
	double uncompensated;
};

static constexpr double kConingFrequency = 50.0;
static constexpr int kConingPeriods = 100;

static ConingDrift integrateConing(SampleModel sample_model, int sample_rate_hz, int samples_per_window)
{
	const ConingMotion coning(math::radians(1.0), kConingFrequency);
	const double sample_dt = 1.0 / sample_rate_hz;

	const auto sample = [&](int k) {
		const double t = k * sample_dt;
		return Vector3f(sample_model == SampleModel::Point ? coning.bodyRate(t) : coning.meanBodyRate(t - sample_dt, t));
	};

	sensors::IntegratorConing integrator;
	sensors::Integrator integrator_uncompensated;

	// Same configuration as VehicleIMU::UpdateIntegratorConfiguration()
	const uint32_t reset_interval_us = roundf((samples_per_window - 0.5f) * 1e6f / sample_rate_hz);
	integrator.set_reset_interval(reset_interval_us);
	integrator.set_reset_samples(samples_per_window);
	integrator_uncompensated.set_reset_interval(reset_interval_us);
	integrator_uncompensated.set_reset_samples(samples_per_window);

	integrator.put(sample(0), 0.f);
	integrator_uncompensated.put(sample(0), 0.f);

	// A trapezoid over block means spans the interval between the block centres
	const double window_offset = (sample_model == SampleModel::BlockMean) ? 0.5 * sample_dt : 0.0;

	// Errors that do not accumulate cancel over whole coning periods, only the drift remains
	const int sample_count = static_cast<int>(lround(kConingPeriods * sample_rate_hz / kConingFrequency));

	Vector3d error_sum;
	Vector3d error_sum_uncompensated;
	int window_start = 0;

	for (int k = 1; k <= sample_count; k++) {
		integrator.put(sample(k), static_cast<float>(sample_dt));
		integrator_uncompensated.put(sample(k), static_cast<float>(sample_dt));

		if (!integrator.integral_ready()) {
			continue;
		}

		Vector3f delta_angle;
		Vector3f delta_angle_uncompensated;
		uint32_t integral_dt_us;
		integrator.reset(delta_angle, integral_dt_us);
		integrator_uncompensated.reset(delta_angle_uncompensated, integral_dt_us);

		const Vector3d truth = coning.rotation(window_start * sample_dt - window_offset, k * sample_dt - window_offset);
		error_sum += Vector3d(delta_angle) - truth;
		error_sum_uncompensated += Vector3d(delta_angle_uncompensated) - truth;
		window_start = k;
	}

	const double duration = window_start * sample_dt;
	printf("drift %.3g deg/h, uncompensated %.3g deg/h\n",
	       math::degrees(error_sum.norm() / duration) * 3600.0,
	       math::degrees(error_sum_uncompensated.norm() / duration) * 3600.0);
	return {error_sum.norm() / duration, error_sum_uncompensated.norm() / duration};
}

TEST(IntegratorConingTest, pointSamples)
{
	const ConingDrift drift = integrateConing(SampleModel::Point, 1000, 5);
	EXPECT_LT(drift.compensated, 0.05 * drift.uncompensated);
}

TEST(IntegratorConingTest, blockMeanSamples)
{
	// Default configuration: 8 kHz FIFO published at IMU_GYRO_RATEMAX 400 Hz, IMU_INTEG_RATE 200 Hz
	const ConingDrift drift = integrateConing(SampleModel::BlockMean, 400, 2);
	EXPECT_LT(drift.compensated, 0.3 * drift.uncompensated);
}
