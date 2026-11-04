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

#include <drivers/drv_hrt.h>
#include <lib/mathlib/math/filter/MedianFilter.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <uORB/Subscription.hpp>

namespace sensors
{

// an enabled IMU takes part in the median while its latest sample is at most this much older than the selected one,
// a few sample periods at the slowest IMU publication rate
static constexpr hrt_abstime IMU_MEDIAN_SAMPLE_TIMEOUT_US = 20'000;

/**
 * Per-axis median of the latest calibrated sample of three IMUs, the selected one included,
 * so that a single misbehaving IMU cannot pull the output away from the other two
 */
template<typename Sample, typename Calibration>
class SensorMedian
{
public:
	explicit SensorMedian(ORB_ID meta) : _sensor_sub{{meta, 0}, {meta, 1}, {meta, 2}, {meta, 3}} {}

	void ParametersUpdate()
	{
		for (auto &calibration : _calibration) {
			calibration.ParametersUpdate();
		}
	}

	/**
	 * @param selected	latest sample of the selected sensor
	 * @param corrected	that sample calibrated
	 * @return the median, or corrected without three recent samples to take it from
	 */
	matrix::Vector3f Apply(const Sample &selected, const matrix::Vector3f &corrected)
	{
		math::MedianFilter<float, 3> median[3];
		int count = 0;

		auto insert = [&](const matrix::Vector3f & sample) {
			for (int axis = 0; axis < 3; axis++) {
				median[axis].insert(sample(axis));
			}

			count++;
		};

		insert(corrected);

		for (int i = 0; (i < MAX_SENSOR_COUNT) && (count < 3); i++) {
			Sample sample;

			if (_sensor_sub[i].copy(&sample) && (sample.device_id != selected.device_id)
			    && (selected.timestamp_sample < sample.timestamp_sample + IMU_MEDIAN_SAMPLE_TIMEOUT_US)) {

				if (_calibration[i].device_id() != sample.device_id) {
					_calibration[i].set_device_id(sample.device_id, sample.is_external);
				}

				if (_calibration[i].enabled()) {
					_calibration[i].SensorCorrectionsUpdate();
					insert(_calibration[i].Correct(matrix::Vector3f{sample.x, sample.y, sample.z}));
				}
			}
		}

		return (count == 3) ? matrix::Vector3f{median[0].median(), median[1].median(), median[2].median()} : corrected;
	}

private:
	static constexpr int MAX_SENSOR_COUNT = 4;

	uORB::Subscription _sensor_sub[MAX_SENSOR_COUNT];
	Calibration _calibration[MAX_SENSOR_COUNT] {};
};

} // namespace sensors
