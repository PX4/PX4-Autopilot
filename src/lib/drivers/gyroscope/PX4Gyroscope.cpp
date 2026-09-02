/****************************************************************************
 *
 *   Copyright (c) 2018-2022 PX4 Development Team. All rights reserved.
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


#include "PX4Gyroscope.hpp"

#include <lib/drivers/device/Device.hpp>
#include <lib/parameters/param.h>

using namespace time_literals;

static constexpr int32_t sum(const int16_t samples[], uint8_t len)
{
	int32_t sum = 0;

	for (int n = 0; n < len; n++) {
		sum += samples[n];
	}

	return sum;
}

static constexpr uint8_t clipping(const int16_t samples[], uint8_t len, int16_t clip_limit)
{
	unsigned clip_count = 0;

	for (int n = 0; n < len; n++) {
		// symmetric about zero: rotation negates samples and |INT16_MIN| = INT16_MAX + 1
		if ((samples[n] <= -clip_limit) || (samples[n] >= clip_limit)) {
			clip_count++;
		}
	}

	return clip_count;
}

PX4Gyroscope::PX4Gyroscope(uint32_t device_id, enum Rotation rotation) :
	PX4Gyroscope(device_id, rotation, device::device_is_external(device_id))
{
	_external_forced = false;
}

PX4Gyroscope::PX4Gyroscope(uint32_t device_id, enum Rotation rotation, bool external) :
	_device_id{device_id},
	_rotation{rotation},
	_is_external{external},
	_external_forced{true}
{
	// advertise immediately to keep instance numbering in sync
	_sensor_pub.advertise();

	param_get(param_find("IMU_GYRO_RATEMAX"), &_imu_gyro_rate_max);
}

void PX4Gyroscope::set_device_id(uint32_t device_id)
{
	_device_id = device_id;

	if (!_external_forced) {
		_is_external = device::device_is_external(device_id);
	}
}

void PX4Gyroscope::set_external(bool external)
{
	_is_external = external;
	_external_forced = true;
}

PX4Gyroscope::~PX4Gyroscope()
{
	_sensor_pub.unadvertise();
	_sensor_fifo_pub.unadvertise();
}

void PX4Gyroscope::set_device_type(uint8_t devtype)
{
	// current DeviceStructure
	union device::Device::DeviceId device_id;
	device_id.devid = _device_id;

	// update to new device type
	device_id.devid_s.devtype = devtype;

	// copy back
	_device_id = device_id.devid;
}

void PX4Gyroscope::set_scale(float scale)
{
	if (fabsf(scale - _scale) > FLT_EPSILON) {
		// rescale last sample on scale change
		float rescale = _scale / scale;

		for (auto &s : _last_sample) {
			s = roundf(s * rescale);
		}

		_scale = scale;

		UpdateClipLimit();
	}
}

void PX4Gyroscope::update(const hrt_abstime &timestamp_sample, float x, float y, float z)
{
	// Apply rotation (before scaling)
	rotate_3f(_rotation, x, y, z);

	sensor_gyro_s report;

	report.timestamp_sample = timestamp_sample;
	report.device_id = _device_id;
	report.is_external = _is_external;
	report.temperature = _temperature;
	report.error_count = _error_count;
	report.x = x * _scale;
	report.y = y * _scale;
	report.z = z * _scale;
	report.clip_counter[0] = (fabsf(x) >= _clip_limit);
	report.clip_counter[1] = (fabsf(y) >= _clip_limit);
	report.clip_counter[2] = (fabsf(z) >= _clip_limit);
	report.samples = 1;
	report.timestamp = hrt_absolute_time();

	_failure_config.update();

	if (!failure_injection::process(_failure_config, failure_injection_s::FAILURE_UNIT_SENSOR_GYRO,
					_sensor_pub.get_instance(), report, _stuck)) {
		return;
	}

	_sensor_pub.publish(report);
}

void PX4Gyroscope::updateFIFO(sensor_gyro_fifo_s &sample)
{
	// rotate all raw samples and publish fifo
	for (int n = 0; n < sample.samples; n++) {
		rotate_3i(_rotation, sample.x[n], sample.y[n], sample.z[n]);
	}

	sample.device_id = _device_id;
	sample.is_external = _is_external;
	sample.scale = _scale;
	sample.timestamp = hrt_absolute_time();

	_failure_config.update();

	if (!failure_injection::process(_failure_config, failure_injection_s::FAILURE_UNIT_SENSOR_GYRO,
					_sensor_pub.get_instance(), sample, _stuck_fifo)) {
		return;
	}

	_sensor_fifo_pub.publish(sample);

	const uint8_t N = sample.samples;

	// publish
	sensor_gyro_s report;
	report.timestamp_sample = sample.timestamp_sample;
	report.device_id = _device_id;
	report.is_external = _is_external;
	report.temperature = _temperature;
	report.error_count = _error_count;

	// trapezoidal integration (equally spaced)
	const float scale = _scale / (float)N;
	report.x = (0.5f * (_last_sample[0] + sample.x[N - 1]) + sum(sample.x, N - 1)) * scale;
	report.y = (0.5f * (_last_sample[1] + sample.y[N - 1]) + sum(sample.y, N - 1)) * scale;
	report.z = (0.5f * (_last_sample[2] + sample.z[N - 1]) + sum(sample.z, N - 1)) * scale;

	_last_sample[0] = sample.x[N - 1];
	_last_sample[1] = sample.y[N - 1];
	_last_sample[2] = sample.z[N - 1];

	// The declared range is not the int16 rail on every part (ST high-g and dps scales leave
	// headroom in the word), so clip against range/scale like update() does. The 0.999 margin
	// in _clip_limit also covers sensors that reuse the lowest bit for a sync flag.
	const int16_t clip_limit = static_cast<int16_t>(math::min(_clip_limit, (float)(INT16_MAX - 1)));
	report.clip_counter[0] = clipping(sample.x, N, clip_limit);
	report.clip_counter[1] = clipping(sample.y, N, clip_limit);
	report.clip_counter[2] = clipping(sample.z, N, clip_limit);
	report.samples = N;
	report.timestamp = hrt_absolute_time();

	if (!failure_injection::process(_failure_config, failure_injection_s::FAILURE_UNIT_SENSOR_GYRO,
					_sensor_pub.get_instance(), report, _stuck)) {
		return;
	}

	_sensor_pub.publish(report);
}

void PX4Gyroscope::UpdateClipLimit()
{
	// 99.9% of potential max
	_clip_limit = fabsf(_range / _scale * 0.999f);
}
