/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include <drivers/drv_gyro.h>
#include <drivers/drv_hrt.h>
#include <lib/cdev/CDev.hpp>
#include <lib/conversion/rotation.h>
#include <lib/drivers/device/integrator.h>
#include <lib/mathlib/math/filter/LowPassFilter2pArray.hpp>
#include <lib/mathlib/math/filter/LowPassFilter2pVector3f.hpp>
#include <lib/mathlib/math/filter/NotchFilter.hpp>
#include <px4_platform_common/module_params.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_gyro_fifo.h>
#include <uORB/topics/sensor_gyro_integrated.h>
#include <uORB/topics/sensor_gyro_status.h>

class PX4Gyroscope : public cdev::CDev, public ModuleParams
{
public:
	PX4Gyroscope(uint32_t device_id, uint8_t priority = ORB_PRIO_DEFAULT, enum Rotation rotation = ROTATION_NONE);
	~PX4Gyroscope() override;

	int	ioctl(cdev::file_t *filp, int cmd, unsigned long arg) override;

	uint32_t get_device_id() const { return _device_id; }

	void set_device_id(uint32_t device_id) { _device_id = device_id; }
	void set_device_type(uint8_t devtype);
	void set_error_count(uint64_t error_count) { _error_count += error_count; }
	void set_range(float range) { _range = range; }
	void set_sample_rate(uint16_t rate);
	void set_scale(float scale) { _scale = scale; }
	void set_temperature(float temperature) { _temperature = temperature; }
	void set_update_rate(uint16_t rate);

	void update(hrt_abstime timestamp_sample, float x, float y, float z);

	void print_status();

	struct FIFOSample {
		hrt_abstime timestamp_sample;
		uint8_t samples; // number of samples
		float dt; // in microseconds

		int16_t x[16];
		int16_t y[16];
		int16_t z[16];
	};
	static_assert(sizeof(FIFOSample::x) == sizeof(sensor_gyro_fifo_s::x), "FIFOSample.x invalid size");
	static_assert(sizeof(FIFOSample::y) == sizeof(sensor_gyro_fifo_s::y), "FIFOSample.y invalid size");
	static_assert(sizeof(FIFOSample::z) == sizeof(sensor_gyro_fifo_s::z), "FIFOSample.z invalid size");

	void updateFIFO(const FIFOSample &sample);

private:

	void ConfigureFilter(float cutoff_freq);
	void ConfigureNotchFilter(float notch_freq, float bandwidth);
	void PublishStatus();
	void ResetIntegrator();
	void UpdateVibrationMetrics(const matrix::Vector3f &delta_angle);

	uORB::PublicationMulti<sensor_gyro_s>            _sensor_pub;
	uORB::PublicationMulti<sensor_gyro_fifo_s>       _sensor_fifo_pub;
	uORB::PublicationMulti<sensor_gyro_integrated_s> _sensor_integrated_pub;
	uORB::PublicationMulti<sensor_gyro_status_s>     _sensor_status_pub;

	math::LowPassFilter2pVector3f _filter{1000, 100};
	math::NotchFilter<matrix::Vector3f> _notch_filter{};

	hrt_abstime	_control_last_publish{0};
	hrt_abstime	_status_last_publish{0};

	math::LowPassFilter2pArray _filterArrayX{8000, 100};
	math::LowPassFilter2pArray _filterArrayY{8000, 100};
	math::LowPassFilter2pArray _filterArrayZ{8000, 100};

	Integrator		_integrator{4000, true};

	matrix::Vector3f	_calibration_offset{0.0f, 0.0f, 0.0f};

	matrix::Vector3f _delta_angle_prev{0.0f, 0.0f, 0.0f};	// delta angle from the previous IMU measurement
	float _vibration_metric{0.0f};	// high frequency vibration level in the IMU delta angle data (rad)
	float _coning_vibration{0.0f};	// Level of coning vibration in the IMU delta angles (rad^2)

	int			_class_device_instance{-1};

	uint32_t		_device_id{0};
	const enum Rotation	_rotation;

	float			_range{math::radians(2000.0f)};
	float			_scale{1.0f};
	float			_temperature{0.0f};

	uint64_t		_error_count{0};

	uint32_t		_clipping[3] {};

	uint16_t		_sample_rate{1000};
	uint16_t		_update_rate{1000};

	// integrator
	hrt_abstime		_integrator_timestamp_sample{0};
	hrt_abstime		_timestamp_sample_prev{0};
	float			_integrator_accum[3] {};
	uint8_t			_integrator_reset_samples{4};
	uint8_t			_integrator_samples{0};
	uint8_t			_integrator_fifo_samples{0};
	uint8_t			_integrator_clipping{0};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::IMU_GYRO_CUTOFF>) _param_imu_gyro_cutoff,
		(ParamFloat<px4::params::IMU_GYRO_NF_FREQ>) _param_imu_gyro_nf_freq,
		(ParamFloat<px4::params::IMU_GYRO_NF_BW>) _param_imu_gyro_nf_bw,
		(ParamInt<px4::params::IMU_GYRO_RATEMAX>) _param_imu_gyro_rate_max
	)
};
