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


#include <drivers/device/integrator.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_hrt.h>
#include <lib/cdev/CDev.hpp>
#include <lib/conversion/rotation.h>
#include <mathlib/math/filter/LowPassFilter2pVector3f.hpp>
#include <px4_module_params.h>
#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/sensor_gyro.h>

class PX4Gyroscope : public cdev::CDev, public ModuleParams
{

public:
	PX4Gyroscope(uint32_t device_id, uint8_t priority, enum Rotation rotation);
	~PX4Gyroscope() override;

	int	ioctl(cdev::file_t *filp, int cmd, unsigned long arg) override;

	void set_device_type(uint8_t devtype);
	void set_error_count(uint64_t error_count) { _sensor_gyro_pub.get().error_count = error_count; }
	void set_scale(float scale) { _sensor_gyro_pub.get().scaling = scale; }
	void set_temperature(float temperature) { _sensor_gyro_pub.get().temperature = temperature; }

	void set_sample_rate(unsigned rate) { _sample_rate = rate; }

	void configure_filter(float cutoff_freq) { _filter.set_cutoff_frequency(_sample_rate, cutoff_freq); }

	void update(hrt_abstime timestamp, int16_t x, int16_t y, int16_t z);

	void print_status();

private:

	uORB::Publication<sensor_gyro_s>	_sensor_gyro_pub;

	math::LowPassFilter2pVector3f _filter{1000, 100};
	Integrator _integrator{4000, true};

	const enum Rotation	_rotation;

	matrix::Vector3f	_calibration_scale{1.0f, 1.0f, 1.0f};
	matrix::Vector3f	_calibration_offset{0.0f, 0.0f, 0.0f};

	int			_class_device_instance{-1};

	unsigned	_sample_rate{1000};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::IMU_GYRO_CUTOFF>) _param_imu_gyro_cutoff
	)

};
