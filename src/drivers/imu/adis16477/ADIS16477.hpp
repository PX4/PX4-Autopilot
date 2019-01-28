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

/*
 * ADIS16477.hpp
 *
 */

#ifndef DRIVERS_IMU_ADIS16477_ADIS16477_HPP_
#define DRIVERS_IMU_ADIS16477_ADIS16477_HPP_

#include <drivers/device/ringbuffer.h>
#include <drivers/device/spi.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <drivers/device/integrator.h>
#include <lib/conversion/rotation.h>
#include <perf/perf_counter.h>
#include <ecl/geo/geo.h>
#include <systemlib/err.h>
#include <px4_work_queue/ScheduledWorkItem.hpp>

#define ADIS16477_GYRO_DEFAULT_RATE					250
#define ADIS16477_GYRO_DEFAULT_DRIVER_FILTER_FREQ	30

#define ADIS16477_ACCEL_DEFAULT_RATE				250
#define ADIS16477_ACCEL_DEFAULT_DRIVER_FILTER_FREQ	30

#define ADIS16477_ACCEL_MAX_OUTPUT_RATE              1221
#define ADIS16477_GYRO_MAX_OUTPUT_RATE               1221

class ADIS16477_gyro;

class ADIS16477 : public device::SPI, public px4::ScheduledWorkItem
{
public:
	ADIS16477(int bus, const char *path_accel, const char *path_gyro, uint32_t device, enum Rotation rotation);
	virtual ~ADIS16477();

	virtual int		init();

	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	void			print_info();

protected:
	virtual int		probe();

	friend class ADIS16477_gyro;

	virtual int		gyro_ioctl(struct file *filp, int cmd, unsigned long arg);

private:
	ADIS16477_gyro		*_gyro{nullptr};

	uint16_t			_product{0};	/** product code */

	unsigned			_call_interval{0};

	struct gyro_calibration_s	_gyro_scale {};

	// gyro 0.025 °/sec/LSB
	float				_gyro_range_scale{0.025f};
	float				_gyro_range_rad_s{math::radians(500.0f)};

	struct accel_calibration_s	_accel_scale {};

	// accel 1.25 mg/LSB
	float				_accel_range_scale{1.25f * CONSTANTS_ONE_G / 1000.0f};
	float				_accel_range_m_s2{40.0f * CONSTANTS_ONE_G};

	orb_advert_t		_accel_topic{nullptr};

	int					_accel_orb_class_instance{-1};
	int					_accel_class_instance{-1};

	unsigned			_sample_rate{100};

	perf_counter_t		_sample_perf;
	perf_counter_t		_bad_transfers;

	math::LowPassFilter2p	_gyro_filter_x{ADIS16477_GYRO_DEFAULT_RATE, ADIS16477_GYRO_DEFAULT_DRIVER_FILTER_FREQ};
	math::LowPassFilter2p	_gyro_filter_y{ADIS16477_GYRO_DEFAULT_RATE, ADIS16477_GYRO_DEFAULT_DRIVER_FILTER_FREQ};
	math::LowPassFilter2p	_gyro_filter_z{ADIS16477_GYRO_DEFAULT_RATE, ADIS16477_GYRO_DEFAULT_DRIVER_FILTER_FREQ};

	math::LowPassFilter2p	_accel_filter_x{ADIS16477_ACCEL_DEFAULT_RATE, ADIS16477_ACCEL_DEFAULT_DRIVER_FILTER_FREQ};
	math::LowPassFilter2p	_accel_filter_y{ADIS16477_ACCEL_DEFAULT_RATE, ADIS16477_ACCEL_DEFAULT_DRIVER_FILTER_FREQ};
	math::LowPassFilter2p	_accel_filter_z{ADIS16477_ACCEL_DEFAULT_RATE, ADIS16477_ACCEL_DEFAULT_DRIVER_FILTER_FREQ};

	Integrator			_accel_int{1000000 / ADIS16477_ACCEL_MAX_OUTPUT_RATE, false};
	Integrator			_gyro_int{1000000 / ADIS16477_GYRO_MAX_OUTPUT_RATE, true};

	enum Rotation		_rotation;

	perf_counter_t		_controller_latency_perf;

#pragma pack(push, 1)
	/**
	 * Report conversation with in the ADIS16477, including command byte and interrupt status.
	 */
	struct ADISReport {
		uint16_t	cmd;
		uint16_t	diag_stat;
		int16_t		gyro_x;
		int16_t		gyro_y;
		int16_t		gyro_z;
		int16_t		accel_x;
		int16_t		accel_y;
		int16_t		accel_z;
		uint16_t	temp;
		uint16_t	DATA_CNTR;
		uint8_t		checksum;
		uint8_t		_padding; // 16 bit SPI mode
	};
#pragma pack(pop)

	/**
	 * Start automatic measurement.
	 */
	void		start();

	/**
	 * Stop automatic measurement.
	 */
	void		stop();

	/**
	 * Reset chip.
	 *
	 * Resets the chip and measurements ranges, but not scale and offset.
	 */
	int			reset();

	void		Run() override;

	/**
	 * Fetch measurements from the sensor and update the report buffers.
	 */
	int			measure();

	bool			publish_accel(const ADISReport &report);
	bool			publish_gyro(const ADISReport &report);

	uint16_t		read_reg16(uint8_t reg);

	void			write_reg(uint8_t reg, uint8_t value);
	void			write_reg16(uint8_t reg, uint16_t value);

	// ADIS16477 onboard self test
	bool 			self_test();

	/*
	  set low pass filter frequency
	 */
	void _set_dlpf_filter(uint16_t frequency_hz);

	/*
	  set IMU to factory default
	 */
	void _set_factory_default();

	/*
	  set sample rate (approximate) - 1kHz to 5Hz
	*/
	void _set_sample_rate(uint16_t desired_sample_rate_hz);

	/*
	  set the gyroscope dynamic range
	*/
	void _set_gyro_dyn_range(uint16_t desired_gyro_dyn_range);

	ADIS16477(const ADIS16477 &);
	ADIS16477 operator=(const ADIS16477 &);
};

#endif /* DRIVERS_IMU_ADIS16477_ADIS16477_HPP_ */
