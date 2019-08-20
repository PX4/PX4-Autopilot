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

#ifndef DRIVERS_IMU_LSM303AGR_LSM303AGR_HPP_
#define DRIVERS_IMU_LSM303AGR_LSM303AGR_HPP_

#include <drivers/drv_hrt.h>
#include <drivers/device/spi.h>
#include <drivers/drv_mag.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/device/integrator.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/conversion/rotation.h>
#include <perf/perf_counter.h>
#include <px4_work_queue/ScheduledWorkItem.hpp>
#include <systemlib/err.h>

// Register mapping
static constexpr uint8_t WHO_AM_I_M = 0x4F;

static constexpr uint8_t CFG_REG_A_M = 0x60;

struct CFG_REG_A_M_BITS {
	uint8_t
	: 1, // unused
	: 1, // unused
	SOFT_RST	: 1,
	LP			: 1,
	ODR1		: 1,
	ODR0		: 1,
	MD1			: 1,
	MD0			: 1;
};

static constexpr uint8_t CFG_REG_A_M_SOFT_RST	= (1 << 5);
static constexpr uint8_t CFG_REG_A_M_ODR1		= (1 << 3);
static constexpr uint8_t CFG_REG_A_M_ODR0		= (1 << 2);
static constexpr uint8_t CFG_REG_A_M_MD1		= (1 << 1);
static constexpr uint8_t CFG_REG_A_M_MD0		= (1 << 0);

static constexpr uint8_t CFG_REG_B_M = 0x61;
static constexpr uint8_t CFG_REG_B_M_OFF_CANC	= (1 << 1);
static constexpr uint8_t CFG_REG_B_M_OFF_LPF	= (1 << 0);

static constexpr uint8_t CFG_REG_C_M = 0x62;
static constexpr uint8_t CFG_REG_C_M_I2C_DIS	= (1 << 5);
static constexpr uint8_t CFG_REG_C_M_BDU		= (1 << 4);
static constexpr uint8_t CFG_REG_C_M_Self_test	= (1 << 1);

static constexpr uint8_t STATUS_REG_M = 0x67;
static constexpr uint8_t STATUS_REG_M_Zyxda		= (1 << 3);

// Magnetometer output registers
static constexpr uint8_t OUTX_L_REG_M = 0x68;
static constexpr uint8_t OUTX_H_REG_M = 0x69;
static constexpr uint8_t OUTY_L_REG_M = 0x6A;
static constexpr uint8_t OUTY_H_REG_M = 0x6B;
static constexpr uint8_t OUTZ_L_REG_M = 0x6C;
static constexpr uint8_t OUTZ_H_REG_M = 0x6D;

class LSM303AGR : public device::SPI, public px4::ScheduledWorkItem
{
public:
	LSM303AGR(int bus, const char *path, uint32_t device, enum Rotation rotation);
	virtual ~LSM303AGR();

	virtual int		init();

	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

protected:
	virtual int		probe();

private:

	bool			_collect_phase{false};

	unsigned		_measure_interval{0};

	unsigned		_call_mag_interval{0};

	mag_calibration_s	_mag_scale{};

	static constexpr float	_mag_range_scale{1.5f / 1000.0f}; // 1.5 milligauss/LSB
	static constexpr float	_mag_range_ga{49.152f};

	int			_class_instance{-1};

	unsigned		_mag_samplerate{100};

	perf_counter_t		_mag_sample_perf;
	perf_counter_t		_bad_registers;
	perf_counter_t		_bad_values;

	enum Rotation		_rotation;

	orb_advert_t		_mag_topic{nullptr};
	int					_mag_orb_class_instance{-1};

	/**
	 * Start automatic measurement.
	 */
	void			start();

	/**
	 * Stop automatic measurement.
	 */
	void			stop();

	/**
	 * Reset chip.
	 *
	 * Resets the chip and measurements ranges, but not scale and offset.
	 */
	int				reset();


	bool			self_test();

	/**
	 * Issue a measurement command.
	 *
	 * @return		OK if the measurement command was successful.
	 */
	void			measure();

	/**
	 * Collect the result of the most recent measurement.
	 */
	int			collect();

	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 *
	 * This is the heart of the measurement state machine.  This function
	 * alternately starts a measurement, or collects the data from the
	 * previous measurement.
	 *
	 * When the interval between measurements is greater than the minimum
	 * measurement interval, a gap is inserted between collection
	 * and measurement to provide the most recent measurement possible
	 * at the next interval.
	 */
	void			Run() override;

	/**
	 * Read a register from the LSM303AGR
	 *
	 * @param		The register to read.
	 * @return		The value that was read.
	 */
	uint8_t			read_reg(unsigned reg);

	/**
	 * Write a register in the LSM303AGR
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void			write_reg(unsigned reg, uint8_t value);

	/* this class cannot be copied */
	LSM303AGR(const LSM303AGR &);
	LSM303AGR operator=(const LSM303AGR &);
};

#endif /* DRIVERS_IMU_LSM303AGR_LSM303AGR_HPP_ */
