/****************************************************************************
 *
 *   Copyright (c) 2017-2019 PX4 Development Team. All rights reserved.
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
 * @file FXAS21002C.hpp
 * Driver for the NXP FXAS21002C 3-Axis Digital Angular Rate Gyroscope
 * connected via SPI
 */

#pragma once

#include <drivers/device/spi.h>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <perf/perf_counter.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>


class FXAS21002C : public device::SPI, public px4::ScheduledWorkItem
{
public:
	FXAS21002C(int bus, uint32_t device, enum Rotation rotation);
	virtual ~FXAS21002C();

	virtual int init();

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void print_info();

	/**
	 * dump register values
	 */
	void print_registers();

	/**
	 * deliberately trigger an error
	 */
	void test_error();

protected:
	virtual int probe();

private:

	PX4Gyroscope _px4_gyro;

	unsigned _current_rate{800};

	unsigned _read{0};

	perf_counter_t _sample_perf;
	perf_counter_t _errors;
	perf_counter_t _bad_registers;
	perf_counter_t _duplicates;

	uint8_t _register_wait{0};

	/* this is used to support runtime checking of key
	 *configuration registers to detect SPI bus errors and sensor
	 * reset
	 */
	static constexpr int FXAS21002C_NUM_CHECKED_REGISTERS{6};

	uint8_t _checked_values[FXAS21002C_NUM_CHECKED_REGISTERS] {};
	uint8_t _checked_next{0};

	/**
	 * Start automatic measurement.
	 */
	void start();

	/**
	 * Stop automatic measurement.
	 */
	void stop();

	/**
	 * Reset chip.
	 *
	 * Resets the chip and measurements ranges, but not scale and offset.
	 */
	void reset();

	/**
	 * Put the chip In stand by
	 */
	void set_standby(int rate, bool standby_true);

	void Run() override;

	/**
	 * check key registers for correct values
	 */
	void check_registers(void);

	/**
	 * Fetch accel measurements from the sensor and update the report ring.
	 */
	void measure();

	/**
	 * Read a register from the FXAS21002C
	 *
	 * @param		The register to read.
	 * @return		The value that was read.
	 */
	uint8_t read_reg(unsigned reg);

	/**
	 * Write a register in the FXAS21002C
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void write_reg(unsigned reg, uint8_t value);

	/**
	 * Modify a register in the FXAS21002C
	 *
	 * Bits are cleared before bits are set.
	 *
	 * @param reg		The register to modify.
	 * @param clearbits	Bits in the register to clear.
	 * @param setbits	Bits in the register to set.
	 */
	void modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits);

	/**
	 * Write a register in the FXAS21002C, updating _checked_values
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void write_checked_reg(unsigned reg, uint8_t value);

	/**
	 * Set the FXAS21002C measurement range.
	 *
	 * @param max_dps	The measurement range is set to permit reading at least
	 *			this rate in degrees per second.
	 *			Zero selects the maximum supported range.
	 * @return		OK if the value can be supported, -ERANGE otherwise.
	 */
	int set_range(unsigned max_dps);

	/**
	 * Set the FXAS21002C internal sampling frequency.
	 *
	 * @param frequency	The internal sampling frequency is set to not less than
	 *			this value.
	 *			Zero selects the maximum rate supported.
	 * @return		OK if the value can be supported.
	 */
	int set_samplerate(unsigned frequency);

	/*
	  set onchip low pass filter frequency
	 */
	void set_onchip_lowpass_filter(int frequency_hz);
};
