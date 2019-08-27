/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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


#include <px4_platform_common/config.h>
#include <lib/perf/perf_counter.h>
#include <systemlib/conversions.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/i2c.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>

#define AK09916_DEVICE_PATH_MAG              "/dev/ak09916_i2c_int"
#define AK09916_DEVICE_PATH_MAG_EXT          "/dev/ak09916_i2c_ext"

/* in 16-bit sampling mode the mag resolution is 1.5 milli Gauss per bit */
static constexpr float AK09916_MAG_RANGE_GA{1.5e-3f};

/* ak09916 deviating register addresses and bit definitions */
#define AK09916_I2C_ADDR         0x0C

#define AK09916_DEVICE_ID_A		0x48
#define AK09916_DEVICE_ID_B		0x09	// additional ID byte ("INFO" on AK9063 without content specification.)

#define AK09916REG_WIA           0x00

#define AK09916REG_ST1        0x10
#define AK09916REG_CNTL2          0x31
#define AK09916REG_CNTL3          0x32

#define AK09916_RESET				0x01
#define AK09916_CNTL2_SINGLE_MODE               0x01 /* default */
#define AK09916_CNTL2_CONTINOUS_MODE_100HZ      0x08

#pragma pack(push, 1)
struct ak09916_regs {
	uint8_t st1;
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t tmps;
	uint8_t st2;
};
#pragma pack(pop)

/**
 * Helper class implementing the magnetometer driver node.
 */
class AK09916 : public device::I2C, px4::ScheduledWorkItem
{
public:
	AK09916(int bus, const char *path, enum Rotation rotation);
	~AK09916();

	virtual int init();

	void read_block(uint8_t reg, uint8_t *val, uint8_t count);

	int reset(void);
	int probe(void);
	int setup(void);
	void print_info(void);
	int setup_master_i2c(void);
	bool check_id(uint8_t &id);

	void Run();

	void start(void);
	void stop(void);

protected:

	/* Directly measure from the _interface if possible */
	void measure();

	uint8_t read_reg(uint8_t reg);
	void write_reg(uint8_t reg, uint8_t value);

private:

	PX4Magnetometer _px4_mag;

	uint32_t _measure_interval{0};

	perf_counter_t _mag_reads;
	perf_counter_t _mag_errors;
	perf_counter_t _mag_overruns;
	perf_counter_t _mag_overflows;
	perf_counter_t _mag_duplicates;

	bool check_duplicate(uint8_t *mag_data);

	// keep last mag reading for duplicate detection
	uint8_t			_last_mag_data[6] {};

	/* do not allow to copy this class due to pointer data members */
	AK09916(const AK09916 &);
	AK09916 operator=(const AK09916 &);
};
