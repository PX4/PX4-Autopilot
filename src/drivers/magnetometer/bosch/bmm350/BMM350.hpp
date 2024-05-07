/****************************************************************************
 *
 *   Copyright (c) 2024 Technology Innovation Institute. All rights reserved.
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
 * @file BMM350.hpp
 *
 * Driver for the Bosch BMM350 connected via I2C.
 *
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/drivers/device/i2c.h>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>

#include "BMM350_registers.hpp"

using namespace Bosch_BMM350;

class BMM350 : public device::I2C, public I2CSPIDriver<BMM350>
{
public:
	BMM350(const I2CSPIDriverConfig &config);
	~BMM350() override;

	static void print_usage();

	void RunImpl();

	int init() override;
	void print_status() override;

private:
	// Result from measure call
	enum BMM350_measure_res {
		BMM350_MEAS_DONE = 0,
		BMM350_MEAS_NODATA,
		BMM350_MEAS_IO_ERR
	};

	// Cross axis compensation structure
	struct bmm350_cross_axis {
		float cross_x_y;
		float cross_y_x;
		float cross_z_x;
		float cross_z_y;
	};

	// Compensate structure
	struct bmm350_mag_compensate {
		float offset_coef[3]; // x, y, z
		float offset_coef_t;  // t
		float sensit_coef[3]; // x, y, z
		float sensit_coef_t;  // t
		float tco[3];
		float tcs[3];
		float t0;
		struct bmm350_cross_axis cross_axis;
	};

	int probe() override;
	bool Reset();
	bool Configure();

	int RegisterRead(Register reg, uint8_t *val);
	int RegisterWrite(Register reg, uint8_t value);
	int RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits);

	// Set ODR and averaging (performance)

	int SetODR_AVG(uint8_t aggr);

	// Read one 16-bit word from OTP memory

	int read_otp_word(uint8_t addr, uint16_t *lsb_msb);

	// Read factory compensation values from OTP

	int update_mag_off_sens();

	// Obtain the compensated magnetometer data in micro-tesla.

	void compensate_xyzt(float *xyzt);

	// Extend register value sign to 32-bit signed integer

	inline int32_t fix_sign(uint32_t inval, int8_t number_of_bits)
	{
		uint32_t mask = 0xffffffff << (number_of_bits - 1);
		return (int32_t)(inval & mask ? inval | mask : inval);
	}

	// Read measurement. ignore_time flag reads out the data always, regardless of
	// the timestamp, i.e. whether the data is new or not. This is used when reading
	// in forced mode, where the timer is not running.

	enum BMM350_measure_res measure(float *raw_xyzt, bool ignore_time);

	// Structure for mag compensate

	struct bmm350_mag_compensate mag_comp;

	PX4Magnetometer _px4_mag;

	perf_counter_t _bad_transfer_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad transfer")};
	perf_counter_t _reset_perf{perf_alloc(PC_COUNT, MODULE_NAME": reset")};
	perf_counter_t _overflow_perf{perf_alloc(PC_COUNT, MODULE_NAME": overflow")};
	perf_counter_t _self_test_failed_perf{perf_alloc(PC_COUNT, MODULE_NAME": self test failed")};

	int _failure_count{0};

	enum class STATE : uint8_t {
		RESET,
		WAIT_FOR_RESET,
		CONFIGURE,
		FGR,
		BR,
		MAG_RESET_DONE,
		MEASURE_FORCED,
		SELF_TEST,
		SET_NORMAL_MODE,
		READ,
	} _state{STATE::RESET};

	enum class SELFTEST_STATE : uint8_t {
		ST_INIT,
		ST_POS_X,
		ST_NEG_X,
		ST_POS_Y,
		ST_NEG_Y,
	} _selftest_state{SELFTEST_STATE::ST_INIT};

	uint32_t _prev_sensortime{0};

	// Initial measurement before self test execution
	float _initial_xyzt[4];

	// Constants

	static constexpr uint16_t BMM350_LSB_MASK = 0x00FF;
	static constexpr uint16_t BMM350_MSB_MASK = 0xFF00;

	// Offsets to data in OTP

	static constexpr uint8_t BMM350_TEMP_OFF_SENS = 0x0D;
	static constexpr uint8_t BMM350_MAG_OFFSET_X = 0x0E;
	static constexpr uint8_t BMM350_MAG_OFFSET_Y = 0x0F;
	static constexpr uint8_t BMM350_MAG_OFFSET_Z = 0x10;
	static constexpr uint8_t BMM350_MAG_SENS_X = 0x10;
	static constexpr uint8_t BMM350_MAG_SENS_Y = 0x11;
	static constexpr uint8_t BMM350_MAG_SENS_Z = 0x11;
	static constexpr uint8_t BMM350_MAG_TCO_X = 0x12;
	static constexpr uint8_t BMM350_MAG_TCO_Y = 0x13;
	static constexpr uint8_t BMM350_MAG_TCO_Z = 0x14;
	static constexpr uint8_t BMM350_MAG_TCS_X = 0x12;
	static constexpr uint8_t BMM350_MAG_TCS_Y = 0x13;
	static constexpr uint8_t BMM350_MAG_TCS_Z = 0x14;
	static constexpr uint8_t BMM350_MAG_T_0 = 0x18;
	static constexpr uint8_t BMM350_CROSS_X_Y = 0x15;
	static constexpr uint8_t BMM350_CROSS_Y_X = 0x15;
	static constexpr uint8_t BMM350_CROSS_Z_X = 0x16;
	static constexpr uint8_t BMM350_CROSS_Z_Y = 0x16;
	static constexpr float BMM350_SENS_CORR_Y = 0.01f;
	static constexpr float BMM350_TCS_CORR_Z = 0.0001f;

	// These constants are just copied from Bosch SensorAPI

	static constexpr float BXY_SENS = 14.55f;
	static constexpr float BZ_SENS = 9.0f;
	static constexpr float TEMP_SENS = 0.00204f;
	static constexpr float INA_XY_GAIN_TRGT = 19.46f;
	static constexpr float INA_Z_GAIN_TRGT = 31.0;
	static constexpr float ADC_GAIN = 1 / 1.5f;
	static constexpr float LUT_GAIN = 0.714607238769531f;
	static constexpr float POWER = (1000000.0f / 1048576.0f);
	float lsb_to_ut_degc[4] = {
		(POWER / (BXY_SENS *INA_XY_GAIN_TRGT *ADC_GAIN * LUT_GAIN)),
		(POWER / (BXY_SENS *INA_XY_GAIN_TRGT *ADC_GAIN * LUT_GAIN)),
		(POWER / (BZ_SENS *INA_Z_GAIN_TRGT *ADC_GAIN * LUT_GAIN)),
		1 / (TEMP_SENS *ADC_GAIN *LUT_GAIN * 1048576)
	};
};
