/****************************************************************************
 *
 *   Copyright (c) 2013-2022 PX4 Development Team. All rights reserved.
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
 * @file LSM303D.hpp
 *
 * Driver for the ST LSM303D connected via SPI.
 *
 */

#pragma once

#include "ST_LSM303D_registers.hpp"

#include <drivers/drv_hrt.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/device/spi.h>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>
#include <lib/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/i2c_spi_buses.h>

using namespace ST_LSM303D;

class LSM303D : public device::SPI, public I2CSPIDriver<LSM303D>
{
public:
	LSM303D(const I2CSPIDriverConfig &config);
	~LSM303D() override;

	static void print_usage();

	void RunImpl();

	int init() override;
	void print_status() override;

private:
	// Sensor Configuration
	static constexpr float FIFO_SAMPLE_DT{1e6f / ST_LSM303D::LA_ODR};
	static constexpr float ACCEL_RATE{ST_LSM303D::LA_ODR};

	static constexpr uint32_t FIFO_MAX_SAMPLES{math::min(FIFO::SIZE / sizeof(FIFO::DATA), sizeof(sensor_accel_fifo_s::x) / sizeof(sensor_accel_fifo_s::x[0]))};

	// Transfer data
	struct FIFOTransferBuffer {
		uint8_t cmd{static_cast<uint8_t>(Register::OUT_X_L_A) | DIR_READ | AUTO_INCREMENT};
		FIFO::DATA f[FIFO_MAX_SAMPLES] {};
	};
	// ensure no struct padding
	static_assert(sizeof(FIFOTransferBuffer) == (1 + FIFO_MAX_SAMPLES *sizeof(FIFO::DATA)));

	struct register_config_t {
		Register reg;
		uint8_t set_bits{0};
		uint8_t clear_bits{0};
	};

	int probe() override;

	bool Reset();

	bool Configure();
	void ConfigureSampleRate(int sample_rate);
	void ConfigureFIFOWatermark(uint8_t samples);

	bool RegisterCheck(const register_config_t &reg_cfg);

	uint8_t RegisterRead(Register reg);
	void RegisterWrite(Register reg, uint8_t value);
	void RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits);
	void RegisterSetBits(Register reg, uint8_t setbits) { RegisterSetAndClearBits(reg, setbits, 0); }
	void RegisterClearBits(Register reg, uint8_t clearbits) { RegisterSetAndClearBits(reg, 0, clearbits); }

	bool FIFORead(const hrt_abstime &timestamp_sample, uint8_t samples);
	void FIFOReset();

	void ProcessAccel(const hrt_abstime &timestamp_sample, const FIFO::DATA fifo[], const uint8_t samples);
	bool UpdateTemperatureAndMagnetometer();

	PX4Accelerometer _px4_accel;
	PX4Magnetometer _px4_mag;

	perf_counter_t _bad_register_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad register")};
	perf_counter_t _bad_transfer_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad transfer")};
	perf_counter_t _fifo_empty_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO empty")};
	perf_counter_t _fifo_overflow_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO overflow")};
	perf_counter_t _fifo_reset_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO reset")};

	hrt_abstime _reset_timestamp{0};
	hrt_abstime _last_config_check_timestamp{0};
	hrt_abstime _temperature_and_magnetometer_update_timestamp{0};
	int _failure_count{0};

	enum class STATE : uint8_t {
		RESET,
		WAIT_FOR_RESET,
		CONFIGURE,
		FIFO_READ,
	} _state{STATE::RESET};

	uint16_t _fifo_empty_interval_us{1000};
	uint32_t _fifo_samples{static_cast<uint32_t>(_fifo_empty_interval_us / (1000000 / ACCEL_RATE))};

	uint8_t _checked_register{0};
	static constexpr uint8_t size_register_cfg{8};
	register_config_t _register_cfg[size_register_cfg] {
		// Register                | Set bits, Clear bits
		{ Register::CTRL0,         CTRL0_BIT::FIFO_EN, CTRL0_BIT::BOOT | CTRL0_BIT::BIT4_MUST_BE_CLEARED | CTRL0_BIT::BIT3_MUST_BE_CLEARED },
		{ Register::CTRL1,         CTRL1_BIT::AODR_1600HZ_SET | CTRL1_BIT::BDU | CTRL1_BIT::AZEN | CTRL1_BIT::AYEN | CTRL1_BIT::AXEN, CTRL1_BIT::AODR_1600HZ_CLEAR },
		{ Register::CTRL2,         CTRL2_BIT::AFS_16G_SET, CTRL2_BIT::ABW_773_HZ | CTRL2_BIT::AFS_16G_CLEAR | CTRL2_BIT::BIT2_MUST_BE_CLEARED },
		{ Register::CTRL5,         CTRL5_BIT::TEMP_EN | CTRL5_BIT::M_RES_HIGH | CTRL5_BIT::M_ODR_50_HZ_SET, CTRL5_BIT::M_ODR_50_HZ_CLEAR },
		{ Register::CTRL6,         CTRL6_BIT::MFS_12_GAUSS, 0 },
		{ Register::CTRL7,         0, CTRL7_BIT::MD },
		{ Register::FIFO_CTRL,     FIFO_CTRL_BIT::FIFO_MODE_SET, FIFO_CTRL_BIT::FIFO_MODE_CLEAR },
	};
};
