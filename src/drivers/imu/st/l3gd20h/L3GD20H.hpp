/****************************************************************************
 *
 *   Copyright (c) 2012-2022 PX4 Development Team. All rights reserved.
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
 * @file L3GD20H.hpp
 *
 * Driver for the ST L3GD20H connected via SPI.
 *
 */

#pragma once

#include "ST_L3GD20H_registers.hpp"

#include <drivers/drv_hrt.h>
#include <lib/drivers/device/spi.h>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/i2c_spi_buses.h>

using namespace ST_L3GD20H;

class L3GD20H : public device::SPI, public I2CSPIDriver<L3GD20H>
{
public:
	L3GD20H(const I2CSPIDriverConfig &config);
	~L3GD20H() override;

	static void print_usage();

	void RunImpl();

	int init() override;
	void print_status() override;

private:
	void exit_and_cleanup() override;

	// Sensor Configuration
	static constexpr float FIFO_SAMPLE_DT{1e6f / ST_L3GD20H::ODR};
	static constexpr float GYRO_RATE{ST_L3GD20H::ODR};

	static constexpr uint32_t FIFO_MAX_SAMPLES{math::min(FIFO::SIZE / sizeof(FIFO::DATA), sizeof(sensor_gyro_fifo_s::x) / sizeof(sensor_gyro_fifo_s::x[0]))};

	// Transfer data
	struct FIFOTransferBuffer {
		uint8_t cmd{static_cast<uint8_t>(Register::OUT_X_L) | DIR_READ | AUTO_INCREMENT};
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

	static int DataReadyInterruptCallback(int irq, void *context, void *arg);
	void DataReady();
	bool DataReadyInterruptConfigure();
	bool DataReadyInterruptDisable();

	bool RegisterCheck(const register_config_t &reg_cfg);

	uint8_t RegisterRead(Register reg);
	void RegisterWrite(Register reg, uint8_t value);
	void RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits);
	void RegisterSetBits(Register reg, uint8_t setbits) { RegisterSetAndClearBits(reg, setbits, 0); }
	void RegisterClearBits(Register reg, uint8_t clearbits) { RegisterSetAndClearBits(reg, 0, clearbits); }

	bool FIFORead(const hrt_abstime &timestamp_sample, uint8_t samples);
	void FIFOReset();

	void ProcessGyro(const hrt_abstime &timestamp_sample, const FIFO::DATA fifo[], const uint8_t samples);
	void UpdateTemperature();

	const spi_drdy_gpio_t _drdy_gpio;

	PX4Gyroscope _px4_gyro;

	perf_counter_t _bad_register_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad register")};
	perf_counter_t _bad_transfer_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad transfer")};
	perf_counter_t _fifo_empty_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO empty")};
	perf_counter_t _fifo_overflow_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO overflow")};
	perf_counter_t _fifo_reset_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO reset")};
	perf_counter_t _drdy_missed_perf{nullptr};

	hrt_abstime _reset_timestamp{0};
	hrt_abstime _last_config_check_timestamp{0};
	hrt_abstime _temperature_update_timestamp{0};
	int _failure_count{0};

	px4::atomic<hrt_abstime> _drdy_timestamp_sample{0};
	bool _data_ready_interrupt_enabled{false};

	enum class STATE : uint8_t {
		RESET,
		WAIT_FOR_RESET,
		CONFIGURE,
		FIFO_READ,
	} _state{STATE::RESET};

	uint16_t _fifo_empty_interval_us{1000};
	uint32_t _fifo_samples{static_cast<uint32_t>(_fifo_empty_interval_us / (1000000 / GYRO_RATE))};

	uint8_t _checked_register{0};
	static constexpr uint8_t size_register_cfg{6};
	register_config_t _register_cfg[size_register_cfg] {
		// Register            | Set bits, Clear bits
		{ Register::CTRL1,     CTRL1_BIT::ODR_800HZ_CUTOFF_100HZ | CTRL1_BIT::PD | CTRL1_BIT::Zen | CTRL1_BIT::Yen | CTRL1_BIT::Xen, 0 },
		{ Register::CTRL3,     CTRL3_BIT::H_Lactive | CTRL3_BIT::INT2_DRDY, CTRL3_BIT::INT2_FTH },
		{ Register::CTRL4,     CTRL4_BIT::BDU | CTRL4_BIT::FS_2000DPS, 0 },
		{ Register::CTRL5,     CTRL5_BIT::FIFO_EN, 0 },
		{ Register::FIFO_CTRL, FIFO_CTRL_BIT::FIFO_mode_SET, FIFO_CTRL_BIT::FIFO_mode_CLEAR },
		{ Register::LOW_ODR,   LOW_ODR_BIT::DRDY_HL | LOW_ODR_BIT::I2C_dis, LOW_ODR_BIT::SW_RES | LOW_ODR_BIT::Low_ODR },
	};
};
