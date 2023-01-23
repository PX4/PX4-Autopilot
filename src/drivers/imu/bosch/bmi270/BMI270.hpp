/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * @file BMI270.hpp
 *
 * Driver for the Bosch BMI270 connected via SPI.
 *
 */

#pragma once

#include "Bosch_BMI270_registers.hpp"

#include <drivers/drv_hrt.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/device/spi.h>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/i2c_spi_buses.h>

using namespace Bosch_BMI270;

class BMI270 : public device::SPI, public I2CSPIDriver<BMI270>
{
public:
	BMI270(const I2CSPIDriverConfig &config);
	~BMI270() override;

	static void print_usage();

	void RunImpl();

	int init() override;
	void print_status() override;

private:
	void exit_and_cleanup() override;

	// Sensor Configuration
	static constexpr uint32_t RATE{1600}; // 1600 Hz
	static constexpr float FIFO_SAMPLE_DT{1e6f / RATE};

	static constexpr uint8_t ID_088 = 0x1E;
	static constexpr uint8_t ID_090L = 0x1A;

	static constexpr int32_t FIFO_MAX_SAMPLES{math::min(FIFO::SIZE / sizeof(FIFO::Data), sizeof(sensor_accel_fifo_s::x) / sizeof(sensor_accel_fifo_s::x[0]))};


	hrt_abstime _temperature_update_timestamp{0};

	struct FIFOLengthReadBuffer {
		uint8_t cmd{static_cast<uint8_t>(Register::FIFO_LENGTH_0) | DIR_READ};
		uint8_t dummy{0};
		uint8_t FIFO_LENGTH_0{0};
		uint8_t FIFO_LENGTH_1{0};
	};

	struct FIFOReadBuffer {
		uint8_t cmd{static_cast<uint8_t>(Register::FIFO_DATA) | DIR_READ};
		uint8_t dummy{0};
		FIFO::Data f[FIFO_MAX_SAMPLES] {};
	};

	// ensure no struct padding
	static_assert(sizeof(FIFOReadBuffer) == (2 + FIFO_MAX_SAMPLES *sizeof(FIFO::Data)));

	struct register_config_t {
		Register reg;
		uint8_t set_bits{0};
		uint8_t clear_bits{0};
	};

	int probe() override;

	bool Reset();

	bool Configure();

	void ProcessGyro(sensor_gyro_fifo_s *gyro, FIFO::Data *gyro_frame);
	void ProcessAccel(sensor_accel_fifo_s *accel, FIFO::Data *accel_frame);

	bool readAccelFrame(FIFO::Data *accel_frame);
	bool readGyroFrame(FIFO::Data *gyro_frame);

	void ConfigurePwr();
	void SetAccelScaleAndRange();
	void SetGyroScale();

	void CheckErrorRegister();

	void ConfigureFifo();
	void ConfigureSampleRate(int sample_rate = 0);
	void ConfigureFIFOWatermark(uint8_t samples);

	static int DataReadyInterruptCallback(int irq, void *context, void *arg);
	void DataReady();
	bool DataReadyInterruptConfigure();
	bool DataReadyInterruptDisable();

	bool RegisterCheck(const register_config_t &reg_cfg);

	uint8_t RegisterRead(Register reg);
	void RegisterWrite(Register reg, uint8_t value);
	void RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits);

	uint16_t FIFOReadCount();
	//bool FIFORead(const hrt_abstime &timestamp_sample, uint8_t samples);
	bool FIFORead(const hrt_abstime &timestamp_sample, uint16_t fifo_bytes);
	void FIFOReset();

	void UpdateTemperature();

	const spi_drdy_gpio_t _drdy_gpio;

	PX4Accelerometer _px4_accel;
	PX4Gyroscope _px4_gyro;

	perf_counter_t _bad_register_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad register")};
	perf_counter_t _bad_transfer_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad transfer")};
	perf_counter_t _fifo_empty_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO empty")};
	perf_counter_t _fifo_overflow_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO overflow")};
	perf_counter_t _fifo_reset_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO reset")};

	perf_counter_t _drdy_missed_perf{nullptr};

	hrt_abstime _reset_timestamp{0};
	hrt_abstime _last_config_check_timestamp{0};
	int _failure_count{0};

	px4::atomic<hrt_abstime> _drdy_timestamp_sample{0};
	bool _data_ready_interrupt_enabled{false};

	enum class STATE : uint8_t {
		WAIT_FOR_RESET,
		RESET,
		MICROCODE_LOAD,
		CONFIGURE,
		FIFO_READ,
	} _state{STATE::RESET};

	uint16_t _fifo_empty_interval_us{1250}; // default 1250 us / 800 Hz transfer interval
	int32_t _fifo_gyro_samples{static_cast<int32_t>(_fifo_empty_interval_us / (1000000 / RATE))}; // checks out to be 2 ...

	uint8_t _checked_register{0};
	static constexpr uint8_t size_register_cfg{11};
	register_config_t _register_cfg[size_register_cfg] {
		// Register                        | Set bits, Clear bits
		{ Register::PWR_CONF,          0, ACC_PWR_CONF_BIT::acc_pwr_save },
		{ Register::PWR_CTRL,          PWR_CTRL_BIT::accel_en | PWR_CTRL_BIT::gyr_en | PWR_CTRL_BIT::temp_en,  0 },
		{ Register::ACC_CONF,              ACC_CONF_BIT::acc_bwp_Normal | ACC_CONF_BIT::acc_odr_1600, Bit1 | Bit0 },
		{ Register::GYR_CONF,              GYR_CONF_BIT::gyr_odr_1k6 | GYR_CONF_BIT::gyr_flt_mode_normal | GYR_CONF_BIT::gyr_noise_hp | GYR_CONF_BIT::gyr_flt_hp, Bit0 | Bit1 | Bit4},
		{ Register::ACC_RANGE,             ACC_RANGE_BIT::acc_range_16g, 0 },
		{ Register::FIFO_WTM_0,            0, 0 },
		{ Register::FIFO_WTM_1,            0, 0 },
		{ Register::FIFO_CONFIG_0,         FIFO_CONFIG_0_BIT::BIT1_ALWAYS | FIFO_CONFIG_0_BIT::FIFO_mode, 0 },
		{ Register::FIFO_CONFIG_1,         FIFO_CONFIG_1_BIT::BIT4_ALWAYS | FIFO_CONFIG_1_BIT::Acc_en | FIFO_CONFIG_1_BIT::Gyr_en, 0 },
		{ Register::INT1_IO_CTRL,          INT1_IO_CONF_BIT::int1_out, 0 },
		{ Register::INT_MAP_DATA,    INT1_INT2_MAP_DATA_BIT::int1_fwm, 0},
	};
};
