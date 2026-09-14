/**
 * @file ASM330LHH.hpp
 *
 * Driver for the ST ASM330LHH automotive 6-axis IMU connected via SPI.
 *
 * Also covers the ASM330LHHX and ASM330LHHXG1, which share WHO_AM_I 0x6B and an
 * identical accelerometer, gyroscope and FIFO register map. Their additional MLC /
 * FSM / sensor-hub blocks are left in their reset state and are not used.
 *
 * Copyright (c) 2026, STMicroelectronics.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     1. Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *     2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     3. Neither the name of the STMicroelectronics nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **/

#pragma once

#include "ST_ASM330LHH_Registers.hpp"

#include <drivers/drv_hrt.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/device/spi.h>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/i2c_spi_buses.h>

using namespace ST_ASM330LHH;

class ASM330LHH : public device::SPI, public I2CSPIDriver<ASM330LHH>
{
public:
	ASM330LHH(const I2CSPIDriverConfig &config);
	~ASM330LHH() override;

	static void print_usage();

	void RunImpl();

	int init() override;
	void print_status() override;

private:
	void exit_and_cleanup() override;

	// Sensor configuration
	static constexpr float FIFO_SAMPLE_DT{1e6f / GYRO_ODR};

	static constexpr int32_t FIFO_MAX_SAMPLES{static_cast<int32_t>(FIFO::MAX_DRAIN_SAMPLES)};
	static_assert(FIFO_MAX_SAMPLES <= (int32_t)(sizeof(sensor_gyro_fifo_s::x) / sizeof(sensor_gyro_fifo_s::x[0])),
		      "FIFO drain exceeds sensor_gyro_fifo capacity");
	static_assert(FIFO_MAX_SAMPLES <= (int32_t)(sizeof(sensor_accel_fifo_s::x) / sizeof(sensor_accel_fifo_s::x[0])),
		      "FIFO drain exceeds sensor_accel_fifo capacity");

	// A FIFO word is a tag byte plus 6 data bytes. With IF_INC set the address wraps from
	// FIFO_DATA_OUT_Z_H (0x7E) back to FIFO_DATA_OUT_TAG (0x78) at every word boundary, so N
	// queued words drain as a single N*7 byte burst (AN5296 section 9, "FIFO reading procedure").
	struct FIFOWord {
		uint8_t TAG;
		uint8_t DATA_X_L;
		uint8_t DATA_X_H;
		uint8_t DATA_Y_L;
		uint8_t DATA_Y_H;
		uint8_t DATA_Z_L;
		uint8_t DATA_Z_H;
	};
	static_assert(sizeof(FIFOWord) == FIFO::WORD_SIZE, "FIFO word must be 7 bytes");

	// RunImpl() drains whole sample periods only, at most FIFO_MAX_SAMPLES of them
	static constexpr uint16_t FIFO_MAX_WORDS{static_cast<uint16_t>(FIFO_MAX_SAMPLES * FIFO::MAX_WORDS_PER_PERIOD)};

	struct FIFOTransferBuffer {
		uint8_t cmd{static_cast<uint8_t>(Register::FIFO_DATA_OUT_TAG) | DIR_READ};
		FIFOWord words[FIFO_MAX_WORDS] {};
	};
	static_assert(sizeof(FIFOTransferBuffer) == (1 + FIFO_MAX_WORDS * FIFO::WORD_SIZE), "Invalid transfer buffer size");

	// Held here rather than on the work queue stack: a wq:SPIx frame is not the place for a
	// buffer this size, and reusing it avoids re-zeroing memory that transfer() overwrites anyway.
	FIFOTransferBuffer _fifo_buffer{};

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

	bool FIFORead(const hrt_abstime &timestamp_sample, uint16_t words);
	void FIFOReset();

	void UpdateTemperature();

	static int DataReadyInterruptCallback(int irq, void *context, void *arg);
	void DataReady();
	bool DataReadyInterruptConfigure();
	bool DataReadyInterruptDisable();

	const spi_drdy_gpio_t _drdy_gpio;
	PX4Accelerometer _px4_accel;
	PX4Gyroscope _px4_gyro;

	perf_counter_t _bad_transfer_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad transfer")};
	perf_counter_t _fifo_empty_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO empty")};
	perf_counter_t _fifo_overflow_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO overflow")};
	// Last timestamp_sample handed to the uORB publications. vehicle_imu rejects a batch whose
	// timestamp_sample does not advance, so the jitter adjustment above is checked against it.
	hrt_abstime _last_timestamp_sample{0};
	perf_counter_t _fifo_reset_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO reset")};
	perf_counter_t _drdy_missed_perf{nullptr};

	hrt_abstime _reset_timestamp{0};
	hrt_abstime _temperature_update_timestamp{0};
	int _failure_count{0};

	px4::atomic<hrt_abstime> _drdy_timestamp_sample{0};
	bool _data_ready_interrupt_enabled{false};

	enum class STATE : uint8_t {
		RESET,
		WAIT_FOR_RESET,
		CONFIGURE,
		FIFO_RESET,
		FIFO_READ,
	} _state{STATE::RESET};

	uint16_t _fifo_empty_interval_us{300}; // default 300 us / 3.33 kHz
	int32_t _fifo_gyro_samples{static_cast<int32_t>(_fifo_empty_interval_us / (1000000 / GYRO_ODR))};

	static constexpr uint8_t size_register_cfg{14};

	// Configure() writes these in array order and the order is significant: full-scale, filter,
	// interrupt and FIFO settings must be in place before CTRL1_XL / CTRL2_G, which set the ODRs
	// and thereby bring the sensors out of power-down.
	register_config_t _register_cfg[size_register_cfg] {
		// Register                   | Set bits                                          | Clear bits
		{ Register::CTRL3_C,            CTRL3_C_BIT::BDU | CTRL3_C_BIT::IF_INC,             CTRL3_C_BIT::SW_RESET },
		{ Register::CTRL9_XL,           CTRL9_XL_BIT::DEVICE_CONF,                          CTRL9_XL_BIT::DEN_XL_EN },
		// SPI-only wiring: disable the I2C interface and use a pulsed (not latched) INT1
		{ Register::CTRL4_C,            CTRL4_C_BIT::I2C_DISABLE,                           CTRL4_C_BIT::LPF1_SEL_G | CTRL4_C_BIT::SLEEP_G },
		{ Register::COUNTER_BDR_REG1,   COUNTER_BDR_REG1_BIT::DATAREADY_PULSED,             0 },
		{ Register::CTRL6_C,            0,                                                  CTRL6_C_BIT::XL_HM_MODE },
		{ Register::CTRL7_G,            0,                                                  CTRL7_G_BIT::G_HM_MODE | CTRL7_G_BIT::HP_EN_G },
		{ Register::CTRL8_XL,           CTRL8_XL_BIT::LPF2_BW_ODR_DIV_10,                   CTRL8_XL_BIT::HP_SLOPE_XL_EN },
		{ Register::INT1_CTRL,          INT1_CTRL_BIT::INT1_FIFO_TH,                        0 },
		{ Register::FIFO_CTRL1,         0, 0 }, // WTM[7:0] set at runtime by ConfigureFIFOWatermark()
		{ Register::FIFO_CTRL2,         0,                                                  FIFO_CTRL2_BIT::WTM8 | FIFO_CTRL2_BIT::STOP_ON_WTM },
		{
			Register::FIFO_CTRL3,         static_cast<uint8_t>(FIFO_CTRL3_BIT::BDR_GY_3330HZ) |
			static_cast<uint8_t>(FIFO_CTRL3_BIT::BDR_XL_3330HZ),                              0
		},
		// FIFO_CTRL4 leaves ODR_T_BATCH and DEC_TS_BATCH at 0: neither temperature nor timestamp
		// is batched, so every sample period is exactly two words (gyro + accel).
		{ Register::FIFO_CTRL4,         FIFO_CTRL4_BIT::FIFO_MODE_CONTINUOUS,               0 },
		// CTRL1_XL / CTRL2_G come last: writing ODR_XL / ODR_G is what brings the sensors out
		// of power-down, so every full-scale and filter bit above is already latched by then.
		{
			Register::CTRL1_XL,           ODR_BIT::ODR_3330HZ | CTRL1_XL_BIT::FS_XL_16G |
			CTRL1_XL_BIT::LPF2_XL_EN,                                                         0
		},
		{ Register::CTRL2_G,            ODR_BIT::ODR_3330HZ | CTRL2_G_BIT::FS_G_2000DPS,     0 },
	};
};
