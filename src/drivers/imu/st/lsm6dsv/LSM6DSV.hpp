/****************************************************************************
 *
 *   Copyright (c) 2024-2026 PX4 Development Team. All rights reserved.
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
 * @file LSM6DSV.hpp
 *
 * Driver for the ST LSM6DSV connected via SPI.
 *
 */

#pragma once

#include "ST_LSM6DSV_Registers.hpp"

#include <drivers/drv_hrt.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/device/spi.h>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/i2c_spi_buses.h>

using namespace ST_LSM6DSV;

class LSM6DSV : public device::SPI, public I2CSPIDriver<LSM6DSV>
{
public:
	LSM6DSV(const I2CSPIDriverConfig &config);
	~LSM6DSV() override;

	static void print_usage();

	void RunImpl();

	int init() override;
	void print_status() override;

private:
	void exit_and_cleanup() override;

	// Sensor Configuration
	static constexpr int32_t FIFO_MAX_SAMPLES{static_cast<int32_t>(FIFO::MAX_DRAIN_SAMPLES)};

	// A FIFO word is a tag byte plus 6 data bytes. With IF_INC set the address rounds from
	// FIFO_DATA_OUT_Z_H back to FIFO_DATA_OUT_TAG at every word boundary, so the whole FIFO drains
	// as a single N*7 byte burst (AN5763 / AN6119 section 9.8).
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

	// RunImpl() bounds a drain to FIFO_MAX_SAMPLES whole sample periods; a period only partially
	// batched when the status was read adds up to one word short of a further period on top.
	static constexpr uint16_t FIFO_MAX_WORDS{
		static_cast<uint16_t>(FIFO_MAX_SAMPLES * FIFO::MAX_WORDS_PER_PERIOD + FIFO::MAX_WORDS_PER_PERIOD - 1)};

	struct FIFOTransferBuffer {
		uint8_t cmd{static_cast<uint8_t>(Register::FIFO_DATA_OUT_TAG) | DIR_READ};
		FIFOWord words[FIFO_MAX_WORDS] {};
	};
	static_assert(sizeof(FIFOTransferBuffer) == (1 + FIFO_MAX_WORDS * FIFO::WORD_SIZE), "Invalid transfer buffer size");

	// Sensor ODR and FIFO layout are variant-dependent (set in UpdateVariantRegisterConfig()):
	//   default (16X/32X/DSK320X): 2000 Hz, 2 FIFO words/period (gyro + low-g)
	//   LSM6DSV80X:                7680 Hz, 3 FIFO words/period (gyro + low-g + high-g)
	uint32_t _sensor_odr{GYRO_ODR};
	float    _fifo_sample_dt{1e6f / GYRO_ODR};
	uint8_t  _fifo_words_per_period{2};

	struct register_config_t {
		Register reg;
		uint8_t set_bits{0};
		uint8_t clear_bits{0};
	};

	int probe() override;

	bool Reset();

	bool Configure();
	void ConfigureSampleRate(int sample_rate);

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
	void ConfigureFIFOWatermark(uint8_t samples);
	void UpdateVariantRegisterConfig();

	// High-g accelerometer fallback (LSM6DSV80X / LSM6DSV320X)
	void ManageHighGFullScale(bool high_g_clipping);
	void ApplyHighGFullScale(float prev_scale_mg_per_lsb);

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

	enum class DeviceVariant : uint8_t {
		LSM6DSV16X,
		LSM6DSV32X,
		LSM6DSK320X,
		LSM6DSV80X,
		LSM6DSV320X,
	};
	DeviceVariant _device_variant{DeviceVariant::LSM6DSV16X};

	// The LSM6DSV80X and LSM6DSV320X share WHO_AM_I 0x73 and cannot be distinguished over SPI, so
	// the physically-installed part is selected explicitly at start via the -T argument (config.custom1):
	// 320 -> LSM6DSV320X, anything else (incl. 80 / unset) -> LSM6DSV80X.
	const int _highg_variant_arg;

	// High-g accelerometer fallback (LSM6DSV80X / LSM6DSV320X): published in place of the
	// low-g channel whenever the low-g channel clips. Full-scale auto-escalates through the
	// variant's _hg_table on high-g clipping and de-escalates after a sustained quiet period.
	bool _high_g_enabled{false};
	const HighGFullScale *_hg_table{nullptr}; // ascending full-scale steps for this variant
	uint8_t _hg_table_size{0};
	uint8_t _hg_index{0};                     // current step within _hg_table
	bool _hg_scale_changed{false};            // high-g samples from before a full-scale change may still be batched
	float _hg_stale_scale_ratio{1.f};         // previous/current sensitivity ratio to normalize those samples
	hrt_abstime _hg_scale_change_timestamp{0};
	hrt_abstime _hg_last_clip_timestamp{0};
	static constexpr hrt_abstime HG_DEESCALATE_TIMEOUT_US{2'000'000}; // 2 s

	uint16_t _fifo_empty_interval_us{500}; // default 500 us / 2000 Hz
	int32_t _fifo_gyro_samples{static_cast<int32_t>(_fifo_empty_interval_us / (1000000 / GYRO_ODR))};

	uint8_t _checked_register{0};
	static constexpr uint8_t size_register_cfg{14};
	// Variant-dependent fields (HAODR_CFG, CTRL1/2/6/8, FIFO_CTRL3, COUNTER_BDR_REG1,
	// CTRL1_XL_HG) are overwritten in UpdateVariantRegisterConfig(); initializers below are
	// the default-variant (2000 Hz, no high-g) values. The high-g entries are no-ops
	// (set/clear = 0) unless enabled for the LSM6DSV80X.
	//
	// Configure() writes these in array order, and the order is significant: FS_G (CTRL6) must be
	// set while the gyro is in power-down, and the reset default FS_G=000 is reserved on the 80X /
	// 320X / DSK320X. So every full-scale, filter and FIFO register comes first, and CTRL1/CTRL2 —
	// which set the ODRs and thereby power the sensors up — come last. HAODR_CFG selects the ODR
	// set that the CTRL1/CTRL2 codes index into, so it must also precede them.
	register_config_t _register_cfg[size_register_cfg] {
		// Register                | Set bits                                              | Clear bits
		{ Register::CTRL3,          CTRL3_BIT::BDU | CTRL3_BIT::IF_INC,                     CTRL3_BIT::SW_RESET },
		{ Register::HAODR_CFG,      HAODR_CFG_BIT::HAODR_MODE1,                             0 },
		{ Register::CTRL6,          CTRL6_BIT::FS_G_2000DPS,                                 0 },
		{ Register::CTRL8,          CTRL8_BIT::FS_XL_16G | CTRL8_BIT::LPF2_BW_ODR_DIV_10,   0 },
		{ Register::CTRL9,          CTRL9_BIT::LPF2_XL_EN,                                   0 },
		{ Register::CTRL4,          CTRL4_BIT::DRDY_PULSED,                                  0 },
		{ Register::INT1_CTRL,      INT1_CTRL_BIT::INT1_FIFO_TH,                             0 },
		{ Register::FIFO_CTRL1,     0, 0 }, // WTM[7:0] set at runtime by ConfigureFIFOWatermark()
		{
			Register::FIFO_CTRL3,     static_cast<uint8_t>(FIFO_CTRL3_BIT::BDR_GY_HAODR) |
			static_cast<uint8_t>(FIFO_CTRL3_BIT::BDR_XL_HAODR),      0
		},
		{ Register::FIFO_CTRL4,     FIFO_CTRL4_BIT::FIFO_MODE_CONTINUOUS,                    0 },
		{ Register::COUNTER_BDR_REG1, 0, 0 }, // XL_HG_BATCH_EN set per-variant (80X high-g)
		{ Register::CTRL1_XL_HG,    0, 0 }, // high-g ODR+FS set per-variant (80X high-g)
		{ Register::CTRL1,          HAODR_MODE1_ODR_2000HZ | CTRL1_BIT::CTRL1_MODE_HAODR,    0 },
		{ Register::CTRL2,          HAODR_MODE1_ODR_2000HZ | CTRL2_BIT::CTRL2_MODE_HAODR,    0 },
	};
};
