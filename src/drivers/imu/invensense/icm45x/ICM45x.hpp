/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file ICM45x.hpp
 * @brief Gen-4 BANK_0 + IREG 6400 Hz little-endian FIFO driver for ICM-45686 and ICM-56686.
 */

#pragma once

#include "ICM45xConfigs.hpp"

#include <drivers/drv_hrt.h>
#include <drivers/drv_sensor.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/device/spi.h>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/drivers/imu/FifoSampleStats.hpp>
#include <lib/drivers/imu/ImuPerf.hpp>
#include <lib/drivers/imu/SpiFamily.hpp>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/px4_config.h>
#include <uORB/topics/sensor_accel_fifo.h>
#include <uORB/topics/sensor_gyro_fifo.h>

class ICM45x final : public device::SPI, public I2CSPIDriver<ICM45x>
{
public:
	ICM45x(const I2CSPIDriverConfig &config);
	~ICM45x() override;

	static void print_usage();

	static constexpr uint16_t maxTransferSize(uint8_t packet_size, uint8_t prefix)
	{
		return prefix + kFifoMaxSamples * packet_size;
	}

	void RunImpl();
	int init() override;
	void print_status() override;

	using Variant = icm45x_config::Variant;
	using Bank0Config = icm45x_config::Bank0Config;
	using IregConfig = icm45x_config::IregConfig;
	using Profile = icm45x_config::Profile;

	static constexpr uint8_t kFifoMaxSamples {
		static_cast<uint8_t>(sizeof(sensor_gyro_fifo_s::x) / sizeof(sensor_gyro_fifo_s::x[0]))
	};

private:
	static constexpr uint8_t kFifoTransferPrefix { 1 };

	bool isIcm56686() const { return _profile.variant == Variant::kIcm56686; }

	void exit_and_cleanup() override;
	int probe() override;
	bool reset();
	void configureSampleRate(int sample_rate);

	bool configureIcm45686();
	bool fifoBypassIcm45686();
	bool fifoResetIcm45686();
	void updateTemperature();

	bool configureIcm56686();
	bool fifoBypassIcm56686();
	bool fifoResetIcm56686();
	bool processTemperature(const imu::FifoSampleStats &temperatures);

	bool fifoFlush(uint8_t fifo_config2_reg, uint8_t flush_bit);
	bool fifoReset();

	bool registerRead(uint8_t reg, uint8_t &value);
	bool registerWrite(uint8_t reg, uint8_t value);
	bool registerSetAndClearBits(uint8_t reg, uint8_t set_bits, uint8_t clear_bits);
	bool registerConfigure(const Bank0Config &register_config);
	bool registerConfigure(uint8_t reg);
	bool registerForceWrite(uint8_t reg);
	bool registerCheck(const Bank0Config &register_config);

	bool waitForIregReady();
	bool iregRead(uint16_t reg, uint8_t &value);
	bool iregWrite(uint16_t reg, uint8_t value);
	bool iregSetAndClearBits(uint16_t reg, uint8_t set_bits, uint8_t clear_bits);

	uint16_t fifoReadCount();
	bool fifoRead(const hrt_abstime &timestamp_sample);
	uint64_t errorCount() const;

	static int dataReadyInterruptCallback(int irq, void *context, void *arg);
	void dataReady();
	bool dataReadyInterruptConfigure();
	bool dataReadyInterruptDisable();

	imu::SpiEndpointGuard _endpoint_guard;
	const Profile        &_profile;
	const bool            _int2;
	const spi_mode_e      _spi_mode;
	const int             _register_frequency;
	const int             _data_frequency;
	const spi_drdy_gpio_t _drdy_gpio;
	const bool            _enable_clock_input;
	const float           _sample_dt_us;
	const uint8_t         _fifo_interrupt_config0;
	const uint8_t         _fifo_interrupt_config2;

	PX4Accelerometer  _px4_accel;
	PX4Gyroscope      _px4_gyro;
	imu::SpiDmaBuffer _dma_buffer { _profile.device.max_transfer_bytes };

	imu::TransferPerfCounters _transfer_perf {
		_profile.periodic_register_check ? MODULE_NAME ": bad register" : nullptr,
		MODULE_NAME ": bad transfer"
	};

	imu::FifoPerfCounters _fifo_perf {
		MODULE_NAME ": FIFO empty",
		MODULE_NAME ": FIFO overflow",
		MODULE_NAME ": FIFO reset"
	};

	imu::PerfCounter<PC_COUNT> _drdy_missed_perf {
		_drdy_gpio != 0
		? MODULE_NAME ": DRDY missed"
		: nullptr
	};

	hrt_abstime _reset_timestamp              { 0 };
	hrt_abstime _state_ready_at               { 0 };
	hrt_abstime _last_config_check_timestamp  { 0 };
	hrt_abstime _temperature_update_timestamp { 0 };
	int         _failure_count                { 0 };

	px4::atomic<hrt_abstime> _drdy_timestamp_sample { 0 };
	bool _data_ready_interrupt_enabled { false };
	bool _drdy_backlog_drain           { false };

	bool    _overrides_saved       { false };
	uint8_t _intf_config1_override { 0 };
	uint8_t _drive_config0         { 0 };

	enum class State : uint8_t {
		kReset,
		kWaitForReset,
		kConfigure,
		kFifoReset,
		kFifoRead,
	} _state { State::kReset };

	uint32_t    _fifo_empty_interval_us { 1250 };
	uint8_t     _fifo_gyro_samples      { 8 };
	uint8_t     _checked_register       { 0 };
	uint8_t     _bank0_config_count        { 0 };
	uint8_t     _ireg_config_count         { 0 };
	Bank0Config _bank0_config[icm45x_config::kMaxBank0Configs] {};
	IregConfig  _ireg_config[icm45x_config::kMaxIregConfigs] {};
};
