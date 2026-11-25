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
 * @file ICM42x.hpp
 * @brief Banked big-endian 8 kHz TDK ICM42x SPI IMU family driver.
 */

#pragma once

#include "ICM42xConfigs.hpp"

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
#include <uORB/topics/sensor_accel_fifo.h>
#include <uORB/topics/sensor_gyro_fifo.h>

class ICM42x final : public device::SPI, public I2CSPIDriver<ICM42x>
{
public:
	ICM42x(const I2CSPIDriverConfig &config);
	~ICM42x() override;

	static void print_usage();

	static constexpr uint16_t maxTransferSize(uint8_t packet_size, uint8_t prefix = icm42x_config::kFifoTransferPrefix)
	{
		return prefix + kFifoMaxSamples * packet_size;
	}

	void RunImpl();
	int init() override;
	void print_status() override;

	using Variant = icm42x_config::Variant;
	using AddressSpace = icm42x_config::AddressSpace;
	using RegisterConfig = icm42x_config::RegisterConfig;
	using PacketFormat = icm42x_config::PacketFormat;
	using Profile = icm42x_config::Profile;

	static constexpr uint8_t kFifoMaxSamples {
		static_cast<uint8_t>(sizeof(sensor_gyro_fifo_s::x) / sizeof(sensor_gyro_fifo_s::x[0]))
	};

private:
	void exit_and_cleanup() override;
	int probe() override;
	bool reset();
	bool resetComplete();
	bool configure();
	bool deferredConfiguration(const RegisterConfig &config) const;
	bool checkConfiguration();
	void startFifoRead();
	void configureSampleRate(int sample_rate);

	static int dataReadyInterruptCallback(int irq, void *context, void *arg);
	void dataReady();
	bool dataReadyInterruptConfigure();
	bool dataReadyInterruptDisable();

	void selectRegisterBank(uint8_t bank, bool force = false);
	uint8_t registerRead(AddressSpace space, uint16_t reg);
	void registerWrite(AddressSpace space, uint16_t reg, uint8_t value);
	void registerSetAndClearBits(const RegisterConfig &register_config);
	bool registerCheck(const RegisterConfig &register_config);

	uint16_t fifoReadCount();
	bool fifoRead(const hrt_abstime &timestamp_sample, uint8_t requested_samples);
	void fifoReset();
	bool processTemperature(const imu::FifoSampleStats &temperatures);
	void updateTemperature();
	uint64_t errorCount() const;

	bool transferChecked(uint8_t *data, size_t size);

	imu::SpiEndpointGuard _endpoint_guard;
	const Profile        &_profile;
	const uint16_t        _aaf_hz;
	const spi_mode_e      _spi_mode;
	const int             _register_frequency;
	const int             _data_frequency;
	bool                  _transfer_failed     { false };
	bool                  _register_bank_valid { false };
	const spi_drdy_gpio_t _drdy_gpio;
	const bool            _enable_clock_input;
	const float           _sample_dt_us;
	const float           _timestamp_scale_us;

	PX4Accelerometer  _px4_accel;
	PX4Gyroscope      _px4_gyro;
	imu::SpiDmaBuffer _dma_buffer { _profile.device.max_transfer_bytes };

	imu::TransferPerfCounters _transfer_perf {
		MODULE_NAME ": bad register",
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
	hrt_abstime _fifo_state_ready_at          { 0 };
	hrt_abstime _last_config_check_timestamp  { 0 };
	hrt_abstime _temperature_update_timestamp { 0 };
	int         _failure_count                { 0 };
	uint8_t     _config_check_failure_count   { 0 };

	px4::atomic<hrt_abstime> _drdy_timestamp_sample { 0 };
	bool    _data_ready_interrupt_enabled { false };
	uint8_t _last_register_bank           { 0 };

	enum class State : uint8_t {
		kReset,
		kWaitForReset,
		kConfigure,
		kFifoReset,
		kFifoRead,
	} _state { State::kReset };

	uint32_t _fifo_empty_interval_us { 1250 };
	uint8_t  _fifo_gyro_samples      { 1 };
	uint8_t  _checked_register[3]    {};
	uint8_t  _register_config_count     { 0 };
	RegisterConfig _register_config[icm42x_config::kMaxRegisterConfigs] {};
};
