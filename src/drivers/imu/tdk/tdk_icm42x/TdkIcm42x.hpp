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

#pragma once

#include "../../common/FifoPerfCounters.hpp"
#include "../../common/FifoSampleStats.hpp"

#include "TdkIcm42xConfigs.hpp"
#include "../TdkPacketModel.hpp"
#include "../../common/SpiFamily.hpp"

#include <drivers/drv_hrt.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/device/spi.h>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <uORB/topics/sensor_accel_fifo.h>
#include <uORB/topics/sensor_gyro_fifo.h>

/** @brief TDK packet-FIFO SPI family retaining per-model register, wire and recovery policies. */
class TdkIcm42x final : public device::SPI, public I2CSPIDriver<TdkIcm42x>
{
public:
	TdkIcm42x(const I2CSPIDriverConfig &config);
	~TdkIcm42x() override;

	static void print_usage();

	/** SPI prefix plus the largest batch of complete wire packets accepted by the FIFO publishers. */
	static constexpr uint16_t maxTransferSize(uint8_t packet_size, uint8_t prefix)
	{
		return prefix + kFifoMaxSamples * packet_size;
	}

	void RunImpl();
	int init() override;
	void print_status() override;

	using Variant = tdk_packet::Variant;
	using AddressSpace = tdk_packet::AddressSpace;
	using RegisterConfig = tdk_packet::RegisterConfig;
	using Protocol = tdk_packet::Protocol;
	using PacketFormat = tdk_packet::PacketFormat;
	using Profile = tdk_packet::Profile;

	static constexpr uint8_t kFifoMaxSamples{
		static_cast<uint8_t>(sizeof(sensor_gyro_fifo_s::x) / sizeof(sensor_gyro_fifo_s::x[0]))
	};

private:
	static constexpr uint8_t kFifoPacketSizeMax     { 20 };
	static constexpr uint8_t kFifoTransferPrefixMax { 4 };

	void exit_and_cleanup() override;
	int probe() override;
	bool reset();
	bool resetComplete();
	bool configure();

	/** Registers enabled only after static configuration and sensor startup have completed. */
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
	void registerSetAndClearBits(const RegisterConfig &reg_cfg);
	bool registerCheck(const RegisterConfig &reg_cfg);

	uint16_t fifoReadCount();

	/** Validate packet headers, counts and temperature before publishing the local accel/gyro batches. */
	bool fifoRead(const hrt_abstime &timestamp_sample, uint8_t requested_samples);

	void fifoReset();
	bool processTemperature(const imu::FifoSampleStats &temperatures);
	void updateTemperature();
	uint64_t errorCount() const;

	bool transferChecked(uint8_t *data, size_t size);

	const Profile &_profile;
	const int     _register_frequency;
	const int     _data_frequency;
	bool _transfer_failed     { false };
	bool _register_bank_valid { false };
	const spi_drdy_gpio_t _drdy_gpio;
	const bool            _enable_clock_input;
	const float           _sample_dt_us;
	const float           _timestamp_scale_us;

	PX4Accelerometer _px4_accel;
	PX4Gyroscope     _px4_gyro;

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
		_drdy_gpio != 0 && _profile.data_ready_interrupt
		? MODULE_NAME ": DRDY missed"
		: nullptr
	};

	hrt_abstime _reset_timestamp              { 0 };
	hrt_abstime _fifo_state_ready_at          { 0 }; ///< Minimum startup wait, independent of queued wakeups.
	hrt_abstime _last_config_check_timestamp  { 0 };
	hrt_abstime _temperature_update_timestamp { 0 };
	int         _failure_count                { 0 };

	px4::atomic<hrt_abstime> _drdy_timestamp_sample { 0 };
	bool    _data_ready_interrupt_enabled { false };
	uint8_t _last_register_bank           { 0 };

	enum class State : uint8_t {
		kReset,
		kWaitForReset,
		kConfigure,
		kFifoEnable, ///< Wait for sensor startup before enabling FIFO acquisition.
		kFifoReset, ///< Allow startup FIFO records to arrive before flushing.
		kFifoRead,
	} _state{State::kReset};

	uint16_t _fifo_empty_interval_us { 1250 };
	uint8_t  _fifo_gyro_samples      { 1 };
	uint8_t  _checked_register[3]    {}; ///< Independent cursors for bank 0, 1 and 2 checks.
	uint8_t  _register_cfg_count     { 0 };
	RegisterConfig _register_cfg[tdk_icm42x_config::kMaxRegisterConfigs] {};
	uint8_t _fifo_transfer[kFifoTransferPrefixMax + kFifoMaxSamples * kFifoPacketSizeMax] {};
};
