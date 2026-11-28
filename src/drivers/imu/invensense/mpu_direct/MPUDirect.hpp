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
 * @file MPUDirect.hpp
 * @brief Flat-register TDK SPI IMU driver for MPU6000, MPU6500, ICM20602/08G/89, and IAM20680HP.
 */

#pragma once

#include "MPUDirectRegisters.hpp"

#include <drivers/drv_hrt.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/device/spi.h>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/drivers/imu/FifoBatch.hpp>
#include <lib/drivers/imu/ImuPerf.hpp>
#include <lib/drivers/imu/SpiFamily.hpp>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/px4_config.h>

/** @brief TDK flat-register SPI family with fixed-frame acquisition and native FIFO publication. */
class MPUDirect final : public device::SPI, public I2CSPIDriver<MPUDirect>
{
public:
	MPUDirect(const I2CSPIDriverConfig &config);
	~MPUDirect() override;

	static void print_usage();
	static constexpr int32_t kFifoMaxSamples { sizeof(sensor_gyro_fifo_s::x) / sizeof(sensor_gyro_fifo_s::x[0]) };
	static_assert(kFifoMaxSamples <= 512 / mpu_direct_registers::FIFO_PACKET_SIZE_MAX,
		      "Batch capacity exceeds smallest FIFO sample limit");

	static constexpr uint16_t maxTransferSize(uint8_t packet_size, uint8_t prefix = 1)
	{
		return prefix + kFifoMaxSamples * packet_size;
	}

	void RunImpl();
	int init() override;
	void print_status() override;

	using Register = mpu_direct_registers::Register;

	enum class Variant : uint8_t {
		kMpu6000,
		kMpu6500,
		kIcm20602,
		kIcm20608G,
		kIcm20689,
		kIam20680HP,
	};

	/** Immutable wire layout, scaling and initialization policy for one exact model. */
	struct Profile {
		imu::SpiModel device {};
		Variant       variant                   { Variant::kIcm20689 };
		uint8_t       whoami                    { 0 };
		uint16_t      fifo_size                 { 512 }; ///< Conservative software FIFO bound [bytes].
		uint8_t       fifo_packet_size          { mpu_direct_registers::FIFO_PACKET_SIZE_CLASSIC }; ///< Wire frame size [bytes].
		uint8_t       gyro_offset               { 6 };   ///< Byte offset of gyro X within a frame [bytes].
		uint8_t       samples_per_transfer      { 2 };   ///< Gyro frames per new accel sample.
		float         temperature_sensitivity   { 326.8f }; ///< Temperature sensitivity [LSB/degC].
		float         temperature_offset        { 25.f };   ///< Temperature offset at zero raw count [degC].
		uint8_t       reset_pwr_value           { static_cast<uint8_t>(mpu_direct_registers::PWR_MGMT_1_BIT::SLEEP) };
		uint32_t      reset_wait_us             { 100000 }; ///< Device reset wait [us].
		uint32_t      configure_wait_us         { 35000 };  ///< Post-configuration settle wait [us].
		bool          check_reset_pwr           { true };
		bool          check_reset_config        { false };
		bool          has_factory_accel_offsets { true };
		bool          has_fifo_temperature      { false };
	};

private:
	bool isMpu6000() const { return _profile.variant == Variant::kMpu6000; }
	bool isIcm20602() const { return _profile.variant == Variant::kIcm20602; }

	struct RegisterConfig {
		Register reg;
		uint8_t  set_bits   { 0 };
		uint8_t  clear_bits { 0 };
	};

	static constexpr float   kFifoSampleDt       { 1e6f / 8000.f };
	static constexpr float   kGyroRate           { 1e6f / kFifoSampleDt };
	static constexpr uint8_t kMaxRegisterConfigs { 32 };

	void buildRegisterConfig();
	void addRegisterConfig(Register reg, uint8_t set_bits, uint8_t clear_bits = 0);

	void exit_and_cleanup() override;
	int probe() override;
	void deviceReset();
	bool reset(uint32_t delay_us = 0);
	bool resetComplete();
	[[nodiscard]] bool wakeAndResetSignalPath();

	bool configure();
	void configureAccel();
	void configureGyro();
	void configureSampleRate(int sample_rate);
	void configureFifoWatermark();

	static int dataReadyInterruptCallback(int irq, void *context, void *arg);
	void dataReady();
	bool dataReadyInterruptConfigure();
	bool dataReadyInterruptDisable();

	bool registerCheck(const RegisterConfig &register_config);
	bool storeCheckedRegisterValue(Register reg);
	uint8_t registerRead(Register reg);
	void registerWrite(Register reg, uint8_t value);
	void registerSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits);

	bool fifoReadCount(uint16_t &fifo_count);
	bool fifoRead(hrt_abstime timestamp_sample, uint8_t samples, bool using_drdy_timestamp);
	[[nodiscard]] bool fifoReset();
	bool decodeAccel(const hrt_abstime &timestamp_sample, const uint8_t fifo[], uint8_t samples,
			 sensor_accel_fifo_s &accel);
	bool decodeGyro(const hrt_abstime &timestamp_sample, const uint8_t fifo[], uint8_t samples,
			sensor_gyro_fifo_s &gyro);
	bool processTemperature(const uint8_t fifo[], uint8_t samples);
	void updateTemperature();
	uint64_t errorCount() const;

	imu::SpiEndpointGuard _endpoint_guard;
	const Profile    &_profile;
	const spi_mode_e  _spi_mode;
	const int         _register_frequency;
	const int         _data_frequency;
	bool _transfer_failed     { false };
	bool _factory_read_failed { false };
	bool _configuration_valid { true };
	const spi_drdy_gpio_t _drdy_gpio;

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
		_drdy_gpio
		? MODULE_NAME ": DRDY missed"
		: nullptr
	};

	hrt_abstime _reset_timestamp              { 0 };
	hrt_abstime _last_sample_timestamp        { 0 };
	hrt_abstime _last_config_check_timestamp  { 0 };
	hrt_abstime _temperature_update_timestamp { 0 };
	int         _failure_count                { 0 };
	uint8_t     _config_check_failure_count   { 0 };

	px4::atomic<hrt_abstime> _drdy_timestamp_sample { 0 };
	imu::DrdySampleAnchor    _drdy_anchor {};
	uint8_t _fifo_accel_samples_count { 0 };
	uint8_t _last_accel[6]            {};
	bool    _has_last_accel           { false };
	bool    _accel_phase_synced       { false };
	bool    _drdy_backlog_drain       { false };
	bool    _data_ready_interrupt_enabled { false };

	enum class State : uint8_t {
		kReset,
		kWaitForReset,
		kConfigure,
		kFifoRead,
	} _state{State::kReset};

	uint16_t _fifo_empty_interval_us { 1250 };
	int32_t  _fifo_gyro_samples      { 10 };
	uint8_t  _checked_register       { 0 };
	uint8_t  _register_config_count     { 0 };
	RegisterConfig _register_config[kMaxRegisterConfigs] {};
};
