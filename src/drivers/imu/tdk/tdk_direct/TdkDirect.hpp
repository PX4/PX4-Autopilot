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

#include "TdkDirectRegisters.hpp"
#include "../../common/SpiFamily.hpp"

#include <drivers/drv_hrt.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/device/spi.h>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/i2c_spi_buses.h>

/** @brief TDK flat-register SPI family with fixed-frame acquisition and native FIFO publication. */
class TdkDirect final : public device::SPI, public I2CSPIDriver<TdkDirect>
{
public:
	TdkDirect(const I2CSPIDriverConfig &config);
	~TdkDirect() override;

	static void print_usage();
	static constexpr uint16_t maxTransferSize(uint8_t packet_size, uint8_t prefix = 1)
	{
		return prefix + kFifoMaxSamples * packet_size;
	}
	static constexpr int32_t kFifoMaxSamples { sizeof(sensor_gyro_fifo_s::x) / sizeof(sensor_gyro_fifo_s::x[0]) };

	void RunImpl();
	int init() override;
	void print_status() override;

	using Register = tdk_direct_registers::Register;

	enum class Variant : uint8_t {
		kMpu6000,
		kMpu6500,
		kMpu9250,
		kIcm20602,
		kIcm20608G,
		kIcm20689,
		kIam20680HP,
	};

	/** Immutable wire layout, scaling and initialization policy for one exact model. */
	struct Profile {
		imu::SpiModel device;
		Variant       variant;
		uint8_t       whoami;
		uint16_t      fifo_size; ///< Conservative software FIFO bound in bytes; may be smaller than the hardware capacity.
		uint8_t       fifo_packet_size; ///< Wire frame size in bytes, excluding the SPI command.
		uint8_t       gyro_offset; ///< Byte offset of gyro X within a frame.
		uint8_t       samples_per_transfer; ///< Gyro frames per new accel sample, not the total SPI transfer count.
		float    temperature_sensitivity; ///< Raw counts per degree Celsius.
		float    temperature_offset; ///< Degrees Celsius added after dividing raw temperature by sensitivity.
		uint8_t  reset_pwr_value;
		uint32_t reset_wait_us;
		uint32_t configure_wait_us;
		bool     check_reset_pwr;
		bool     check_reset_config;
		bool     has_factory_accel_offsets;
		bool     has_fifo_temperature;
	};

private:
	struct RegisterConfig {
		Register reg;
		uint8_t set_bits   { 0 };
		uint8_t clear_bits { 0 };
	};

	static constexpr float   kFifoSampleDt       { 1e6f / 8000.f };
	static constexpr float   kGyroRate           { 1e6f / kFifoSampleDt };
	static constexpr uint8_t kMaxRegisterConfigs { 32 };

	struct FifoTransferBuffer {
		uint8_t cmd { static_cast<uint8_t>(Register::FIFO_R_W) | tdk_direct_registers::DIR_READ };
		// ICM20602 includes two FIFO count bytes before the same fixed-frame payload.
		uint8_t data[2 + kFifoMaxSamples * tdk_direct_registers::FIFO_PACKET_SIZE_MAX] {};
	};

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

	bool registerCheck(const RegisterConfig &reg_cfg);
	bool storeCheckedRegisterValue(Register reg);
	uint8_t registerRead(Register reg);
	void registerWrite(Register reg, uint8_t value);
	void registerSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits);

	uint16_t fifoReadCount();
	bool fifoRead(const hrt_abstime &timestamp_sample, uint8_t samples);
	[[nodiscard]] bool fifoReset();
	bool processAccel(const hrt_abstime &timestamp_sample, const uint8_t fifo[], uint8_t samples);
	bool processGyro(const hrt_abstime &timestamp_sample, const uint8_t fifo[], uint8_t samples);
	bool processTemperature(const uint8_t fifo[], uint8_t samples);
	void updateTemperature();
	uint64_t errorCount() const;

	const Profile &_profile;
	const int     _register_frequency;
	const int     _data_frequency;
	bool _transfer_failed     { false };
	bool _factory_read_failed { false };
	bool _configuration_valid { true };
	const spi_drdy_gpio_t _drdy_gpio;

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
		_drdy_gpio
		? MODULE_NAME ": DRDY missed"
		: nullptr
	};

	hrt_abstime _reset_timestamp              { 0 };
	hrt_abstime _last_config_check_timestamp  { 0 };
	hrt_abstime _temperature_update_timestamp { 0 };
	int         _failure_count                { 0 };

	px4::atomic<hrt_abstime> _drdy_timestamp_sample { 0 };
	px4::atomic<int32_t>     _drdy_count            { 0 };
	uint8_t _fifo_accel_samples_count     { 0 };
	uint8_t _last_accel[6]                {};
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
	uint8_t  _register_cfg_count     { 0 };
	RegisterConfig _register_cfg[kMaxRegisterConfigs] {};
};
