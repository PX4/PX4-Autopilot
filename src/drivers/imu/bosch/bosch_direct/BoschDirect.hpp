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

#include "BoschDirectModel.hpp"
#include "../../common/FifoPerfCounters.hpp"

#include <drivers/drv_hrt.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/drivers/device/spi.h>
#include <px4_platform_common/atomic.h>
#include <uORB/topics/sensor_accel_fifo.h>
#include <uORB/topics/sensor_gyro_fifo.h>

/** @brief One native Bosch SPI endpoint with fixed or tagged FIFO acquisition; split devices remain independent. */
class BoschDirect final : public device::SPI, public I2CSPIDriver<BoschDirect>
{
public:
	explicit BoschDirect(const I2CSPIDriverConfig &config);
	~BoschDirect() override;
	BoschDirect(const BoschDirect &) = delete;
	BoschDirect &operator=(const BoschDirect &) = delete;

	int init() override;
	void RunImpl();
	void print_status() override;
	static void print_usage();

	static constexpr uint8_t kMaxSamples { sizeof(sensor_accel_fifo_s::x) / sizeof(sensor_accel_fifo_s::x[0]) };

	/** Maximum SPI transaction size, including command/dummy bytes and tagged-frame overhead. */
	static constexpr uint16_t maxTransferSize(bosch_direct_fifo::Format format, uint8_t prefix)
	{
		// A BMI270 batch may contain separate accel/gyro frames instead of pairs.
		const uint8_t data_bytes    = format == bosch_direct_fifo::Format::kTaggedImu ? 14 : bosch_direct_fifo::frameBytes(format);
		const uint8_t control_bytes = bosch_direct_fifo::tagged(format) ? 16 : 0;

		return prefix + kMaxSamples * data_bytes + control_bytes;
	}

private:
	using Profile = bosch_direct_model::Profile;
	using RegisterConfig = bosch_direct_model::RegisterConfig;
	using Format = bosch_direct_fifo::Format;
	using Variant = bosch_direct_model::Variant;

	enum class State : uint8_t {
		kReset,
		kWaitReset,
		kEnableAccel, ///< BMI08x: enable after leaving suspend, then wait before configuring registers.
		kLoadConfig, ///< Upload the BMI270 configuration image before writing measurement settings.
		kWaitConfig,
		kConfigure,
		kFifoReset,
		kRead,
	};

	int probe() override;
	void exit_and_cleanup() override;
	void reset();
	bool identity();
	void configureSampleRate();
	bool configure();

	bool readFifo(hrt_abstime now, hrt_abstime interrupt_time);
	bool readFixed(hrt_abstime now, hrt_abstime interrupt_time);
	bool readTagged(hrt_abstime now, hrt_abstime interrupt_time);

	/** Decode and validate the complete tagged batch before publishing either enabled channel. */
	template<bool integrated>
	bool processTagged(const uint8_t *data, size_t size, hrt_abstime now, hrt_abstime interrupt_time);

	bool fifoReset();
	void updateTemperature();

	bool readRegisters(uint8_t reg, uint8_t *data, size_t count);
	uint8_t readRegister(uint8_t reg);
	bool writeRegister(uint8_t reg, uint8_t value);
	bool modifyRegister(const RegisterConfig &cfg);
	bool checkRegister(const RegisterConfig &cfg);

	/** Enforce model-specific transfer spacing; latch a failure until the next work-queue pass. */
	bool transferChecked(const uint8_t *tx, uint8_t *rx, size_t size);

	uint64_t errorCount() const;
	hrt_abstime sampleTime(hrt_abstime now, hrt_abstime interrupt_time, uint8_t samples) const;

	static int dataReadyCallback(int irq, void *context, void *arg);
	bool enableInterrupt();
	void disableInterrupt();

	const Profile           &_profile;
	const int               _register_frequency;
	const int               _data_frequency;
	const spi_drdy_gpio_t   _drdy_gpio;
	const float             _sample_dt;
	PX4Accelerometer *const _accel;
	PX4Gyroscope *const     _gyro;
	uint8_t *const          _buffer;

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
		_drdy_gpio != 0 && _profile.interrupt_enabled
		? MODULE_NAME ": DRDY missed"
		: nullptr
	};

	px4::atomic<hrt_abstime> _drdy_timestamp { 0 };
	State    _state             { State::kReset };
	bool     _transfer_failed   { false };
	bool     _interrupt_enabled { false };
	uint8_t  _failure_count     { 0 };
	uint8_t  _watermark_samples { 1 };
	uint8_t  _register_count    { 0 };
	uint8_t  _checked_register  { 0 };
	uint32_t _interval_us       { 0 };
	uint32_t _accel_startup_us  { 1000 }; ///< BMI085/088 guard; BMI090L identity selects 50 ms.

	hrt_abstime _reset_timestamp   { 0 };
	hrt_abstime _last_config_check { 0 };
	hrt_abstime _last_temperature  { 0 };
#if defined(CONFIG_BOSCH_DIRECT_BMI270) \
	|| defined(CONFIG_BOSCH_DIRECT_BMI085) \
	|| defined(CONFIG_BOSCH_DIRECT_BMI088)
	hrt_abstime _next_transfer { 0 }; ///< Earliest next SPI transaction after the previous transfer's stall time.
#endif // BMI08x or BMI270 register-write timing

	RegisterConfig _registers[bosch_direct_model::kMaxRegisterConfigs] {};
};
