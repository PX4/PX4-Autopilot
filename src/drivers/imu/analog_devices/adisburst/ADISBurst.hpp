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
 * @file ADISBurst.hpp
 * @brief Flat-register Analog Devices 16/32-bit SPI burst IMU family driver.
 */

#pragma once

#include "ADISBurstDecoder.hpp"

#include <drivers/drv_hrt.h>
#include <drivers/drv_sensor.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/device/spi.h>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/drivers/imu/ImuPerf.hpp>
#include <lib/drivers/imu/SpiFamily.hpp>
#include <lib/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/px4_config.h>


/** Register and initialization dialect for the selected flat-register ADI model. */
enum class ADISBurstVariant : uint8_t {
	kAdis16470,
	kAdis16477,
	kAdis165xx,
};

/** Immutable acquisition and startup policy; model-specific units are explicit. */
struct ADISBurstProfile {
	imu::SpiModel             device {};                  ///< Board identity and SPI phase limits.
	ADISBurstVariant           variant { ADISBurstVariant::kAdis165xx }; ///< Register and initialization policy.
	uint16_t                  product_id {};              ///< Expected PROD_ID response.
	adis_burst_decoder::Format format { adis_burst_decoder::Format::kBurst32 }; ///< Validated single-sample wire layout.
	float                     accel_range {};             ///< Accelerometer full-scale range [m/s^2].
	float                     accel_scale {};             ///< Accelerometer scale factor in burst mode [(m/s^2)/LSB].
	uint16_t                  gyro_range_mask {};         ///< Allowed RANG_MDL encodings, one bit per encoding.
	uint16_t                  accel_base_delay_us {};     ///< Unfiltered accelerometer base group delay [us].
	uint16_t                  gyro_base_delay_us {};      ///< Unfiltered gyroscope base group delay [us].
	uint8_t                   stall_us {};                ///< Minimum idle time between independent SPI transactions [us].
	bool                      range_450_dps {};           ///< Range encoding 1 means 450 deg/s instead of 500 deg/s.
	uint32_t                  reset_wait_us { 350000 };   ///< Reset recovery delay including hardware-reset margin [us].
	uint32_t                  self_test_wait_us { 50000 }; ///< Initial self-test guard [us]; not a device completion indication.
	const char               *range_descriptions[4] {};   ///< Human-readable model + gyro range per RANG_MDL[3:2] code.
};

/** Startup options copied by the constructor; only model must retain static storage duration. */
struct ADISBurstOptions {
	const ADISBurstProfile *model { nullptr }; ///< Selected immutable profile, with static lifetime.
	int decimation {};                      ///< ADIS165xx DEC_RATE setting, validated to 0..1999.
	int filter     {};                      ///< ADIS165xx Bartlett filter setting, validated to 0..6.
};

/** @brief ADI 16/32-bit burst acquisition with model-specific validation and native single-sample publication. */
class ADISBurst final : public device::SPI, public I2CSPIDriver<ADISBurst>
{
public:
	explicit ADISBurst(const I2CSPIDriverConfig &config);
	~ADISBurst() override;

	static void print_usage();

	int init() override;
	void print_status() override;
	void RunImpl();

private:
	/** Track validated sample progress in host monotonic microseconds, not compensated sample time. */
	class SampleProgress
	{
	public:
		void reset(hrt_abstime timestamp_now)
		{
			_last_update = timestamp_now;
			_initialized = false;
		}

		[[nodiscard]] bool observe(uint16_t counter, hrt_abstime timestamp_now)
		{
			if (_initialized && counter == _counter) {
				return false;
			}

			_counter     = counter;
			_last_update = timestamp_now;
			_initialized = true;

			return true;
		}

		bool expired(hrt_abstime timestamp_now, hrt_abstime timeout) const
		{
			return timestamp_now >= _last_update && (timestamp_now - _last_update) >= timeout;
		}

		hrt_abstime lastUpdate() const { return _last_update; }

	private:
		hrt_abstime _last_update {};
		uint16_t    _counter {};
		bool        _initialized {};
	};

	template<typename Duration, size_t count>
	static constexpr Duration retryDelay(unsigned attempt, const Duration(&delays)[count])
	{
		static_assert(count > 0, "Retry schedule must not be empty");
		return delays[attempt < count ? attempt : count - 1];
	}

	enum class Register : uint8_t {
		DiagStat    = 0x02,
		FifoControl = 0x5a, ///< ADIS1657x only: flash-backed FIFO/direct-output selection.
		Filter      = 0x5c,
		Range       = 0x5e,
		Control     = 0x60,
		Decimation  = 0x64,
		Command     = 0x68,
		FirmRev     = 0x6c,
		FirmDm      = 0x6e,
		FirmY       = 0x70,
		Product     = 0x72,
		SerialNum   = 0x74,
	};

	enum class State : uint8_t {
		kReset,
		kWaitReset,
		kStartSelfTest,
		kSelfTest,
		kConfigure,
		kFlush,
		kValidate,
		kRead,
		kFault,
	};

	/** Separate communication, integrity, device health and sample-progress outcomes. */
	enum class ReadResult : uint8_t {
		kPublished,
		kWarmingUp,
		kDuplicate,
		kTransferError,
		kInvalid,
		kDeviceError,
	};

	enum class RecoveryReason : uint8_t {
		kNone,
		kTransfer,
		kIntegrity,
		kNoData,
		kConfiguration,
		kIdentity,
		kSelfTest,
		kDiagnostic,
	};

	int probe() override;
	void exit_and_cleanup() override;
	void reset(uint32_t delay_us = 0);
	void recover(RecoveryReason reason);
	void fault(RecoveryReason reason);
	void selfTestFailed(RecoveryReason reason);
	void recordDiagnostic(uint16_t diagnostic);
	void handleDiagnostic();
	static const char *stateName(State state);
	static const char *reasonName(RecoveryReason reason);
	bool hasBoardResetPin() const;
	bool hardwareReset();
	bool readIdentification();
	bool configure();
	bool configure16();
	bool configureWide();
	bool writeRetried(Register reg, uint16_t value, unsigned delay_us);
	void startReading();
	bool checkConfiguration();

	bool classic() const
	{
		return _profile.variant == ADISBurstVariant::kAdis16470
		       || _profile.variant == ADISBurstVariant::kAdis16477;
	}

	bool probeConfigured() const
	{
		return _profile.variant == ADISBurstVariant::kAdis16477;
	}

	bool wide() const
	{
		return _profile.variant == ADISBurstVariant::kAdis165xx
		       && _profile.format != adis_burst_decoder::Format::kBurst16;
	}

	bool wordTransfers() const { return !wide(); }
	ReadResult measure(hrt_abstime timestamp);
	uint16_t readRegister(Register reg);
	bool writeRegister(Register reg, uint16_t value);

	/** Write a register and wait delay_us microseconds before validating its readback. */
	bool writeVerified(Register reg, uint16_t value, unsigned delay_us);

	bool transferBytes(uint8_t *tx, uint8_t *rx, size_t count);
	bool transferWords(uint16_t *tx, uint16_t *rx, size_t words);
	static int dataReadyCallback(int irq, void *context, void *arg);
	void disableInterrupt();

	imu::SpiEndpointGuard    _endpoint_guard;
	const ADISBurstProfile  &_profile;
	const spi_mode_e       _spi_mode;
	const int              _register_frequency;
	const int             _data_frequency;
	const int             _decimation;
	const int             _filter;
	const spi_drdy_gpio_t _drdy_gpio;

	PX4Accelerometer  _px4_accel;
	PX4Gyroscope      _px4_gyro;
	imu::SpiDmaBuffer _dma_buffer { 64 };

	imu::PerfCounter<PC_ELAPSED> _read_perf         { MODULE_NAME ": read" };
	imu::PerfCounter<PC_COUNT>   _error_perf        { MODULE_NAME ": errors" };
	imu::PerfCounter<PC_COUNT>   _bad_transfer_perf { MODULE_NAME ": bad transfer" };
	imu::PerfCounter<PC_COUNT>   _bad_register_perf { MODULE_NAME ": bad register" };
	imu::PerfCounter<PC_COUNT>   _drdy_missed_perf  { _drdy_gpio ? MODULE_NAME ": DRDY missed" : nullptr };
	imu::PerfCounter<PC_COUNT>   _reset_perf        { MODULE_NAME ": resets" };
	imu::PerfCounter<PC_COUNT>   _stale_perf        { MODULE_NAME ": stale data" };
	imu::PerfCounter<PC_COUNT>   _diagnostic_perf   { MODULE_NAME ": device diagnostic" };

	px4::atomic<hrt_abstime> _drdy_timestamp_sample { 0 };

	SampleProgress _progress {};
	const char    *_model_description  { "unknown" };
	hrt_abstime    _reset_timestamp    {};
	hrt_abstime    _self_test_deadline {};
	hrt_abstime    _healthy_since      {};
	hrt_abstime    _last_config_check_timestamp {};
	uint32_t       _interval_us        {};
	uint32_t       _irq_timeout_us     {};
	uint32_t       _sample_timeout_us  {};
	uint32_t       _accel_delay_us     {};
	uint32_t       _gyro_delay_us      {};
	uint16_t       _serial_number      {};
	uint16_t       _firmware_revision  {};
	uint16_t       _firmware_day_month {};
	uint16_t       _firmware_year      {};
	uint16_t       _last_diagnostic    {}; ///< Last nonzero status; never lost through a read-clear follow-up.
	uint16_t       _diagnostic_history {}; ///< OR of observed device flags for the instance lifetime.
	uint8_t        _self_test_attempts {};
	uint8_t        _recovery_attempt   {};
	uint8_t        _diagnostic_streak  {};
	uint8_t        _flush_remaining    {};
	uint8_t        _ready_samples      {};
	uint8_t        _irq_streak         {};
	uint8_t        _checked_config     {};
	bool           _id_valid                  {};
	bool           _memory_self_test_passed   { _profile.variant != ADISBurstVariant::kAdis16477 };
	bool           _self_test_passed          {};
	bool           _transfer_failed           {};
	bool           _data_ready_interrupt_enabled  {};
	bool           _fallback_polling   {};
	RecoveryReason _recovery_reason    { RecoveryReason::kNone };
	State          _state              { State::kReset };
};
