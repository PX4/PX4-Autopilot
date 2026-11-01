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

#include "AdiBurstDecoder.hpp"
#include "../../common/SpiFamily.hpp"
#include "../../common/SpiEndpointClaim.hpp"
#include "../../common/PerfCounter.hpp"
#include "../../common/SensorTiming.hpp"

#include <drivers/drv_hrt.h>
#include <lib/drivers/device/spi.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/atomic.h>

/** Register and initialization dialect for the selected ADI model. */
enum class AdiBurstVariant : uint8_t {
	kAdis16470,
	kAdis16477,
	kAdis165xx,
	kAdis16497,
};

/** Immutable acquisition and startup policy; model-specific units are explicit. */
struct AdiBurstModel {
	imu::SpiModel device; ///< Board identity and SPI phase limits.
	AdiBurstVariant variant { AdiBurstVariant::kAdis165xx }; ///< Register and initialization policy.
	const char *instance_key; ///< Static exact-model/mode key for typed stop/status.
	uint16_t   product_id; ///< Expected PROD_ID response.
	adi_burst_decoder::Format format; ///< Validated single-sample wire layout.
	float    accel_range; ///< m/s^2, not g.
	float    accel_scale; ///< (m/s^2) per raw count in the selected burst mode.
	uint16_t gyro_range_mask; ///< Allowed RANG_MDL encodings, one bit per encoding.
	uint8_t  stall_us; ///< Minimum idle time between independent SPI transactions.
	bool     range_450_dps; ///< Range encoding 1 means 450 dps instead of 500 dps.
	uint32_t reset_wait_us     { 350000 }; ///< Reset recovery, including hardware-reset margin.
	uint32_t self_test_wait_us { 50000 }; ///< Initial self-test guard; not a device completion indication.
	uint16_t hard_fault_mask { (1u << 2) | (1u << 5) | (1u << 6) }; ///< Flat DIAG_STAT: flash update, self-test, memory.
};

/** Startup options copied by the constructor; only model must retain static storage duration. */
struct AdiBurstOptions {
	const AdiBurstModel *model; ///< Selected immutable profile, with static lifetime.
	int decimation {}; ///< ADIS165xx DEC_RATE setting, validated to 0..1999.
	int filter     {}; ///< ADIS165xx Bartlett filter setting, validated to 0..6.
};

/** @brief ADI 16/32-bit burst acquisition with model-specific validation and native single-sample publication. */
class AdiBurst final : public device::SPI, public I2CSPIDriver<AdiBurst>
{
public:
	/**
	 * @brief Copy startup options and bind the immutable model.
	 * @param[in] config Native bus configuration; custom_data points to validated AdiBurstOptions.
	 */
	explicit AdiBurst(const I2CSPIDriverConfig &config);
	~AdiBurst() override;

	/** Print native CLI help and compiled exact-model choices. */
	static void print_usage();

	/** @return PX4_OK after successful SPI probe and scheduling, or a negative error. */
	int init() override;

	/** Print the bus identity, requested phase rates and acquisition counters. */
	void print_status() override;

	/** Execute one scheduled initialization or single-sample acquisition step. */
	void RunImpl();

private:
	enum class Register : uint16_t {
		Page = 0x0000, ///< ADIS16497 PAGE_ID; available on every page.
		DiagStat = 0x02,
		FifoControl = 0x5a, ///< ADIS1657x only: flash-backed FIFO/direct-output selection.
		Filter = 0x5c,
		Range = 0x5e,
		Control = 0x60,
		Decimation = 0x64,
		Command = 0x68,
		Product = 0x72,
		PagedProduct = 0x007e,
		PagedSystemFlags = 0x0008,
		PagedDiag = 0x000a,
		PagedCommand = 0x0302,
		PagedControl = 0x0306,
		PagedConfig = 0x030a,
		PagedDecimation = 0x030c,
		PagedNull = 0x030e,
		PagedRange = 0x0312,
		PagedFilter0 = 0x0316,
		PagedFilter1 = 0x0318,
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
	bool hardwareReset();
	bool configure();
	bool configure16();
	bool configureCrc32();
	bool configurePagedRange();
	bool configureWide();
	bool resetAndConfigure();
	bool selfTest(bool memory = false);
	void startReading();
	bool checkConfiguration();
	bool selectPage(uint8_t page);
	bool paged() const;
	bool classic() const;
	bool probeConfigured() const
	{
#if defined(CONFIG_ADI_BURST_ADIS16477) || defined(CONFIG_ADI_BURST_ADIS16497)

		return _model.variant == AdiBurstVariant::kAdis16477 || paged();

#else

		return false;

#endif // CONFIG_ADI_BURST_ADIS16477 || CONFIG_ADI_BURST_ADIS16497
	}

	bool wide() const
	{
#if defined(CONFIG_ADI_BURST_ADIS16500) \
	|| defined(CONFIG_ADI_BURST_ADIS16501) \
	|| defined(CONFIG_ADI_BURST_ADIS16505) \
	|| defined(CONFIG_ADI_BURST_ADIS16507) \
	|| defined(CONFIG_ADI_BURST_ADIS16575) \
	|| defined(CONFIG_ADI_BURST_ADIS16576) \
	|| defined(CONFIG_ADI_BURST_ADIS16577)

		return _model.variant == AdiBurstVariant::kAdis165xx && _model.format != adi_burst_decoder::Format::kBurst16;

#else

		return false;

#endif // Compiled wide ADIS165xx profiles
	}
	bool wordTransfers() const { return !wide(); }
	Register commandRegister() const { return paged() ? Register::PagedCommand : Register::Command; }
	Register productRegister() const { return paged() ? Register::PagedProduct : Register::Product; }
	Register diagnosticRegister() const { return paged() ? Register::PagedSystemFlags : Register::DiagStat; }
	ReadResult measure(hrt_abstime timestamp);
	uint16_t readRegister(Register reg);
	bool writeRegister(Register reg, uint16_t value);

	/** Write a register and wait delay_us microseconds before validating its readback. */
	bool writeVerified(Register reg, uint16_t value, unsigned delay_us);
	bool writeRetried(Register reg, uint16_t value, unsigned delay_us);

	bool transfer(uint8_t *tx, uint8_t *rx, size_t size);
	bool transfer(uint16_t *tx, uint16_t *rx, size_t words);
	static int dataReadyCallback(int irq, void *context, void *arg);
	void disableInterrupt();

	imu::SpiEndpointClaim _endpoint_claim; ///< Held across initialization, recovery and Fault; released on stop/destruction.
	const AdiBurstModel   &_model;
	const int             _register_frequency;
	const int             _data_frequency;
	const int             _decimation;
	const int             _filter;
	const spi_drdy_gpio_t _drdy_gpio;

	PX4Accelerometer _accel;
	PX4Gyroscope     _gyro;

	imu::PerfCounter<PC_ELAPSED> _read_perf         { MODULE_NAME ": read" };
	imu::PerfCounter<PC_COUNT>   _error_perf        { MODULE_NAME ": errors" };
	imu::PerfCounter<PC_COUNT>   _bad_transfer_perf { MODULE_NAME ": bad transfer" };
	imu::PerfCounter<PC_COUNT>   _bad_register_perf { MODULE_NAME ": bad register" };
	imu::PerfCounter<PC_COUNT> _drdy_missed_perf {
		_drdy_gpio
		? MODULE_NAME ": DRDY missed"
		: nullptr
	};

	imu::PerfCounter<PC_COUNT> _reset_perf { MODULE_NAME ": resets" };
	imu::PerfCounter<PC_COUNT> _stale_perf { MODULE_NAME ": stale data" };
	imu::PerfCounter<PC_COUNT> _diagnostic_perf { MODULE_NAME ": device diagnostic" };

	px4::atomic<hrt_abstime> _drdy_timestamp { 0 };

	imu::SampleProgress<uint16_t> _progress;
	hrt_abstime _reset_timestamp      {};
	hrt_abstime _self_test_deadline    {};
	hrt_abstime _healthy_since         {};
	uint32_t    _interval_us           {};
	uint32_t    _irq_timeout_us        {};
	uint32_t    _sample_timeout_us     {};
	uint32_t    _accel_delay_us        {};
	uint32_t    _gyro_delay_us         {};
	uint16_t    _last_diagnostic        {}; ///< Last nonzero status; never lost through a read-clear follow-up.
	uint16_t    _diagnostic_history     {}; ///< OR of observed device flags for the instance lifetime.
	uint8_t     _self_test_attempts     {};
	uint8_t     _recovery_attempt       {};
	uint8_t     _flush_remaining        {};
	uint8_t     _ready_samples          {};
	uint8_t     _irq_streak             {};
	hrt_abstime _last_config_check      {};
	uint8_t     _checked_config         {};
	uint8_t _page               { UINT8_MAX };
	bool    _self_test_passed   {};
	bool    _transfer_failed    {};
	bool    _interrupt_enabled  {};
	bool    _fallback_polling   {};
	RecoveryReason _recovery_reason { RecoveryReason::kNone };
	State   _state              { State::kReset };
};
