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
 * @file ADISBurst.cpp
 * @brief Implementation of the ADISBurst flat-register SPI IMU family driver.
 */

#include "ADISBurst.hpp"

#include <lib/mathlib/mathlib.h>

using namespace time_literals;
using adis_burst_decoder::Format;
using Variant = ADISBurstVariant;

namespace
{
constexpr uint8_t  kWriteBit            { 0x80 };
constexpr uint8_t  kBurstCommand        { 0x68 };
#if defined(GPIO_ADIS16507_RESET)
constexpr uint16_t kProductAdis16507    { 0x407b };
#endif // GPIO_ADIS16507_RESET
constexpr uint16_t kSoftwareReset       { 1u << 7 };
constexpr uint16_t kMemorySelfTest      { 1u << 4 };
constexpr uint16_t kDiagMemoryError     { 1u << 6 };
constexpr uint16_t kSensorSelfTest      { 1u << 2 };
constexpr uint16_t kDiagSelfTestError   { 1u << 5 };
constexpr uint16_t kDataReadyActiveHigh { 1u << 0 };
constexpr uint16_t kPointOfPercussion   { 1u << 6 };
constexpr uint16_t kLinearGCompensation { 1u << 7 };
constexpr uint16_t kBurst32Enable       { 1u << 9 };
constexpr uint16_t kRangeReservedMask   { 0xfff3 };
constexpr uint16_t kRangeReservedValue  { 0x0003 };
constexpr unsigned kSamplePeriodUs      { 500 };
constexpr unsigned kControlUpdateUs     { 200 };
constexpr unsigned kFilterUpdateUs      { 30 };
constexpr unsigned kDecimationUpdateUs  { 30 };
#if defined(GPIO_ADIS16507_RESET) || defined(GPIO_ADIS1650X_RESET) || defined(GPIO_ADIS1657X_RESET)
constexpr unsigned kResetPulseUs        { 100 };
#endif // Hardware reset pulse duration
constexpr float    kWideWordFactor      { 65536.f };
constexpr float    kTemperatureScale    { 0.1f };
constexpr unsigned kSelfTestAttempts    { 3 };
constexpr unsigned kReadySamples        { 2 };      // Require progress across two validated bursts after configuration.
constexpr uint16_t kSamplingMask        { 0x003f }; // SYNC mode/polarity, DR polarity and must-zero bit 5.
constexpr unsigned kIrqRecoverySamples  { 3 };      // Hysteresis before leaving fallback polling.
constexpr uint32_t kRecoveryDelays[]    { 100_ms, 500_ms, 2_s, 5_s };
constexpr uint32_t kSelfTestDelays[]    { 1_s, 2_s };
constexpr imu::ByteOrder kNativeWordOrder {
	__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__ ? imu::ByteOrder::kBigEndian : imu::ByteOrder::kLittleEndian
};
} // namespace

ADISBurst::ADISBurst(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_endpoint_guard(config),
	_profile(*static_cast<const ADISBurstOptions *>(config.custom_data)->model),
	_spi_mode(config.spi_mode),
	_register_frequency(imu::spiConfigFrequency(_profile.device, config.custom2)),
	_data_frequency(imu::spiDataFrequency(_profile.device, config.custom2)),
	_decimation(_profile.variant == Variant::kAdis16477 ? 1 : static_cast<const ADISBurstOptions *>(config.custom_data)->decimation),
	_filter(_profile.variant == Variant::kAdis16477 ? 4 : static_cast<const ADISBurstOptions *>(config.custom_data)->filter),
	_drdy_gpio(config.drdy_gpio),
	_px4_accel(get_device_id(), config.rotation, config.external),
	_px4_gyro(get_device_id(), config.rotation, config.external)
{
	_interval_us = probeConfigured() ? 1000u : static_cast<uint32_t>(_decimation + 1) * kSamplePeriodUs;

	// Host liveness uses the exact polling period, not compensated publication timestamps.
	_irq_timeout_us    = math::max(3 * _interval_us, uint32_t(2_ms));
	_sample_timeout_us = math::max(10 * _interval_us, uint32_t(20_ms));
}

ADISBurst::~ADISBurst()
{
	disableInterrupt();
	_endpoint_guard.release();
}

bool ADISBurst::transferBytes(uint8_t *tx, uint8_t *rx, size_t count)
{
	if (_transfer_failed || SPI::transfer(tx, rx, count) != PX4_OK) {
		_transfer_failed = true;
		return false;
	}

	return true;
}

bool ADISBurst::transferWords(uint16_t *tx, uint16_t *rx, size_t words)
{
	if (_transfer_failed || SPI::transferhword(tx, rx, words) != PX4_OK) {
		_transfer_failed = true;
		return false;
	}

	return true;
}

uint16_t ADISBurst::readRegister(Register reg)
{
	set_frequency(_register_frequency);

	if (wordTransfers()) {
		uint16_t *words = _dma_buffer.words();
		words[0] = static_cast<uint16_t>(reg) << 8;

		transferWords(words, nullptr, 1);
		px4_udelay(_profile.stall_us);
		words[0] = 0;
		transferWords(nullptr, words, 1);
		// tSTALL also separates this reply from the next register command.
		px4_udelay(_profile.stall_us);

		return words[0];
	}

	uint8_t *buf = _dma_buffer.data();
	buf[0] = static_cast<uint8_t>(reg);
	buf[1] = 0;

	// Register reads are pipelined: send the address, then clock out its reply in a separate transaction.
	transferBytes(buf, nullptr, 2);
	px4_udelay(_profile.stall_us);
	buf[0] = 0;
	buf[1] = 0;
	transferBytes(nullptr, buf, 2);
	px4_udelay(_profile.stall_us);

	return adis_burst_decoder::unsignedWord(buf);
}

bool ADISBurst::writeRegister(Register reg, uint16_t value)
{
	set_frequency(_register_frequency);

	const uint8_t address = static_cast<uint8_t>(reg);

	if (wordTransfers()) {
		uint16_t *words = _dma_buffer.words();
		words[0] = static_cast<uint16_t>((static_cast<uint16_t>(address | kWriteBit) << 8) | (value & 0xff));

		transferWords(words, nullptr, 1);
		px4_udelay(_profile.stall_us);
		words[0] = static_cast<uint16_t>((static_cast<uint16_t>((address + 1) | kWriteBit) << 8) | (value >> 8));
		transferWords(words, nullptr, 1);
		px4_udelay(_profile.stall_us);

		return !_transfer_failed;
	}

	uint8_t *buf = _dma_buffer.data();

	// Write the low and high bytes separately, retaining the model's stall time after each transaction.
	buf[0] = static_cast<uint8_t>(address | kWriteBit);
	buf[1] = static_cast<uint8_t>(value);
	transferBytes(buf, nullptr, 2);
	px4_udelay(_profile.stall_us);

	buf[0] = static_cast<uint8_t>((address + 1) | kWriteBit);
	buf[1] = static_cast<uint8_t>(value >> 8);
	transferBytes(buf, nullptr, 2);
	px4_udelay(_profile.stall_us);

	return !_transfer_failed;
}

bool ADISBurst::writeVerified(Register reg, uint16_t value, unsigned delay_us)
{
	if (!writeRegister(reg, value)) {
		return false;
	}

	// MSC_CTRL readback may lag by 200 us; wait before comparing.
	px4_udelay(delay_us);

	return readRegister(reg) == value && !_transfer_failed;
}

bool ADISBurst::writeRetried(Register reg, uint16_t value, unsigned delay_us)
{
	// ADIS1650x/1657x retry each write/readback three times before resetting.
	for (unsigned attempt = 0; attempt < 3; ++attempt) {
		_transfer_failed = false;

		if (writeVerified(reg, value, delay_us)) {
			return true;
		}
	}

	PX4_WARN("register 0x%02x did not accept 0x%04x", unsigned(reg), unsigned(value));
	return false;
}

bool ADISBurst::readIdentification()
{
	const uint16_t serial = readRegister(Register::SerialNum);
	const uint16_t rev    = readRegister(Register::FirmRev);
	const uint16_t dm     = readRegister(Register::FirmDm);
	const uint16_t year   = readRegister(Register::FirmY);

	if (_transfer_failed) {
		return false;
	}

	_serial_number      = serial;
	_firmware_revision  = rev;
	_firmware_day_month = dm;
	_firmware_year      = year;
	_id_valid           = true;

	return true;
}

int ADISBurst::probe()
{
	_transfer_failed = false;

	// Always verify identity with read-only access before any reset command or GPIO pulse.
	if (readRegister(Register::Product) != _profile.product_id || _transfer_failed) {
		if (hasBoardResetPin() && hardwareReset()) {
			px4_usleep(_profile.reset_wait_us);
			_transfer_failed = false;

			if (readRegister(Register::Product) == _profile.product_id && !_transfer_failed) {
				return readIdentification() ? PX4_OK : PX4_ERROR;
			}
		}

		return PX4_ERROR;
	}

	return readIdentification() ? PX4_OK : PX4_ERROR;
}

int ADISBurst::init()
{
	if (!_endpoint_guard.acquire()) {
		return PX4_ERROR;
	}

	if (!_dma_buffer) {
		_endpoint_guard.release();
		return PX4_ERROR;
	}

#ifdef GPIO_SPI1_RESET_ADIS16477

	if (_profile.variant == Variant::kAdis16477 && hasBoardResetPin()) {
		px4_arch_configgpio(GPIO_SPI1_RESET_ADIS16477);
	}

#endif // GPIO_SPI1_RESET_ADIS16477

#ifdef GPIO_ADIS1650X_RESET

	if (_profile.variant == Variant::kAdis165xx && _profile.format != Format::kTimestamp32 && hasBoardResetPin()) {
		px4_arch_configgpio(GPIO_ADIS1650X_RESET);
	}

#endif // GPIO_ADIS1650X_RESET

#ifdef GPIO_ADIS1657X_RESET

	if (_profile.format == Format::kTimestamp32 && hasBoardResetPin()) {
		px4_arch_configgpio(GPIO_ADIS1657X_RESET);
	}

#endif // GPIO_ADIS1657X_RESET

	const int result = SPI::init();

	if (result == PX4_OK) {
		// All models complete reset, self-test, and configuration asynchronously on the work queue.
		reset();

	} else {
		_endpoint_guard.release();
	}

	return result;
}

void ADISBurst::disableInterrupt()
{
	if (_data_ready_interrupt_enabled && _drdy_gpio != 0) {
		px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr);
	}

	_data_ready_interrupt_enabled = false;
}

void ADISBurst::exit_and_cleanup()
{
	disableInterrupt();
	I2CSPIDriverBase::exit_and_cleanup();
}

void ADISBurst::reset(uint32_t delay_us)
{
	disableInterrupt();
	ScheduleClear();
	_drdy_timestamp_sample.store(0);
	_healthy_since     = 0;
	_diagnostic_streak = 0;
	_ready_samples     = 0;
	_flush_remaining   = 0;
	_state             = State::kReset;

	if (delay_us > 0) {
		ScheduleDelayed(delay_us);

	} else {
		ScheduleNow();
	}
}

void ADISBurst::recover(RecoveryReason reason)
{
	const uint32_t delay = _recovery_attempt == 0 ? 0u : retryDelay(static_cast<unsigned>(_recovery_attempt - 1u), kRecoveryDelays);

	if (_recovery_attempt <= sizeof(kRecoveryDelays) / sizeof(kRecoveryDelays[0])) {
		++_recovery_attempt;
	}

	_recovery_reason = reason;
	_error_perf.count();
	// ScheduleDelayed does not remove a runnable item. Never queue ScheduleNow before the backoff.
	reset(delay);
}

void ADISBurst::fault(RecoveryReason reason)
{
	disableInterrupt();
	ScheduleClear();
	_drdy_timestamp_sample.store(0);
	_recovery_reason = reason;
	_state           = State::kFault;
	_error_perf.count();

	PX4_ERR("%s fault: %s, diagnostic 0x%04x; restart required",
		_profile.device.name, reasonName(reason), unsigned(_last_diagnostic));
}

void ADISBurst::selfTestFailed(RecoveryReason reason)
{
	_self_test_passed = false;
	_recovery_reason  = reason;
	_error_perf.count();

	if (_self_test_attempts >= kSelfTestAttempts) {
		fault(RecoveryReason::kSelfTest);

	} else {
		_state = State::kStartSelfTest;
		ScheduleDelayed(retryDelay(_self_test_attempts - 1, kSelfTestDelays));
	}
}

void ADISBurst::recordDiagnostic(uint16_t diagnostic)
{
	if (diagnostic != 0) {
		_last_diagnostic     = diagnostic;
		_diagnostic_history |= diagnostic;
		_diagnostic_perf.count();
	}
}

void ADISBurst::handleDiagnostic()
{
	if (_state == State::kRead || _state == State::kValidate) {
		if (_diagnostic_streak < UINT8_MAX) {
			++_diagnostic_streak;
		}

		if (adis_burst_decoder::shouldRecoverOnDiagnostic(_last_diagnostic, _diagnostic_streak)) {
			recover(RecoveryReason::kDiagnostic);

		} else {
			// Clear ADI's sticky DIAG_STAT latch (0x02) on ADIS16470/16477/1650x without resetting.
			px4_udelay(_profile.stall_us);
			(void)readRegister(Register::DiagStat);
			_transfer_failed = false;
			_error_perf.count();
		}

		return;
	}

	// State::kWaitReset just read Register::DiagStat directly; only non-transient faults require another reset.
	if ((_last_diagnostic & ~adis_burst_decoder::kTransientDiagnosticMask) != 0) {
		recover(RecoveryReason::kDiagnostic);

	} else {
		_error_perf.count();
	}
}

const char *ADISBurst::stateName(State state)
{
	switch (state) {
	case State::kReset: { return "reset"; }

	case State::kWaitReset: { return "reset wait"; }

	case State::kStartSelfTest: { return "self-test start"; }

	case State::kSelfTest: { return "self-test wait"; }

	case State::kConfigure: { return "configure"; }

	case State::kFlush: { return "filter flush"; }

	case State::kValidate: { return "validate samples"; }

	case State::kRead: { return "read"; }

	case State::kFault: { return "fault"; }
	}

	return "unknown";
}

const char *ADISBurst::reasonName(RecoveryReason reason)
{
	switch (reason) {
	case RecoveryReason::kNone: { return "none"; }

	case RecoveryReason::kTransfer: { return "SPI transfer"; }

	case RecoveryReason::kIntegrity: { return "burst integrity"; }

	case RecoveryReason::kNoData: { return "no fresh sample"; }

	case RecoveryReason::kConfiguration: { return "configuration"; }

	case RecoveryReason::kIdentity: { return "identity/readiness"; }

	case RecoveryReason::kSelfTest: { return "self-test"; }

	case RecoveryReason::kDiagnostic: { return "device diagnostic"; }
	}

	return "unknown";
}

bool ADISBurst::hasBoardResetPin() const
{
#ifdef GPIO_SPI1_RESET_ADIS16477

	if (_profile.variant == Variant::kAdis16477 && !external() && get_device_bus() == 1) {
		return true;
	}

#endif // GPIO_SPI1_RESET_ADIS16477

#if defined(GPIO_ADIS16507_RESET)
# ifdef SPI6_nRESET_EXTERNAL1

	if (_profile.product_id == kProductAdis16507 && external() && get_device_bus() == 6) {
		return true;
	}

# else

	if (_profile.product_id == kProductAdis16507
	    && !external()
	    && px4_find_spi_bus(get_device_id()) == static_cast<int>(get_device_bus())) {
		return true;
	}

# endif // SPI6_nRESET_EXTERNAL1
#endif // GPIO_ADIS16507_RESET

#ifdef GPIO_ADIS1650X_RESET

	if (_profile.variant == Variant::kAdis165xx
	    && _profile.format != Format::kTimestamp32
	    && !external()
	    && px4_find_spi_bus(get_device_id()) == static_cast<int>(get_device_bus())) {
		return true;
	}

#endif // GPIO_ADIS1650X_RESET

#ifdef GPIO_ADIS1657X_RESET

	if (_profile.format == Format::kTimestamp32
	    && !external()
	    && px4_find_spi_bus(get_device_id()) == static_cast<int>(get_device_bus())) {
		return true;
	}

#endif // GPIO_ADIS1657X_RESET

	return false;
}

bool ADISBurst::hardwareReset()
{
	if (!hasBoardResetPin()) {
		return false;
	}

#ifdef GPIO_SPI1_RESET_ADIS16477

	if (_profile.variant == Variant::kAdis16477) {
		px4_arch_gpiowrite(GPIO_SPI1_RESET_ADIS16477, 0);
		px4_udelay(10);
		px4_arch_gpiowrite(GPIO_SPI1_RESET_ADIS16477, 1);

		return true;
	}

#endif // GPIO_SPI1_RESET_ADIS16477

#if defined(GPIO_ADIS16507_RESET)

	if (_profile.product_id == kProductAdis16507) {
		GPIO_ADIS16507_RESET(1);
		px4_udelay(_profile.format == Format::kBurst16 ? 15u : kResetPulseUs);
		GPIO_ADIS16507_RESET(0);

		return true;
	}

#endif // GPIO_ADIS16507_RESET

#ifdef GPIO_ADIS1650X_RESET

	if (_profile.variant == Variant::kAdis165xx && _profile.format != Format::kTimestamp32) {
		px4_arch_configgpio(GPIO_ADIS1650X_RESET);
		px4_arch_gpiowrite(GPIO_ADIS1650X_RESET, 0);
		px4_udelay(kResetPulseUs);
		px4_arch_gpiowrite(GPIO_ADIS1650X_RESET, 1);

		return true;
	}

#endif // GPIO_ADIS1650X_RESET

#ifdef GPIO_ADIS1657X_RESET

	if (_profile.format == Format::kTimestamp32) {
		px4_arch_configgpio(GPIO_ADIS1657X_RESET);
		px4_arch_gpiowrite(GPIO_ADIS1657X_RESET, 0);
		px4_udelay(kResetPulseUs);
		px4_arch_gpiowrite(GPIO_ADIS1657X_RESET, 1);

		return true;
	}

#endif // GPIO_ADIS1657X_RESET

	return false;
}

bool ADISBurst::configureWide()
{
	const bool native_wide = wide();

	if (!_id_valid && !readIdentification()) {
		return false;
	}

	if (_profile.format == Format::kTimestamp32) {
		// ADIS1657x FIFO_CTRL is flash-backed. Explicitly select direct mode and disable
		// watermark output on every configuration pass without issuing a Flash backup command.
		if (!writeRetried(Register::FifoControl, 0, kSamplePeriodUs)) {
			return false;
		}
	}

	if (!native_wide) {
		// Preserve the original unfiltered, undecimated 16-bit ADIS16507 acquisition policy.
		if (!writeVerified(Register::Control, kPointOfPercussion | kLinearGCompensation, kControlUpdateUs)
		    || !writeVerified(Register::Decimation, 0, kDecimationUpdateUs)
		    || !writeVerified(Register::Filter, 0, kFilterUpdateUs)) {
			return false;
		}
	}

	const uint16_t rang_mdl = readRegister(Register::Range);
	const unsigned range    = (rang_mdl >> 2) & 3u; // RANG_MDL bits 3:2, not physical units.

	if (_transfer_failed
	    || (rang_mdl & kRangeReservedMask) != kRangeReservedValue
	    || (_profile.gyro_range_mask & (1u << range)) == 0
	    || !_profile.range_descriptions[range]) {
		PX4_WARN("%s unexpected RANG_MDL 0x%04x", _profile.device.name, unsigned(rang_mdl));
		return false;
	}

	float gyro_range = 0.f;
	float gyro_scale = 0.f;

	switch (range) {
	case 0: {
			gyro_range = 125.f;
			gyro_scale = 1.f / 160.f;
			break;
		}

	case 1: {
			gyro_range = _profile.range_450_dps ? 450.f : 500.f;
			gyro_scale = 1.f / 40.f;
			break;
		}

	case 3: {
			gyro_range = 2000.f;
			gyro_scale = 1.f / 10.f;
			break;
		}

	default: {
			return false;
		}
	}

	_model_description = _profile.range_descriptions[range];
	_px4_accel.set_range(_profile.accel_range);
	_px4_accel.set_scale(_profile.accel_scale);
	_px4_gyro.set_range(math::radians(gyro_range));
	_px4_gyro.set_scale(math::radians(gyro_scale) / (native_wide ? kWideWordFactor : 1.f));

	if (!native_wide) {
		_accel_delay_us = _profile.accel_base_delay_us;
		_gyro_delay_us  = _profile.gyro_base_delay_us;
		return true;
	}

	const uint16_t control = kPointOfPercussion | kLinearGCompensation | kBurst32Enable | kDataReadyActiveHigh;

	if (!writeRetried(Register::Control, control, kControlUpdateUs)
	    || !writeRetried(Register::Decimation, static_cast<uint16_t>(_decimation), kDecimationUpdateUs)) {
		return false;
	}

	// FILT_CTRL is flash-backed. Explicitly program zero as well so reset cannot restore a nonzero filter.
	if (!writeRetried(Register::Filter, static_cast<uint16_t>(_filter), kFilterUpdateUs)) {
		return false;
	}

	if (_filter > 0) {
		_flush_remaining = static_cast<uint8_t>((1u << _filter) + 4u);
	}

	// The latency specification uses tap count N = 2^B, not the encoded FILT_CTRL value B.
	const uint32_t filter_delay     = _filter > 0 ? (1u << _filter) * kSamplePeriodUs : 0;
	const uint32_t decimation_delay = adis_burst_decoder::decimationDelayUs(
			_profile.format, static_cast<uint16_t>(_decimation), kSamplePeriodUs);

	_accel_delay_us = _profile.accel_base_delay_us + filter_delay + decimation_delay;
	_gyro_delay_us  = _profile.gyro_base_delay_us + filter_delay + decimation_delay;

	return true;
}

bool ADISBurst::configure16()
{
	const bool adis16477  = _profile.variant == Variant::kAdis16477;
	float      gyro_range = 2000.f;
	float      gyro_scale = 0.1f;

	if (!adis16477 && !_id_valid && !readIdentification()) {
		return false;
	}

	if (adis16477) {
		const uint16_t rang_mdl = readRegister(Register::Range);
		const unsigned range    = (rang_mdl >> 2) & 3u;

		if (_transfer_failed
		    || (rang_mdl & kRangeReservedMask) != kRangeReservedValue
		    || (_profile.gyro_range_mask & (1u << range)) == 0
		    || !_profile.range_descriptions[range]) {
			PX4_ERR("ADIS16477 unexpected RANG_MDL 0x%04x", unsigned(rang_mdl));
			return false;
		}

		switch (range) {
		case 0: {
				gyro_range = 125.f;
				gyro_scale = 1.f / 160.f;
				break;
			}

		case 1: {
				gyro_range = 500.f;
				gyro_scale = 1.f / 40.f;
				break;
			}

		case 3: {
				gyro_range = 2000.f;
				gyro_scale = 1.f / 10.f;
				break;
			}

		default: {
				return false;
			}
		}

		// Preserve factory linear-g gyro compensation, point-of-percussion alignment and active-high DRDY (0x00C1).
		if (!writeVerified(Register::Control, kLinearGCompensation | kPointOfPercussion | kDataReadyActiveHigh,
				   kControlUpdateUs)
		    || !writeVerified(Register::Filter, 4, 100)     // Original 16-tap Bartlett FIR and readback delay.
		    || !writeVerified(Register::Decimation, 1, 100)) { // Original 1000 samples/s and readback delay.
			return false;
		}

		_model_description = _profile.range_descriptions[range];

	} else {
		// Startup-only host validation bound, not an ADIS16470 worst-case timing specification.
		constexpr uint32_t kReadbackTimeout  { 1_ms };
		constexpr uint32_t kReadbackInterval { 50_us };

		const uint16_t control = readRegister(Register::Control);

		if (_transfer_failed) {
			return false;
		}

		// Always select internal sampling and active-low DRDY without modifying calibration controls.
		if (!writeRegister(Register::Control, control & ~kSamplingMask)) {
			return false;
		}

		const hrt_abstime deadline = hrt_absolute_time() + kReadbackTimeout;

		px4_udelay(kControlUpdateUs);

		uint16_t actual = readRegister(Register::Control);

		while (!_transfer_failed
		       && (actual & kSamplingMask)
		       && hrt_absolute_time() < deadline) {
			px4_udelay(kReadbackInterval);
			actual = readRegister(Register::Control);
		}

		if (_transfer_failed || (actual & kSamplingMask)) {
			PX4_WARN("MSC_CTRL sampling: 0x%04x -> 0x%04x, SPI error %u",
				 unsigned(control), unsigned(actual), unsigned(_transfer_failed));
			return false;
		}

		if (!writeVerified(Register::Decimation, 0, kDecimationUpdateUs)
		    || !writeVerified(Register::Filter, 0, kFilterUpdateUs)) {
			return false;
		}

		_model_description = _profile.range_descriptions[3];
	}

	_accel_delay_us = _profile.accel_base_delay_us;
	_gyro_delay_us  = _profile.gyro_base_delay_us;
	_px4_accel.set_range(_profile.accel_range);
	_px4_accel.set_scale(_profile.accel_scale);
	_px4_gyro.set_range(math::radians(gyro_range));
	_px4_gyro.set_scale(math::radians(gyro_scale));

	return true;
}

bool ADISBurst::configure()
{
	return classic() ? configure16() : configureWide();
}

bool ADISBurst::checkConfiguration()
{
	const uint8_t count = _profile.format == Format::kTimestamp32 ? 4u : 3u;
	const uint8_t slot  = _checked_config % count;

	px4_udelay(_profile.stall_us);

	bool valid = false;

	if (_profile.variant == Variant::kAdis16470) {
		// ADIS16470 preserves calibration bits in MSC_CTRL; check only sampling/filter masks.
		static constexpr Register registers[] { Register::Control, Register::Decimation, Register::Filter };
		static constexpr uint16_t masks[]     { kSamplingMask, 0x07ff, 0x0007 };
		valid = (readRegister(registers[slot]) & masks[slot]) == 0;

	} else if (_profile.variant == Variant::kAdis16477) {
		static constexpr Register registers[] { Register::Control, Register::Decimation, Register::Filter };
		static constexpr uint16_t expected[] {
			kLinearGCompensation | kPointOfPercussion | kDataReadyActiveHigh,
			1,
			4,
		};
		valid = readRegister(registers[slot]) == expected[slot];

	} else if (!wide()) {
		static constexpr Register registers[] { Register::Control, Register::Decimation, Register::Filter };
		static constexpr uint16_t expected[]  { kPointOfPercussion | kLinearGCompensation, 0, 0 };
		valid = readRegister(registers[slot]) == expected[slot];

	} else {
		const Register registers[] {
			Register::Control,
			Register::Decimation,
			Register::Filter,
			Register::FifoControl,
		};
		const uint16_t expected[] {
			kPointOfPercussion | kLinearGCompensation | kBurst32Enable | kDataReadyActiveHigh,
			static_cast<uint16_t>(_decimation),
			static_cast<uint16_t>(_filter),
			0,
		};
		valid = readRegister(registers[slot]) == expected[slot];
	}

	if (!valid || _transfer_failed) {
		_bad_register_perf.count();
		return false;
	}

	_checked_config              = (slot + 1u) % count;
	_last_config_check_timestamp = hrt_absolute_time();

	return true;
}

void ADISBurst::startReading()
{
	_progress.reset(hrt_absolute_time());
	_ready_samples     = 0;
	_checked_config    = 0;
	_irq_streak        = 0;
	_diagnostic_streak = 0;
	_healthy_since     = 0;
	_drdy_timestamp_sample.store(0);
	_state = State::kValidate;

	const bool rising = probeConfigured() || wide();
	const bool event  = rising;

	_data_ready_interrupt_enabled = _drdy_gpio != 0
					&& px4_arch_gpiosetevent(_drdy_gpio, rising, !rising, event, dataReadyCallback, this) == 0;

	_fallback_polling = !_data_ready_interrupt_enabled;
	ScheduleDelayed(_fallback_polling ? _interval_us : _irq_timeout_us);
}

int ADISBurst::dataReadyCallback(int irq, void *context, void *arg)
{
	(void)irq;
	(void)context;

	auto *driver = static_cast<ADISBurst *>(arg);

	// DR already marks an output update after DEC_RATE. Do not decimate it again in software.
	driver->_drdy_timestamp_sample.store(hrt_absolute_time());
	driver->ScheduleNow();

	return 0;
}

ADISBurst::ReadResult ADISBurst::measure(hrt_abstime timestamp)
{
	const size_t burst_length = adis_burst_decoder::size(_profile.format);

	if (burst_length > _dma_buffer.capacity()) {
		return ReadResult::kTransferError;
	}

	uint8_t  *buffer = _dma_buffer.data();
	uint16_t *words  = _dma_buffer.words();
	memset(buffer, 0, burst_length);

	if (wordTransfers()) {
		words[0] = static_cast<uint16_t>(kBurstCommand) << 8;

	} else {
		buffer[0] = kBurstCommand;
	}

	set_frequency(_data_frequency);

	adis_burst_decoder::Sample sample {};

	const bool transferred = wordTransfers() ? transferWords(words, words, burst_length / sizeof(uint16_t))
				 : transferBytes(buffer, buffer, burst_length);

	if (!transferred) {
		return ReadResult::kTransferError;
	}

	uint16_t diagnostic = 0;
	const bool valid = wordTransfers()
			   ? adis_burst_decoder::decode<kNativeWordOrder>(buffer, burst_length, _profile.format, sample, &diagnostic)
			   : adis_burst_decoder::decode(buffer, burst_length, _profile.format, sample, &diagnostic);

	if (!valid) {
		recordDiagnostic(diagnostic);

		return diagnostic != 0 ? ReadResult::kDeviceError : ReadResult::kInvalid;
	}

	// Only complete, healthy frames may advance the epoch or its host-time deadline.
	if (!_progress.observe(sample.counter, hrt_absolute_time())) {
		return ReadResult::kDuplicate;
	}

	if (_state == State::kValidate) {
		if (++_ready_samples < kReadySamples) {
			return ReadResult::kWarmingUp;
		}

		_state = State::kRead;
	}

	const float temperature = static_cast<float>(sample.temperature) * kTemperatureScale;

	_px4_accel.set_temperature(temperature);
	_px4_gyro.set_temperature(temperature);

	const uint64_t errors = _bad_transfer_perf.eventCount() + _diagnostic_perf.eventCount()
				+ _stale_perf.eventCount() + _bad_register_perf.eventCount();

	_px4_accel.set_error_count(errors);
	_px4_gyro.set_error_count(errors);

	// ADIS16470/1650x/1657x sensor frames are FLU (+X forward, +Y left, +Z up); flip Y and Z to publish PX4 FRD.
	// ADIS16470 saturates int16 negation; ADIS16477 preserves unflipped axes for boards/av/x-v1 (-R 8).
	if (_profile.variant == Variant::kAdis16470) {
		for (size_t axis = 1; axis < 3; ++axis) {
			sample.accel[axis] = imu::negateSaturated(static_cast<int16_t>(sample.accel[axis]));
			sample.gyro[axis]  = imu::negateSaturated(static_cast<int16_t>(sample.gyro[axis]));
		}
	}

	const float yz_sign = _profile.variant == Variant::kAdis165xx ? -1.f : 1.f;

	const hrt_abstime accel_timestamp = timestamp > _accel_delay_us ? timestamp - _accel_delay_us : 0;
	const hrt_abstime gyro_timestamp  = timestamp > _gyro_delay_us ? timestamp - _gyro_delay_us : 0;

	const auto publish_accel = [&]() {
		_px4_accel.update(accel_timestamp,
				  static_cast<float>(sample.accel[0]),
				  static_cast<float>(sample.accel[1]) * yz_sign,
				  static_cast<float>(sample.accel[2]) * yz_sign);
	};
	const auto publish_gyro = [&]() {
		_px4_gyro.update(gyro_timestamp,
				 static_cast<float>(sample.gyro[0]),
				 static_cast<float>(sample.gyro[1]) * yz_sign,
				 static_cast<float>(sample.gyro[2]) * yz_sign);
	};

	// Preserve original publication order: 1650x/1657x gyro first, other paths accel first.
	if (wide()) {
		publish_gyro();
		publish_accel();

	} else {
		publish_accel();
		publish_gyro();
	}

	return ReadResult::kPublished;
}

void ADISBurst::RunImpl()
{
	_transfer_failed = false;

	// Fetch _drdy_timestamp_sample BEFORE hrt_absolute_time() so an ISR firing in between cannot underflow (now - irq).
	const hrt_abstime irq = _data_ready_interrupt_enabled ? _drdy_timestamp_sample.fetch_and(0) : 0;
	const hrt_abstime now = hrt_absolute_time();

	// The steady-state path remains first; validation shares acquisition without a second parser.
	if (__builtin_expect(_state == State::kRead, 1) || _state == State::kValidate) {
		if (_healthy_since != 0 && _progress.expired(now, _irq_timeout_us)) {
			_healthy_since = 0; // A long gap followed by one good frame is not sustained recovery.
		}

		const bool        fresh_irq = _data_ready_interrupt_enabled && irq != 0 && now >= irq && (now - irq) < _interval_us;
		const hrt_abstime timestamp = fresh_irq ? irq : now;

		if (fresh_irq) {
			if (_irq_streak < kIrqRecoverySamples) {
				++_irq_streak;
			}

			if (_irq_streak == kIrqRecoverySamples) {
				_fallback_polling = false;
			}

		} else {
			if (_data_ready_interrupt_enabled && !_fallback_polling) {
				_drdy_missed_perf.count();
			}

			_irq_streak       = 0;
			_fallback_polling = true;
		}

		// Always schedule _irq_timeout_us on fresh_irq to prevent 2x SPI livelock on slightly slower oscillators.
		ScheduleDelayed(fresh_irq ? _irq_timeout_us : _interval_us);
		_read_perf.begin();

		const ReadResult result = measure(timestamp);

		_read_perf.end();

		const hrt_abstime read_completed = hrt_absolute_time();

		if (result == ReadResult::kPublished) {
			_diagnostic_streak = 0;

			if (_healthy_since == 0) {
				_healthy_since = read_completed;

			} else if (read_completed - _healthy_since >= 1_s) {
				_recovery_attempt = 0;
			}

		} else if (result != ReadResult::kDuplicate && result != ReadResult::kWarmingUp) {
			_healthy_since = 0;
			_ready_samples = 0;

			if (result == ReadResult::kDeviceError) {
				handleDiagnostic();

				if (_state == State::kReset) {
					return;
				}

			} else {
				_bad_transfer_perf.count();
			}
		}

		if (_progress.expired(read_completed, _sample_timeout_us)) {
			_stale_perf.count();

			const RecoveryReason reason = result == ReadResult::kTransferError ? RecoveryReason::kTransfer
						      : result == ReadResult::kInvalid ? RecoveryReason::kIntegrity
						      : result == ReadResult::kDeviceError ? RecoveryReason::kDiagnostic
						      : RecoveryReason::kNoData;
			recover(reason);

			return;
		}

		if (_state == State::kRead
		    && hrt_elapsed_time(&_last_config_check_timestamp) > 100_ms
		    && !checkConfiguration()) {
			recover(RecoveryReason::kConfiguration);
		}

		return;
	}

	switch (_state) {
	case State::kReset: {
			_reset_perf.count();

			// Preserve the compatibility endpoint's software-reset-before-GPIO sequence.
			if (_profile.variant == Variant::kAdis165xx && !wide()) {
				const bool sent = writeRegister(Register::Command, kSoftwareReset);

				if (!hardwareReset() && !sent) {
					recover(RecoveryReason::kTransfer);
					break;
				}

			} else if (!hardwareReset() && !writeRegister(Register::Command, kSoftwareReset)) {
				recover(RecoveryReason::kTransfer);
				break;
			}

			_reset_timestamp = now;
			_state           = State::kWaitReset;
			ScheduleDelayed(_profile.reset_wait_us);
			break;
		}

	case State::kWaitReset: {
			const uint16_t product = readRegister(Register::Product);

			if (product != _profile.product_id || _transfer_failed) {
				if (now - _reset_timestamp >= _profile.reset_wait_us + 200_ms) {
					recover(RecoveryReason::kIdentity);

				} else {
					ScheduleDelayed(10_ms);
				}

				break;
			}

			if (!_self_test_passed) {
				_state = State::kStartSelfTest;
				ScheduleNow();
				break;
			}

			const uint16_t diagnostic = readRegister(Register::DiagStat);

			if (_transfer_failed) {
				recover(RecoveryReason::kTransfer);
				break;
			}

			if (diagnostic != 0) {
				recordDiagnostic(diagnostic);
				handleDiagnostic();

				if (_state == State::kReset) {
					break;
				}
			}

			_state = State::kConfigure;
			ScheduleNow();
			break;
		}

	case State::kStartSelfTest: {
			++_self_test_attempts;

			uint16_t command = kSensorSelfTest;
			uint32_t wait_us = _profile.self_test_wait_us;

			if (!_memory_self_test_passed) {
				command = kMemorySelfTest;
				wait_us = 32_ms;
			}

			if (!writeRegister(Register::Command, command)) {
				selfTestFailed(RecoveryReason::kTransfer);
				break;
			}

			// The command wait and SPI-readiness bound are separate. DIAG_STAT is read only once.
			_self_test_deadline = now + wait_us + 200_ms;
			_state              = State::kSelfTest;
			ScheduleDelayed(wait_us);
			break;
		}

	case State::kSelfTest: {
			if (readRegister(Register::Product) != _profile.product_id || _transfer_failed) {
				if (now >= _self_test_deadline) {
					selfTestFailed(RecoveryReason::kIdentity);

				} else {
					ScheduleDelayed(1_ms);
				}

				break;
			}

			const uint16_t diagnostic   = readRegister(Register::DiagStat);
			const bool     memory_phase = !_memory_self_test_passed;
			uint16_t       fail_mask    = kDiagSelfTestError;

			if (memory_phase) {
				fail_mask |= kDiagMemoryError;
			}

			if (_transfer_failed || (diagnostic & fail_mask) != 0) {
				recordDiagnostic(diagnostic);
				selfTestFailed(_transfer_failed ? RecoveryReason::kTransfer : RecoveryReason::kSelfTest);
				break;
			}

			recordDiagnostic(diagnostic);

			if (memory_phase) {
				_memory_self_test_passed = true;
				_self_test_attempts      = 0;
				_state                   = State::kStartSelfTest;
				ScheduleNow();
				break;
			}

			_self_test_passed = true;
			reset(); // Require a second reset after self-test; do not invalidate completed self-test flags.
			break;
		}

	case State::kConfigure: {
			if (!configure()) {
				_bad_register_perf.count();
				recover(RecoveryReason::kConfiguration);
				break;
			}

			_last_config_check_timestamp = now;

			if (_flush_remaining > 0) {
				_state = State::kFlush;
				ScheduleNow();

			} else {
				startReading();
			}

			break;
		}

	case State::kFlush: {
			if (_flush_remaining == 0) {
				startReading();
				break;
			}

			// One discarded burst per pass; no sleeping across filter cycles on a shared work queue.
			const size_t burst_length = adis_burst_decoder::size(_profile.format);
			uint8_t     *discard      = _dma_buffer.data();
			memset(discard, 0, burst_length);
			discard[0] = kBurstCommand;
			set_frequency(_data_frequency);

			if (burst_length > _dma_buffer.capacity() || !transferBytes(discard, discard, burst_length)) {
				_bad_transfer_perf.count();
				recover(RecoveryReason::kTransfer);
				break;
			}

			--_flush_remaining;
			ScheduleDelayed(1100_us);
			break;
		}

	case State::kValidate:
	case State::kRead: {
			break; // Handled by the acquisition path.
		}

	case State::kFault: {
			break; // Kept visible to status/stop when startup self-test fails all retries.
		}
	}
}

void ADISBurst::print_status()
{
	I2CSPIDriverBase::print_status();

	PX4_INFO("Model:    %s", _model_description);
	PX4_INFO("Serial:   0x%X", _serial_number);
	PX4_INFO("Firmware: revision 0x%X, date Y %X DM %X", _firmware_revision, _firmware_year, _firmware_day_month);

	PX4_INFO("state: %s, recovery: %s, level: %u, sampling: %s",
		 stateName(_state), reasonName(_recovery_reason), unsigned(_recovery_attempt),
		 _fallback_polling ? "poll fallback" : "DRDY");
	PX4_INFO("diagnostic: last 0x%04x, history 0x%04x, new-sample timeout %u us",
		 unsigned(_last_diagnostic), unsigned(_diagnostic_history), unsigned(_sample_timeout_us));

	PX4_INFO("type %s, %u-bit, poll %.3f Hz, decimation %d, filter %d",
		 _profile.device.name,
		 _profile.format == Format::kBurst16 ? 16u : 32u,
		 1e6 / static_cast<double>(_interval_us),
		 _decimation,
		 _filter);

	imu::printSpiStatus(_profile.device,
			    _spi_mode,
			    _register_frequency,
			    _data_frequency,
			    get_frequency());

	_read_perf.print();
	_error_perf.print();
	_bad_transfer_perf.print();
	_reset_perf.print();
	_bad_register_perf.print();
	_drdy_missed_perf.print();
	_stale_perf.print();
	_diagnostic_perf.print();
}
