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

#include "AdiBurst.hpp"
#include <lib/mathlib/mathlib.h>

using namespace time_literals;
using adi_burst_decoder::Format;
using Variant = AdiBurstVariant;

namespace
{
constexpr uint8_t  kWriteBit            { 0x80 };
constexpr uint8_t  kBurstCommand        { 0x68 };
constexpr uint16_t kSoftwareReset       { 1u << 7 };
constexpr uint16_t kSensorSelfTest      { 1u << 2 };
constexpr uint16_t kDataReadyActiveHigh { 1u << 0 };
constexpr uint16_t kPointOfPercussion   { 1u << 6 };
constexpr uint16_t kLinearGCompensation { 1u << 7 };
constexpr uint16_t kBurst32Enable       { 1u << 9 };
constexpr unsigned kSamplePeriodUs      { 500 };
constexpr unsigned kControlUpdateUs     { 200 };
constexpr unsigned kFilterUpdateUs      { 30 };
constexpr unsigned kDecimationUpdateUs  { 30 };
constexpr unsigned kResetPulseUs        { 100 };
constexpr unsigned kAccelBaseDelayUs    { 1570 };
constexpr unsigned kGyroBaseDelayUs     { 1437 };
constexpr float    kTemperatureScale    { 0.1f };
constexpr float    kWideWordFactor      { 65536.f };
constexpr unsigned kSelfTestAttempts    { 3 };
constexpr unsigned kReadySamples        { 2 }; // Require progress across two validated bursts after configuration.
constexpr unsigned kIrqRecoverySamples  { 3 }; // Hysteresis before leaving fallback polling.
constexpr uint32_t kRecoveryDelays[]    { 100_ms, 500_ms, 2_s, 5_s };
constexpr uint32_t kSelfTestDelays[]    { 1_s, 2_s };
constexpr imu::ByteOrder kNativeWordOrder {
	__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__ ? imu::ByteOrder::kBigEndian : imu::ByteOrder::kLittleEndian
};
}

AdiBurst::AdiBurst(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_endpoint_claim(config),
	_model(*static_cast<const AdiBurstOptions *>(config.custom_data)->model),
	_register_frequency(imu::spiConfigFrequency(_model.device, config.custom2)),
	_data_frequency(imu::spiDataFrequency(_model.device, config.custom2)),
	_decimation(_model.variant == Variant::kAdis16497 ? 3 : _model.variant == Variant::kAdis16477 ? 1 : static_cast<const AdiBurstOptions *>
		    (config.custom_data)->decimation),
	_filter(_model.variant == Variant::kAdis16477 ? 4 : static_cast<const AdiBurstOptions *>(config.custom_data)->filter),
	_drdy_gpio(config.drdy_gpio),
	_accel(get_device_id(), config.rotation, config.external),
	_gyro(get_device_id(), config.rotation, config.external)
{
	// Preserve the original integer-Hz conversion for non-integral output rates.
	const unsigned rate_hz = 2000 / (_decimation + 1);

	_interval_us = probeConfigured() ? 1000 : 1000000 / rate_hz;

	// Host liveness uses the actual polling period, not compensated publication timestamps.
	_irq_timeout_us    = math::max(3 * _interval_us, uint32_t(2_ms));
	_sample_timeout_us = math::max(10 * _interval_us, uint32_t(20_ms));
}

AdiBurst::~AdiBurst() = default;

bool AdiBurst::transfer(uint8_t *tx, uint8_t *rx, size_t size)
{
	// A failed transaction invalidates the page cache and suppresses the remaining transfers in this work-queue pass.
	if (_transfer_failed || SPI::transfer(tx, rx, size) != PX4_OK) {
		_transfer_failed = true;
		_page            = UINT8_MAX;

		return false;
	}

	return true;
}

bool AdiBurst::transfer(uint16_t *tx, uint16_t *rx, size_t words)
{
	if (_transfer_failed || SPI::transferhword(tx, rx, words) != PX4_OK) {
		_transfer_failed = true;
		_page = UINT8_MAX;
		return false;
	}

	return true;
}

bool AdiBurst::paged() const
{
#if defined(CONFIG_ADI_BURST_ADIS16497)

	return _model.variant == Variant::kAdis16497;

#else

	return false;

#endif // CONFIG_ADI_BURST_ADIS16497
}

bool AdiBurst::classic() const
{
#if defined(CONFIG_ADI_BURST_ADIS16470) || defined(CONFIG_ADI_BURST_ADIS16477)

	return _model.variant == Variant::kAdis16470 || _model.variant == Variant::kAdis16477;

#else

	return false;

#endif // CONFIG_ADI_BURST_ADIS16470 || CONFIG_ADI_BURST_ADIS16477
}

bool AdiBurst::selectPage(uint8_t page)
{
	if (!paged() || _page == page) {
		return !_transfer_failed;
	}

	set_frequency(_register_frequency);

	// ADIS16497 writes PAGE_ID as a pair of native 16-bit commands, like the original driver.
	uint16_t command[2] { static_cast<uint16_t>(0x8000u | page), 0x8100 };

	if (!transfer(command, nullptr, 1)) {
		return false;
	}

	px4_udelay(_model.stall_us);

	if (!transfer(command + 1, nullptr, 1)) {
		return false;
	}

	px4_udelay(_model.stall_us);
	_page = page;

	return true;
}

uint16_t AdiBurst::readRegister(Register reg)
{
	if (!selectPage(static_cast<uint16_t>(reg) >> 8)) {
		return 0;
	}

	set_frequency(_register_frequency);

	if (wordTransfers()) {
		uint16_t command = static_cast<uint8_t>(reg) << 8;
		uint16_t reply {};

		transfer(&command, nullptr, 1);
		px4_udelay(_model.stall_us);
		transfer(nullptr, &reply, 1);
		// tSTALL also separates this reply from the next register command.
		px4_udelay(_model.stall_us);

		return reply;
	}

	uint8_t command[2] { static_cast<uint8_t>(reg), 0 };
	uint8_t reply[2]   {};

	// Register reads are pipelined: send the address, then clock out its reply in a separate transaction.
	transfer(command, nullptr, sizeof(command));
	px4_udelay(_model.stall_us);
	transfer(nullptr, reply, sizeof(reply));
	px4_udelay(_model.stall_us);

	return adi_burst_decoder::unsignedWord(reply);
}

bool AdiBurst::writeRegister(Register reg, uint16_t value)
{
	if (!selectPage(static_cast<uint16_t>(reg) >> 8)) {
		return false;
	}

	set_frequency(_register_frequency);

	const uint8_t address = static_cast<uint8_t>(reg);

	if (wordTransfers()) {
		unsigned stall_us = _model.stall_us;

		if (paged()) {
			// ADIS16497 Rev. D Table 3 overrides the normal tSTALL for these
			// configuration registers. Respect it after each byte-write transaction.
			switch (reg) {
			case Register::PagedControl:
			case Register::PagedDecimation: {
					stall_us = 340;
					break;
				}

			case Register::PagedConfig: {
					stall_us = 45;
					break;
				}

			case Register::PagedNull: {
					stall_us = 71;
					break;
				}

			case Register::PagedFilter0:
			case Register::PagedFilter1: {
					stall_us = 65;
					break;
				}

			default: {
					break;
				}
			}
		}

		uint16_t command[2] {
			static_cast<uint16_t>(((address | kWriteBit) << 8) | (value & 0xff)),
			static_cast<uint16_t>((((address + 1) | kWriteBit) << 8) | (value >> 8))
		};

		transfer(command, nullptr, 1);
		px4_udelay(stall_us);
		transfer(command + 1, nullptr, 1);
		px4_udelay(stall_us);

		return !_transfer_failed;
	}

	uint8_t low[2]  { static_cast<uint8_t>(address | kWriteBit), static_cast<uint8_t>(value) };
	uint8_t high[2] { static_cast<uint8_t>((address + 1) | kWriteBit), static_cast<uint8_t>(value >> 8) };

	// Write the low and high bytes separately, retaining the model's stall time after each transaction.
	transfer(low, nullptr, sizeof(low));
	px4_udelay(_model.stall_us);
	transfer(high, nullptr, sizeof(high));
	px4_udelay(_model.stall_us);

	return !_transfer_failed;
}

bool AdiBurst::writeVerified(Register reg, uint16_t value, unsigned delay_us)
{
	if (!writeRegister(reg, value)) {
		return false;
	}

	// MSC_CTRL readback may lag by 200 us; wait before comparing.
	px4_udelay(delay_us);

	return readRegister(reg) == value && !_transfer_failed;
}

bool AdiBurst::writeRetried(Register reg, uint16_t value, unsigned delay_us)
{
	// ADIS1650x/1657x retry each write/readback three times before resetting.
	for (unsigned attempt = 0; attempt < 3; ++attempt) {
		_transfer_failed = false;

		// Each attempt must wait for register settling before checking readback.
		if (writeVerified(reg, value, delay_us)) {
			return true;
		}
	}

	PX4_WARN("register 0x%04x did not accept 0x%04x", unsigned(reg), unsigned(value));
	return false;
}

bool AdiBurst::selfTest(bool memory)
{
	constexpr uint16_t kMemoryTest { 1u << 4 };
	constexpr uint16_t kPagedSelfTest { 1u << 1 };

	const uint16_t command = memory ? kMemoryTest : paged() ? kPagedSelfTest : kSensorSelfTest;

	// This bounded synchronous path is used only by the original probe lifecycle.
	// Runtime recovery never calls it or introduces a new mechanical self-test.
	for (unsigned attempt = 0; attempt < kSelfTestAttempts; ++attempt) {
		_transfer_failed = false;

		if (writeRegister(commandRegister(), command)) {
			px4_usleep(memory ? 32_ms : _model.self_test_wait_us);
			const hrt_abstime deadline = hrt_absolute_time() + 200_ms;

			// Check SPI availability before diagnostics. ADIS1657x keeps SPI accessible
			// during self-test, so PROD_ID is NOT a completion indication on that family.
			// Its profile includes a host time guard; Rev. A only specifies typical timing.
			do {
				_transfer_failed = false;

				if (readRegister(productRegister()) == _model.product_id && !_transfer_failed) {
					const uint16_t diagnostic = readRegister(diagnosticRegister());

					if (!_transfer_failed) {
						recordDiagnostic(diagnostic);

						if (diagnostic == 0) {
							return true;
						}
					}

					break; // Reissue the test after a failure; never pass by re-reading cleared flags.
				}

				px4_usleep(1_ms);
			} while (hrt_absolute_time() < deadline);
		}

		_error_perf.count();

		if (attempt + 1 < kSelfTestAttempts) {
			px4_usleep(imu::retryDelay(attempt, kSelfTestDelays));
		}
	}

	PX4_ERR("%s %s test failed after %u attempts, status 0x%04x",
		_model.device.name, memory ? "memory" : "sensor", kSelfTestAttempts, unsigned(_last_diagnostic));
	return false;
}

bool AdiBurst::resetAndConfigure()
{
	// ADIS16477/16497 configure before probing and self-testing, not afterwards.
	_transfer_failed = false;
	_page = UINT8_MAX;
	_reset_perf.count();

	if (!hardwareReset() && !writeRegister(commandRegister(), kSoftwareReset)) {
		return false;
	}

	_page = UINT8_MAX;
	px4_usleep(_model.reset_wait_us);

	const hrt_abstime deadline = hrt_absolute_time() + 200_ms;

	// Probe must obey the same readiness gate as scheduled recovery. A reset
	// delay is a first-check time, not evidence that a device accepted reset.
	do {
		_transfer_failed = false;
		_page = UINT8_MAX;

		if (readRegister(productRegister()) == _model.product_id && !_transfer_failed) {
			return configure();
		}

		px4_usleep(10_ms);
	} while (hrt_absolute_time() < deadline);

	return false;
}

int AdiBurst::probe()
{
	_transfer_failed = false;

	if (probeConfigured()) {
		bool configured = resetAndConfigure();

		// Retain the original bounded probe lifecycle: start reports failure after five attempts.
		for (unsigned attempt = 0; attempt < 5; ++attempt) {
			if (configured && readRegister(productRegister()) == _model.product_id && !_transfer_failed) {
				if ((_model.variant == Variant::kAdis16477 && !selfTest(true)) || !selfTest()) {
					return PX4_ERROR;
				}

				_self_test_passed = true;

				return !paged() || configurePagedRange() ? PX4_OK : PX4_ERROR;
			}

			if (_state == State::kFault) {
				return PX4_ERROR;
			}

			configured = resetAndConfigure();
		}

		return PX4_ERROR;
	}

	if (readRegister(productRegister()) != _model.product_id || _transfer_failed) {
		return PX4_ERROR;
	}

	// The 32-bit product drivers self-test in probe, then reset before configuration.
	if (wide()) {
		_self_test_passed = selfTest();
		return _self_test_passed ? PX4_OK : PX4_ERROR;
	}

	return PX4_OK;
}

int AdiBurst::init()
{
	// Model/mode keys preserve typed stop/status, but must not permit two
	// instances to reset or reconfigure the same physical SPI endpoint.
	if (!_endpoint_claim.acquire()) {
		PX4_ERR("SPI endpoint unavailable or already owned by a family instance");
		return PX4_ERROR;
	}

#ifdef GPIO_SPI1_RESET_ADIS16477

	if (_model.variant == Variant::kAdis16477) {
		px4_arch_configgpio(GPIO_SPI1_RESET_ADIS16477);
	}

#endif // GPIO_SPI1_RESET_ADIS16477

#ifdef GPIO_SPI1_RESET_ADIS16497

	if (paged()) {
		px4_arch_configgpio(GPIO_SPI1_RESET_ADIS16497);
	}

#endif // GPIO_SPI1_RESET_ADIS16497

	// Preserve the pre-probe hardware reset used by ADIS1650x/1657x on driver restarts.
	if (wide() && hardwareReset()) {
		px4_usleep(_model.reset_wait_us);
	}

	const int result = SPI::init();

	if (result == PX4_OK) {
		if (probeConfigured()) {
			startReading();

		} else {
			reset();
		}

	} else {
		_endpoint_claim.release();
	}

	return result;
}

void AdiBurst::disableInterrupt()
{
	if (_interrupt_enabled) {
		px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr);
	}

	_interrupt_enabled = false;
}

void AdiBurst::exit_and_cleanup()
{
	disableInterrupt();
	I2CSPIDriverBase::exit_and_cleanup();
}

void AdiBurst::reset(uint32_t delay_us)
{
	disableInterrupt();
	ScheduleClear();
	_drdy_timestamp.store(0);
	_healthy_since   = 0;
	_ready_samples   = 0;
	_flush_remaining = 0;
	_state           = State::kReset;

	if (delay_us > 0) {
		ScheduleDelayed(delay_us);

	} else {
		ScheduleNow();
	}
}

void AdiBurst::recover(RecoveryReason reason)
{
	const uint32_t delay = imu::retryDelay(_recovery_attempt, kRecoveryDelays);

	if (_recovery_attempt < sizeof(kRecoveryDelays) / sizeof(kRecoveryDelays[0])) {
		++_recovery_attempt;
	}

	_recovery_reason = reason;
	_error_perf.count();
	// ScheduleDelayed does not remove a runnable item. Never queue ScheduleNow before the backoff.
	reset(delay);
}

void AdiBurst::fault(RecoveryReason reason)
{
	disableInterrupt();
	ScheduleClear();
	_drdy_timestamp.store(0);
	_recovery_reason = reason;
	_state = State::kFault;
	_error_perf.count();

	PX4_ERR("%s fault: %s, diagnostic 0x%04x; restart required",
		_model.device.name, reasonName(reason), unsigned(_last_diagnostic));
}

void AdiBurst::selfTestFailed(RecoveryReason reason)
{
	_self_test_passed = false;
	_recovery_reason = reason;
	_error_perf.count();

	if (_self_test_attempts >= kSelfTestAttempts) {
		fault(RecoveryReason::kSelfTest);

	} else {
		_state = State::kStartSelfTest;
		ScheduleDelayed(imu::retryDelay(_self_test_attempts - 1, kSelfTestDelays));
	}
}

void AdiBurst::recordDiagnostic(uint16_t diagnostic)
{
	if (diagnostic != 0) {
		_last_diagnostic     = diagnostic;
		_diagnostic_history |= diagnostic;
		_diagnostic_perf.count();
	}
}

void AdiBurst::handleDiagnostic()
{
	// Diagnostic meanings come from the model, not from 16/32-bit acquisition width.
	constexpr uint16_t kSpiError { 1u << 3 };

	// Runtime never attempts to clear a reported hardware failure by repeatedly resetting or re-reading it.
	if (_last_diagnostic & _model.hard_fault_mask) {
		fault(RecoveryReason::kDiagnostic);

	} else if ((_last_diagnostic & ~kSpiError) != 0) {
		// Includes overrun, watchdog restart, supply and synchronization faults.
		recover(RecoveryReason::kDiagnostic);
	}

	// An isolated SPI-framing flag drops this sample. Persistent errors reach the liveness deadline.
}

const char *AdiBurst::stateName(State state)
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

const char *AdiBurst::reasonName(RecoveryReason reason)
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

bool AdiBurst::hardwareReset()
{
#ifdef GPIO_SPI1_RESET_ADIS16477

	if (_model.variant == Variant::kAdis16477) {
		px4_arch_gpiowrite(GPIO_SPI1_RESET_ADIS16477, 0);
		px4_udelay(10);
		px4_arch_gpiowrite(GPIO_SPI1_RESET_ADIS16477, 1);

		return true;
	}

#endif // GPIO_SPI1_RESET_ADIS16477

#ifdef GPIO_SPI1_RESET_ADIS16497

	if (paged()) {
		px4_arch_gpiowrite(GPIO_SPI1_RESET_ADIS16497, 0);
		px4_udelay(10);
		px4_arch_gpiowrite(GPIO_SPI1_RESET_ADIS16497, 1);

		return true;
	}

#endif // GPIO_SPI1_RESET_ADIS16497

#ifdef GPIO_ADIS16507_RESET

	if (_model.variant == Variant::kAdis165xx && _model.format == Format::kBurst16) {
		GPIO_ADIS16507_RESET(1);
		px4_udelay(15); // Original 16-bit path: minimum reset pulse is 10 us.
		GPIO_ADIS16507_RESET(0);

		return true;
	}

#endif // GPIO_ADIS16507_RESET

#ifdef GPIO_ADIS1650X_RESET

	if (_model.format == Format::kBurst32) {
		px4_arch_configgpio(GPIO_ADIS1650X_RESET);
		px4_arch_gpiowrite(GPIO_ADIS1650X_RESET, 0);
		px4_udelay(kResetPulseUs);
		px4_arch_gpiowrite(GPIO_ADIS1650X_RESET, 1);

		return true;
	}

#endif // GPIO_ADIS1650X_RESET

#ifdef GPIO_ADIS1657X_RESET

	if (_model.format == Format::kTimestamp32) {
		px4_arch_configgpio(GPIO_ADIS1657X_RESET);
		px4_arch_gpiowrite(GPIO_ADIS1657X_RESET, 0);
		px4_udelay(kResetPulseUs);
		px4_arch_gpiowrite(GPIO_ADIS1657X_RESET, 1);

		return true;
	}

#endif // GPIO_ADIS1657X_RESET
	return false;
}

bool AdiBurst::configureWide()
{
	const bool native_wide = wide();

	if (_model.format == Format::kTimestamp32) {
		// ADIS1657x FIFO_CTRL is flash-backed. This driver timestamps live output,
		// never queued FIFO records. Explicitly select direct mode and disable the
		// watermark output on every configuration pass. No Flash backup command.
		if (!writeRetried(Register::FifoControl, 0, kSamplePeriodUs)) {
			return false;
		}
	}

	if (!native_wide) {
		// Preserve the original unfiltered, undecimated 16-bit acquisition policy,
		// even if a previous application saved different settings in device Flash.
		// MSC_CTRL readback may lag by 200 us in either burst format (ADIS16507 Table 106).
		if (!writeVerified(Register::Control, kPointOfPercussion | kLinearGCompensation, kControlUpdateUs)
		    || !writeVerified(Register::Decimation, 0, kDecimationUpdateUs)
		    || !writeVerified(Register::Filter, 0, kFilterUpdateUs)) {
			return false;
		}
	}

	const uint16_t range_word = readRegister(Register::Range);
	const unsigned range = (range_word >> 2) & 3; // RANG_MDL bits 3:2, not physical units.

	if (_transfer_failed || (_model.gyro_range_mask & (1u << range)) == 0) {
		if (!_transfer_failed) {
			fault(RecoveryReason::kConfiguration); // A different range suffix cannot be repaired by reset.
		}

		return false;
	}

	float gyro_range;
	float gyro_scale;

	switch (range) {
	case 0: {
			gyro_range = 125.f;
			gyro_scale = 1.f / 160.f;
			break;
		}

	case 1: {
			gyro_range = _model.range_450_dps ? 450.f : 500.f;
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

	_accel.set_range(_model.accel_range);
	_accel.set_scale(_model.accel_scale);
	_gyro.set_range(math::radians(gyro_range));
	_gyro.set_scale(math::radians(gyro_scale) / (native_wide ? kWideWordFactor : 1.f));

	if (!native_wide) {
		_accel_delay_us = kAccelBaseDelayUs;
		_gyro_delay_us = (1510 + 1510 + 1290) / 3; // Original mean of the three gyro group delays, us.
		return true;
	}

	const uint16_t control = kPointOfPercussion | kLinearGCompensation | kBurst32Enable | kDataReadyActiveHigh;

	if (!writeRetried(Register::Control, control, kControlUpdateUs)
	    || !writeRetried(Register::Decimation, _decimation, kDecimationUpdateUs)) {
		return false;
	}

	// FILT_CTRL is flash-backed. Explicitly program zero as well: reset may have
	// restored a nonzero filter, which would invalidate the latency compensation.
	if (!writeRetried(Register::Filter, _filter, kFilterUpdateUs)) {
		return false;
	}

	if (_filter > 0) {
		// Preserve actual discarded bursts, but schedule them individually outside this function.
		_flush_remaining = (1u << _filter) + 4;
	}

	if (_model.format == Format::kBurst32) {
		// The latency specification uses tap count N = 2^B, not the encoded FILT_CTRL value B.
		// A disabled filter/decimator adds no delay (ADIS16507 Rev. A, Latency and Table 102).
		const unsigned filter_delay     = _filter > 0 ? (1u << _filter) * kSamplePeriodUs : 0;
		const unsigned decimation_delay = _decimation > 0 ? (_decimation + 1) * kSamplePeriodUs / 2 : 0;

		_accel_delay_us = kAccelBaseDelayUs + filter_delay + decimation_delay;
		_gyro_delay_us = kGyroBaseDelayUs + filter_delay + decimation_delay;
	}

	return true;
}

bool AdiBurst::configure16()
{
	const bool adis16477 = _model.variant == Variant::kAdis16477;

	if (adis16477) {
		const uint16_t range = readRegister(Register::Range);

		if (_transfer_failed) {
			return false;
		}

		if ((range & 0x0c) != 0x04) {
			PX4_ERR("ADIS16477 requires the 500 dps (-2) model");
			fault(RecoveryReason::kConfiguration);
			return false;
		}

		// Rev. E, 16-bit burst: disable linear-g compensation; retain point-of-percussion and DRDY.
		if (!writeVerified(Register::Control, kPointOfPercussion | kDataReadyActiveHigh, kControlUpdateUs)
		    || !writeVerified(Register::Filter, 4, 100)     // Original 16-tap Bartlett FIR and readback delay.
		    || !writeVerified(Register::Decimation, 1, 100)) { // Original 1000 samples/s and readback delay.
			return false;
		}

	} else {
		// Startup-only host validation bound, not an ADIS16470 worst-case timing specification.
		constexpr uint32_t kReadbackTimeout  { 1_ms };
		constexpr uint32_t kReadbackInterval { 50_us };
		constexpr uint16_t kSamplingMask     { 0x003f }; // SYNC mode/polarity, DR polarity and must-zero bit 5.

		const uint16_t control = readRegister(Register::Control);

		if (_transfer_failed) {
			return false;
		}

		// Always select internal sampling and active-low DRDY, without modifying
		// calibration controls or unused upper bits. Never depend on saved SYNC mode.
		if (!writeRegister(Register::Control, control & ~kSamplingMask)) {
			return false;
		}

		// tSTALL covers bus framing, not register-update latency. Keep the bounded
		// settling window that was verified on ADIS16470, without repeated writes.
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

		// The 2 kHz polling/latency policy requires no saved filter or decimation.
		if (!writeVerified(Register::Decimation, 0, kDecimationUpdateUs)
		    || !writeVerified(Register::Filter, 0, kFilterUpdateUs)) {
			return false;
		}
	}

	_accel.set_range(_model.accel_range);
	_accel.set_scale(_model.accel_scale);
	_gyro.set_range(math::radians(adis16477 ? 500.f : 2000.f));
	_gyro.set_scale(math::radians(adis16477 ? 0.025f : 0.1f));

	return true;
}

bool AdiBurst::configurePagedRange()
{
	const uint16_t range = readRegister(Register::PagedRange);
	float gyro_range;
	float gyro_scale;

	if (_transfer_failed) {
		return false;
	}

	switch (range) {
	case 3: {
			gyro_range = 125.f;
			gyro_scale = 0.00625f;
			break;
		}

	case 7: {
			gyro_range = 450.f;
			gyro_scale = 0.025f;
			break;
		}

	case 15: {
			gyro_range = 2000.f;
			gyro_scale = 0.1f;
			break;
		}

	default: {
			fault(RecoveryReason::kConfiguration);
			return false;
		}
	}

	_accel.set_range(_model.accel_range);
	_accel.set_scale(_model.accel_scale);

	_gyro.set_range(math::radians(gyro_range));
	_gyro.set_scale(math::radians(gyro_scale / kWideWordFactor));

	return !_transfer_failed;
}

bool AdiBurst::configureCrc32()
{
	// Retain the original ADIS16497 settings: internal clock, active-high
	// DRDY, factory calibration, no FIR filtering or continuous bias update.
	// Register update delays follow Rev. D Table 3; do not write flash.
	// Columns: register, required value, settling time in microseconds before readback.
	if (!writeVerified(Register::PagedControl, 0x000d, 340)  // Functional I/O configuration.
	    || !writeVerified(Register::PagedConfig, 0x00c0, 45) // Clock/correction settings.
	    || !writeVerified(Register::PagedDecimation, 3, 340) // 4250 / (3 + 1) samples/s.
	    || !writeVerified(Register::PagedNull, 0, 71)) {     // No continuous bias estimation.
		return false;
	}

	// Each FIR bank needs its own settling interval before the next access.
	return writeVerified(Register::PagedFilter0, 0, 65)
	       && writeVerified(Register::PagedFilter1, 0, 65);
}

bool AdiBurst::configure()
{
	// Keep unselected initialization policies out of single-model images.
	// Acquisition still uses one implementation; no hot-path indirection is added.
#if defined(CONFIG_ADI_BURST_ADIS16470) \
	|| defined(CONFIG_ADI_BURST_ADIS16477)

	if (classic()) {
		return configure16();
	}

#endif // CONFIG_ADI_BURST_ADIS16470 || CONFIG_ADI_BURST_ADIS16477

#if defined(CONFIG_ADI_BURST_ADIS16497)

	if (paged()) {
		return configureCrc32();
	}

#endif // CONFIG_ADI_BURST_ADIS16497

#if defined(CONFIG_ADI_BURST_ADIS16500) \
 || defined(CONFIG_ADI_BURST_ADIS16501) \
 || defined(CONFIG_ADI_BURST_ADIS16505) \
 || defined(CONFIG_ADI_BURST_ADIS16507) \
 || defined(CONFIG_ADI_BURST_ADIS16507_16) \
 || defined(CONFIG_ADI_BURST_ADIS16575) \
 || defined(CONFIG_ADI_BURST_ADIS16576) \
 || defined(CONFIG_ADI_BURST_ADIS16577)

	return configureWide();

#else

	return false;

#endif // Compiled ADIS165xx profiles
}

bool AdiBurst::checkConfiguration()
{
	// Only ADIS16470 had a periodic register check; only DR polarity belongs to that check.
	px4_udelay(_model.stall_us);

	if ((readRegister(Register::Control) & kDataReadyActiveHigh) != 0 || _transfer_failed) {
		_bad_register_perf.count();
		return false;
	}

	_last_config_check = hrt_absolute_time();
	return true;
}

void AdiBurst::startReading()
{
	// RANG_MDL leaves ADIS16497 on page 3. BURST_CMD belongs to page 0;
	// qualify that page before either startup or recovery enables acquisition.
	if (paged()) {
		if (!selectPage(0)) {
			recover(RecoveryReason::kTransfer);
			return;
		}

		const uint16_t page = readRegister(Register::Page);

		if (_transfer_failed) {
			recover(RecoveryReason::kTransfer);
			return;
		}

		if (page != 0) {
			_page = UINT8_MAX;
			_bad_register_perf.count();
			recover(RecoveryReason::kConfiguration);
			return;
		}
	}

	_progress.reset(hrt_absolute_time());
	_ready_samples = 0;
	_irq_streak    = 0;
	_healthy_since = 0;
	_drdy_timestamp.store(0);
	_state = State::kValidate;

	const bool rising = probeConfigured() || wide();
	const bool event = rising;

	_interrupt_enabled = _drdy_gpio != 0
			     && px4_arch_gpiosetevent(_drdy_gpio, rising, !rising, event, dataReadyCallback, this) == 0;

	_fallback_polling = !_interrupt_enabled;
	ScheduleDelayed(_fallback_polling ? _interval_us : _irq_timeout_us);
}

int AdiBurst::dataReadyCallback(int irq, void *context, void *arg)
{
	auto *driver = static_cast<AdiBurst *>(arg);

	// DR already marks an output update after DEC_RATE. Do not decimate it again in software.
	driver->_drdy_timestamp.store(hrt_absolute_time());
	driver->ScheduleNow();

	return 0;
}

AdiBurst::ReadResult AdiBurst::measure(hrt_abstime timestamp)
{
	// One aligned buffer serves both controller widths. Native words are decoded in place.
	alignas(4) uint16_t words[adi_burst_decoder::kSizeCrc32 / sizeof(uint16_t)] {};
	auto *buffer = reinterpret_cast<uint8_t *>(words);
	const uint8_t command = paged() ? 0x7c : kBurstCommand;

	if (wordTransfers()) {
		words[0] = static_cast<uint16_t>(command) << 8;

	} else {
		buffer[0] = command;
	}

	set_frequency(_data_frequency);

	adi_burst_decoder::Sample sample {};
	const size_t size = adi_burst_decoder::size(_model.format);

	const bool transferred = wordTransfers() ? transfer(words, words, size / sizeof(uint16_t))
				 : transfer(buffer, buffer, size);

	if (!transferred) {
		return ReadResult::kTransferError;
	}

	uint16_t diagnostic = 0;
	bool valid;
#if defined(CONFIG_ADI_BURST_ADIS16497)

	if (paged()) {
		valid = adi_burst_decoder::decodeCrc32<kNativeWordOrder>(buffer, size, sample, &diagnostic);

	} else
#endif // CONFIG_ADI_BURST_ADIS16497
	{
		valid = wordTransfers() ? adi_burst_decoder::decode<kNativeWordOrder>(buffer, size, _model.format, sample, &diagnostic)
			: adi_burst_decoder::decode(buffer, size, _model.format, sample, &diagnostic);
	}

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

	// Apply the model's temperature conversion to both channels of the validated burst.
	const float temperature = paged() ? sample.temperature * 0.0125f + 25.f : sample.temperature * kTemperatureScale;

	_accel.set_temperature(temperature);
	_gyro.set_temperature(temperature);

	const uint64_t errors = _bad_transfer_perf.eventCount() + _diagnostic_perf.eventCount()
				+ _stale_perf.eventCount() + _bad_register_perf.eventCount();

	_accel.set_error_count(errors);
	_gyro.set_error_count(errors);

	// Preserve each original driver's coordinate and timestamp conventions.
	// ADIS16470 saturates the int16 negation; ADIS16507 16-bit flips in float.
	if (_model.variant == Variant::kAdis16470) {
		for (size_t axis = 1; axis < 3; ++axis) {
			sample.accel[axis] = imu::negateSaturated(static_cast<int16_t>(sample.accel[axis]));
			sample.gyro[axis]  = imu::negateSaturated(static_cast<int16_t>(sample.gyro[axis]));
		}
	}

	const float yz_sign = _model.variant == Variant::kAdis165xx
			      && _model.format  == Format::kBurst16 ? -1.f : 1.f;

	const auto publish_accel = [&]() {
		_accel.update(timestamp - _accel_delay_us,
			      static_cast<float>(sample.accel[0]),
			      static_cast<float>(sample.accel[1]) * yz_sign,
			      static_cast<float>(sample.accel[2]) * yz_sign);
	};
	const auto publish_gyro = [&]() {
		_gyro.update(timestamp - _gyro_delay_us,
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

void AdiBurst::RunImpl()
{
	_transfer_failed = false;
	const hrt_abstime now = hrt_absolute_time();

	// The steady-state path remains first; validation shares acquisition without a second parser.
	if (__builtin_expect(_state == State::kRead, 1) || _state == State::kValidate) {
		if (_healthy_since != 0 && _progress.expired(now, _irq_timeout_us)) {
			_healthy_since = 0; // A long gap followed by one good frame is not sustained recovery.
		}

		const hrt_abstime irq       = _drdy_timestamp.fetch_and(0);
		const bool       fresh_irq = _interrupt_enabled && irq != 0 && now - irq < _interval_us;
		const hrt_abstime timestamp = fresh_irq ? irq : now;

		if (fresh_irq) {
			if (_irq_streak < kIrqRecoverySamples) {
				++_irq_streak;
			}

			if (_irq_streak == kIrqRecoverySamples) {
				_fallback_polling = false;
			}

		} else {
			if (_interrupt_enabled && !_fallback_polling) {
				_drdy_missed_perf.count();
			}

			_irq_streak       = 0;
			_fallback_polling = true;
		}

		// Missing IRQs alone never reset a sensor that continues producing healthy samples.
		ScheduleDelayed(_fallback_polling ? _interval_us : _irq_timeout_us);
		_read_perf.begin();

		const ReadResult result = measure(timestamp);

		_read_perf.end();

		const hrt_abstime read_completed = hrt_absolute_time();

		if (result == ReadResult::kPublished) {
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

				if (_state == State::kFault || _state == State::kReset) {
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
						      : RecoveryReason::kNoData;
			recover(reason);

			return;
		}

		if (_model.variant == Variant::kAdis16470
		    && hrt_elapsed_time(&_last_config_check) > 100_ms
		    && !checkConfiguration()) {
			recover(RecoveryReason::kConfiguration);
		}

		return;
	}

	switch (_state) {
	case State::kReset: {
			_reset_perf.count();
			_page = UINT8_MAX;

			// Preserve the compatibility endpoint's software-reset-before-GPIO sequence.
			if (_model.variant == Variant::kAdis165xx && !wide()) {
				const bool sent = writeRegister(commandRegister(), kSoftwareReset);

				if (!hardwareReset() && !sent) {
					recover(RecoveryReason::kTransfer);
					break;
				}

			} else if (!hardwareReset() && !writeRegister(commandRegister(), kSoftwareReset)) {
				recover(RecoveryReason::kTransfer);
				break;
			}

			_page = UINT8_MAX;
			_reset_timestamp = now;
			_state = State::kWaitReset;
			ScheduleDelayed(_model.reset_wait_us);
			break;
		}

	case State::kWaitReset: {
			const uint16_t product = readRegister(productRegister());

			if (product != _model.product_id || _transfer_failed) {
				if (now - _reset_timestamp >= _model.reset_wait_us + 200_ms) {
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

			const uint16_t diagnostic = readRegister(diagnosticRegister());

			if (_transfer_failed) {
				recover(RecoveryReason::kTransfer);
				break;
			}

			if (diagnostic != 0) {
				recordDiagnostic(diagnostic);
				handleDiagnostic();

				if (_state == State::kFault || _state == State::kReset) {
					break;
				}
			}

			_state = State::kConfigure;
			ScheduleNow();
			break;
		}

	case State::kStartSelfTest: {
			++_self_test_attempts;
			constexpr uint16_t kPagedSelfTest { 1u << 1 };

			if (!writeRegister(commandRegister(), paged() ? kPagedSelfTest : kSensorSelfTest)) {
				selfTestFailed(RecoveryReason::kTransfer);
				break;
			}

			// The command wait and SPI-readiness bound are separate. DIAG_STAT is read only once.
			_self_test_deadline = now + _model.self_test_wait_us + 200_ms;
			_state = State::kSelfTest;
			ScheduleDelayed(_model.self_test_wait_us);
			break;
		}

	case State::kSelfTest: {
			if (readRegister(productRegister()) != _model.product_id || _transfer_failed) {
				if (now >= _self_test_deadline) {
					selfTestFailed(RecoveryReason::kIdentity);

				} else {
					ScheduleDelayed(1_ms);
				}

				break;
			}

			const uint16_t diagnostic = readRegister(diagnosticRegister());

			if (_transfer_failed || diagnostic != 0) {
				recordDiagnostic(diagnostic);
				selfTestFailed(_transfer_failed ? RecoveryReason::kTransfer : RecoveryReason::kSelfTest);
				break;
			}

			_self_test_passed = true;
			reset(); // Classic startup requires a second reset; do not invalidate the completed self-test.
			break;
		}

	case State::kConfigure: {
			if (!configure() || (paged() && !configurePagedRange())) {
				if (_state != State::kFault) {
					_bad_register_perf.count();
					recover(RecoveryReason::kConfiguration);
				}

				break;
			}

			_last_config_check = now;

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
			uint8_t discard[adi_burst_decoder::kSizeTimestamp32] {};
			discard[0] = kBurstCommand;
			set_frequency(_data_frequency);

			if (!transfer(discard, discard, adi_burst_decoder::size(_model.format))) {
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
			break; // Kept visible to status/stop; only an explicit restart clears a hard fault.
		}
	}
}

void AdiBurst::print_status()
{
	I2CSPIDriverBase::print_status();

	PX4_INFO("state: %s, recovery: %s, level: %u, sampling: %s",
		 stateName(_state), reasonName(_recovery_reason), unsigned(_recovery_attempt),
		 _fallback_polling ? "poll fallback" : "DRDY");
	PX4_INFO("diagnostic: last 0x%04x, history 0x%04x, new-sample timeout %u us",
		 unsigned(_last_diagnostic), unsigned(_diagnostic_history), unsigned(_sample_timeout_us));

	PX4_INFO("type %s, %u-bit, poll %.3f Hz, decimation %d, filter %d",
		 _model.device.name,
		 _model.format == Format::kBurst16 ? 16u : 32u,
		 1e6 / _interval_us,
		 _decimation,
		 _filter);

	imu::printSpiStatus(_model.device,
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
