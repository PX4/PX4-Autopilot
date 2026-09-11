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

#include "ICM56686.hpp"

#include <cstdlib>

using namespace time_literals;

static constexpr int16_t combine(const uint8_t high, const uint8_t low)
{
	return static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | low);
}

static constexpr uint16_t combineUnsigned(const uint8_t high, const uint8_t low)
{
	return (static_cast<uint16_t>(high) << 8) | low;
}

ICM56686::ICM56686(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_drdy_gpio(config.drdy_gpio),
	_fifo_interrupt_config0(config.custom1 == 2 ? Register::BANK_0::INT2_CONFIG0 : Register::BANK_0::INT1_CONFIG0),
	_fifo_interrupt_config2(config.custom1 == 2 ? Register::BANK_0::INT2_CONFIG2 : Register::BANK_0::INT1_CONFIG2),
	_px4_accel(get_device_id(), config.rotation),
	_px4_gyro(get_device_id(), config.rotation)
{
	if (_fifo_interrupt_config0 == Register::BANK_0::INT2_CONFIG0) {
		for (auto &reg_cfg : _register_cfg) {
			if (reg_cfg.reg == Register::BANK_0::INT1_CONFIG0) {
				reg_cfg.reg = Register::BANK_0::INT2_CONFIG0;

			} else if (reg_cfg.reg == Register::BANK_0::INT1_CONFIG2) {
				reg_cfg.reg = Register::BANK_0::INT2_CONFIG2;
			}
		}
	}

	if (_drdy_gpio != 0) {
		_drdy_missed_perf = perf_alloc(PC_COUNT, MODULE_NAME": DRDY missed");

	} else {
		for (auto &reg_cfg : _register_cfg) {
			if (reg_cfg.reg == _fifo_interrupt_config0) {
				reg_cfg.value = 0;
			}
		}
	}

	ConfigureSampleRate(_px4_gyro.get_max_rate_hz());
}

ICM56686::~ICM56686()
{
	DataReadyInterruptDisable();
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_fifo_empty_perf);
	perf_free(_fifo_overflow_perf);
	perf_free(_fifo_reset_perf);
	perf_free(_drdy_missed_perf);
}

int ICM56686::init()
{
	const int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? PX4_OK : PX4_ERROR;
}

bool ICM56686::Reset()
{
	_state = STATE::RESET;
	ScheduleClear();
	ScheduleNow();
	return true;
}

void ICM56686::exit_and_cleanup()
{
	DataReadyInterruptDisable();
	I2CSPIDriverBase::exit_and_cleanup();
}

void ICM56686::print_status()
{
	I2CSPIDriverBase::print_status();

	PX4_INFO("FIFO empty interval: %" PRIu32 " us (%.1f Hz)", _fifo_empty_interval_us,
		 1e6 / _fifo_empty_interval_us);
	PX4_INFO("DRDY interrupt: %s", _data_ready_interrupt_enabled ? "enabled" : "disabled");

	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_fifo_empty_perf);
	perf_print_counter(_fifo_overflow_perf);
	perf_print_counter(_fifo_reset_perf);
	perf_print_counter(_drdy_missed_perf);
}

int ICM56686::probe()
{
	for (int i = 0; i < 3; i++) {
		uint8_t whoami{0};

		if (RegisterRead(Register::BANK_0::WHO_AM_I, whoami) && whoami == WHOAMI) {
			return PX4_OK;
		}

		DEVICE_DEBUG("unexpected WHO_AM_I 0x%02x", whoami);
	}

	return PX4_ERROR;
}

void ICM56686::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET:
		DataReadyInterruptDisable();
		_data_ready_interrupt_enabled = false;

		// Soft reset restores INTF_CONFIG1_OVRD / DRIVE_CONFIG0 to OTP defaults (SPI
		// 4-wire / slew). Save them so WAIT_FOR_RESET can put the board's pad config back
		// before the next WHO_AM_I read.
		if (!RegisterRead(Register::BANK_0::INTF_CONFIG1_OVRD, _intf_config1_ovrd)
		    || !RegisterRead(Register::BANK_0::DRIVE_CONFIG0, _drive_config0)) {
			ScheduleDelayed(10_ms);
			break;
		}

		// INT1_STATUS0 only latches the reset done status while its source is enabled in
		// INT1_CONFIG0 (reset value 0x80). Configure() disables all INT1 sources, so the source
		// is re-enabled here to keep WAIT_FOR_RESET working on a runtime re-reset, independent
		// of whether the soft reset restores the register defaults.
		if (!RegisterWrite(Register::BANK_0::INT1_CONFIG0, INT_CONFIG0_BIT::INT1_STATUS_EN_RESET_DONE)
		    || !RegisterWrite(Register::BANK_0::REG_MISC2, REG_MISC2_BIT::SOFT_RST)) {
			ScheduleDelayed(10_ms);
			break;
		}

		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;
		ScheduleDelayed(10_ms);
		break;

	case STATE::WAIT_FOR_RESET: {
			uint8_t whoami{0};
			uint8_t status{0};

			if (!RegisterWrite(Register::BANK_0::DRIVE_CONFIG0, _drive_config0)
			    || !RegisterWrite(Register::BANK_0::INTF_CONFIG1_OVRD, _intf_config1_ovrd)) {
				ScheduleDelayed(10_ms);
				break;
			}

			// INT1_STATUS_RESET_DONE is cleared on read, so it is only read once WHO_AM_I matches.
			if (RegisterRead(Register::BANK_0::WHO_AM_I, whoami)
			    && whoami == WHOAMI
			    && RegisterRead(Register::BANK_0::INT1_STATUS0, status)
			    && (status & INT1_STATUS0_BIT::INT1_STATUS_RESET_DONE)) {
				_state = STATE::CONFIGURE;
				ScheduleNow();

			} else if (hrt_elapsed_time(&_reset_timestamp) > 100_ms) {
				PX4_DEBUG("Reset failed, retrying");
				_state = STATE::RESET;
				ScheduleDelayed(100_ms);

			} else {
				ScheduleDelayed(10_ms);
			}
		}
		break;

	case STATE::CONFIGURE:
		if (Configure()) {
			_state = STATE::FIFO_RESET;
			// 35 ms gyro start-up time, 10 ms accel from sleep to valid data
			ScheduleDelayed(50_ms);

		} else if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
			PX4_DEBUG("Configure failed, resetting");
			_state = STATE::RESET;
			ScheduleDelayed(100_ms);

		} else {
			PX4_DEBUG("Configure failed, retrying");
			ScheduleDelayed(100_ms);
		}

		break;

	case STATE::FIFO_RESET:
		if (!FIFOReset()) {
			_state = STATE::RESET;
			ScheduleDelayed(100_ms);
			break;
		}

		_state = STATE::FIFO_READ;

		if (DataReadyInterruptConfigure()) {
			_data_ready_interrupt_enabled = true;
			ScheduleDelayed(100_ms);

		} else {
			_data_ready_interrupt_enabled = false;
			ScheduleOnInterval(_fifo_empty_interval_us, _fifo_empty_interval_us);
		}

		break;

	case STATE::FIFO_READ: {
			hrt_abstime timestamp_sample = now;

			if (_data_ready_interrupt_enabled) {
				const hrt_abstime drdy_timestamp_sample = _drdy_timestamp_sample.fetch_and(0);

				if ((now - drdy_timestamp_sample) < (_fifo_empty_interval_us * 2)) {
					timestamp_sample = drdy_timestamp_sample;

				} else {
					perf_count(_drdy_missed_perf);
				}

				ScheduleDelayed(_fifo_empty_interval_us * 2);
			}

			if (FIFORead(timestamp_sample)) {
				if (_failure_count > 0) {
					_failure_count--;
				}

			} else if (++_failure_count > 10) {
				Reset();
				return;
			}

			if (hrt_elapsed_time(&_last_config_check_timestamp) > 100_ms) {
				if (RegisterCheck(_register_cfg[_checked_register])) {
					_last_config_check_timestamp = now;
					_checked_register = (_checked_register + 1) % REGISTER_CFG_COUNT;

				} else {
					perf_count(_bad_register_perf);
					Reset();
				}
			}
		}
		break;
	}
}

void ICM56686::ConfigureSampleRate(const int sample_rate)
{
	const float minimum_interval = FIFO_SAMPLE_DT;
	_fifo_empty_interval_us = math::max(roundf((1e6f / sample_rate) / minimum_interval) * minimum_interval,
					    minimum_interval);
	_fifo_samples = roundf(math::min(_fifo_empty_interval_us / FIFO_SAMPLE_DT,
					 static_cast<float>(FIFO_MAX_SAMPLES)));
	_fifo_empty_interval_us = roundf(_fifo_samples * FIFO_SAMPLE_DT);
	ConfigureFIFOWatermark(static_cast<uint8_t>(_fifo_samples));
}

void ICM56686::ConfigureFIFOWatermark(const uint8_t samples)
{
	for (auto &config : _register_cfg) {
		if (config.reg == Register::BANK_0::FIFO_CONFIG1_0) {
			config.value = samples;

		} else if (config.reg == Register::BANK_0::FIFO_CONFIG1_1) {
			config.value = 0;
		}
	}
}

bool ICM56686::Configure()
{
	bool success = true;

	// Registers without dynamic-change support must be configured with both sensors off.
	success &= RegisterWrite(Register::BANK_0::INT1_CONFIG0, 0);

	if (_fifo_interrupt_config0 != Register::BANK_0::INT1_CONFIG0) {
		success &= RegisterWrite(_fifo_interrupt_config0, 0);
	}

	success &= RegisterWrite(Register::BANK_0::PWR_MGMT0, 0);
	success &= RegisterWrite(Register::BANK_0::PWR_MGMT_AUX1, 0);

	success &= FIFOBypass();

	success &= RegisterConfigure(Register::BANK_0::ACCEL_CONFIG0);
	success &= RegisterConfigure(Register::BANK_0::GYRO_CONFIG0);
	success &= RegisterConfigure(Register::BANK_0::FIFO_CONFIG4);

	// FIFO_HIRES_EN, FIFO_GYRO_EN and FIFO_ACCEL_EN do not support dynamic change, so they are
	// configured here while both sensors are off. ConfigureFIFO() only touches the fields that
	// may be changed at runtime (FIFO_MODE, FIFO_WM_TH, FIFO_FLUSH, FIFO_IF_EN).
	success &= RegisterWrite(Register::BANK_0::FIFO_CONFIG3,
				 FIFO_CONFIG3_BIT::FIFO_HIRES_EN | FIFO_CONFIG3_BIT::FIFO_GYRO_EN |
				 FIFO_CONFIG3_BIT::FIFO_ACCEL_EN);

	// Select 20-bit sensor/FIFO data in little-endian format.
	success &= IregSetAndClearBits(IREG::SREG_CTRL, SREG_CTRL_BIT::SREG_SIFS_20BITS_EN,
				       SREG_CTRL_BIT::SREG_DATA_ENDIAN_BIG);
	success &= IregSetAndClearBits(IREG::SMC_CONTROL_0, SMC_CONTROL_0_BIT::TMST_EN, 0);

	// ODRs at or above 4 kHz require an 8 us interrupt pulse.
	success &= IregSetAndClearBits(IREG::INT_PULSE_MIN_ON_INTF0,
				       INTF0_PULSE_BIT::DURATION_8_US,
				       INTF0_PULSE_BIT::DURATION_MASK);
	success &= IregSetAndClearBits(IREG::INT_PULSE_MIN_OFF_INTF0,
				       INTF0_PULSE_BIT::DURATION_8_US,
				       INTF0_PULSE_BIT::DURATION_MASK);

	// Run both UI paths with the sample rate converter and the pre-filter enabled, and with the
	// UI low pass filter bypassed (ODR/2) to get the full bandwidth. The datasheet gives no
	// recommendation for the SRC fields (reset default is off), so they are kept explicit here and
	// have to be re-checked against the InvenSense reference driver if the ODR is changed.
	// NOTE: IREG contents are not covered by the periodic register check in RunImpl().
	success &= IregSetAndClearBits(IREG::GYRO_SRC_CTRL,
				       GYRO_SRC_CTRL_BIT::GYRO_SRC_CTRL_SRC_ON_PREFILTER_ON,
				       GYRO_SRC_CTRL_BIT::GYRO_SRC_CTRL_MASK);
	success &= IregSetAndClearBits(IREG::GYRO_UI_LPF_CFG,
				       GYRO_UI_LPF_CFG_BIT::GYRO_UI_LPFBW_ODR_DIV_2,
				       GYRO_UI_LPF_CFG_BIT::GYRO_UI_LPFBW_MASK);
	success &= IregSetAndClearBits(IREG::ACCEL_SRC_CTRL,
				       ACCEL_SRC_CTRL_BIT::ACCEL_SRC_CTRL_SRC_ON_PREFILTER_ON,
				       ACCEL_SRC_CTRL_BIT::ACCEL_SRC_CTRL_MASK);
	success &= IregSetAndClearBits(IREG::ACCEL_UI_LPF_CFG,
				       ACCEL_UI_LPF_CFG_BIT::ACCEL_UI_LPFBW_ODR_DIV_2,
				       ACCEL_UI_LPF_CFG_BIT::ACCEL_UI_LPFBW_MASK);

	success &= RegisterConfigure(Register::BANK_0::PWR_MGMT0);

	_px4_accel.set_range(32.f * CONSTANTS_ONE_G);
	_px4_gyro.set_range(math::radians(4000.f));

	return success;
}

bool ICM56686::FIFOBypass()
{
	// Disable the SREG-FIFO interface before the FIFO itself, then switch to bypass mode.
	bool success = RegisterSetAndClearBits(Register::BANK_0::FIFO_CONFIG3, 0, FIFO_CONFIG3_BIT::FIFO_IF_EN);
	success &= RegisterSetAndClearBits(Register::BANK_0::FIFO_CONFIG0, FIFO_CONFIG0_BIT::FIFO_MODE_BYPASS,
					   FIFO_CONFIG0_BIT::FIFO_MODE_MASK);

	// FIFO_DEPTH may only be written while the FIFO is disabled (bypass mode).
	success &= RegisterSetAndClearBits(Register::BANK_0::FIFO_CONFIG0, FIFO_CONFIG0_BIT::FIFO_DEPTH_2K,
					   static_cast<uint8_t>(FIFO_CONFIG0_BIT::FIFO_DEPTH_MASK & ~FIFO_CONFIG0_BIT::FIFO_DEPTH_2K));

	return success;
}

bool ICM56686::ConfigureFIFO()
{
	bool success = FIFOBypass();

	// The watermark threshold latches on the FIFO_CONFIG1_1 write pulse. Write the LSByte first
	// and force both writes, because skipping an unchanged MSByte would leave the threshold at
	// its disabled reset value.
	success &= RegisterForceWrite(Register::BANK_0::FIFO_CONFIG1_0);
	success &= RegisterForceWrite(Register::BANK_0::FIFO_CONFIG1_1);

	// FIFO_INT_OVFL and FIFO_WR_WM_GT_TH may only be written while the FIFO is in bypass mode.
	success &= RegisterConfigure(Register::BANK_0::FIFO_CONFIG2);

	success &= FIFOFlush();

	// Enable the FIFO first, the SREG-FIFO interface (FIFO_IF_EN) last.
	success &= RegisterConfigure(Register::BANK_0::FIFO_CONFIG0);
	success &= RegisterConfigure(Register::BANK_0::FIFO_CONFIG3);
	success &= RegisterConfigure(_fifo_interrupt_config2);
	success &= RegisterConfigure(_fifo_interrupt_config0);

	for (const auto &config : _register_cfg) {
		success &= RegisterCheck(config);
	}

	return success;
}

bool ICM56686::FIFOFlush()
{
	uint8_t fifo_config2{0};

	if (!RegisterRead(Register::BANK_0::FIFO_CONFIG2, fifo_config2)
	    || !RegisterWrite(Register::BANK_0::FIFO_CONFIG2, fifo_config2 | FIFO_CONFIG2_BIT::FIFO_FLUSH)) {
		return false;
	}

	px4_udelay(10);

	for (int i = 0; i < 100; i++) {
		if (!RegisterRead(Register::BANK_0::FIFO_CONFIG2, fifo_config2)) {
			return false;
		}

		if ((fifo_config2 & FIFO_CONFIG2_BIT::FIFO_FLUSH) == 0) {
			return true;
		}

		px4_udelay(10);
	}

	return false;
}

bool ICM56686::RegisterCheck(const register_config_t &reg_cfg)
{
	uint8_t value{0};

	if (!RegisterRead(reg_cfg.reg, value)) {
		return false;
	}

	return (value & reg_cfg.mask) == (reg_cfg.value & reg_cfg.mask);
}

bool ICM56686::RegisterConfigure(const register_config_t &reg_cfg)
{
	return RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.value & reg_cfg.mask,
				       static_cast<uint8_t>(~reg_cfg.value) & reg_cfg.mask);
}

bool ICM56686::RegisterConfigure(const Register::BANK_0 reg)
{
	for (const auto &config : _register_cfg) {
		if (config.reg == reg) {
			return RegisterConfigure(config);
		}
	}

	return false;
}

bool ICM56686::RegisterForceWrite(const Register::BANK_0 reg)
{
	for (const auto &config : _register_cfg) {
		if (config.reg == reg) {
			return RegisterWrite(config.reg, config.value);
		}
	}

	return false;
}

bool ICM56686::RegisterRead(const Register::BANK_0 reg, uint8_t &value)
{
	uint8_t cmd[2] {static_cast<uint8_t>(static_cast<uint8_t>(reg) | DIR_READ), 0};

	if (transfer(cmd, cmd, sizeof(cmd)) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return false;
	}

	value = cmd[1];
	return true;
}

bool ICM56686::RegisterWrite(const Register::BANK_0 reg, const uint8_t value)
{
	uint8_t cmd[2] {static_cast<uint8_t>(reg), value};

	if (transfer(cmd, cmd, sizeof(cmd)) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return false;
	}

	return true;
}

bool ICM56686::RegisterSetAndClearBits(const Register::BANK_0 reg, const uint8_t set_bits,
				       const uint8_t clear_bits)
{
	uint8_t value{0};

	if (!RegisterRead(reg, value)) {
		return false;
	}

	const uint8_t new_value = (value & ~clear_bits) | set_bits;
	return value == new_value || RegisterWrite(reg, new_value);
}

bool ICM56686::WaitForIregReady()
{
	// IREG_DONE reads 1 at idle. Honour the datasheet 4 us gap first or a poll can
	// observe a stale "done" from the previous access.
	px4_udelay(4);

	for (int i = 0; i < 20; i++) {
		uint8_t status{0};

		if (!RegisterRead(Register::BANK_0::REG_MISC2, status)) {
			return false;
		}

		if (status & REG_MISC2_BIT::IREG_DONE) {
			return true;
		}

		px4_udelay(10);
	}

	return false;
}

bool ICM56686::IregRead(const uint16_t reg, uint8_t &value)
{
	if (!WaitForIregReady()) {
		return false;
	}

	uint8_t address[3] {
		static_cast<uint8_t>(Register::BANK_0::IREG_ADDR_15_8),
		static_cast<uint8_t>(reg >> 8),
		static_cast<uint8_t>(reg & 0xFF)
	};

	if (transfer(address, address, sizeof(address)) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return false;
	}

	px4_udelay(10);

	if (!WaitForIregReady() || !RegisterRead(Register::BANK_0::IREG_DATA, value)) {
		return false;
	}

	// Reading IREG_DATA starts a prefetch of the next indirect address.
	px4_udelay(10);
	return WaitForIregReady();
}

bool ICM56686::IregWrite(const uint16_t reg, const uint8_t value)
{
	if (!WaitForIregReady()) {
		return false;
	}

	uint8_t cmd[4] {
		static_cast<uint8_t>(Register::BANK_0::IREG_ADDR_15_8),
		static_cast<uint8_t>(reg >> 8),
		static_cast<uint8_t>(reg & 0xFF),
		value
	};

	if (transfer(cmd, cmd, sizeof(cmd)) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return false;
	}

	px4_udelay(10);
	return WaitForIregReady();
}

bool ICM56686::IregSetAndClearBits(const uint16_t reg, const uint8_t set_bits, const uint8_t clear_bits)
{
	uint8_t value{0};

	if (!IregRead(reg, value)) {
		return false;
	}

	const uint8_t new_value = (value & ~clear_bits) | set_bits;

	if (value != new_value && !IregWrite(reg, new_value)) {
		return false;
	}

	uint8_t check{0};
	return IregRead(reg, check) && check == new_value;
}

uint16_t ICM56686::FIFOReadCount()
{
	uint8_t fifo_count[3] {
		static_cast<uint8_t>(static_cast<uint8_t>(Register::BANK_0::FIFO_COUNT_0) | DIR_READ), 0, 0
	};

	if (transfer(fifo_count, fifo_count, sizeof(fifo_count)) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return 0;
	}

	// FIFO_COUNT_0 is documented as FIFO_DATA_CNT[15:8] and FIFO_COUNT_1 as [7:0], but the
	// SREG_CTRL little-endian selection applies to the count registers as well, so the burst
	// returns the LSByte first (same behaviour as the ICM-45686). Verified on hardware: the
	// resulting frame count tracks the configured ODR (6344 Hz measured against 6400 Hz nominal,
	// within the +-1.25% internal clock tolerance) and never trips the overflow path.
	return combineUnsigned(fifo_count[2], fifo_count[1]);
}

bool ICM56686::FIFORead(const hrt_abstime &timestamp_sample)
{
	const uint16_t fifo_frames = FIFOReadCount();

	if (fifo_frames == 0) {
		perf_count(_fifo_empty_perf);
		return false;
	}

	if (fifo_frames >= FIFO::MAX_FRAMES) {
		perf_count(_fifo_overflow_perf);

		if (!FIFOReset()) {
			PX4_DEBUG("FIFO reset failed");
		}

		return false;
	}

	const uint8_t frames = math::min(static_cast<uint16_t>(FIFO_MAX_SAMPLES), fifo_frames);
	FIFOTransferBuffer buffer{};
	const size_t transfer_size = 1 + frames * sizeof(FIFO::DATA);

	if (transfer(reinterpret_cast<uint8_t *>(&buffer), reinterpret_cast<uint8_t *>(&buffer), transfer_size) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return false;
	}

	// Only the leading frames with a valid header are used. An invalid header means the FIFO stream
	// is out of sync, the remaining frames of this transfer are dropped and the next transfer will
	// start on a bad header again, which resets the FIFO below.
	uint8_t samples = 0;

	while ((samples < frames) && ((buffer.samples[samples].header & FIFO::HEADER_MASK) == FIFO::HEADER)) {
		samples++;
	}

	if (samples < frames) {
		perf_count(_bad_transfer_perf);
	}

	if (samples == 0) {
		if (!FIFOReset()) {
			PX4_DEBUG("FIFO reset failed");
		}

		return false;
	}

	if (!ProcessTemperature(buffer.samples, samples)) {
		return false;
	}

	ProcessGyro(timestamp_sample, buffer.samples, samples);
	ProcessAccel(timestamp_sample, buffer.samples, samples);
	return true;
}

bool ICM56686::FIFOReset()
{
	perf_count(_fifo_reset_perf);
	_drdy_timestamp_sample.store(0);
	return ConfigureFIFO();
}

void ICM56686::ProcessAccel(const hrt_abstime &timestamp_sample, const FIFO::DATA fifo[], const uint8_t samples)
{
	// In high resolution mode the accel data is 19 bit at the configured UI filter bandwidth of
	// ODR/2 (the LSB is always zero), so dropping the LSB is lossless and doubles the headroom
	// before the batch has to be scaled down: 2^18 / 32 g = 8192 LSB/g.
	static constexpr float ACCEL_SCALE{CONSTANTS_ONE_G / 8192.f};

	// A rotation of up to 45 degrees can make a component sqrt(2) longer, so the raw samples have
	// to stay well inside the int16 range to avoid false clipping, plus a bit of margin.
	static constexpr int32_t SAFE_RAW_LIMIT{static_cast<int32_t>(INT16_MAX * M_SQRT1_2_F) - 100};

	// Shift applied to the whole batch if any sample exceeds the limit, 2^18 >> 4 still fits.
	static constexpr uint8_t SCALE_SHIFT{4};

	uint8_t shift{0};

	for (int i = 0; i < samples; i++) {
		if ((std::abs(FIFO::accelX(fifo[i]) / 2) > SAFE_RAW_LIMIT)
		    || (std::abs(FIFO::accelY(fifo[i]) / 2) > SAFE_RAW_LIMIT)
		    || (std::abs(FIFO::accelZ(fifo[i]) / 2) > SAFE_RAW_LIMIT)) {
			shift = SCALE_SHIFT;
			break;
		}
	}

	const int32_t divider = 2 * (1 << shift);

	sensor_accel_fifo_s accel{};
	accel.timestamp_sample = timestamp_sample;
	accel.dt = FIFO_SAMPLE_DT;

	for (int i = 0; i < samples; i++) {
		const int32_t x = FIFO::accelX(fifo[i]);
		const int32_t y = FIFO::accelY(fifo[i]);
		const int32_t z = FIFO::accelZ(fifo[i]);

		if ((x != FIFO::INVALID_SAMPLE) && (y != FIFO::INVALID_SAMPLE) && (z != FIFO::INVALID_SAMPLE)) {
			// the sensor frame is +x forward, +y left, +z up, flip y & z for the FRD board frame
			accel.x[accel.samples] = x / divider;
			accel.y[accel.samples] = -(y / divider);
			accel.z[accel.samples] = -(z / divider);
			accel.samples++;
		}
	}

	_px4_accel.set_scale(ACCEL_SCALE * (1 << shift));
	_px4_accel.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf) +
				   perf_event_count(_fifo_empty_perf) + perf_event_count(_fifo_overflow_perf));

	if (accel.samples > 0) {
		_px4_accel.updateFIFO(accel);
	}
}

void ICM56686::ProcessGyro(const hrt_abstime &timestamp_sample, const FIFO::DATA fifo[], const uint8_t samples)
{
	// In high resolution mode the gyro data is 20 bit: 2^19 / 4000 dps = 131.1 LSB/dps.
	static constexpr float GYRO_SCALE{4000.f / 524288.f};

	// A rotation of up to 45 degrees can make a component sqrt(2) longer, so the raw samples have
	// to stay well inside the int16 range to avoid false clipping, plus a bit of margin.
	static constexpr int32_t SAFE_RAW_LIMIT{static_cast<int32_t>(INT16_MAX * M_SQRT1_2_F) - 100};

	// Shift applied to the whole batch if any sample exceeds the limit. 2^19 >> 5 still fits, a
	// shift of 4 would overflow the int16 samples at full scale.
	static constexpr uint8_t SCALE_SHIFT{5};

	uint8_t shift{0};

	for (int i = 0; i < samples; i++) {
		if ((std::abs(FIFO::gyroX(fifo[i])) > SAFE_RAW_LIMIT)
		    || (std::abs(FIFO::gyroY(fifo[i])) > SAFE_RAW_LIMIT)
		    || (std::abs(FIFO::gyroZ(fifo[i])) > SAFE_RAW_LIMIT)) {
			shift = SCALE_SHIFT;
			break;
		}
	}

	const int32_t divider = 1 << shift;

	sensor_gyro_fifo_s gyro{};
	gyro.timestamp_sample = timestamp_sample;
	gyro.dt = FIFO_SAMPLE_DT;

	for (int i = 0; i < samples; i++) {
		const int32_t x = FIFO::gyroX(fifo[i]);
		const int32_t y = FIFO::gyroY(fifo[i]);
		const int32_t z = FIFO::gyroZ(fifo[i]);

		if ((x != FIFO::INVALID_SAMPLE) && (y != FIFO::INVALID_SAMPLE) && (z != FIFO::INVALID_SAMPLE)) {
			// the sensor frame is +x forward, +y left, +z up, flip y & z for the FRD board frame
			gyro.x[gyro.samples] = x / divider;
			gyro.y[gyro.samples] = -(y / divider);
			gyro.z[gyro.samples] = -(z / divider);
			gyro.samples++;
		}
	}

	_px4_gyro.set_scale(math::radians(GYRO_SCALE) * (1 << shift));
	_px4_gyro.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf) +
				  perf_event_count(_fifo_empty_perf) + perf_event_count(_fifo_overflow_perf));

	if (gyro.samples > 0) {
		_px4_gyro.updateFIFO(gyro);
	}
}

bool ICM56686::ProcessTemperature(const FIFO::DATA fifo[], const uint8_t samples)
{
	int16_t temperature_samples[FIFO_MAX_SAMPLES] {};
	float temperature_sum{0};
	int valid_samples{0};

	for (int i = 0; i < samples; i++) {
		const int16_t temperature = combine(fifo[i].temperature_h, fifo[i].temperature_l);

		if (temperature != FIFO::INVALID_TEMPERATURE) {
			temperature_sum += temperature;
			temperature_samples[valid_samples] = temperature;
			valid_samples++;
		}
	}

	if (valid_samples == 0) {
		perf_count(_bad_transfer_perf);
		return false;
	}

	const float temperature_average = temperature_sum / valid_samples;

	for (int i = 0; i < valid_samples; i++) {
		// A temperature jump within one FIFO transfer is an indication of a transfer error.
		if (fabsf(temperature_samples[i] - temperature_average) > 1000.f) {
			perf_count(_bad_transfer_perf);
			return false;
		}
	}

	const float temperature = (temperature_average / TEMPERATURE_SENSITIVITY) + TEMPERATURE_OFFSET;

	if (!PX4_ISFINITE(temperature)) {
		perf_count(_bad_transfer_perf);
		return false;
	}

	_px4_accel.set_temperature(temperature);
	_px4_gyro.set_temperature(temperature);
	return true;
}

int ICM56686::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<ICM56686 *>(arg)->DataReady();
	return 0;
}

void ICM56686::DataReady()
{
	_drdy_timestamp_sample.store(hrt_absolute_time());
	ScheduleNow();
}

bool ICM56686::DataReadyInterruptConfigure()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	return px4_arch_gpiosetevent(_drdy_gpio, true, false, true, &DataReadyInterruptCallback, this) == 0;
}

bool ICM56686::DataReadyInterruptDisable()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	return px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0;
}
