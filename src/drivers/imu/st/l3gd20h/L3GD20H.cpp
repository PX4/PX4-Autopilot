/****************************************************************************
 *
 *   Copyright (c) 2012-2022 PX4 Development Team. All rights reserved.
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

#include "L3GD20H.hpp"

using namespace time_literals;

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

L3GD20H::L3GD20H(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_drdy_gpio(config.drdy_gpio),
	_px4_gyro(get_device_id(), config.rotation)
{
	if (_drdy_gpio != 0) {
		_drdy_missed_perf = perf_alloc(PC_COUNT, MODULE_NAME": DRDY missed");
	}

	ConfigureSampleRate(ST_L3GD20H::ODR); // default to max ODR
}

L3GD20H::~L3GD20H()
{
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_fifo_empty_perf);
	perf_free(_fifo_overflow_perf);
	perf_free(_fifo_reset_perf);
	perf_free(_drdy_missed_perf);
}

int L3GD20H::init()
{
	int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool L3GD20H::Reset()
{
	_state = STATE::RESET;
	DataReadyInterruptDisable();
	ScheduleClear();
	ScheduleNow();
	return true;
}

void L3GD20H::exit_and_cleanup()
{
	DataReadyInterruptDisable();
	I2CSPIDriverBase::exit_and_cleanup();
}

void L3GD20H::print_status()
{
	I2CSPIDriverBase::print_status();

	PX4_INFO("FIFO empty interval: %d us (%.1f Hz)", _fifo_empty_interval_us, 1e6 / _fifo_empty_interval_us);

	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_fifo_empty_perf);
	perf_print_counter(_fifo_overflow_perf);
	perf_print_counter(_fifo_reset_perf);
	perf_print_counter(_drdy_missed_perf);
}

int L3GD20H::probe()
{
	const uint8_t whoami = RegisterRead(Register::WHO_AM_I);

	if (whoami != WHOAMI) {
		DEVICE_DEBUG("unexpected WHO_AM_I 0x%02x", whoami);
		return PX4_ERROR;
	}

	return PX4_OK;
}

void L3GD20H::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET:
		// LOW_ODR SW_RES
		RegisterWrite(Register::LOW_ODR, LOW_ODR_BIT::SW_RES);
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;
		ScheduleDelayed(150_ms);
		break;

	case STATE::WAIT_FOR_RESET:
		if ((RegisterRead(Register::WHO_AM_I) == WHOAMI)
		    && ((RegisterRead(Register::LOW_ODR) & LOW_ODR_BIT::SW_RES) == 0)) {

			// if reset succeeded then configure
			RegisterSetBits(Register::CTRL1, CTRL1_BIT::PD);
			_state = STATE::CONFIGURE;
			ScheduleDelayed(50_ms); // Turn-on time 50 ms

		} else {
			// RESET not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				PX4_DEBUG("Reset failed, retrying");
				_state = STATE::RESET;
				ScheduleDelayed(100_ms);

			} else {
				PX4_DEBUG("Reset not complete, check again in 10 ms");
				ScheduleDelayed(10_ms);
			}
		}

		break;

	case STATE::CONFIGURE:
		if (Configure()) {
			// if configure succeeded then start reading from FIFO
			_state = STATE::FIFO_READ;

			if (DataReadyInterruptConfigure()) {
				_data_ready_interrupt_enabled = true;

				// default to max ODR with data ready available
				ConfigureSampleRate(ST_L3GD20H::ODR);

				// backup schedule as a watchdog timeout
				ScheduleDelayed(100_ms);

			} else {
				_data_ready_interrupt_enabled = false;

				// default to half of max ODR if no data ready available
				ConfigureSampleRate(ST_L3GD20H::ODR / 2);

				ScheduleOnInterval(_fifo_empty_interval_us, _fifo_empty_interval_us);
			}

			FIFOReset();

		} else {
			// CONFIGURE not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				PX4_DEBUG("Configure failed, resetting");
				_state = STATE::RESET;

			} else {
				PX4_DEBUG("Configure failed, retrying");
			}

			ScheduleDelayed(100_ms);
		}

		break;

	case STATE::FIFO_READ: {
			hrt_abstime timestamp_sample = now;

			if (_data_ready_interrupt_enabled) {
				// scheduled from interrupt if _drdy_timestamp_sample was set as expected
				const hrt_abstime drdy_timestamp_sample = _drdy_timestamp_sample.fetch_and(0);

				if ((now - drdy_timestamp_sample) < _fifo_empty_interval_us) {
					timestamp_sample = drdy_timestamp_sample;

				} else {
					perf_count(_drdy_missed_perf);
				}

				// push backup schedule back
				ScheduleDelayed(_fifo_empty_interval_us * 2);
			}

			bool success = false;

			// always check FIFO status and count
			const uint8_t FIFO_SRC = RegisterRead(Register::FIFO_SRC);
			const uint8_t samples = FIFO_SRC & FIFO_SRC_BIT::FSS; // FSS4-FSS0 FIFO stored data level

			if ((FIFO_SRC & FIFO_SRC_BIT::OVRN) || (samples > FIFO_MAX_SAMPLES)) {
				// not necessarily an overflow, but more samples than we expected or can publish
				perf_count(_fifo_overflow_perf);
				FIFOReset();

			} else if ((FIFO_SRC & FIFO_SRC_BIT::EMPTY) || (samples == 0)) {
				perf_count(_fifo_empty_perf);

			} else {
				if (FIFORead(timestamp_sample, samples)) {
					success = true;

					if (_failure_count > 0) {
						_failure_count--;
					}
				}
			}

			if (!success) {
				_failure_count++;

				// full reset if things are failing consistently
				if (_failure_count > 100) {
					Reset();
					return;
				}
			}

			if (!success || hrt_elapsed_time(&_last_config_check_timestamp) > 100_ms) {
				// check configuration registers periodically or immediately following any failure
				if (RegisterCheck(_register_cfg[_checked_register])) {
					_last_config_check_timestamp = now;
					_checked_register = (_checked_register + 1) % size_register_cfg;

				} else {
					// register check failed, force reset
					perf_count(_bad_register_perf);
					Reset();
				}

			} else {
				// periodically update temperature (~1 Hz)
				if (now >= _temperature_update_timestamp + 1_s) {
					UpdateTemperature();
					_temperature_update_timestamp = now;
				}
			}
		}

		break;
	}
}

void L3GD20H::ConfigureSampleRate(int sample_rate)
{
	if (sample_rate == 0) {
		sample_rate = ST_L3GD20H::ODR; // default to max ODR
	}

	// round down to nearest FIFO sample dt
	const float min_interval = FIFO_SAMPLE_DT;
	_fifo_empty_interval_us = math::max(roundf((1e6f / (float)sample_rate) / min_interval) * min_interval, min_interval);

	_fifo_samples = roundf(math::min((float)_fifo_empty_interval_us / (1e6f / GYRO_RATE), (float)FIFO_MAX_SAMPLES));

	// recompute FIFO empty interval (us) with actual gyro sample limit
	_fifo_empty_interval_us = _fifo_samples * (1e6f / GYRO_RATE);
}

bool L3GD20H::Configure()
{
	// first set and clear all configured register bits
	for (const auto &reg_cfg : _register_cfg) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
	}

	// now check that all are configured
	bool success = true;

	for (const auto &reg_cfg : _register_cfg) {
		if (!RegisterCheck(reg_cfg)) {
			success = false;
		}
	}

	_px4_gyro.set_scale(math::radians(70.f * 1e-3f)); // 70 mdps/LSB (FS = 2000 dps)
	_px4_gyro.set_range(math::radians(2000.f));       // ±2000 dps

	return success;
}

int L3GD20H::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<L3GD20H *>(arg)->DataReady();
	return 0;
}

void L3GD20H::DataReady()
{
	_drdy_timestamp_sample.store(hrt_absolute_time());
	ScheduleNow();
}

bool L3GD20H::DataReadyInterruptConfigure()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	// Setup data ready on falling edge
	return px4_arch_gpiosetevent(_drdy_gpio, false, true, false, &DataReadyInterruptCallback, this) == 0;
}

bool L3GD20H::DataReadyInterruptDisable()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	return px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0;
}

bool L3GD20H::RegisterCheck(const register_config_t &reg_cfg)
{
	bool success = true;

	const uint8_t reg_value = RegisterRead(reg_cfg.reg);

	if (reg_cfg.set_bits && ((reg_value & reg_cfg.set_bits) != reg_cfg.set_bits)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not set)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.set_bits);
		success = false;
	}

	if (reg_cfg.clear_bits && ((reg_value & reg_cfg.clear_bits) != 0)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not cleared)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.clear_bits);
		success = false;
	}

	return success;
}

uint8_t L3GD20H::RegisterRead(Register reg)
{
	uint8_t cmd[2] {};
	cmd[0] = static_cast<uint8_t>(reg) | DIR_READ;
	transfer(cmd, cmd, sizeof(cmd));
	return cmd[1];
}

void L3GD20H::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t cmd[2] { (uint8_t)reg, value };
	transfer(cmd, cmd, sizeof(cmd));
}

void L3GD20H::RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);

	uint8_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}

bool L3GD20H::FIFORead(const hrt_abstime &timestamp_sample, uint8_t samples)
{
	FIFOTransferBuffer buffer{};
	const size_t transfer_size = math::min(samples * sizeof(FIFO::DATA) + 1, FIFO::SIZE);

	if (transfer((uint8_t *)&buffer, (uint8_t *)&buffer, transfer_size) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return false;
	}

	ProcessGyro(timestamp_sample, buffer.f, samples);

	return true;
}

void L3GD20H::FIFOReset()
{
	perf_count(_fifo_reset_perf);

	// To reset FIFO content Bypass mode should be written in FIFO_CTRL
	RegisterClearBits(Register::FIFO_CTRL, FIFO_CTRL_BIT::Bypass_mode);

	// reset while FIFO is disabled
	_drdy_timestamp_sample.store(0);

	// FIFO_CTRL: mode + watermark
	for (auto &r : _register_cfg) {
		if (r.reg == Register::FIFO_CTRL) {
			RegisterSetAndClearBits(Register::FIFO_CTRL, r.set_bits, r.clear_bits);
		}
	}
}

void L3GD20H::ProcessGyro(const hrt_abstime &timestamp_sample, const FIFO::DATA fifo[], const uint8_t samples)
{
	sensor_gyro_fifo_s gyro{};
	gyro.timestamp_sample = timestamp_sample;
	gyro.samples = samples;
	gyro.dt = FIFO_SAMPLE_DT;

	for (int i = 0; i < samples; i++) {
		int16_t gyro_x = combine(fifo[i].OUT_X_H, fifo[i].OUT_X_L);
		int16_t gyro_y = combine(fifo[i].OUT_Y_H, fifo[i].OUT_Y_L);
		int16_t gyro_z = combine(fifo[i].OUT_Z_H, fifo[i].OUT_Z_L);

		// sensor's frame is +x forward, +y left, +z up
		//  flip y & z to publish right handed with z down (x forward, y right, z down)
		gyro.x[i] = gyro_x;
		gyro.y[i] = math::negate(gyro_y);
		gyro.z[i] = math::negate(gyro_z);
	}

	_px4_gyro.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf) +
				  perf_event_count(_fifo_empty_perf) + perf_event_count(_fifo_overflow_perf));

	_px4_gyro.updateFIFO(gyro);
}

void L3GD20H::UpdateTemperature()
{
	// Temperature data (-1LSB/deg with 8 bit resolution). The value is expressed as two’s complement.
	const int8_t OUT_TEMP = RegisterRead(Register::OUT_TEMP);

	const float temperature_c = (OUT_TEMP / TEMPERATURE_SENSITIVITY) + TEMPERATURE_OFFSET;

	if (PX4_ISFINITE(temperature_c)
	    && (temperature_c >= TEMPERATURE_SENSOR_MIN)
	    && (temperature_c <= TEMPERATURE_SENSOR_MAX)) {

		_px4_gyro.set_temperature(temperature_c);
	}

}
