/****************************************************************************
 *
 *   Copyright (c) 2024-2025 PX4 Development Team. All rights reserved.
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

#include "IIS2MDC.hpp"

using namespace time_literals;

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

IIS2MDC::IIS2MDC(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_px4_mag(get_device_id(), config.rotation)
{
}

IIS2MDC::~IIS2MDC()
{
	perf_free(_reboot_perf);
	perf_free(_reset_perf);
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
}

int IIS2MDC::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		PX4_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	return Reboot() ? 0 : -1;
}

bool IIS2MDC::Reboot()
{
	_state = STATE::REBOOT;
	ScheduleClear();
	ScheduleNow();
	return true;
}

bool IIS2MDC::Reset()
{
	_state = STATE::RESET;
	ScheduleClear();
	ScheduleNow();
	return true;
}

void IIS2MDC::print_status()
{
	I2CSPIDriverBase::print_status();

	perf_print_counter(_reboot_perf);
	perf_print_counter(_reset_perf);
	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
}

int IIS2MDC::probe()
{
	for (int retry = 0; retry < 3; retry++) {

		uint8_t id = RegisterRead(Register::WHO_AM_I);

		if (id == Device_ID) {
			_retries = 2;
			return PX4_OK;

		} else {
			PX4_DEBUG("unexpected WHO_AM_I 0x%02x", id);
		}
	}

	return PX4_ERROR;
}

void IIS2MDC::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::REBOOT:
		// CFG_REG_A: Reboot
		RegisterWrite(Register::CFG_REG_A, CFG_REG_A_BIT::REBOOT);
		perf_count(_reboot_perf);
		_state = STATE::RESET;
		ScheduleDelayed(40_ms); // Wait > 20 ms
		break;

	case STATE::RESET:
		// CFG_REG_A: Software Reset
		RegisterWrite(Register::CFG_REG_A, CFG_REG_A_BIT::SOFT_RST);
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;
		perf_count(_reset_perf);
		ScheduleDelayed(10_ms);
		break;

	case STATE::WAIT_FOR_RESET:

		// SOFT_RST: This bit is automatically reset to zero after reset
		if ((RegisterRead(Register::WHO_AM_I) == Device_ID)
		    && ((RegisterRead(Register::CFG_REG_A) & CFG_REG_A_BIT::SOFT_RST) == 0)) {

			// if reset succeeded then configure
			_state = STATE::CONFIGURE;
			ScheduleDelayed(10_ms);

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
			// if configure succeeded then start read cycle
			_state = STATE::READ;
			ScheduleOnInterval(20_ms); // 50 Hz

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

	case STATE::READ: {
			struct TransferBuffer {
				uint8_t STATUS_REG;
				uint8_t OUTX_L_REG;
				uint8_t OUTX_H_REG;
				uint8_t OUTY_L_REG;
				uint8_t OUTY_H_REG;
				uint8_t OUTZ_L_REG;
				uint8_t OUTZ_H_REG;
				uint8_t TEMP_OUT_L_REG;
				uint8_t TEMP_OUT_H_REG;
			} buffer{};

			bool success = false;
			uint8_t cmd = static_cast<uint8_t>(Register::STATUS_REG);

			if (transfer(&cmd, 1, (uint8_t *)&buffer, sizeof(buffer)) == PX4_OK) {

				// new set of data is available (Zyxda = 1)
				if (buffer.STATUS_REG & STATUS_REG_BIT::Zyxda) {
					int16_t x = combine(buffer.OUTX_H_REG, buffer.OUTX_L_REG);
					int16_t y = combine(buffer.OUTY_H_REG, buffer.OUTY_L_REG);
					int16_t z = combine(buffer.OUTZ_H_REG, buffer.OUTZ_L_REG);

					// sensor's frame is +x forward, +y right, +z up
					z = math::negate(z);

					int16_t t = combine(buffer.TEMP_OUT_H_REG, buffer.TEMP_OUT_L_REG);

					// temperature sensitivity of 8 LSB/°C. Typically, the output zero level corresponds to 25 °C
					const float temperature = (t / 8.f) + 25.f;
					_px4_mag.set_temperature(temperature);

					_px4_mag.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf));
					_px4_mag.update(now, x, y, z);

					success = true;

					if (_failure_count > 0) {
						_failure_count--;
					}
				}

			} else {
				perf_count(_bad_transfer_perf);
			}

			if (!success) {
				_failure_count++;

				// full reboot/reset if things are failing consistently
				if (_failure_count > 10) {
					Reboot();
					return;
				}
			}

			if (!success || hrt_elapsed_time(&_last_config_check_timestamp) > 100_ms) {
				// check configuration registers periodically or immediately following any failure
				if (RegisterCheck(_register_cfg[_checked_register])) {
					_last_config_check_timestamp = now;
					_checked_register = (_checked_register + 1) % size_register_cfg;

				} else {
					// register check failed, force reboot/reset
					perf_count(_bad_register_perf);
					Reboot();
					return;
				}
			}
		}

		break;
	}
}

bool IIS2MDC::Configure()
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

	_px4_mag.set_scale(0.0015f); // M_So = 1.5 mG / LSB

	return success;
}

bool IIS2MDC::RegisterCheck(const register_config_t &reg_cfg)
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

uint8_t IIS2MDC::RegisterRead(Register reg)
{
	const uint8_t cmd = static_cast<uint8_t>(reg);
	uint8_t buffer{};
	transfer(&cmd, 1, &buffer, 1);
	return buffer;
}

void IIS2MDC::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t buffer[2] { (uint8_t)reg, value };
	transfer(buffer, sizeof(buffer), nullptr, 0);
}

void IIS2MDC::RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);
	uint8_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}
