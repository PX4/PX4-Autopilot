/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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

#include "MPC2520.hpp"

using namespace time_literals;

static constexpr int32_t combine(uint8_t h, uint8_t m, uint8_t l)
{
	// 24 bit sign extend
	int32_t ret = (uint32_t)(h << 24) | (uint32_t)(m << 16) | (uint32_t)(l << 8);
	return ret >> 8;
}

MPC2520::MPC2520(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_px4_baro(get_device_id())
{
	_px4_baro.set_external(external());
}

MPC2520::~MPC2520()
{
	perf_free(_reset_perf);
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
}

int MPC2520::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool MPC2520::Reset()
{
	_state = STATE::RESET;
	ScheduleClear();
	ScheduleNow();
	return true;
}

void MPC2520::print_status()
{
	I2CSPIDriverBase::print_status();

	perf_print_counter(_reset_perf);
	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
}

int MPC2520::probe()
{
	const uint8_t ID = RegisterRead(Register::ID);

	uint8_t PROD_ID = ID & 0xF0 >> 4; // Product ID Bits 7:4

	if (PROD_ID != Product_ID) {
		DEVICE_DEBUG("unexpected PROD_ID 0x%02x", PROD_ID);
		return PX4_ERROR;
	}

	return PX4_OK;
}

void MPC2520::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET:
		// RESET: SOFT_RST
		RegisterWrite(Register::RESET, RESET_BIT::SOFT_RST);
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;
		perf_count(_reset_perf);
		ScheduleDelayed(50_ms); // Power On Reset: max 50ms
		break;

	case STATE::WAIT_FOR_RESET: {
			// check MEAS_CFG SENSOR_RDY
			const uint8_t ID = RegisterRead(Register::ID);

			uint8_t PROD_ID = ID & 0xF0 >> 4; // Product ID Bits 7:4

			if (PROD_ID == Product_ID) {
				// if reset succeeded then read prom
				_state = STATE::READ_PROM;
				ScheduleDelayed(40_ms); // Time to coefficients are available.

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
		}

		break;

	case STATE::READ_PROM: {
			uint8_t	prom_buf[3] {};
			uint8_t cmd = 0x10;
			transfer(&cmd, 1, &prom_buf[0], 2);
			_prom.c0 = (int16_t)prom_buf[0] << 4 | prom_buf[1] >> 4;
			_prom.c0 = (_prom.c0 & 0x0800) ? (0xF000 | _prom.c0) : _prom.c0;

			cmd = 0x11;
			transfer(&cmd, 1, &prom_buf[0], 2);
			_prom.c1 = (int16_t)(prom_buf[0] & 0x0F) << 8 | prom_buf[1];
			_prom.c1 = (_prom.c1 & 0x0800) ? (0xF000 | _prom.c1) : _prom.c1;

			cmd = 0x13;
			transfer(&cmd, 1, &prom_buf[0], 3);
			_prom.c00 = (int32_t)prom_buf[0] << 12 | (int32_t)prom_buf[1] << 4 | (int32_t)prom_buf[2] >> 4;
			_prom.c00 = (_prom.c00 & 0x080000) ? (0xFFF00000 | _prom.c00) : _prom.c00;

			cmd = 0x15;
			transfer(&cmd, 1, &prom_buf[0], 3);
			_prom.c10 = (int32_t)prom_buf[0] << 16 | (int32_t)prom_buf[1] << 8 | prom_buf[2];
			_prom.c10 = (_prom.c10 & 0x080000) ? (0xFFF00000 | _prom.c10) : _prom.c10;

			cmd = 0x18;
			transfer(&cmd, 1, &prom_buf[0], 2);
			_prom.c01 = (int16_t)prom_buf[0] << 8 | prom_buf[1];

			cmd = 0x1A;
			transfer(&cmd, 1, &prom_buf[0], 2);
			_prom.c11 = (int16_t)prom_buf[0] << 8 | prom_buf[1];

			cmd = 0x1C;
			transfer(&cmd, 1, &prom_buf[0], 2);
			_prom.c20 = (int16_t)prom_buf[0] << 8 | prom_buf[1];

			cmd = 0x1E;
			transfer(&cmd, 1, &prom_buf[0], 2);
			_prom.c21 = (int16_t)prom_buf[0] << 8 | prom_buf[1];

			cmd = 0x20;
			transfer(&cmd, 1, &prom_buf[0], 2);
			_prom.c30 = (int16_t)prom_buf[0] << 8 | prom_buf[1];

			_state = STATE::CONFIGURE;
			ScheduleDelayed(10_ms);
		}
		break;

	case STATE::CONFIGURE:
		if (Configure()) {
			// if configure succeeded then start measurement cycle
			_state = STATE::READ;
			ScheduleOnInterval(1000000 / 32); // 32 Hz

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
			bool success = false;

			// read temperature first
			struct TransferBuffer {
				uint8_t PSR_B2;
				uint8_t PSR_B1;
				uint8_t PSR_B0;
				uint8_t TMP_B2;
				uint8_t TMP_B1;
				uint8_t TMP_B0;
			} buffer{};

			uint8_t cmd = static_cast<uint8_t>(Register::PSR_B2);

			if (transfer(&cmd, 1, (uint8_t *)&buffer, sizeof(buffer)) == PX4_OK) {
				int32_t Traw = (int32_t)(buffer.TMP_B2 << 16) | (int32_t)(buffer.TMP_B1 << 8) | (int32_t)buffer.TMP_B0;
				Traw = (Traw & 0x800000) ? (0xFF000000 | Traw) : Traw;

				static constexpr float kT = 7864320; // temperature 8 times oversampling
				float Traw_sc = static_cast<float>(Traw) / kT;
				float Tcomp = _prom.c0 * 0.5f + _prom.c1 * Traw_sc;
				_px4_baro.set_temperature(Tcomp);

				int32_t Praw = (int32_t)(buffer.PSR_B2 << 16) | (int32_t)(buffer.PSR_B1 << 8) | (int32_t)buffer.PSR_B1;
				Praw = (Praw & 0x800000) ? (0xFF000000 | Praw) : Praw;

				static constexpr float kP = 7864320; // pressure 8 times oversampling
				float Praw_sc = static_cast<float>(Praw) / kP;

				// Calculate compensated measurement results.
				float Pcomp = _prom.c00 + Praw_sc * (_prom.c10 + Praw_sc * (_prom.c20 + Praw_sc * _prom.c30)) + Traw_sc * _prom.c01 +
					      Traw_sc * Praw_sc * (_prom.c11 + Praw_sc * _prom.c21);

				float pressure_mbar = Pcomp / 100.0f; // convert to millibar

				_px4_baro.update(now, pressure_mbar);

				success = true;

				if (_failure_count > 0) {
					_failure_count--;
				}

			} else {
				perf_count(_bad_transfer_perf);
			}

			if (!success) {
				_failure_count++;

				// full reset if things are failing consistently
				if (_failure_count > 10) {
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
					return;
				}
			}
		}

		break;
	}
}

bool MPC2520::Configure()
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

	return success;
}

bool MPC2520::RegisterCheck(const register_config_t &reg_cfg)
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

uint8_t MPC2520::RegisterRead(Register reg)
{
	const uint8_t cmd = static_cast<uint8_t>(reg);
	uint8_t buffer{};
	transfer(&cmd, 1, &buffer, 1);
	return buffer;
}

void MPC2520::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t buffer[2] { (uint8_t)reg, value };
	transfer(buffer, sizeof(buffer), nullptr, 0);
}

void MPC2520::RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);
	uint8_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}
