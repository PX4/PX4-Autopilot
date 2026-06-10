/****************************************************************************
 *
 *   Copyright (c) 2019-2026 PX4 Development Team. All rights reserved.
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

#include "AK09940A.hpp"

using namespace time_literals;

static constexpr int32_t combine(uint8_t hh, uint8_t hm,  uint8_t hl)
{
	// Combine into 24-bit word
	int32_t raw = (hh << 16) | (hm << 8) | hl;

	// Overflow marker check (datasheet says 0x1FFFF means overflow)
	if (raw == 0x1FFFF) {
		return raw;
	}

	// Keep only 18 bits
	raw &= 0x3FFFF;

	// Sign-extend from 18 bits
	if (raw & (1 << 17)) {
		raw -= (1 << 18);
	}

	return raw;
}

AK09940A::AK09940A(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_px4_mag(get_device_id(), config.rotation)
{
}

AK09940A::~AK09940A()
{
	perf_free(_reset_perf);
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_magnetic_sensor_overflow_perf);
}

int AK09940A::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool AK09940A::Reset()
{
	_state = STATE::RESET;
	ScheduleClear();
	ScheduleNow();
	return true;
}

void AK09940A::print_status()
{
	I2CSPIDriverBase::print_status();
	PX4_INFO("Variant: %s", device_name());

	perf_print_counter(_reset_perf);
	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_magnetic_sensor_overflow_perf);
}

int AK09940A::probe()
{
	// 3 retries
	for (int i = 0; i < 3; i++) {

		const uint8_t WIA1 = RegisterRead(Register::WIA1);
		const uint8_t WIA2 = RegisterRead(Register::WIA2);

		if ((WIA1 != Company_ID)) {
			PX4_DEBUG("unexpected WIA1 0x%02x", WIA1);
			continue;
		}

		switch (static_cast<AKTYPE>(WIA2)) {
		case AKTYPE::AK09940A:
			_device = AKTYPE::AK09940A;
			return PX4_OK;

		default:
			PX4_DEBUG("unexpected WIA2 0x%02x", WIA2);
		};
	}

	return PX4_ERROR;
}

void AK09940A::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET:
		perf_count(_reset_perf);
		// CNTL4 SRST: Soft reset
		RegisterWrite(Register::CNTL4, static_cast<uint8_t>(CNTL4_BIT::SRST));
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;
		ScheduleDelayed(100_ms);
		break;

	case STATE::WAIT_FOR_RESET: {
			if ((RegisterRead(Register::WIA1) == Company_ID) && (static_cast<AKTYPE>(RegisterRead(Register::WIA2)) == _device)) {
				// if reset succeeded then configure
				_state = STATE::CONFIGURE;
				ScheduleDelayed(100_ms);

			} else {
				// RESET not complete
				if (hrt_elapsed_time(&_reset_timestamp) > 30_s) {
					PX4_ERR("Reset failed, retrying");
					Reset();

				} else {
					PX4_DEBUG("Reset not complete, check again in 100 ms");
					ScheduleDelayed(100_ms);
				}
			}

			break;
		}

	case STATE::CONFIGURE:
		if (Configure()) {
			// if configure succeeded then start reading
			_state = STATE::READ;
			ScheduleOnInterval(20_ms, 20_ms); // 50 Hz

		} else {
			// CONFIGURE not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 30_s) {
				PX4_ERR("Configure failed, resetting");
				Reset();

			} else {
				PX4_DEBUG("Configure failed, retrying");
				ScheduleDelayed(100_ms);
			}
		}

		break;

	case STATE::READ: {

			struct TransferBuffer {
				uint8_t ST1;
				uint8_t HXL;
				uint8_t HXM;
				uint8_t HXH;
				uint8_t HYL;
				uint8_t HYM;
				uint8_t HYH;
				uint8_t HZL;
				uint8_t HZM;
				uint8_t HZH;
				uint8_t TMPS;
				uint8_t ST2;
			} buffer{};

			uint8_t cmd = static_cast<uint8_t>(Register::ST1);
			int ret = transfer(&cmd, 1, (uint8_t *)&buffer, sizeof(TransferBuffer));

			bool success = false;

			if (ret == PX4_OK) {
				if (buffer.ST1 & static_cast<uint8_t>(ST1_BIT::DRDY)) {

					const int32_t x = combine(buffer.HXH, buffer.HXM, buffer.HXL);
					const int32_t y = combine(buffer.HYH, buffer.HYM, buffer.HYL);
					const int32_t z = combine(buffer.HZH, buffer.HZM, buffer.HZL);

					if (buffer.ST2 & static_cast<uint8_t>(ST2_BIT::INV)) {
						// invalid
						perf_count(_bad_transfer_perf);

					} else {
						if (x == 0x1FFFF || y == 0x1FFFF || z == 0x1FFFF) {
							// Magnetic Overflow is present
							perf_count(_magnetic_sensor_overflow_perf);

						} else {

							// sensor's frame is +X forward (X), +Y right (Y), +Z down (Z)
							_px4_mag.update(now, x, y, z);
							_px4_mag.set_temperature((float)(30 - ((int8_t)buffer.TMPS) / 1.7));
						}

						// Submit errors
						_px4_mag.set_error_count(perf_event_count(_bad_register_perf) +
									 perf_event_count(_bad_transfer_perf) +
									 perf_event_count(_magnetic_sensor_overflow_perf));

						success = true;

						if (_failure_count > 0) {
							_failure_count--;
						}
					}

				}
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
				}
			}
		}

		break;
	}
}

bool AK09940A::Configure()
{
	_retries = 2;

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

	// mag resolution is 1.0e-4 Gauss per bit (10 nT/LSB)
	_px4_mag.set_scale(1.0e-4f);

	return success;
}

bool AK09940A::RegisterCheck(const register_config_t &reg_cfg)
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

uint8_t AK09940A::RegisterRead(Register reg)
{
	const uint8_t cmd = static_cast<uint8_t>(reg);
	uint8_t buffer{};
	transfer(&cmd, 1, &buffer, 1);
	return buffer;
}

void AK09940A::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t buffer[2] { (uint8_t)reg, value };
	transfer(buffer, sizeof(buffer), nullptr, 0);
}

void AK09940A::RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);
	uint8_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}
