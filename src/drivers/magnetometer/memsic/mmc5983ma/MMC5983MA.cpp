/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#include "MMC5983MA.hpp"

using namespace time_literals;

MMC5983MA::MMC5983MA(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_px4_mag(get_device_id(), config.rotation)
{
}

MMC5983MA::~MMC5983MA()
{
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_reset_perf);
	perf_free(_self_test_failed_perf);
}

int MMC5983MA::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool MMC5983MA::Reset()
{
	RegisterWrite(Register::CONTROL1, CONTROL1_BIT::SOFTRESET);
	_state = STATE::RESET;
	ScheduleClear();
	ScheduleDelayed(15_ms); // 10 ms for soft restart
	return true;
}

void MMC5983MA::print_status()
{
	I2CSPIDriverBase::print_status();

	perf_print_counter(_reset_perf);
	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_self_test_failed_perf);
}

int MMC5983MA::probe()
{
	// 3 retries
	for (int i = 0; i < 3; i++) {
		const uint8_t STATUS = RegisterRead(Register::STATUS);
		const uint8_t CHIP_ID = RegisterRead(Register::CHIP_ID);

		PX4_DEBUG("STATUS: 0x%02hhX, CHIP_ID: 0x%02hhX", STATUS, CHIP_ID);

		if (CHIP_ID == chip_identification_number) {
			return PX4_OK;
		}
	}

	return PX4_ERROR;
}

static constexpr float convert_xyz(uint32_t mag_data)
{
	const float scale = 0.0625; // 0.0625 milliGauss per LSB
	return ((float) mag_data - 131071.5f) * scale;
}

static constexpr uint32_t combine_x_int18(const uint8_t msb, const uint8_t lsb, const uint8_t lsb10)
{
	// msb: 8-bit MSB part [17:10] of the 18 bit output data
	// lsb: 8-bit LSB part [9:2] of the 18 bit output data
	// lsb10: 2-bit LSB part [1:0] of the 18 bit output data
	uint32_t msb_data = ((uint32_t) msb) << 10;
	uint32_t lsb_data = ((uint32_t) lsb) << 2;
	uint32_t lsb10_data = (((uint32_t) lsb10) >> 6) & 0x3;

	return (uint32_t)(msb_data | lsb_data | lsb10_data);
}

static constexpr uint32_t combine_y_int18(const uint8_t msb, const uint8_t lsb, const uint8_t lsb10)
{
	// msb: 8-bit MSB part [17:10] of the 18 bit output data
	// lsb: 8-bit LSB part [9:2] of the 18 bit output data
	// lsb10: 2-bit LSB part [1:0] of the 18 bit output data
	uint32_t msb_data = ((uint32_t) msb) << 10;
	uint32_t lsb_data = ((uint32_t) lsb) << 2;
	uint32_t lsb10_data = (((uint32_t) lsb10) >> 4) & 0x3;

	return (uint32_t)(msb_data | lsb_data | lsb10_data);
}

static constexpr uint32_t combine_z_int18(const uint8_t msb, const uint8_t lsb, const uint8_t lsb10)
{
	// msb: 8-bit MSB part [17:10] of the 18 bit output data
	// lsb: 8-bit LSB part [9:2] of the 18 bit output data
	// lsb10: 2-bit LSB part [1:0] of the 18 bit output data
	uint32_t msb_data = ((uint32_t) msb) << 10;
	uint32_t lsb_data = ((uint32_t) lsb) << 2;
	uint32_t lsb10_data = (((uint32_t) lsb10) >> 2) & 0x3;

	return (uint32_t)(msb_data | lsb_data | lsb10_data);
}

void MMC5983MA::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET:
		// CONTROL1: soft reset
		RegisterWrite(Register::CONTROL1, CONTROL1_BIT::SOFTRESET);
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;
		perf_count(_reset_perf);
		ScheduleDelayed(15_ms);
		break;

	case STATE::WAIT_FOR_RESET:
		if ((RegisterRead(Register::CHIP_ID) == chip_identification_number)) {
			// if reset succeeded then start self test

			// clear all control registers
			RegisterWrite(Register::CONTROL0, 0);
			RegisterWrite(Register::CONTROL1, 0);
			RegisterWrite(Register::CONTROL2, 0);
			RegisterWrite(Register::CONTROL3, 0);

			// enable set current, read once
			RegisterSetAndClearBits(Register::CONTROL0, CONTROL0_BIT::SET | CONTROL0_BIT::MEASURE_FIELD,
						CONTROL0_BIT::RESET | CONTROL0_BIT::MEASURE_TEMP);
			_state = STATE::SELF_TEST_RESET;

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

	case STATE::SELF_TEST_RESET: {
			struct TransferBuffer {
				uint8_t DATAX_MSB;
				uint8_t DATAX_LSB;
				uint8_t DATAY_MSB;
				uint8_t DATAY_LSB;
				uint8_t DATAZ_MSB;
				uint8_t DATAZ_LSB;
			} buffer{};
			// 0x00 to 0x05 with a burst read.
			uint8_t cmd = static_cast<uint8_t>(Register::DATAX_MSB);

			if (transfer(&cmd, 1, (uint8_t *)&buffer, sizeof(buffer)) == PX4_OK) {
				_set_data[0] = ((uint16_t) buffer.DATAX_MSB << 8) | buffer.DATAX_LSB;
				_set_data[1] = ((uint16_t) buffer.DATAY_MSB << 8) | buffer.DATAY_LSB;
				_set_data[2] = ((uint16_t) buffer.DATAZ_MSB << 8) | buffer.DATAZ_LSB;

				// now reset (invert) current
				RegisterSetAndClearBits(Register::CONTROL0, CONTROL0_BIT::RESET | CONTROL0_BIT::MEASURE_FIELD,
							CONTROL0_BIT::SET | CONTROL0_BIT::MEASURE_TEMP);
				_state = STATE::SELF_TEST_CHECK;
				ScheduleDelayed(10_ms);

			} else {
				// full reset
				_state = STATE::RESET;
				ScheduleDelayed(10_ms);
			}
		}

		break;

	case STATE::SELF_TEST_CHECK: {
			struct TransferBuffer {
				uint8_t DATAX_MSB;
				uint8_t DATAX_LSB;
				uint8_t DATAY_MSB;
				uint8_t DATAY_LSB;
				uint8_t DATAZ_MSB;
				uint8_t DATAZ_LSB;
			} buffer{};
			// 0x00 to 0x05 with a burst read.
			uint8_t cmd = static_cast<uint8_t>(Register::DATAX_MSB);

			if (transfer(&cmd, 1, (uint8_t *)&buffer, sizeof(buffer)) == PX4_OK) {
				_reset_data[0] = ((uint16_t) buffer.DATAX_MSB << 8) | buffer.DATAX_LSB;
				_reset_data[1] = ((uint16_t) buffer.DATAY_MSB << 8) | buffer.DATAY_LSB;
				_reset_data[2] = ((uint16_t) buffer.DATAZ_MSB << 8) | buffer.DATAZ_LSB;

				bool success = true;

				for (int i = 0; i < 3; i++) {
					int16_t delta = _set_data[i] - _reset_data[i];

					if ((delta <= 100) && (delta >= -100)) {
						PX4_DEBUG("SET[%d]: %d RESET[%d]: %d", _set_data[i], i, _reset_data[i], i);
						success = false;
						break;
					}
				}

				if (success) {
					_state = STATE::CONFIGURE;
					ScheduleDelayed(10_ms);

				} else {
					PX4_DEBUG("self test failed, resetting");
					perf_count(_self_test_failed_perf);
					_state = STATE::RESET;
					ScheduleDelayed(1_s);
				}

			} else {
				// full reset
				_state = STATE::RESET;
				ScheduleDelayed(10_ms);
			}
		}

		break;

	case STATE::CONFIGURE:
		if (Configure()) {
			// if configure succeeded then start reading every 50 ms (20 Hz)
			_state = STATE::READ;
			ScheduleOnInterval(50_ms, 50_ms);

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
				uint8_t DATAX_MSB;
				uint8_t DATAX_LSB;
				uint8_t DATAY_MSB;
				uint8_t DATAY_LSB;
				uint8_t DATAZ_MSB;
				uint8_t DATAZ_LSB;
				uint8_t DATAXYZ_LSB;
				uint8_t TEMPERATURE;
				uint8_t STATUS;
			} buffer{};

			bool success = false;
			// 0x00 to 0x08 with a burst read.
			uint8_t cmd = static_cast<uint8_t>(Register::DATAX_MSB);

			if (transfer(&cmd, 1, (uint8_t *)&buffer, sizeof(buffer)) == PX4_OK) {
				int32_t x18 = combine_x_int18(buffer.DATAX_MSB, buffer.DATAX_LSB, buffer.DATAXYZ_LSB);
				int32_t y18 = combine_y_int18(buffer.DATAY_MSB, buffer.DATAY_LSB, buffer.DATAXYZ_LSB);
				int32_t z18 = combine_z_int18(buffer.DATAZ_MSB, buffer.DATAZ_LSB, buffer.DATAXYZ_LSB);

				float x = convert_xyz(x18);
				float y = convert_xyz(y18);
				float z = convert_xyz(z18);

				_px4_mag.set_error_count(perf_event_count(_bad_register_perf)
							 + perf_event_count(_bad_transfer_perf) + perf_event_count(_self_test_failed_perf));
				_px4_mag.update(now, x, y, z);

				if (_failure_count > 0) {
					_failure_count--;
				}

				success = true;

			} else {
				perf_count(_bad_transfer_perf);
				success = true;
			}

			if (!success) {
				_failure_count++;

				// full reset if things are failing consistently
				if (_failure_count > 10) {
					Reset();
					return;
				}
			}
		}

		break;
	}
}

bool MMC5983MA::Configure()
{
	// first set and clear all configured register bits
	for (const auto &reg_cfg : _register_cfg) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
	}

	// milliGauss -> Gauss
	_px4_mag.set_scale(0.001f);

	return true;
}

uint8_t MMC5983MA::RegisterRead(Register reg)
{
	const uint8_t cmd = static_cast<uint8_t>(reg);
	uint8_t buffer{};
	int ret = transfer(&cmd, 1, &buffer, 1);

	if (ret != PX4_OK) {
		PX4_DEBUG("register read 0x%02hhX failed, ret = %d", cmd, ret);
		return -1;
	}

	return buffer;
}

void MMC5983MA::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t buffer[2] { (uint8_t)reg, value };
	int ret = transfer(buffer, sizeof(buffer), nullptr, 0);

	if (ret != PX4_OK) {
		PX4_DEBUG("register write 0x%02hhX failed, ret = %d", (uint8_t)reg, ret);
	}
}

void MMC5983MA::RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = 0; // chip does not support reading control registers
	uint8_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}
