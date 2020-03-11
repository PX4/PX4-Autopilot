/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include "IST8310.hpp"

using namespace time_literals;

#define IST8310_TEMP_RESOLUTION	(1/67.2)
#define IST8310_TEMP_25C_OFFSET	27100
#define IST8310_OTP_SENSITIVITY 330

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

IST8310::IST8310(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_px4_mag(get_device_id(), config.rotation)
{
}

IST8310::~IST8310()
{
	perf_free(_reset_perf);
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
}

int IST8310::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool IST8310::Reset()
{
	_state = STATE::RESET;
	ScheduleClear();
	ScheduleNow();
	return true;
}

void IST8310::print_status()
{
	I2CSPIDriverBase::print_status();

	perf_print_counter(_reset_perf);
	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
	print_cross_axis_info();
}

void IST8310::print_cross_axis_info()
{
	PX4_INFO("cross axis cal: [%+3.2f,%+3.2f,%+3.2f|%+3.2f,%+3.2f,%+3.2f|%+3.2f,%+3.2f,%+3.2f]",
		 (double)_crossaxis_inv[0],
		 (double)_crossaxis_inv[1],
		 (double)_crossaxis_inv[2],
		 (double)_crossaxis_inv[3],
		 (double)_crossaxis_inv[4],
		 (double)_crossaxis_inv[5],
		 (double)_crossaxis_inv[6],
		 (double)_crossaxis_inv[7],
		 (double)_crossaxis_inv[8]);
}

int IST8310::probe()
{
	uint8_t id = RegisterRead(Register::WAI);

	if (id != Device_ID) {
		DEVICE_DEBUG("unexpected WAI 0x%02x", id);

		// Apparently, the IST8310's WHOAMI register is writeable. Presumably,
		// this can get corrupted by bus noise. It is only reset if powered off
		// for 30s or by a reset.
		RegisterWrite(Register::CNTL2, CNTL2_BIT::SRST);

		auto start_time = hrt_absolute_time();

		while (hrt_elapsed_time(&start_time) < 50_ms) {
			px4_usleep(10'000);
			id = RegisterRead(Register::WAI);

			if (id == Device_ID) {
				return PX4_OK;
			}
		}

		return PX4_ERROR;
	}

	return PX4_OK;
}

void IST8310::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET:
		// CNTL2: Software Reset
		RegisterWrite(Register::CNTL2, CNTL2_BIT::SRST);
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;
		perf_count(_reset_perf);
		ScheduleDelayed(50_ms); // Power On Reset: max 50ms
		break;

	case STATE::WAIT_FOR_RESET:

		// SRST: This bit is automatically reset to zero after POR routine
		if ((RegisterRead(Register::WAI) == Device_ID)
		    && ((RegisterRead(Register::CNTL2) & CNTL2_BIT::SRST) == 0)) {

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
			// if configure succeeded then start measurement cycle
			_state = STATE::MEASURE;
			ScheduleDelayed(20_ms);

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

	case STATE::MEASURE:
		RegisterWrite(Register::CNTL1, CNTL1_BIT::MODE_SINGLE_MEASUREMENT);
		_state = STATE::READ;
		ScheduleDelayed(20_ms); // Wait at least 6ms. (minimum waiting time for 16 times internal average setup)
		break;

	case STATE::READ: {
			struct TransferBuffer {
				uint8_t STAT1;
				uint8_t DATAXL;
				uint8_t DATAXH;
				uint8_t DATAYL;
				uint8_t DATAYH;
				uint8_t DATAZL;
				uint8_t DATAZH;
			} buffer{};

			struct Temp {
				uint8_t TEMPL;
				uint8_t TEMPH;
			} buffer_temp;

			bool success = false;
			uint8_t cmd = static_cast<uint8_t>(Register::STAT1);
			uint8_t cmd_temp = static_cast<uint8_t>(Register::TEMPL);

			if (transfer(&cmd, 1, (uint8_t *)&buffer, sizeof(buffer)) == PX4_OK
			    && transfer(&cmd_temp, 1, (uint8_t *)&buffer_temp, sizeof(buffer_temp)) == PX4_OK) {

				if (buffer.STAT1 & STAT1_BIT::DRDY) {
					int16_t x = combine(buffer.DATAXH, buffer.DATAXL);
					int16_t y = combine(buffer.DATAYH, buffer.DATAYL);
					int16_t z = combine(buffer.DATAZH, buffer.DATAZL);
					int16_t t = combine(buffer_temp.TEMPH, buffer_temp.TEMPL);

					_px4_mag.set_temperature((IST8310_TEMP_25C_OFFSET - t) * IST8310_TEMP_RESOLUTION + 25);

					float xf = (float)x * _crossaxis_inv[0] + (float)y * _crossaxis_inv[1] + (float)z * _crossaxis_inv[2];
					float yf = (float)x * _crossaxis_inv[3] + (float)y * _crossaxis_inv[4] + (float)z * _crossaxis_inv[5];
					float zf = (float)x * _crossaxis_inv[6] + (float)y * _crossaxis_inv[7] + (float)z * _crossaxis_inv[8];

					_px4_mag.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf));

					// sensor's frame is +x forward, +y right, +z up
					_px4_mag.update(now, xf, yf, -zf);

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

			// initiate next measurement
			RegisterWrite(Register::CNTL1, CNTL1_BIT::MODE_SINGLE_MEASUREMENT);
			ScheduleDelayed(20_ms); // Wait at least 6ms. (minimum waiting time for 16 times internal average setup)
		}

		break;
	}
}

bool IST8310::Configure()
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

	_px4_mag.set_scale(1.f / 1320.f); // 1320 LSB/Gauss

	/* Pull & process cross-axis compensation */
	cross_axis_comp();

	return success;
}

void
IST8310::cross_axis_comp()
{
	int ret;
	uint8_t bOTPDataFlags;
	uint8_t tempBuff[2];
	bool crossaxis_enable = 0;
	float inv[9] = {0};
	short OTPcrossaxis[9] = {0};

	uint8_t crossxbuf[6];
	uint8_t crossybuf[6];
	uint8_t crosszbuf[6];

	update_crossaxis();

	//check otp_date (Sensors before some date appears to have the matrix stored transposed)
	bOTPDataFlags = 0;

	ret = read(Register::OTP, (uint8_t *)&tempBuff, sizeof(tempBuff));

	if (OK != ret) {
		perf_count(_bad_transfer_perf);
		return;
	}


	if (tempBuff[0] == 0xff) {
		bOTPDataFlags = 0;

	} else if (tempBuff[0] <= 0x12) {
		if (tempBuff[1] <= 0x07) {
			bOTPDataFlags = 1;
		}
	}

	ret = read(Register::XX_CROSS_L, (uint8_t *)&tempBuff, sizeof(tempBuff));

	if (OK != ret) {
		perf_count(_bad_transfer_perf);
		return;
	}

	if ((tempBuff[0] == 0xFF) && (tempBuff[1] == 0xFF)) {
		crossaxis_enable = 0;

	} else {
		crossaxis_enable = 1;
	}

	if (crossaxis_enable == 0) {
		update_crossaxis();
		return;

	} else {
		ret = read(Register::XX_CROSS_L, (uint8_t *)&crossxbuf, sizeof(crossxbuf));

		if (OK != ret) {
			perf_count(_bad_transfer_perf);
			return;
		}

		ret = read(Register::YX_CROSS_L, (uint8_t *)&crossybuf, sizeof(crossybuf));

		if (OK != ret) {
			perf_count(_bad_transfer_perf);
			return;
		}

		ret = read(Register::ZX_CROSS_L, (uint8_t *)&crosszbuf, sizeof(crosszbuf));

		if (OK != ret) {
			perf_count(_bad_transfer_perf);
			return;
		}

		if (bOTPDataFlags) {
			//before
			OTPcrossaxis[0] = ((int16_t) crossxbuf[1]) << 8 | crossxbuf[0];
			OTPcrossaxis[1] = ((int16_t) crossxbuf[3]) << 8 | crossxbuf[2];
			OTPcrossaxis[2] = ((int16_t) crossxbuf[5]) << 8 | crossxbuf[4];
			OTPcrossaxis[3] = ((int16_t) crossybuf[1]) << 8 | crossybuf[0];
			OTPcrossaxis[4] = ((int16_t) crossybuf[3]) << 8 | crossybuf[2];
			OTPcrossaxis[5] = ((int16_t) crossybuf[5]) << 8 | crossybuf[4];
			OTPcrossaxis[6] = ((int16_t) crosszbuf[1]) << 8 | crosszbuf[0];
			OTPcrossaxis[7] = ((int16_t) crosszbuf[3]) << 8 | crosszbuf[2];
			OTPcrossaxis[8] = ((int16_t) crosszbuf[5]) << 8 | crosszbuf[4];

		} else {
			//after
			OTPcrossaxis[0] = ((int16_t) crossxbuf[1]) << 8 | crossxbuf[0];
			OTPcrossaxis[3] = ((int16_t) crossxbuf[3]) << 8 | crossxbuf[2];
			OTPcrossaxis[6] = ((int16_t) crossxbuf[5]) << 8 | crossxbuf[4];
			OTPcrossaxis[1] = ((int16_t) crossybuf[1]) << 8 | crossybuf[0];
			OTPcrossaxis[4] = ((int16_t) crossybuf[3]) << 8 | crossybuf[2];
			OTPcrossaxis[7] = ((int16_t) crossybuf[5]) << 8 | crossybuf[4];
			OTPcrossaxis[2] = ((int16_t) crosszbuf[1]) << 8 | crosszbuf[0];
			OTPcrossaxis[5] = ((int16_t) crosszbuf[3]) << 8 | crosszbuf[2];
			OTPcrossaxis[8] = ((int16_t) crosszbuf[5]) << 8 | crosszbuf[4];
		}

		_crossaxis_det = ((int32_t)OTPcrossaxis[0]) * ((int32_t)OTPcrossaxis[4]) * ((int32_t)OTPcrossaxis[8]) +
				 ((int32_t)OTPcrossaxis[1]) * ((int32_t)OTPcrossaxis[5]) * ((int32_t)OTPcrossaxis[6]) +
				 ((int32_t)OTPcrossaxis[2]) * ((int32_t)OTPcrossaxis[3]) * ((int32_t)OTPcrossaxis[7]) -
				 ((int32_t)OTPcrossaxis[0]) * ((int32_t)OTPcrossaxis[5]) * ((int32_t)OTPcrossaxis[7]) -
				 ((int32_t)OTPcrossaxis[2]) * ((int32_t)OTPcrossaxis[4]) * ((int32_t)OTPcrossaxis[6]) -
				 ((int32_t)OTPcrossaxis[1]) * ((int32_t)OTPcrossaxis[3]) * ((int32_t)OTPcrossaxis[8]);

		if (_crossaxis_det == 0) {
			update_crossaxis();
		}

		inv[0] = (float)OTPcrossaxis[4] * (float)OTPcrossaxis[8] - (float)OTPcrossaxis[5] * (float)OTPcrossaxis[7];
		inv[1] = (float)OTPcrossaxis[2] * (float)OTPcrossaxis[7] - (float)OTPcrossaxis[1] * (float)OTPcrossaxis[8];
		inv[2] = (float)OTPcrossaxis[1] * (float)OTPcrossaxis[5] - (float)OTPcrossaxis[2] * (float)OTPcrossaxis[4];
		inv[3] = (float)OTPcrossaxis[5] * (float)OTPcrossaxis[6] - (float)OTPcrossaxis[3] * (float)OTPcrossaxis[8];
		inv[4] = (float)OTPcrossaxis[0] * (float)OTPcrossaxis[8] - (float)OTPcrossaxis[2] * (float)OTPcrossaxis[6];
		inv[5] = (float)OTPcrossaxis[2] * (float)OTPcrossaxis[3] - (float)OTPcrossaxis[0] * (float)OTPcrossaxis[5];
		inv[6] = (float)OTPcrossaxis[3] * (float)OTPcrossaxis[7] - (float)OTPcrossaxis[4] * (float)OTPcrossaxis[6];
		inv[7] = (float)OTPcrossaxis[1] * (float)OTPcrossaxis[6] - (float)OTPcrossaxis[0] * (float)OTPcrossaxis[7];
		inv[8] = (float)OTPcrossaxis[0] * (float)OTPcrossaxis[4] - (float)OTPcrossaxis[1] * (float)OTPcrossaxis[3];

		for (int i = 0; i < 9; i++) {
			_crossaxis_inv[i] = inv[i] * ((float)IST8310_OTP_SENSITIVITY) / ((float)_crossaxis_det);
		}

	}

	return;
}

void
IST8310::update_crossaxis()
{
	*_crossaxis_inv = 1;
	*(_crossaxis_inv + 1) = 0;
	*(_crossaxis_inv + 2) = 0;
	*(_crossaxis_inv + 3) = 0;
	*(_crossaxis_inv + 4) = 1;
	*(_crossaxis_inv + 5) = 0;
	*(_crossaxis_inv + 6) = 0;
	*(_crossaxis_inv + 7) = 0;
	*(_crossaxis_inv + 8) = 1;
	_crossaxis_det = 1;
}

bool IST8310::RegisterCheck(const register_config_t &reg_cfg)
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

uint8_t IST8310::RegisterRead(Register reg)
{
	const uint8_t cmd = static_cast<uint8_t>(reg);
	uint8_t buffer{};
	transfer(&cmd, 1, &buffer, 1);
	return buffer;
}

int IST8310::read(Register reg, void *data, unsigned count)
{
	const uint8_t cmd = static_cast<uint8_t>(reg);
	return transfer(&cmd, 1, (uint8_t *)data, count);
}

void IST8310::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t buffer[2] { (uint8_t)reg, value };
	transfer(buffer, sizeof(buffer), nullptr, 0);
}

void IST8310::RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);
	uint8_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}
