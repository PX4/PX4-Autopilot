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

#include "IST8310J.hpp"

using namespace time_literals;

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

IST8310J::IST8310J(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_px4_mag(get_device_id(), config.rotation)
{
}

IST8310J::~IST8310J()
{
	perf_free(_reset_perf);
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
}

int IST8310J::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool IST8310J::Reset()
{
	_state = STATE::RESET;
	ScheduleClear();
	ScheduleNow();
	return true;
}

void IST8310J::print_status()
{
	I2CSPIDriverBase::print_status();

	perf_print_counter(_reset_perf);
	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
}

int IST8310J::probe()
{
	// Reading the WAI register is not always reliable, it can return 0xff or
	// other values if the sensor has been powered up in a certain way. In
	// addition, the I2C address is not always correct, sometimes it boots with
	// 0x0C rather than 0x0E.
	const auto start_time = hrt_absolute_time();
	const uint8_t start_addr = get_device_address();

	while (hrt_elapsed_time(&start_time) < 50_ms) {
		set_device_address(start_addr);
		const int WAI = RegisterRead(Register::WAI);

		if (WAI == Device_ID) {
			// Device has the right I2C address and register content
			return PX4_OK;
		}

		// send reset command to all four possible addresses
		for (uint8_t addr = 0x0C; addr <= 0x0F; addr++) {
			set_device_address(addr);
			RegisterWrite(Register::CNTL2, CNTL2_BIT::SRST);
		}

		px4_usleep(10'000);
	}

	return PX4_ERROR;
}

void IST8310J::RunImpl()
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

			bool success = false;
			uint8_t cmd = static_cast<uint8_t>(Register::STAT1);

			if (transfer(&cmd, 1, (uint8_t *)&buffer, sizeof(buffer)) == PX4_OK) {

				if (buffer.STAT1 & STAT1_BIT::DRDY) {
					int16_t x = combine(buffer.DATAXH, buffer.DATAXL);
					int16_t y = combine(buffer.DATAYH, buffer.DATAYL);
					int16_t z = combine(buffer.DATAZH, buffer.DATAZL);
					//PX4_INFO("Raw mag data before cross-axis transformation: x=%d, y=%d, z=%d", x, y, z);
					// Apply cross-axis transformation if enabled
					int16_t xyz[3] = {x, y, z};
					CrossAxisTransformation(xyz);
					x = xyz[0];
					y = xyz[1];
					z = xyz[2];
					//PX4_INFO("Raw mag data after cross-axis transformation: x=%d, y=%d, z=%d", x, y, z);
					// sensor's frame is +x forward, +y right, +z up
					z = (z == INT16_MIN) ? INT16_MAX : -z; // flip z

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

bool IST8310J::Configure()
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

	// Initialize cross-axis calibration matrix
	if (!InitializeCrossAxisMatrix()) {
		PX4_WARN("Failed to initialize cross-axis calibration matrix");
	}

	return success;
}

bool IST8310J::RegisterCheck(const register_config_t &reg_cfg)
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

int IST8310J::RegisterRead(Register reg)
{
	const uint8_t cmd = static_cast<uint8_t>(reg);
	uint8_t buffer{};
	const int ret = transfer(&cmd, 1, &buffer, 1);

	if (ret != OK) { return -1; }

	return buffer;
}

void IST8310J::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t buffer[2] { (uint8_t)reg, value };
	transfer(buffer, sizeof(buffer), nullptr, 0);
}

void IST8310J::RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);
	uint8_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}

bool IST8310J::InitializeCrossAxisMatrix()
{
	// Check OTP date validity
	uint8_t otp_date[3];
	uint8_t cmd = 0x99;

	if (transfer(&cmd, 1, otp_date, 3) != PX4_OK) {
		PX4_ERR("Failed to read OTP date");
		return false;
	}

	_otp_data_valid = false;

	if (otp_date[0] != 0xFF) {
		if (otp_date[0] <= 0x12 && otp_date[1] <= 0x07) {
			_otp_data_valid = true;
		}
	}

	// Check if cross-axis calibration data is available
	uint8_t crossaxis_check[2];
	cmd = static_cast<uint8_t>(Register::XX_CROSS_L);

	if (transfer(&cmd, 1, crossaxis_check, 2) != PX4_OK) {
		PX4_ERR("Failed to read cross-axis check registers");
		return false;
	}

	_crossaxis_enabled = !((crossaxis_check[0] == 0xFF) && (crossaxis_check[1] == 0xFF));

	if (!_crossaxis_enabled) {
		// Set identity matrix
		_crossaxis_inv[0] = (1 << CROSSAXIS_INV_BITSHIFT);
		_crossaxis_inv[1] = 0;
		_crossaxis_inv[2] = 0;
		_crossaxis_inv[3] = 0;
		_crossaxis_inv[4] = (1 << CROSSAXIS_INV_BITSHIFT);
		_crossaxis_inv[5] = 0;
		_crossaxis_inv[6] = 0;
		_crossaxis_inv[7] = 0;
		_crossaxis_inv[8] = (1 << CROSSAXIS_INV_BITSHIFT);
		_crossaxis_det = 1;
		return true;
	}

	// Read cross-axis matrix data
	uint8_t crossx_buf[6], crossy_buf[6], crossz_buf[6];

	cmd = static_cast<uint8_t>(Register::XX_CROSS_L);

	if (transfer(&cmd, 1, crossx_buf, 6) != PX4_OK) {
		PX4_ERR("Failed to read X cross-axis data");
		return false;
	}

	// PX4_INFO("IST8310J XX_CROSS_L: 0x%02hhX, XX_CROSS_H: 0x%02hhX", crossx_buf[0], crossx_buf[1]);
	// PX4_INFO("IST8310J XY_CROSS_L: 0x%02hhX, XY_CROSS_H: 0x%02hhX", crossx_buf[2], crossx_buf[3]);
	// PX4_INFO("IST8310J XZ_CROSS_L: 0x%02hhX, XZ_CROSS_H: 0x%02hhX", crossx_buf[4], crossx_buf[5]);

	cmd = static_cast<uint8_t>(Register::YX_CROSS_L);

	if (transfer(&cmd, 1, crossy_buf, 6) != PX4_OK) {
		PX4_ERR("Failed to read Y cross-axis data");
		return false;
	}

	// PX4_INFO("IST8310J YX_CROSS_L: 0x%02hhX, YX_CROSS_H: 0x%02hhX", crossy_buf[0], crossy_buf[1]);
	// PX4_INFO("IST8310J YX_CROSS_L: 0x%02hhX, YX_CROSS_H: 0x%02hhX", crossy_buf[2], crossy_buf[3]);
	// PX4_INFO("IST8310J YZ_CROSS_L: 0x%02hhX, YZ_CROSS_H: 0x%02hhX", crossy_buf[4], crossy_buf[5]);

	cmd = static_cast<uint8_t>(Register::ZX_CROSS_L);

	if (transfer(&cmd, 1, crossz_buf, 6) != PX4_OK) {
		PX4_ERR("Failed to read Z cross-axis data");
		return false;
	}

	// PX4_INFO("IST8310J ZX_CROSS_L: 0x%02hhX, ZX_CROSS_H: 0x%02hhX", crossz_buf[0], crossz_buf[1]);
	// PX4_INFO("IST8310J ZY_CROSS_L: 0x%02hhX, ZY_CROSS_H: 0x%02hhX", crossz_buf[2], crossz_buf[3]);
	// PX4_INFO("IST8310J ZZ_CROSS_L: 0x%02hhX, ZZ_CROSS_H: 0x%02hhX", crossz_buf[4], crossz_buf[5]);

	// Parse cross-axis matrix based on OTP data format
	int16_t otp_crossaxis[9];

	if (_otp_data_valid) {
		// Before format
		otp_crossaxis[0] = combine(crossx_buf[1], crossx_buf[0]);
		otp_crossaxis[1] = combine(crossx_buf[3], crossx_buf[2]);
		otp_crossaxis[2] = combine(crossx_buf[5], crossx_buf[4]);
		otp_crossaxis[3] = combine(crossy_buf[1], crossy_buf[0]);
		otp_crossaxis[4] = combine(crossy_buf[3], crossy_buf[2]);
		otp_crossaxis[5] = combine(crossy_buf[5], crossy_buf[4]);
		otp_crossaxis[6] = combine(crossz_buf[1], crossz_buf[0]);
		otp_crossaxis[7] = combine(crossz_buf[3], crossz_buf[2]);
		otp_crossaxis[8] = combine(crossz_buf[5], crossz_buf[4]);

	} else {
		// After format
		otp_crossaxis[0] = combine(crossx_buf[1], crossx_buf[0]);
		otp_crossaxis[3] = combine(crossx_buf[3], crossx_buf[2]);
		otp_crossaxis[6] = combine(crossx_buf[5], crossx_buf[4]);
		otp_crossaxis[1] = combine(crossy_buf[1], crossy_buf[0]);
		otp_crossaxis[4] = combine(crossy_buf[3], crossy_buf[2]);
		otp_crossaxis[7] = combine(crossy_buf[5], crossy_buf[4]);
		otp_crossaxis[2] = combine(crossz_buf[1], crossz_buf[0]);
		otp_crossaxis[5] = combine(crossz_buf[3], crossz_buf[2]);
		otp_crossaxis[8] = combine(crossz_buf[5], crossz_buf[4]);
	}

	// PX4_INFO("Cross-axis matrix from OTP:");
	// PX4_INFO("[[%d, %d, %d],", otp_crossaxis[0], otp_crossaxis[1], otp_crossaxis[2]);
	// PX4_INFO(" [%d, %d, %d],", otp_crossaxis[3], otp_crossaxis[4], otp_crossaxis[5]);
	// PX4_INFO(" [%d, %d, %d]]", otp_crossaxis[6], otp_crossaxis[7], otp_crossaxis[8]);

	// Calculate matrix determinant
	_crossaxis_det = ((int32_t)otp_crossaxis[0]) * otp_crossaxis[4] * otp_crossaxis[8] +
			 ((int32_t)otp_crossaxis[1]) * otp_crossaxis[5] * otp_crossaxis[6] +
			 ((int32_t)otp_crossaxis[2]) * otp_crossaxis[3] * otp_crossaxis[7] -
			 ((int32_t)otp_crossaxis[0]) * otp_crossaxis[5] * otp_crossaxis[7] -
			 ((int32_t)otp_crossaxis[2]) * otp_crossaxis[4] * otp_crossaxis[6] -
			 ((int32_t)otp_crossaxis[1]) * otp_crossaxis[3] * otp_crossaxis[8];

	// PX4_INFO("Cross-axis matrix determinant: %d", _crossaxis_det);
	if (_crossaxis_det == 0) {
		PX4_WARN("Cross-axis determinant is zero, using identity matrix");
		_crossaxis_enabled = false;
		// Directly set identity matrix instead of recursive call
		_crossaxis_inv[0] = (1 << CROSSAXIS_INV_BITSHIFT);
		_crossaxis_inv[1] = 0;
		_crossaxis_inv[2] = 0;
		_crossaxis_inv[3] = 0;
		_crossaxis_inv[4] = (1 << CROSSAXIS_INV_BITSHIFT);
		_crossaxis_inv[5] = 0;
		_crossaxis_inv[6] = 0;
		_crossaxis_inv[7] = 0;
		_crossaxis_inv[8] = (1 << CROSSAXIS_INV_BITSHIFT);
		_crossaxis_det = 1;
		return true;
	}

	// Calculate inverse matrix
	int64_t inv[9];
	inv[0] = (int64_t)otp_crossaxis[4] * otp_crossaxis[8] - (int64_t)otp_crossaxis[5] * otp_crossaxis[7];
	inv[1] = (int64_t)otp_crossaxis[2] * otp_crossaxis[7] - (int64_t)otp_crossaxis[1] * otp_crossaxis[8];
	inv[2] = (int64_t)otp_crossaxis[1] * otp_crossaxis[5] - (int64_t)otp_crossaxis[2] * otp_crossaxis[4];
	inv[3] = (int64_t)otp_crossaxis[5] * otp_crossaxis[6] - (int64_t)otp_crossaxis[3] * otp_crossaxis[8];
	inv[4] = (int64_t)otp_crossaxis[0] * otp_crossaxis[8] - (int64_t)otp_crossaxis[2] * otp_crossaxis[6];
	inv[5] = (int64_t)otp_crossaxis[2] * otp_crossaxis[3] - (int64_t)otp_crossaxis[0] * otp_crossaxis[5];
	inv[6] = (int64_t)otp_crossaxis[3] * otp_crossaxis[7] - (int64_t)otp_crossaxis[4] * otp_crossaxis[6];
	inv[7] = (int64_t)otp_crossaxis[1] * otp_crossaxis[6] - (int64_t)otp_crossaxis[0] * otp_crossaxis[7];
	inv[8] = (int64_t)otp_crossaxis[0] * otp_crossaxis[4] - (int64_t)otp_crossaxis[1] * otp_crossaxis[3];

	for (int i = 0; i < 9; i++) {
		_crossaxis_inv[i] = (inv[i] << CROSSAXIS_INV_BITSHIFT) * OTP_SENSITIVITY;
	}

	// PX4_INFO("Inverse cross-axis matrix:");
	// PX4_INFO("[[%lld, %lld, %lld],", _crossaxis_inv[0], _crossaxis_inv[1], _crossaxis_inv[2]);
	// PX4_INFO(" [%lld, %lld, %lld],", _crossaxis_inv[3], _crossaxis_inv[4], _crossaxis_inv[5]);
	// PX4_INFO(" [%lld, %lld, %lld]]", _crossaxis_inv[6], _crossaxis_inv[7], _crossaxis_inv[8]);

	PX4_INFO("Cross-axis calibration initialized successfully");
	return true;
}

void IST8310J::CrossAxisTransformation(int16_t *xyz)
{
	if (!_crossaxis_enabled) {
		return;
	}

	// Check if crossaxis matrix is initialized
	bool matrix_initialized = false;

	for (int i = 0; i < 9; i++) {
		if (_crossaxis_inv[i] != 0) {
			matrix_initialized = true;
			break;
		}
	}

	if (!matrix_initialized) {
		PX4_WARN("Cross-axis matrix not initialized, reinitializing");
		InitializeCrossAxisMatrix();
		return;
	}

	// Apply cross-axis transformation
	int64_t output_tmp[3];

	output_tmp[0] = (int64_t)xyz[0] * _crossaxis_inv[0] +
			(int64_t)xyz[1] * _crossaxis_inv[1] +
			(int64_t)xyz[2] * _crossaxis_inv[2];

	output_tmp[1] = (int64_t)xyz[0] * _crossaxis_inv[3] +
			(int64_t)xyz[1] * _crossaxis_inv[4] +
			(int64_t)xyz[2] * _crossaxis_inv[5];

	output_tmp[2] = (int64_t)xyz[0] * _crossaxis_inv[6] +
			(int64_t)xyz[1] * _crossaxis_inv[7] +
			(int64_t)xyz[2] * _crossaxis_inv[8];

	// Apply determinant division and bit shift
	for (int i = 0; i < IST8310J_AXES_NUM; i++) {
		output_tmp[i] = output_tmp[i] / _crossaxis_det;
	}

	xyz[0] = static_cast<int16_t>(output_tmp[0] >> CROSSAXIS_INV_BITSHIFT);
	xyz[1] = static_cast<int16_t>(output_tmp[1] >> CROSSAXIS_INV_BITSHIFT);
	xyz[2] = static_cast<int16_t>(output_tmp[2] >> CROSSAXIS_INV_BITSHIFT);
}
