/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "HMC6343.hpp"

using namespace time_literals;

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

HMC6343::HMC6343(I2CSPIBusOption bus_option, int bus, uint8_t addr, int bus_frequency, enum Rotation rotation) :
	I2C(DRV_MAG_DEVTYPE_HMC6343, MODULE_NAME, bus, addr, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus, addr),
	_px4_mag(get_device_id(), rotation)
{
	_px4_mag.set_external(external());
}

HMC6343::~HMC6343()
{
	perf_free(_reset_perf);
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
}

int HMC6343::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool HMC6343::Reset()
{
	_state = STATE::RESET;
	ScheduleClear();
	ScheduleNow();
	return true;
}

void HMC6343::print_status()
{
	I2CSPIDriverBase::print_status();

	perf_print_counter(_reset_perf);
	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
}

int HMC6343::probe()
{
	ScheduleDelayed(500_ms); // Reset Recovery Time 500 ms
		
	// Power-On Start-Up Time 500 ms
	if (hrt_absolute_time() < 500_ms) {
		PX4_WARN("Power-On Start-Up Time is 500 ms");
	}

	uint8_t sn_lsb = ReadEEPROM(Register::SN_LSB);
	uint8_t sn_msb = ReadEEPROM(Register::SN_MSB);
	uint16_t serial_num = combine(sn_msb, sn_lsb);

	uint8_t sw_version = ReadEEPROM(Register::SW_VERSION);

	uint8_t date_code_ww = ReadEEPROM(Register::DATE_CODE_WW);
	uint8_t date_code_yy = ReadEEPROM(Register::DATE_CODE_YY);

	PX4_INFO("Serial Number: 0x%X, Firmware Version: 0x%X, Date (WW): %X, (YY): %X", serial_num, sw_version, date_code_ww, date_code_yy);

	return PX4_OK;
}

void HMC6343::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET:

		// Software Reset
		SendCommand(Command::RESET);
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;
		perf_count(_reset_perf);
		ScheduleDelayed(500_ms); // Reset Recovery Time 500 ms

		break;

	case STATE::WAIT_FOR_RESET:

		// Check if Run Mode bit is set
		if ((ReadOPMode(Command::POST_OPMODE1)) & OP_Mode1_BIT::Run) {

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
			ScheduleDelayed(200_ms);

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
				uint8_t MxMSB;
				uint8_t MxLSB;
				uint8_t MyMSB;
				uint8_t MyLSB;
				uint8_t MzMSB;
				uint8_t MzLSB;
			} buffer{};

			bool success = false;
			uint8_t cmd = static_cast<uint8_t>(Command::POST_MAG);

			transfer(&cmd, 1, nullptr, 0);
			ScheduleDelayed(1_ms); // response delay 1 ms
			if (transfer(nullptr, 0, (uint8_t *)&buffer, sizeof(buffer)) == PX4_OK) {

				int16_t mag_x = combine(buffer.MxMSB, buffer.MxLSB);
				int16_t mag_y = combine(buffer.MyMSB, buffer.MyLSB);
				int16_t mag_z = combine(buffer.MzMSB, buffer.MzLSB);

				// sensor's frame is +x forward, +y right, +z up
				mag_z = (mag_z == INT16_MIN) ? INT16_MAX : -mag_z; // flip z

				_px4_mag.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf));
				_px4_mag.update(now, mag_x, mag_y, mag_z);

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

			// initiate next measurement
			ScheduleDelayed(200_ms); // Wait at least 200 ms to get fresh data from the default 5 Hz update rate
		}

		break;
	}
}

bool HMC6343::Configure()
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

	_px4_mag.set_scale(1.f / 1200.f); // 1200 LSB/Gauss

	return success;
}

bool HMC6343::RegisterCheck(const register_config_t &reg_cfg)
{
	bool success = true;

	const uint8_t reg_value = ReadEEPROM(reg_cfg.reg);

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

uint8_t HMC6343::ReadOPMode(Command cmnd)
{
	const uint8_t cmd = static_cast<uint8_t>(cmnd);
	transfer(&cmd, 1, nullptr, 0);
	ScheduleDelayed(1_ms); // response delay 1 ms
	uint8_t buffer {};
	transfer(nullptr, 0, &buffer, 1);
	return buffer;
}

void HMC6343::SendCommand(Command cmnd)
{
	const uint8_t cmd = static_cast<uint8_t>(cmnd);
	transfer(&cmd, 1, nullptr, 0);
	//ScheduleDelayed(?_ms); // Table 3 of datasheet, Command to Response Delay Times
}

uint8_t HMC6343::ReadEEPROM(Register reg)
{
	const uint8_t cmd[2] { (uint8_t)(Command::READ_EEPROM), (uint8_t)reg };
	transfer(cmd, sizeof(cmd), nullptr, 0);
	ScheduleDelayed(10_ms); // response delay 10 ms
	uint8_t buffer {};
	transfer(nullptr, 0, &buffer, 1);
	return buffer;
}

void HMC6343::WriteEEPROM(Register reg, uint8_t value)
{
	const uint8_t cmd[3] { (uint8_t)(Command::WRITE_EEPROM), (uint8_t)reg, value };
	transfer(cmd, sizeof(cmd), nullptr, 0);
	ScheduleDelayed(10_ms); // response delay 10 ms
}

void HMC6343::RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = ReadEEPROM(reg);

	uint8_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		WriteEEPROM(reg, val);		
	}
}
