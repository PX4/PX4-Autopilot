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

#include "ADIS16607.hpp"
#include <endian.h>

using namespace time_literals;

ADIS16607::ADIS16607(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_drdy_gpio(config.drdy_gpio),
	_px4_accel(get_device_id(), config.rotation),
	_px4_gyro(get_device_id(), config.rotation)
{
	if (_drdy_gpio != 0) {
		_drdy_missed_perf = perf_alloc(PC_COUNT, MODULE_NAME": DRDY missed");
	}
}

ADIS16607::~ADIS16607()
{
	perf_free(_reset_perf);
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_perf_crc_bad);
	perf_free(_drdy_missed_perf);
}

int ADIS16607::init()
{
	int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool ADIS16607::Reset()
{
	_state = STATE::RESET;
	DataReadyInterruptDisable();
	ScheduleClear();
	ScheduleNow();
	return true;
}

void ADIS16607::exit_and_cleanup()
{
	DataReadyInterruptDisable();
	I2CSPIDriverBase::exit_and_cleanup();
}

void ADIS16607::print_status()
{
	I2CSPIDriverBase::print_status();

	perf_print_counter(_reset_perf);
	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_perf_crc_bad);
	perf_print_counter(_drdy_missed_perf);
}

int ADIS16607::probe()
{
	// Power-On Start-Up Time 50 ms
	if (hrt_absolute_time() < 50_ms) {
		PX4_WARN("required Power-On Start-Up Time 50 ms");
	}

	// lock the device to half duplex SPI mode
	RegisterWrite(Register::SPI_HALFDUPLEX_KEY, 0xB4B4);

	const uint16_t DEV_ID = RegisterRead(Register::DEV_ID);

	if (DEV_ID != device_identification) {
		PX4_ERR("unexpected DEV_ID 0x%02x", DEV_ID);
		return PX4_ERROR;
	}

	return PX4_OK;
}

void ADIS16607::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET:
		perf_count(_reset_perf);
		RegisterWrite(Register::SOFT_RESET, 0x01);
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;
		ScheduleDelayed(50_ms); // 50 ms Reset Recovery Time
		break;

	case STATE::WAIT_FOR_RESET:
		// lock the device to half duplex SPI mode
		RegisterWrite(Register::SPI_HALFDUPLEX_KEY, 0xB4B4);
		// These bits are cleared when read
		RegisterRead(Register::DIAG_STAT);

		if (_self_test_passed) {
			if ((RegisterRead(Register::DEV_ID) == device_identification)) {
				// if reset succeeded then configure
				_state = STATE::CONFIGURE;
				ScheduleNow();

			} else {
				// RESET not complete
				if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
					PX4_DEBUG("Reset failed, retrying");
					_state = STATE::RESET;
					ScheduleDelayed(100_ms);

				} else {
					PX4_DEBUG("Reset not complete, check again in 100 ms");
					ScheduleDelayed(100_ms);
				}
			}

		} else {
			RegisterWrite(Register::MSC_CTRL, MSC_CTRL_BIT::Self_test_1);
			_state = STATE::SELF_TEST_CHECK;
			ScheduleDelayed(50_ms); // Self Test Time
		}

		break;

	case STATE::SELF_TEST_CHECK: {
			// read DIAG_STAT to check result
			const uint16_t DIAG_STAT = RegisterRead(Register::DIAG_STAT);

			if (DIAG_STAT != 0) {
				PX4_ERR("self test failed, resetting. DIAG_STAT: %#X", DIAG_STAT);
				_state = STATE::RESET;
				ScheduleDelayed(3_s);

			} else {
				PX4_DEBUG("self test passed");
				_self_test_passed = true;
				_state = STATE::RESET;
				ScheduleNow();
			}
		}
		break;

	case STATE::CONFIGURE:
		if (Configure()) {
			// if configure succeeded then start reading
			_state = STATE::READ;

			if (DataReadyInterruptConfigure()) {
				_data_ready_interrupt_enabled = true;

				// backup schedule as a watchdog timeout
				ScheduleDelayed(100_ms);

			} else {
				_data_ready_interrupt_enabled = false;
				ScheduleOnInterval(SAMPLE_INTERVAL_US, SAMPLE_INTERVAL_US);
			}

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
			hrt_abstime timestamp_sample = now;

			if (_data_ready_interrupt_enabled) {
				// scheduled from interrupt if _drdy_timestamp_sample was set as expected
				const hrt_abstime drdy_timestamp_sample = _drdy_timestamp_sample.fetch_and(0);

				if ((now - drdy_timestamp_sample) < SAMPLE_INTERVAL_US) {
					timestamp_sample = drdy_timestamp_sample;

				} else {
					perf_count(_drdy_missed_perf);
				}

				// push backup schedule back
				ScheduleDelayed(SAMPLE_INTERVAL_US * 2);
			}

			bool success = false;
			// __attribute__((packed))
			struct BurstRead {
				uint16_t cmd;
				uint16_t DIAG_STAT;
				int16_t X_ACCL_HIGH_OUT;
				int16_t X_ACCL_LOW_OUT;
				int16_t Y_ACCL_HIGH_OUT;
				int16_t Y_ACCL_LOW_OUT;
				int16_t Z_ACCL_HIGH_OUT;
				int16_t Z_ACCL_LOW_OUT;
				int16_t X_GYRO_HIGH_OUT;
				int16_t X_GYRO_LOW_OUT;
				int16_t Y_GYRO_HIGH_OUT;
				int16_t Y_GYRO_LOW_OUT;
				int16_t Z_GYRO_HIGH_OUT;
				int16_t Z_GYRO_LOW_OUT;
				int16_t TEMP_OUT;
				uint16_t COUNT;
				uint16_t checksum;
			} buffer{};

			// ADIS16607 burst report should be 272 bits
			static_assert(sizeof(BurstRead) == (272 / 8), "ADIS16607 report not 272 bits");

			buffer.cmd = static_cast<uint8_t>(Register::DIAG_STAT) | DIR_READ;

			if (transfer((uint8_t *)&buffer, ((uint8_t *)&buffer), sizeof(buffer) / sizeof(uint8_t)) == PX4_OK) {

				// Calculate checksum and compare
				uint16_t *checksum_helper = &buffer.DIAG_STAT;

				uint16_t checksum = 0;

				for (int i = 0; i < 15; i++) {
					checksum += be16toh(checksum_helper[i]);
				}

				if (be16toh(buffer.checksum) != checksum) {
					perf_count(_bad_transfer_perf);
					perf_count(_perf_crc_bad);
				}

				// Check all Status/Error Flag Indicators (DIAG_STAT)
				if (be16toh(buffer.DIAG_STAT) != 0) {
					perf_count(_bad_transfer_perf);
				}

				buffer.TEMP_OUT = (int16_t)be16toh(buffer.TEMP_OUT);

				float temperature = 0;
				temperature = (float)(buffer.TEMP_OUT) * 0.005f + 25.0f;

				_px4_accel.set_temperature(temperature);
				_px4_gyro.set_temperature(temperature);

				int32_t accel_x = (int32_t)((int32_t)(be16toh(buffer.X_ACCL_HIGH_OUT) << 16 | be16toh(buffer.X_ACCL_LOW_OUT)) >> 8);
				int32_t accel_y = (int32_t)((int32_t)(be16toh(buffer.Y_ACCL_HIGH_OUT) << 16 | be16toh(buffer.Y_ACCL_LOW_OUT)) >> 8);
				int32_t accel_z = (int32_t)((int32_t)(be16toh(buffer.Z_ACCL_HIGH_OUT) << 16 | be16toh(buffer.Z_ACCL_LOW_OUT)) >> 8);

				// sensor's frame is +x forward, +y left, +z up
				//  flip y & z to publish right handed with z down (x forward, y right, z down)
				accel_y = (accel_y == INT16_MIN) ? INT16_MAX : -accel_y;
				accel_z = (accel_z == INT16_MIN) ? INT16_MAX : -accel_z;

				_px4_accel.update(timestamp_sample, accel_x, accel_y, accel_z);

				int32_t gyro_x = (int32_t)((int32_t)(be16toh(buffer.X_GYRO_HIGH_OUT) << 16 | be16toh(buffer.X_GYRO_LOW_OUT)) >> 8);
				int32_t gyro_y = (int32_t)((int32_t)(be16toh(buffer.Y_GYRO_HIGH_OUT) << 16 | be16toh(buffer.Y_GYRO_LOW_OUT)) >> 8);
				int32_t gyro_z = (int32_t)((int32_t)(be16toh(buffer.Z_GYRO_HIGH_OUT) << 16 | be16toh(buffer.Z_GYRO_LOW_OUT)) >> 8);
				// sensor's frame is +x forward, +y left, +z up
				//  flip y & z to publish right handed with z down (x forward, y right, z down)
				gyro_y = (gyro_y == INT16_MIN) ? INT16_MAX : -gyro_y;
				gyro_z = (gyro_z == INT16_MIN) ? INT16_MAX : -gyro_z;

				_px4_gyro.update(timestamp_sample, gyro_x, gyro_y, gyro_z);

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
				}
			}
		}

		break;
	}
}

bool ADIS16607::Configure()
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

	RegisterWrite(Register::USER_FIFO_CFG, USER_FIFO_CFG_BIT::CLEAR_FIFOB);

	// accel: ±40 g, 200000 LSB/g (24-bit format)
	_px4_accel.set_range(40.f * CONSTANTS_ONE_G);
	_px4_accel.set_scale(CONSTANTS_ONE_G / 200000.f); // scaling 200000 LSB/g -> m/s^2 per LSB

	// gyro: ±2000 °/sec, 4000 LSB/°/sec (24-bit format)
	_px4_gyro.set_range(math::radians(2000.f));
	_px4_gyro.set_scale(math::radians(1.f / 4000.f)); // scaling 4000 LSB/°/sec -> rad/s per LSB

	return success;
}

int ADIS16607::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<ADIS16607 *>(arg)->DataReady();
	return 0;
}

void ADIS16607::DataReady()
{
	_drdy_timestamp_sample.store(hrt_absolute_time());
	ScheduleNow();
}

bool ADIS16607::DataReadyInterruptConfigure()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	// Setup data ready on falling edge
	return px4_arch_gpiosetevent(_drdy_gpio, false, true, false, &DataReadyInterruptCallback, this) == 0;
}

bool ADIS16607::DataReadyInterruptDisable()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	return px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0;
}

bool ADIS16607::RegisterCheck(const register_config_t &reg_cfg)
{
	bool success = true;

	const uint16_t reg_value = RegisterRead(reg_cfg.reg);

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

uint16_t ADIS16607::RegisterRead(Register reg)
{
	uint8_t cmd[4];
	cmd[0] = (static_cast<uint8_t>(reg) | DIR_READ);

	transfer(&cmd[0], &cmd[0], 4);

	return (uint16_t)((cmd[2] << 8) | cmd[3]);
}

void ADIS16607::RegisterWrite(Register reg, uint16_t value)
{
	uint8_t cmd[3];
	cmd[0] = static_cast<uint8_t>(reg);
	cmd[1] = (value >> 8) & 0xFF;
	cmd[2] = value & 0xFF;

	transfer(&cmd[0], nullptr, 3);
}

void ADIS16607::RegisterSetAndClearBits(Register reg, uint16_t setbits, uint16_t clearbits)
{
	const uint16_t orig_val = RegisterRead(reg);

	uint16_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}
