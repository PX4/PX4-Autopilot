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

#include "IIM20670.hpp"

using namespace time_literals;

IIM20670::IIM20670(const I2CSPIDriverConfig &config) :
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

IIM20670::~IIM20670()
{
	perf_free(_reset_perf);
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_bad_crc_perf);
	perf_free(_drdy_missed_perf);
}

int IIM20670::init()
{
	int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool IIM20670::Reset()
{
	_state = STATE::RESET;
	DataReadyInterruptDisable();
	ScheduleClear();
	ScheduleNow();
	return true;
}

void IIM20670::exit_and_cleanup()
{
	DataReadyInterruptDisable();
	I2CSPIDriverBase::exit_and_cleanup();
}

void IIM20670::print_status()
{
	I2CSPIDriverBase::print_status();

	const char *range_str = "16 g";

	if (_accel_range == ACCEL_RANGE::RANGE_32G) {
		range_str = "32 g";

	} else if (_accel_range == ACCEL_RANGE::RANGE_64G) {
		range_str = "64 g";
	}

	PX4_INFO("accel range: %s, gyro range: 1966 dps", range_str);

	perf_print_counter(_reset_perf);
	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_bad_crc_perf);
	perf_print_counter(_drdy_missed_perf);
}

int IIM20670::probe()
{
	// return to bank 0 in case the sensor was left in another bank (no-op if bank_select is still locked)
	SelectRegisterBank(0, true);

	for (int i = 0; i < 3; i++) {
		const uint16_t fixed_value = RegisterRead(Register::BANK_0::FIXED_VALUE_REG);

		if (fixed_value == FIXED_VALUE) {
			return PX4_OK;
		}

		DEVICE_DEBUG("unexpected fixed_value 0x%04x", fixed_value);
	}

	return PX4_ERROR;
}

void IIM20670::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET:
		perf_count(_reset_perf);
		// RESET_CONTROL: hard reset (equivalent to power-on reset, also re-locks the register banks)
		RegisterWrite(Register::BANK_0::RESET_CONTROL, RESET_CONTROL_BIT::hard_reset);
		_reset_timestamp = now;
		_failure_count = 0;
		_last_register_bank = 0;
		_state = STATE::WAIT_FOR_RESET;
		ScheduleDelayed(200_ms); // start-up time for register read/write from power-up
		break;

	case STATE::WAIT_FOR_RESET:
		if (RegisterRead(Register::BANK_0::FIXED_VALUE_REG) == FIXED_VALUE) {
			_state = STATE::CONFIGURE;
			ScheduleNow();

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
			// start reading from the sensor at the full 8 kHz internal sampling rate
			_state = STATE::READ;
			_read_start_timestamp = now;

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

			SensorData data{};

			if (ReadData(&data)) {
				// discard output while the sensor signal path settles after reset/configuration:
				// the first samples can read full-scale, which would both publish garbage and
				// falsely escalate the accel range (the ladder is one-way until reset)
				if (now - _read_start_timestamp > SENSOR_SETTLE_TIME_US) {
					const float temperature = TEMPERATURE_OFFSET + (float)data.temp / TEMPERATURE_SENSITIVITY;
					_px4_accel.set_temperature(temperature);
					_px4_gyro.set_temperature(temperature);

					// sensor's frame is +x forward, +y left, +z up
					//  flip y & z to publish right handed with z down (x forward, y right, z down)
					const int16_t accel_y = (data.accel_y == INT16_MIN) ? INT16_MAX : -data.accel_y;
					const int16_t accel_z = (data.accel_z == INT16_MIN) ? INT16_MAX : -data.accel_z;
					const int16_t gyro_y = (data.gyro_y == INT16_MIN) ? INT16_MAX : -data.gyro_y;
					const int16_t gyro_z = (data.gyro_z == INT16_MIN) ? INT16_MAX : -data.gyro_z;

					_px4_accel.update(timestamp_sample, data.accel_x, accel_y, accel_z);
					_px4_gyro.update(timestamp_sample, data.gyro_x, gyro_y, gyro_z);

					// escalate the accel full-scale range if any axis is clipping
					if (_accel_range != ACCEL_RANGE::RANGE_64G) {
						const int16_t values[3] {data.accel_x, data.accel_y, data.accel_z};

						for (auto v : values) {
							if (v >= ACCEL_CLIP_THRESHOLD || v <= -ACCEL_CLIP_THRESHOLD) {
								EscalateAccelRange();
								break;
							}
						}
					}
				}

				if (_failure_count > 0) {
					_failure_count--;
				}

			} else {
				perf_count(_bad_transfer_perf);
				_failure_count++;

				// full reset if things are failing consistently
				if (_failure_count > 10) {
					Reset();
					return;
				}
			}

			// check configuration registers periodically
			if (hrt_elapsed_time(&_last_config_check_timestamp) > 100_ms) {
				if (RegisterCheck(_register_bank0_cfg[_checked_register_bank0])
				    && RegisterCheck(_register_bank6_cfg[0])
				    && RegisterCheck(_register_bank7_cfg[0])
				   ) {
					_last_config_check_timestamp = now;
					_checked_register_bank0 = (_checked_register_bank0 + 1) % size_register_bank0_cfg;

				} else {
					// register check failed, force reset
					perf_count(_bad_register_perf);
					Reset();
					return;
				}

				// the low resolution accel data registers are only accessible from bank 0
				SelectRegisterBank(0);
			}
		}

		break;
	}
}

bool IIM20670::Configure()
{
	// unlock the full-scale change operation and the ODR pin routing (also unlocks bank_select)
	for (auto frame : UNLOCK_SEQUENCE) {
		TransferSpiFrame(frame);
	}

	// verify identity now that bank 1 is accessible
	const uint8_t whoami = RegisterRead(Register::BANK_1::WHOAMI) & 0xFF;

	if (whoami != WHOAMI) {
		PX4_DEBUG("unexpected WHO_AM_I 0x%02x", whoami);
		return false;
	}

	// start over from the smallest accel range every time the sensor is configured
	_register_bank6_cfg[0].set_bits = ACCEL_FS_SEL_16G_SET;
	_register_bank6_cfg[0].clear_bits = ACCEL_FS_SEL_16G_CLEAR;

	// first set and clear all configured register bits
	for (const auto &reg_cfg : _register_bank0_cfg) {
		if (reg_cfg.reg != Register::BANK_0::FIXED_VALUE_REG) {
			RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
		}
	}

	for (const auto &reg_cfg : _register_bank6_cfg) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
	}

	for (const auto &reg_cfg : _register_bank7_cfg) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
	}

	// route the 8 kHz internal ODR clock to pin 12 for data ready synchronization
	if (_drdy_gpio != 0) {
		ConfigureOdrPin();
	}

	// now check that all are configured
	bool success = true;

	for (const auto &reg_cfg : _register_bank0_cfg) {
		if (!RegisterCheck(reg_cfg)) {
			success = false;
		}
	}

	for (const auto &reg_cfg : _register_bank6_cfg) {
		if (!RegisterCheck(reg_cfg)) {
			success = false;
		}
	}

	for (const auto &reg_cfg : _register_bank7_cfg) {
		if (!RegisterCheck(reg_cfg)) {
			success = false;
		}
	}

	// leave the sensor in bank 0 for sensor data reads
	SelectRegisterBank(0);

	ConfigureAccelRange(ACCEL_RANGE::RANGE_16G);

	// gyro_fs_sel = 0011: ±1966 dps (16.67 LSB/dps)
	_px4_gyro.set_range(math::radians(1966.08f));
	_px4_gyro.set_scale(math::radians(1966.08f / 32768.f));

	return success;
}

void IIM20670::ConfigureAccelRange(ACCEL_RANGE range)
{
	_accel_range = range;

	switch (range) {
	case ACCEL_RANGE::RANGE_16G:
		// accel_fs_sel = 001 high resolution output: ±16.384 g (2000 LSB/g)
		_px4_accel.set_range(16.384f * CONSTANTS_ONE_G);
		_px4_accel.set_scale(CONSTANTS_ONE_G / 2000.f);
		break;

	case ACCEL_RANGE::RANGE_32G:
		// accel_fs_sel = 011 high resolution output: ±32.768 g (1000 LSB/g)
		_px4_accel.set_range(32.768f * CONSTANTS_ONE_G);
		_px4_accel.set_scale(CONSTANTS_ONE_G / 1000.f);
		break;

	case ACCEL_RANGE::RANGE_64G:
		// accel_fs_sel = 011 low resolution output: ±65.536 g (500 LSB/g)
		_px4_accel.set_range(65.536f * CONSTANTS_ONE_G);
		_px4_accel.set_scale(CONSTANTS_ONE_G / 500.f);
		break;
	}
}

void IIM20670::EscalateAccelRange()
{
	if (_accel_range == ACCEL_RANGE::RANGE_16G) {
		// switch the high resolution output from ±16.384 g to ±32.768 g
		_register_bank6_cfg[0].set_bits = ACCEL_FS_SEL_32G_SET;
		_register_bank6_cfg[0].clear_bits = ACCEL_FS_SEL_32G_CLEAR;
		RegisterSetAndClearBits(Register::BANK_6::SENSITIVITY_CONFIG, ACCEL_FS_SEL_32G_SET, ACCEL_FS_SEL_32G_CLEAR);
		SelectRegisterBank(0);
		ConfigureAccelRange(ACCEL_RANGE::RANGE_32G);
		PX4_INFO("accel clipping, range escalated to ±32 g");

	} else if (_accel_range == ACCEL_RANGE::RANGE_32G) {
		// switch to the low resolution output registers (±65.536 g), no register change required
		ConfigureAccelRange(ACCEL_RANGE::RANGE_64G);
		PX4_INFO("accel clipping, range escalated to ±64 g");
	}
}

void IIM20670::ConfigureOdrPin()
{
	// route a copy of the internal 8 kHz ODR clock to pin 12 (datasheet section 4.11)
	RegisterSetBits(Register::BANK_3::ODR_CONFIG_3, Bit9);
	RegisterSetBits(Register::BANK_3::ODR_CONFIG_6, Bit12);
	RegisterSetAndClearBits(Register::BANK_3::ODR_CONFIG_1, 0x21 << 8, 0x3F << 8); // bits 13:8 = 0x21
	RegisterSetAndClearBits(Register::BANK_3::ODR_CONFIG_2, 0x08 << 4, 0x0F << 4); // bits 7:4 = 0x08
	RegisterSetBits(Register::BANK_3::ODR_CONFIG_3, Bit5);
	RegisterSetBits(Register::BANK_3::ODR_CONFIG_5, Bit0);
}

bool IIM20670::ReadData(SensorData *data)
{
	// register reads are out-of-frame: each response is clocked out during the following transfer
	const bool low_res = (_accel_range == ACCEL_RANGE::RANGE_64G);
	const uint8_t accel_x_reg = static_cast<uint8_t>(low_res ? Register::BANK_0::ACCEL_X_DATA_LR : Register::BANK_0::ACCEL_X_DATA);
	const uint8_t accel_y_reg = static_cast<uint8_t>(low_res ? Register::BANK_0::ACCEL_Y_DATA_LR : Register::BANK_0::ACCEL_Y_DATA);
	const uint8_t accel_z_reg = static_cast<uint8_t>(low_res ? Register::BANK_0::ACCEL_Z_DATA_LR : Register::BANK_0::ACCEL_Z_DATA);

	TransferSpiFrame(ReadCommand(static_cast<uint8_t>(Register::BANK_0::GYRO_X_DATA)));
	const uint32_t gyro_x  = TransferSpiFrame(ReadCommand(static_cast<uint8_t>(Register::BANK_0::GYRO_Y_DATA)));
	const uint32_t gyro_y  = TransferSpiFrame(ReadCommand(static_cast<uint8_t>(Register::BANK_0::GYRO_Z_DATA)));
	const uint32_t gyro_z  = TransferSpiFrame(ReadCommand(static_cast<uint8_t>(Register::BANK_0::TEMP1_DATA)));
	const uint32_t temp    = TransferSpiFrame(ReadCommand(accel_x_reg));
	const uint32_t accel_x = TransferSpiFrame(ReadCommand(accel_y_reg));
	const uint32_t accel_y = TransferSpiFrame(ReadCommand(accel_z_reg));
	const uint32_t accel_z = TransferSpiFrame(ReadCommand(static_cast<uint8_t>(Register::BANK_0::TEST)));

	const uint32_t responses[] {gyro_x, gyro_y, gyro_z, temp, accel_x, accel_y, accel_z};

	for (auto r : responses) {
		if (!CheckResponse(r)) {
			return false;
		}
	}

	data->gyro_x  = (int16_t)((gyro_x >> 8) & 0xFFFF);
	data->gyro_y  = (int16_t)((gyro_y >> 8) & 0xFFFF);
	data->gyro_z  = (int16_t)((gyro_z >> 8) & 0xFFFF);
	data->temp    = (int16_t)((temp >> 8) & 0xFFFF);
	data->accel_x = (int16_t)((accel_x >> 8) & 0xFFFF);
	data->accel_y = (int16_t)((accel_y >> 8) & 0xFFFF);
	data->accel_z = (int16_t)((accel_z >> 8) & 0xFFFF);

	return true;
}

void IIM20670::SelectRegisterBank(uint16_t bank, bool force)
{
	if (bank != _last_register_bank || force) {
		TransferSpiFrame(WriteCommand(static_cast<uint8_t>(Register::BANK_0::BANK_SELECT), bank));
		_last_register_bank = bank;
	}
}

template <typename T>
bool IIM20670::RegisterCheck(const T &reg_cfg)
{
	bool success = true;

	const uint16_t reg_value = RegisterRead(reg_cfg.reg);

	if (reg_cfg.set_bits && ((reg_value & reg_cfg.set_bits) != reg_cfg.set_bits)) {
		PX4_DEBUG("0x%02hhX: 0x%04hX (0x%04hX not set)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.set_bits);
		success = false;
	}

	if (reg_cfg.clear_bits && ((reg_value & reg_cfg.clear_bits) != 0)) {
		PX4_DEBUG("0x%02hhX: 0x%04hX (0x%04hX not cleared)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.clear_bits);
		success = false;
	}

	return success;
}

template <typename T>
uint16_t IIM20670::RegisterRead(T reg)
{
	SelectRegisterBank(reg);

	const uint32_t cmd = ReadCommand(static_cast<uint8_t>(reg));

	// the response is returned during the following transfer
	TransferSpiFrame(cmd);
	const uint32_t response = TransferSpiFrame(cmd);

	return (response >> 8) & 0xFFFF;
}

template <typename T>
void IIM20670::RegisterWrite(T reg, uint16_t value)
{
	SelectRegisterBank(reg);
	TransferSpiFrame(WriteCommand(static_cast<uint8_t>(reg), value));
}

template <typename T>
void IIM20670::RegisterSetAndClearBits(T reg, uint16_t setbits, uint16_t clearbits)
{
	const uint16_t orig_val = RegisterRead(reg);

	uint16_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}

uint8_t IIM20670::CalculateCRC8(uint32_t word24)
{
	// CRC-8 (x^8 + x^4 + x^3 + x^2 + 1), seed 0xFF, inverted before appending (datasheet section 5.2)
	uint8_t crc = 0xFF;

	for (int i = 23; i >= 0; i--) {
		const uint8_t bit = (word24 >> i) & 1;
		const uint8_t fb = (crc >> 7) & 1;
		crc <<= 1;

		if (fb) {
			crc ^= 0x1D;
		}

		crc ^= bit;
	}

	return crc ^ 0xFF;
}

uint32_t IIM20670::ReadCommand(uint8_t addr)
{
	uint32_t cmd = (uint32_t)(addr & 0x1F) << 26;
	cmd |= CalculateCRC8(cmd >> 8);
	return cmd;
}

uint32_t IIM20670::WriteCommand(uint8_t addr, uint16_t value)
{
	uint32_t cmd = DIR_WRITE | ((uint32_t)(addr & 0x1F) << 26) | ((uint32_t)value << 8);
	cmd |= CalculateCRC8(cmd >> 8);
	return cmd;
}

bool IIM20670::CheckResponse(uint32_t response)
{
	if (CalculateCRC8(response >> 8) != (response & 0xFF)) {
		perf_count(_bad_crc_perf);
		return false;
	}

	const uint8_t return_status = (response >> 24) & 0x3;

	return return_status == RS_SUCCESS;
}

uint32_t IIM20670::TransferSpiFrame(uint32_t frame)
{
	uint8_t buf[4];

	buf[0] = (frame >> 24) & 0xFF;
	buf[1] = (frame >> 16) & 0xFF;
	buf[2] = (frame >> 8) & 0xFF;
	buf[3] = frame & 0xFF;

	transfer(buf, buf, sizeof(buf));

	return ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8) | buf[3];
}

int IIM20670::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<IIM20670 *>(arg)->DataReady();
	return 0;
}

void IIM20670::DataReady()
{
	_drdy_timestamp_sample.store(hrt_absolute_time());
	ScheduleNow();
}

bool IIM20670::DataReadyInterruptConfigure()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	// Setup data ready on rising edge (sensor data request window opens 5 us after the ODR rising edge)
	return px4_arch_gpiosetevent(_drdy_gpio, true, false, false, &DataReadyInterruptCallback, this) == 0;
}

bool IIM20670::DataReadyInterruptDisable()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	return px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0;
}
