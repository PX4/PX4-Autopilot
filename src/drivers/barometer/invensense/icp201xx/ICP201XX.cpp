/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include "ICP201XX.hpp"
#include <unistd.h>

using namespace time_literals;

ICP201XX::ICP201XX(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config)
{
}

ICP201XX::~ICP201XX()
{
	perf_free(_reset_perf);
	perf_free(_sample_perf);
	perf_free(_bad_transfer_perf);
}

int
ICP201XX::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool
ICP201XX::Reset()
{
	_state = STATE::SOFT_RESET;
	ScheduleClear();
	ScheduleNow();
	return true;
}

void
ICP201XX::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_reset_perf);
	perf_print_counter(_sample_perf);
	perf_print_counter(_bad_transfer_perf);
}

int
ICP201XX::probe()
{
	uint8_t device_id = 0;
	uint8_t ver = 0xFF;
	read_reg(Register::DEVICE_ID, &device_id);
	read_reg(Register::VERSION, &ver);

	if (device_id != EXPECTED_DEVICE_ID) {
		DEVICE_DEBUG("unexpected device id 0x%02x", device_id);
		return PX4_ERROR;
	}

	if (ver != 0x00 && ver != 0xB2) {
		DEVICE_DEBUG("unexpected version 0x%02x", ver);
		return PX4_ERROR;
	}

	return PX4_OK;
}

void
ICP201XX::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::SOFT_RESET: {
			/* Stop the measurement */
			mode_select(0x00);

			ScheduleDelayed(2_ms);

			/* Flush FIFO */
			flush_fifo();

			/* Mask all interrupts */
			write_reg(Register::FIFO_CONFIG, 0x00);
			write_reg(Register::INTERRUPT_MASK, 0xFF);

			_reset_timestamp = now;
			_failure_count = 0;
			perf_count(_reset_perf);
			_state = STATE::OTP_BOOTUP_CFG;
			ScheduleDelayed(10_ms);
		}
		break;

	case STATE::OTP_BOOTUP_CFG: {
			uint8_t reg_value = 0;
			uint8_t offset = 0, gain = 0, Hfosc = 0;
			uint8_t version = 0;
			uint8_t bootup_status = 0;
			int ret = 0;

			/*  read version register */
			if (read_reg(Register::VERSION, &version) != PX4_OK) {
				ScheduleDelayed(10_ms);
				break;
			}

			if (version == 0xB2) {
				/* B2 version Asic is detected. Boot up sequence is not required for B2 Asic, so returning */
				_state = STATE::CONFIG;
				ScheduleDelayed(10_ms);
			}

			/* Read boot up status and avoid re running boot up sequence if it is already done */
			if (read_reg(Register::OTP_MTP_OTP_STATUS2, &bootup_status) != PX4_OK) {
				ScheduleDelayed(10_ms);
				break;
			}

			if (bootup_status & 0x01) {
				/* Boot up sequence is already done, not required to repeat boot up sequence */
				_state = STATE::CONFIG;
				ScheduleDelayed(10_ms);
				break;
			}

			/* Bring the ASIC in power mode to activate the OTP power domain and get access to the main registers */
			mode_select(0x04);
			usleep(4_ms);

			/* Unlock the main registers */
			write_reg(Register::MASTER_LOCK, 0x1F);

			/* Enable the OTP and the write switch */
			read_reg(Register::OTP_MTP_OTP_CFG1, &reg_value);
			reg_value |= 0x03;
			write_reg(Register::OTP_MTP_OTP_CFG1, reg_value);
			usleep(10_us);

			/* Toggle the OTP reset pin */
			read_reg(Register::OTP_DEBUG2, &reg_value);
			reg_value |= 1 << 7;
			write_reg(Register::OTP_DEBUG2, reg_value);
			usleep(10_us);

			read_reg(Register::OTP_DEBUG2, &reg_value);
			reg_value &= ~(1 << 7);
			write_reg(Register::OTP_DEBUG2, reg_value);
			usleep(10_us);

			/* Program redundant read */
			write_reg(Register::OTP_MTP_MRA_LSB, 0x04);
			write_reg(Register::OTP_MTP_MRA_MSB, 0x04);
			write_reg(Register::OTP_MTP_MRB_LSB, 0x21);
			write_reg(Register::OTP_MTP_MRB_MSB, 0x20);
			write_reg(Register::OTP_MTP_MR_LSB, 0x10);
			write_reg(Register::OTP_MTP_MR_MSB, 0x80);

			/* Read the data from register */
			ret |= read_otp_data(0xF8, 0x10, &offset);
			ret |= read_otp_data(0xF9, 0x10, &gain);
			ret |= read_otp_data(0xFA, 0x10, &Hfosc);
			ScheduleDelayed(10_us);

			/* Write OTP values to main registers */
			ret |= read_reg(Register::TRIM1_MSB, &reg_value);

			if (ret == 0) {
				reg_value = (reg_value & (~0x3F)) | (offset & 0x3F);
				ret |= write_reg(Register::TRIM1_MSB, reg_value);
			}

			ret |= read_reg(Register::TRIM2_MSB, &reg_value);

			if (ret == 0) {
				reg_value = (reg_value & (~0x70)) | ((gain & 0x07) << 4);
				ret |= write_reg(Register::TRIM2_MSB, reg_value);
			}

			ret |= read_reg(Register::TRIM2_LSB, &reg_value);

			if (ret == 0) {
				reg_value = (reg_value & (~0x7F)) | (Hfosc & 0x7F);
				ret |= write_reg(Register::TRIM2_LSB, reg_value);
			}

			ScheduleDelayed(10_us);

			/* Update boot up status to 1 */
			if (ret == 0) {
				ret |= read_reg(Register::OTP_MTP_OTP_STATUS2, &reg_value);

				if (ret == 0) {
					reg_value |= 0x01;
					ret |= write_reg(Register::OTP_MTP_OTP_STATUS2, reg_value);
				}
			}

			/* Disable OTP and write switch */
			read_reg(Register::OTP_MTP_OTP_CFG1, &reg_value);
			reg_value &= ~0x03;
			write_reg(Register::OTP_MTP_OTP_CFG1, reg_value);

			/* Lock the main register */
			write_reg(Register::MASTER_LOCK, 0x00);

			/* Move to standby */
			mode_select(0x00);

			ScheduleDelayed(10_ms);
		}
		break;

	case STATE::CONFIG: {
			if (configure()) {
				_state = STATE::WAIT_READ;
				ScheduleDelayed(10_ms);

			} else {
				if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
					PX4_WARN("Configure failed, resetting");
					_state = STATE::SOFT_RESET;

				} else {
					PX4_WARN("Configure failed, retrying");
				}

				ScheduleDelayed(100_ms);
			}
		}
		break;

	case STATE::WAIT_READ: {
			/*
			* If FIR filter is enabled, it will cause a settling effect on the first 14 pressure values.
			* Therefore the first 14 pressure output values are discarded.
			**/
			uint8_t fifo_packets = 0;
			uint8_t fifo_packets_to_skip = 14;

			do {
				ScheduleDelayed(10_ms);
				read_reg(Register::FIFO_FILL, &fifo_packets);
				fifo_packets = (uint8_t)(fifo_packets & 0x1F);
			} while (fifo_packets >= fifo_packets_to_skip);

			flush_fifo();
			fifo_packets = 0;

			do {
				ScheduleDelayed(10_ms);
				read_reg(Register::FIFO_FILL, &fifo_packets);
				fifo_packets = (uint8_t)(fifo_packets & 0x1F);
			} while (fifo_packets == 0);

			_state = STATE::READ;
			perf_begin(_sample_perf);
			ScheduleOnInterval(1_s / 30);
		}
		break;

	case STATE::READ: {
			bool success = false;
			float pressure, temperature;

			if (get_sensor_data(&pressure, &temperature)) {
				perf_end(_sample_perf);
				perf_begin(_sample_perf);

				sensor_baro_s sensor_baro{};
				sensor_baro.timestamp_sample = now;
				sensor_baro.device_id = get_device_id();
				sensor_baro.pressure = pressure;
				sensor_baro.temperature = temperature;
				sensor_baro.error_count = perf_event_count(_bad_transfer_perf);
				sensor_baro.timestamp = hrt_absolute_time();
				_sensor_baro_pub.publish(sensor_baro);

				success = true;

				if (_failure_count > 0) {
					_failure_count--;
				}

			} else {
				perf_count(_bad_transfer_perf);
			}

			if (!success) {
				_failure_count++;

				if (_failure_count > 10) {
					Reset();
					return;
				}
			}
		}
		break;
	}
}

void
ICP201XX::dummy_reg()
{
	do {
		uint8_t reg = (uint8_t)Register::EMPTY;
		uint8_t val = 0;
		transfer((uint8_t *)&reg, 1, &val, 1);
	} while (0);
}

int
ICP201XX::read_reg(Register reg, uint8_t *buf, uint8_t len)
{
	int ret;
	ret = transfer((uint8_t *)&reg, 1, buf, len);
	dummy_reg();
	return ret;
}

int
ICP201XX::read_reg(Register reg, uint8_t *val)
{
	return read_reg(reg, val, 1);
}

int
ICP201XX::write_reg(Register reg, uint8_t val)
{
	uint8_t data[2] = { (uint8_t)reg, val };
	int ret;
	ret = transfer(data, sizeof(data), nullptr, 0);
	dummy_reg();
	return ret;
}

int
ICP201XX::mode_select(uint8_t mode)
{
	uint8_t mode_sync_status = 0;

	do {
		read_reg(Register::DEVICE_STATUS, &mode_sync_status, 1);

		if (mode_sync_status & 0x01) {
			break;
		}

		ScheduleDelayed(500_us);
	} while (1);

	if (write_reg(Register::MODE_SELECT, mode) != PX4_OK) {
		return PX4_ERROR;
	}

	return PX4_OK;
}

int
ICP201XX::read_otp_data(uint8_t addr, uint8_t cmd, uint8_t *val)
{
	uint8_t otp_status = 0xFF;

	/* Write the address content and read command */
	if (write_reg(Register::OTP_MTP_OTP_ADDR, addr) != PX4_OK) {
		return PX4_ERROR;
	}

	if (write_reg(Register::OTP_MTP_OTP_CMD, cmd) != PX4_OK) {
		return PX4_ERROR;
	}

	/* Wait for the OTP read to finish Monitor otp_status */
	do 	{
		read_reg(Register::OTP_MTP_OTP_STATUS, &otp_status);

		if (otp_status == 0) {
			break;
		}

		ScheduleDelayed(1_us);
	} while (1);

	/* Read the data from register */
	if (read_reg(Register::OTP_MTP_RD_DATA, val) != PX4_OK) {
		return PX4_ERROR;
	}

	return PX4_OK;
}

bool
ICP201XX::get_sensor_data(float *pressure, float *temperature)
{
	bool success = false;
	uint8_t fifo_packets = 0;
	uint8_t fifo_data[96] {0};
	int32_t data_temp[16] {0};
	int32_t data_press[16] {0};

	if (read_reg(Register::FIFO_FILL, &fifo_packets) == PX4_OK) {
		fifo_packets  = (uint8_t)(fifo_packets & 0x1F);

		if (fifo_packets > 0 && fifo_packets <= 16 && !read_reg(Register::FIFO_BASE, fifo_data, fifo_packets * 2 * 3)) {
			uint8_t offset = 0;

			for (uint8_t i = 0; i < fifo_packets; i++) {
				data_press[i] = (int32_t)(((fifo_data[offset + 2] & 0x0f) << 16) | (fifo_data[offset + 1] << 8) | fifo_data[offset]) ;
				offset += 3;
				data_temp[i] = (int32_t)(((fifo_data[offset + 2] & 0x0f) << 16) | (fifo_data[offset + 1] << 8) | fifo_data[offset]) ;
				offset += 3;
			}

			*pressure = 0;
			*temperature = 0;

			for (uint8_t i = 0; i < fifo_packets; i++) {
				/** P = (POUT/2^17)*40kPa + 70kPa **/
				if (data_press[i] & 0x080000) {
					data_press[i] |= 0xFFF00000;
				}

				*pressure += ((float)(data_press[i]) * 40 / 131072) + 70;

				/* T = (TOUT/2^18)*65C + 25C */
				if (data_temp[i] & 0x080000) {
					data_temp[i] |= 0xFFF00000;
				}

				*temperature += ((float)(data_temp[i]) * 65 / 262144) + 25;
			}

			*pressure = *pressure * 1000 / fifo_packets;
			*temperature = *temperature / fifo_packets;
			success = true;
		}
	}

	return success;
}

bool
ICP201XX::configure()
{
	uint8_t reg_value = 0;

	/* Initiate Triggered Operation: Stay in Standby mode */
	reg_value |= (reg_value & (~0x10)) | ((uint8_t)_forced_meas_trigger << 4);

	/* Power Mode Selection: Normal Mode */
	reg_value |= (reg_value & (~0x04)) | ((uint8_t)_power_mode << 2);

	/* FIFO Readout Mode Selection: Pressure first. */
	reg_value |= (reg_value & (~0x03)) | ((uint8_t)(_fifo_readout_mode));

	/* Measurement Configuration: Mode2*/
	reg_value |= (reg_value & (~0xE0)) | (((uint8_t)_op_mode) << 5);

	/* Measurement Mode Selection: Continuous Measurements (duty cycled) */
	reg_value |= (reg_value & (~0x08)) | ((uint8_t)_meas_mode << 3);

	if (mode_select(reg_value) != PX4_OK) {
		return false;
	}

	return true;
}

bool
ICP201XX::flush_fifo()
{
	uint8_t reg_value;

	if (read_reg(Register::FIFO_FILL, &reg_value) != PX4_OK) {
		return false;
	}

	reg_value |= 0x80;

	if (write_reg(Register::FIFO_FILL, reg_value) != PX4_OK) {
		return false;
	}

	return true;
}
