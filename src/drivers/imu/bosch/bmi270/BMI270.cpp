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
#include "BMI270.hpp"
#define BMI270_DEBUG
#define BMI270_MAX_FIFO_SAMPLES 8

using namespace time_literals;

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

// Microcode that enables max fifo
uint8_t maximum_fifo_config_file[] = { 0x5E,
    0xc8, 0x2e, 0x00, 0x2e, 0x80, 0x2e, 0x1a, 0x00, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00,
    0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0x90, 0x32, 0x21, 0x2e, 0x59, 0xf5,
    0x10, 0x30, 0x21, 0x2e, 0x6a, 0xf5, 0x1a, 0x24, 0x22, 0x00, 0x80, 0x2e, 0x3b, 0x00, 0xc8, 0x2e, 0x44, 0x47, 0x22,
    0x00, 0x37, 0x00, 0xa4, 0x00, 0xff, 0x0f, 0xd1, 0x00, 0x07, 0xad, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
    0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00,
    0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x11, 0x24, 0xfc, 0xf5, 0x80, 0x30, 0x40, 0x42, 0x50, 0x50, 0x00, 0x30, 0x12, 0x24, 0xeb,
    0x00, 0x03, 0x30, 0x00, 0x2e, 0xc1, 0x86, 0x5a, 0x0e, 0xfb, 0x2f, 0x21, 0x2e, 0xfc, 0xf5, 0x13, 0x24, 0x63, 0xf5,
    0xe0, 0x3c, 0x48, 0x00, 0x22, 0x30, 0xf7, 0x80, 0xc2, 0x42, 0xe1, 0x7f, 0x3a, 0x25, 0xfc, 0x86, 0xf0, 0x7f, 0x41,
    0x33, 0x98, 0x2e, 0xc2, 0xc4, 0xd6, 0x6f, 0xf1, 0x30, 0xf1, 0x08, 0xc4, 0x6f, 0x11, 0x24, 0xff, 0x03, 0x12, 0x24,
    0x00, 0xfc, 0x61, 0x09, 0xa2, 0x08, 0x36, 0xbe, 0x2a, 0xb9, 0x13, 0x24, 0x38, 0x00, 0x64, 0xbb, 0xd1, 0xbe, 0x94,
    0x0a, 0x71, 0x08, 0xd5, 0x42, 0x21, 0xbd, 0x91, 0xbc, 0xd2, 0x42, 0xc1, 0x42, 0x00, 0xb2, 0xfe, 0x82, 0x05, 0x2f,
    0x50, 0x30, 0x21, 0x2e, 0x21, 0xf2, 0x00, 0x2e, 0x00, 0x2e, 0xd0, 0x2e, 0xf0, 0x6f, 0x02, 0x30, 0x02, 0x42, 0x20,
    0x26, 0xe0, 0x6f, 0x02, 0x31, 0x03, 0x40, 0x9a, 0x0a, 0x02, 0x42, 0xf0, 0x37, 0x05, 0x2e, 0x5e, 0xf7, 0x10, 0x08,
    0x12, 0x24, 0x1e, 0xf2, 0x80, 0x42, 0x83, 0x84, 0xf1, 0x7f, 0x0a, 0x25, 0x13, 0x30, 0x83, 0x42, 0x3b, 0x82, 0xf0,
    0x6f, 0x00, 0x2e, 0x00, 0x2e, 0xd0, 0x2e, 0x12, 0x40, 0x52, 0x42, 0x00, 0x2e, 0x12, 0x40, 0x52, 0x42, 0x3e, 0x84,
    0x00, 0x40, 0x40, 0x42, 0x7e, 0x82, 0xe1, 0x7f, 0xf2, 0x7f, 0x98, 0x2e, 0x6a, 0xd6, 0x21, 0x30, 0x23, 0x2e, 0x61,
    0xf5, 0xeb, 0x2c, 0xe1, 0x6f
};



BMI270::BMI270(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_drdy_gpio(config.drdy_gpio),
	_px4_accel(get_device_id(), config.rotation),
	_px4_gyro(get_device_id(), config.rotation)
{
	if (_drdy_gpio != 0) {
		_drdy_missed_perf = perf_alloc(PC_COUNT, MODULE_NAME": DRDY missed");
	}

	ConfigureSampleRate(_px4_gyro.get_max_rate_hz());
}

BMI270::~BMI270()
{
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_fifo_empty_perf);
	perf_free(_fifo_overflow_perf);
	perf_free(_fifo_reset_perf);
	perf_free(_drdy_missed_perf);
}

int BMI270::init()
{
	int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);
		return ret;
	}

	PX4_DEBUG("init function called, resetting...");


	return Reset() ? 0 : -1;
}

bool BMI270::Reset()
{
	_state = STATE::RESET;
	DataReadyInterruptDisable();
	ScheduleClear();
	ScheduleNow();
	return true;
}

// Debug function that Ardupilot is equipped with
void BMI270::CheckErrorRegister()
{
	#ifdef BMI270_DEBUG
	uint8_t err = RegisterRead(Register::ERR_REG);

	if (err) {
		if ((err & 1) == 1) {
		uint8_t status =  RegisterRead(Register::INTERNAL_STATUS);
		switch (status & 0xF) {
		case 0:
			PX4_DEBUG("BMI270: not_init");
			break;
		case 2:
			PX4_DEBUG("BMI270: init_err");
			break;
		case 3:
			PX4_DEBUG("BMI270: drv_err");
			break;
		case 4:
			PX4_DEBUG("BMI270: sns_stop");
			break;
		case 5:
			PX4_DEBUG("BMI270: nvm_error");
			break;
		case 6:
			PX4_DEBUG("BMI270: start_up_error");
			break;
		case 7:
			PX4_DEBUG("BMI270: compat_error");
			break;
		case 1: // init ok
			if ((status>>5 & 1) == 1) {
			PX4_DEBUG("BMI270: axes_remap_error");
			} else if ((status>>6 & 1) == 1) {
			PX4_DEBUG("BMI270: odr_50hz_error");
			}
			break;
		}
		} else if ((err>>6 & 1) == 1) {
		PX4_DEBUG("BMI270: fifo_err");
		} else if ((err>>7 & 1) == 1) {
		PX4_DEBUG("BMI270: aux_err");
		} else {
		PX4_DEBUG("BMI270: internal error detected %d", err>>1 & 0xF);
		}
	}
	#endif

}

void BMI270::exit_and_cleanup()
{
	DataReadyInterruptDisable();
	I2CSPIDriverBase::exit_and_cleanup();
}

void BMI270::print_status()
{
	I2CSPIDriverBase::print_status();

	PX4_INFO("FIFO empty interval: %d us (%.1f Hz)", _fifo_empty_interval_us, 1e6 / _fifo_empty_interval_us);

	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_fifo_empty_perf);
	perf_print_counter(_fifo_overflow_perf);
	perf_print_counter(_fifo_reset_perf);
	perf_print_counter(_drdy_missed_perf);
}

int BMI270::probe()
{
	// Interface selection (SPI only): Read an arbitrary register of the device, discard the read response
	RegisterRead(Register::CHIP_ID);

	const uint8_t CHIP_ID = RegisterRead(Register::CHIP_ID);

	if (CHIP_ID != chip_id) {
		DEVICE_DEBUG("unexpected CHIP_ID 0x%02x", CHIP_ID);
		return PX4_ERROR;
	}

	return PX4_OK;
}

void BMI270::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET:

		// 0xB6 is written to the CMD register for a soft reset
		RegisterWrite(Register::CMD, 0xB6);
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;
		ScheduleDelayed(1_ms); // Following a delay of 1 ms, all configuration settings are overwritten with their reset value.
		break;

	case STATE::WAIT_FOR_RESET:

		/*
		Hardware initialization steps according to datasheet:
		1. Disable PWR_CONF.adv_power_save and wait for 450us
		2. Write 0x00 to INIT_CTRL
		3. Burst write initialization file to INIT_DATA
		4. Write 0x01 to INIT_CTRL
		5. Wait 150ms and read register INTERNAL_STATUS for value 0b001
		6. If step 5 passed, enter configure state


		*/
		if ((RegisterRead(Register::CHIP_ID) == chip_id) ) {
			PX4_DEBUG("Read from CHIP_ID register and the IDs match");

			// 1. Disable PWR_CONF.adv_power_save and wait for 450us

			RegisterWrite(Register::PWR_CONF, 0x00);
			px4_udelay(450);

			// 2. Write 0x00 to INIT_CTRL

			RegisterWrite(Register::CONFIG1, 0x00);
			// give it the maximum FIFO config file
			PX4_DEBUG("attempting to upload initialization file onto BMI270");

			//  3. Burst write initialization file to INIT_DATA

			int res = transfer(maximum_fifo_config_file, nullptr, sizeof(maximum_fifo_config_file));
			if (res == PX4_OK) {
				RegisterWrite(Register::CONFIG1, 1);
				PX4_DEBUG("Successfully uploaded initialization file onto BMI270");
				px4_mdelay(150);
				PX4_DEBUG("Preparing to read INTERNAL_STATUS register");

			} else {
				PX4_DEBUG("Failed to upload initialization file onto BMI270, resetting");
				_state = STATE::RESET;
				ScheduleDelayed(10_ms);
			}

			uint8_t internal_status = RegisterRead(Register::INTERNAL_STATUS);
			PX4_DEBUG("Internal status register value: 0x%02hhX", internal_status);

			if ((internal_status & 1) == 1)
			{
				PX4_DEBUG("INTERNAL_STATUS 0x01, ready for configure");
				// if reset succeeded then configure
				_state = STATE::CONFIGURE;
				ScheduleDelayed(10_ms);
			} else {

				PX4_DEBUG("INTERNAL_STATUS check failed, resetting");
				_state = STATE::RESET;
				ScheduleDelayed(10_ms);

			};

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

		PX4_DEBUG("IMU now in configure state");
		if (Configure()) {

			// if configure succeeded then start reading from FIFO

			if (DataReadyInterruptConfigure()) {
				_data_ready_interrupt_enabled = true;

				// backup schedule as a watchdog timeout
				ScheduleDelayed(100_ms);

			} else {
				_data_ready_interrupt_enabled = false;
				ScheduleOnInterval(_fifo_empty_interval_us, _fifo_empty_interval_us);
			}

			FIFOReset();
			_state = STATE::FIFO_READ;


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
	case STATE::FIFO_READ: {

			PX4_DEBUG("reading from FIFO");

			hrt_abstime timestamp_sample = now;

			if (_data_ready_interrupt_enabled) {
				PX4_DEBUG("data ready interrupt enabled");
				// scheduled from interrupt if _drdy_timestamp_sample was set as expected
				const hrt_abstime drdy_timestamp_sample = _drdy_timestamp_sample.fetch_and(0);

				if ((now - drdy_timestamp_sample) < _fifo_empty_interval_us) {
					timestamp_sample = drdy_timestamp_sample;

				} else {
					perf_count(_drdy_missed_perf);
				}

				// push backup schedule back
				ScheduleDelayed(_fifo_empty_interval_us * 2);
			}



			bool success = false;
			const uint16_t fifo_count = FIFOReadCount();

			if (!success) {
				_failure_count++;

				// full reset if things are failing consistently
				if (_failure_count > 10) {
					PX4_DEBUG("failure count > 10, resetting...");
					Reset();
					return;
				}
			}

			// more bytes than what the buffer takes so an overflow
			if (fifo_count >= FIFO::SIZE) {
				FIFOReset();
				perf_count(_fifo_overflow_perf);
			} else if (fifo_count == 0) {
				perf_count(_fifo_empty_perf);
			} else {

				uint8_t samples = fifo_count / sizeof(FIFO::DATA);

				// tolerate minor jitter, leave sample to next iteration if behind by only 1
				if (samples == _fifo_gyro_samples + 1) {
					timestamp_sample -= static_cast<int>(FIFO_SAMPLE_DT);
					samples--;
				}

				if (samples > FIFO_MAX_SAMPLES) {
					// not technically an overflow, but more samples than we expected or can publish
					FIFOReset();
					perf_count(_fifo_overflow_perf);

				} else if (samples >= _fifo_gyro_samples) {
					if (FIFORead(timestamp_sample, fifo_count)) {
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
					PX4_DEBUG("failure count > 10, resetting...");
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
					PX4_DEBUG("register check failed, resetting...");

					// register check failed, force reset
					perf_count(_bad_register_perf);
					// Reset();
				}

			} else {
				// periodically update temperature (~1 Hz)
				if (hrt_elapsed_time(&_temperature_update_timestamp) >= 1_s) {
					UpdateTemperature();
					_temperature_update_timestamp = now;
				}
			}

		}

		break;
	}
}

void BMI270::SetAccelScaleAndRange()
{
	const uint8_t ACC_RANGE = RegisterRead(Register::ACC_RANGE) & (Bit1 | Bit0);

	switch (ACC_RANGE) {
	case acc_range_3g:
		_px4_accel.set_scale(CONSTANTS_ONE_G * (powf(2, ACC_RANGE + 1) * 1.5f) / 32768.f);
		_px4_accel.set_range(3.f * CONSTANTS_ONE_G);
		break;

	case acc_range_6g:
		_px4_accel.set_scale(CONSTANTS_ONE_G * (powf(2, ACC_RANGE + 1) * 1.5f) / 32768.f);
		_px4_accel.set_range(6.f * CONSTANTS_ONE_G);
		break;

	case acc_range_12g:
		_px4_accel.set_scale(CONSTANTS_ONE_G * (powf(2, ACC_RANGE + 1) * 1.5f) / 32768.f);
		_px4_accel.set_range(12.f * CONSTANTS_ONE_G);
		break;

	case acc_range_24g:
		_px4_accel.set_scale(CONSTANTS_ONE_G * (powf(2, ACC_RANGE + 1) * 1.5f) / 32768.f);
		_px4_accel.set_range(24.f * CONSTANTS_ONE_G);
		break;
	}
}

void BMI270::SetGyroScale()
{
	// data is 16 bits with 2000dps range
    	const float scale = math::radians(2000.0f) / 32767.0f;
	_px4_gyro.set_scale(scale);

}

void BMI270::ConfigureSampleRate(int sample_rate)
{
	// round down to nearest FIFO sample dt * SAMPLES_PER_TRANSFER
	const float min_interval = FIFO_SAMPLE_DT;
	_fifo_empty_interval_us = math::max(roundf((1e6f / (float)sample_rate) / min_interval) * min_interval, min_interval);

	// works out to be 2 ...
	_fifo_gyro_samples = math::min((float)_fifo_empty_interval_us / (1e6f / RATE), (float)FIFO_MAX_SAMPLES);

	// recompute FIFO empty interval (us) with actual sample limit
	_fifo_empty_interval_us = _fifo_gyro_samples * (1e6f / RATE);

	ConfigureFIFOWatermark(_fifo_gyro_samples);
}


// when this register is set an interrupt is triggered when the FIFO reaches this many samples
void BMI270::ConfigureFIFOWatermark(uint8_t samples)
{
	// FIFO_WTM: 13 bit FIFO watermark level value
	// unit of the fifo watermark is one byte
	const uint16_t fifo_watermark_threshold = samples * sizeof(FIFO::DATA);

	for (auto &r : _register_cfg) {
		if (r.reg == Register::FIFO_WTM_0) {
			// fifo_water_mark[7:0]
			r.set_bits = fifo_watermark_threshold & 0x00FF;
			r.clear_bits = ~r.set_bits;

		} else if (r.reg == Register::FIFO_WTM_1) {
			// fifo_water_mark[12:8]
			r.set_bits = (fifo_watermark_threshold & 0x0700) >> 8;
			r.clear_bits = ~r.set_bits;
		}
	}
}

bool BMI270::Configure()
{

	bool success = false;


	// first set and clear all configured register bits
	for (const auto &reg_cfg : _register_cfg) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
	}

	// now check that all are configured

	for (const auto &reg_cfg : _register_cfg) {
		if (!RegisterCheck(reg_cfg)) {
			success = false;
		}
	}

	SetAccelScaleAndRange();
	SetGyroScale();

	success = true;
	return success;
}

int BMI270::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	PX4_DEBUG("actual data ready interrupt called");
	static_cast<BMI270 *>(arg)->DataReady();
	return 0;
}

void BMI270::DataReady()
{
	_drdy_timestamp_sample.store(hrt_absolute_time());
	ScheduleNow();
}

bool BMI270::DataReadyInterruptConfigure()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	// Setup data ready on falling edge
	return px4_arch_gpiosetevent(_drdy_gpio, false, true, true, &DataReadyInterruptCallback, this) == 0;
}

bool BMI270::DataReadyInterruptDisable()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	return px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0;
}

bool BMI270::RegisterCheck(const register_config_t &reg_cfg)
{
	bool success = true;

	const uint8_t reg_value = RegisterRead(reg_cfg.reg);

	PX4_DEBUG("0x%02hhX: 0x%02hhX", (uint8_t)reg_cfg.reg, reg_value);

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

uint8_t BMI270::RegisterRead(Register reg)
{
	// 6.1.2 SPI interface of accelerometer part
	//
	// In case of read operations of the accelerometer part, the requested data
	// is not sent immediately, but instead first a dummy byte is sent, and
	// after this dummy byte the actual requested register content is transmitted.
	uint8_t cmd[3] {};
	cmd[0] = static_cast<uint8_t>(reg) | DIR_READ;
	// cmd[1] dummy byte
	transfer(cmd, cmd, sizeof(cmd));
	return cmd[2];
}

void BMI270::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t cmd[2] { (uint8_t)reg, value };
	transfer(cmd, cmd, sizeof(cmd));
}

void BMI270::RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);

	uint8_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}


// Checks how many bytes are in the FIFO
uint16_t BMI270::FIFOReadCount()
{
	CheckErrorRegister();
	PX4_DEBUG("Attempting to determine FIFO fill level");
	// FIFO length registers FIFO_LENGTH_1 and FIFO_LENGTH_0 contain the 14 bit FIFO byte
	uint8_t fifo_len_buf[4] {};
	fifo_len_buf[0] = static_cast<uint8_t>(Register::FIFO_LENGTH_0) | DIR_READ;
	// fifo_len_buf[1] dummy byte


	if (transfer(&fifo_len_buf[0], &fifo_len_buf[0], sizeof(fifo_len_buf)) != PX4_OK) {
		PX4_DEBUG("Bad transfer");
		perf_count(_bad_transfer_perf);
		return 0;
	}


	const uint8_t FIFO_LENGTH_0 = fifo_len_buf[2];        // fifo_byte_counter[7:0]
	const uint8_t FIFO_LENGTH_1 = fifo_len_buf[3] & 0x3F; // fifo_byte_counter[13:8]

	uint16_t fifo_fill_level = combine(FIFO_LENGTH_1, FIFO_LENGTH_0);

	PX4_DEBUG("FIFO fill level: %d", fifo_fill_level);

	return fifo_fill_level;
}


// writes a gyro frame into the FIFO buffer the first argument points to
void BMI270::ProcessGyro(sensor_gyro_fifo_s *gyro, FIFO::GYRO_DATA *gyro_frame) {


	const uint8_t samples = gyro->samples;

	const int16_t gyro_x = combine(gyro_frame->GYR_X_MSB, gyro_frame->GYR_X_LSB);
	const int16_t gyro_y = combine(gyro_frame->GYR_Y_MSB, gyro_frame->GYR_Y_LSB);
	const int16_t gyro_z = combine(gyro_frame->GYR_Z_MSB, gyro_frame->GYR_Z_LSB);

	// Rotate from FLU to NED
	gyro->x[samples] = gyro_x;
	gyro->y[samples] = (gyro_y == INT16_MIN) ? INT16_MAX : -gyro_y;
	gyro->z[samples] = (gyro_z == INT16_MIN) ? INT16_MAX : -gyro_z;

	gyro->samples++;
}

// writes an accelerometer frame into the FIFO buffer the first argument points to
void BMI270::ProcessAccel(sensor_accel_fifo_s *accel, FIFO::ACCEL_DATA *accel_frame) {

	const uint8_t samples = accel->samples;

	const int16_t accel_x = combine(accel_frame->ACC_X_MSB, accel_frame->ACC_X_LSB);
	const int16_t accel_y = combine(accel_frame->ACC_Y_MSB, accel_frame->ACC_Y_LSB);
	const int16_t accel_z = combine(accel_frame->ACC_Z_MSB, accel_frame->ACC_Z_LSB);

	// Rotate from FLU to NED
	accel->x[samples] = accel_x;
	accel->y[samples] = (accel_y == INT16_MIN) ? INT16_MAX : -accel_y;
	accel->z[samples] = (accel_z == INT16_MIN) ? INT16_MAX : -accel_z;

	accel->samples++;
}


bool BMI270::FIFORead(const hrt_abstime &timestamp_sample, uint16_t fifo_bytes)
{

	uint8_t err = RegisterRead(Register::ERR_REG);
	if ((err>>6 & 1) == 1) {
		FIFOReset();
		return false;
	}

	// don't read more than 8 frames at a time, need to find out why this is the case...
	if (fifo_bytes > BMI270_MAX_FIFO_SAMPLES*13)
	{
	fifo_bytes = BMI270_MAX_FIFO_SAMPLES*13;
	}
	if (fifo_bytes == 0)
	{
		perf_count(_fifo_empty_perf);
		return false;
	}

	FIFOTransferBuffer buffer{};

	// transfers the buffer from the IMU into PX4-land
	if (transfer((uint8_t *)&buffer, (uint8_t *)&buffer, fifo_bytes) != PX4_OK)
	{
		PX4_DEBUG("buffer transfer failed");
		perf_count(_bad_transfer_perf);
		return false;
	}

	sensor_accel_fifo_s accel_buffer{};
	accel_buffer.timestamp_sample = timestamp_sample;
	accel_buffer.dt = FIFO_SAMPLE_DT;

	sensor_gyro_fifo_s gyro_buffer{};
	gyro_buffer.timestamp_sample = timestamp_sample;
	gyro_buffer.dt = FIFO_SAMPLE_DT;

	uint8_t *data_buffer = (uint8_t *)&buffer.f[0];
	unsigned fifo_buffer_index = 0; // start of buffer

		while (fifo_buffer_index < fifo_bytes) {
			// look for header signature (first 6 bits) followed by two bits indicating the status of INT1 and INT2
			switch (data_buffer[fifo_buffer_index] & 0xFC) {
			case FIFO::header::sensor_accel_frame:
			{

				// Acceleration sensor data frame
				// Frame length: 7 bytes (1 byte header + 6 bytes payload)

				// casts the 7 bytes of the buffer into a FIFO struct
				FIFO::ACCEL_DATA *fifo_sample = (FIFO::ACCEL_DATA *)&data_buffer[fifo_buffer_index];
				BMI270::ProcessAccel(&accel_buffer, fifo_sample);
				fifo_buffer_index += 7; // move forward to next record

				}
				break;

			case FIFO::header::sensor_gyro_frame:
			{



				FIFO::GYRO_DATA *fifo_sample = (FIFO::GYRO_DATA *)&data_buffer[fifo_buffer_index];
				BMI270::ProcessGyro(&gyro_buffer, fifo_sample);

				fifo_buffer_index += 7; // move forward to next record

			}
				break;
			case FIFO::header::sensor_accel_and_gyro_frame:
				{

				FIFO::GYRO_DATA *fifo_sample_gyr = (FIFO::GYRO_DATA *)&data_buffer[fifo_buffer_index];
				BMI270::ProcessGyro(&gyro_buffer, fifo_sample_gyr);

				FIFO::ACCEL_DATA *fifo_sample_acc = (FIFO::ACCEL_DATA *)&data_buffer[fifo_buffer_index + 6];
				BMI270::ProcessAccel(&accel_buffer, fifo_sample_acc);

				fifo_buffer_index += 13; // move forward to next record

				}
				break;

			case FIFO::header::skip_frame:
				// Skip Frame
				// Frame length: 2 bytes (1 byte header + 1 byte payload)
				PX4_DEBUG("Skip Frame");
				fifo_buffer_index += 2;
				break;

			case FIFO::header::sensor_time_frame:
				// Sensortime Frame
				// Frame length: 4 bytes (1 byte header + 3 bytes payload)
				PX4_DEBUG("Sensortime Frame");
				fifo_buffer_index += 4;
				break;

			case FIFO::header::FIFO_input_config_frame:
				// FIFO input config Frame
				// Frame length: 2 bytes (1 byte header + 1 byte payload)
				PX4_DEBUG("FIFO input config Frame");
				fifo_buffer_index += 2;
				break;

			case FIFO::header::sample_drop_frame:
				// Sample drop Frame
				// Frame length: 2 bytes (1 byte header + 1 byte payload)
				PX4_DEBUG("Sample drop Frame");
				fifo_buffer_index += 2;
				break;

			default:
				fifo_buffer_index++;
				break;
			}
	}

	_px4_accel.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf) +
				   perf_event_count(_fifo_empty_perf) + perf_event_count(_fifo_overflow_perf));


	if ((accel_buffer.samples == 0) && (gyro_buffer.samples == 0)) {

		return false;

	} else {
		_px4_accel.updateFIFO(accel_buffer);
		_px4_gyro.updateFIFO(gyro_buffer);
		return true;
	}

}



void BMI270::FIFOReset()
{
	PX4_DEBUG("Resetting FIFO...");
	perf_count(_fifo_reset_perf);

	// ACC_SOFTRESET: trigger a FIFO reset by writing 0xB0 to ACC_SOFTRESET (register 0x7E).
	RegisterWrite(Register::CMD, 0xB0);

	// reset while FIFO is disabled
	_drdy_timestamp_sample.store(0);
}

void BMI270::UpdateTemperature()
{
	// stored in an 11-bit value in 2’s complement format
	uint8_t temperature_buf[4] {};
	temperature_buf[0] = static_cast<uint8_t>(Register::TEMP_MSB) | DIR_READ;
	// temperature_buf[1] dummy byte

	if (transfer(&temperature_buf[0], &temperature_buf[0], sizeof(temperature_buf)) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return;
	}

	const uint8_t TEMP_MSB = temperature_buf[2];
	const uint8_t TEMP_LSB = temperature_buf[3];

	// Datasheet 5.3.7: Register 0x22 – 0x23: Temperature sensor data
	uint16_t Temp_uint11 = (TEMP_MSB * 8) + (TEMP_LSB / 32);
	int16_t Temp_int11 = 0;

	if (Temp_uint11 > 1023) {
		Temp_int11 = Temp_uint11 - 2048;

	} else {
		Temp_int11 = Temp_uint11;
	}

	float temperature = (Temp_int11 * 0.125f) + 23.f; // Temp_int11 * 0.125°C/LSB + 23°C

	if (PX4_ISFINITE(temperature)) {
		_px4_accel.set_temperature(temperature);

	} else {
		perf_count(_bad_transfer_perf);
	}
}
