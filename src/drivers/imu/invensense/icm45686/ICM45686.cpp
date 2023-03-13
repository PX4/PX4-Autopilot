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

#include "ICM45686.hpp"

using namespace time_literals;

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

static constexpr uint16_t combine_uint(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

static constexpr int32_t reassemble_20bit(const uint32_t a, const uint32_t b, const uint32_t c)
{
	// 0xXXXAABBC
	uint32_t high   = ((a << 12) & 0x000FF000);
	uint32_t low    = ((b << 4)  & 0x00000FF0);
	uint32_t lowest = (c         & 0x0000000F);

	uint32_t x = high | low | lowest;

	if (a & Bit7) {
		// sign extend
		x |= 0xFFF00000u;
	}

	return static_cast<int32_t>(x);
}


ICM45686::ICM45686(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_px4_accel(get_device_id(), config.rotation),
	_px4_gyro(get_device_id(), config.rotation)
{
	if (config.custom1 != 0) {
		_enable_clock_input = true;
		_input_clock_freq = config.custom1;
		// TODO: this is not tested
		ConfigureCLKIN();

	} else {
		_enable_clock_input = false;
	}

	ConfigureSampleRate(_px4_gyro.get_max_rate_hz());
}

ICM45686::~ICM45686()
{
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_fifo_empty_perf);
	perf_free(_fifo_overflow_perf);
	perf_free(_fifo_reset_perf);
}

int ICM45686::init()
{
	int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool ICM45686::Reset()
{
	_state = STATE::RESET;
	ScheduleClear();
	ScheduleNow();
	return true;
}

void ICM45686::exit_and_cleanup()
{
	I2CSPIDriverBase::exit_and_cleanup();
}

void ICM45686::print_status()
{
	I2CSPIDriverBase::print_status();

	PX4_INFO("FIFO empty interval: %d us (%.1f Hz)", _fifo_empty_interval_us, 1e6 / _fifo_empty_interval_us);
	PX4_INFO("Clock input: %s", _enable_clock_input ? "enabled" : "disabled");

	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_fifo_empty_perf);
	perf_print_counter(_fifo_overflow_perf);
	perf_print_counter(_fifo_reset_perf);
}

int ICM45686::probe()
{
	for (int i = 0; i < 3; i++) {
		const uint8_t whoami = RegisterRead(Register::BANK_0::WHO_AM_I);

		if (whoami != WHOAMI) {
			DEVICE_DEBUG("unexpected WHO_AM_I 0x%02x", whoami);
			return PX4_ERROR;
		}
	}

	return PX4_OK;
}

void ICM45686::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET:
		// DEVICE_CONFIG: Software reset configuration
		RegisterWrite(Register::BANK_0::REG_MISC2, REG_MISC2_BIT::SOFT_RST);
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;
		ScheduleDelayed(1_ms); // wait 1 ms for soft reset to be effective
		break;

	case STATE::WAIT_FOR_RESET:
		if ((RegisterRead(Register::BANK_0::WHO_AM_I) == WHOAMI)
		    && ((RegisterRead(Register::BANK_0::REG_MISC2) & Bit1) == 0x0)) {

			// Wakeup accel and gyro and schedule remaining configuration
			RegisterWrite(Register::BANK_0::PWR_MGMT0, PWR_MGMT0_BIT::GYRO_MODE_LOW_NOISE | PWR_MGMT0_BIT::ACCEL_MODE_LOW_NOISE);
			_state = STATE::CONFIGURE;
			ScheduleDelayed(30_ms); // 30 ms gyro startup time, 10 ms accel from sleep to valid data

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
			// if configure succeeded then reset the FIFO
			_state = STATE::FIFO_RESET;
			ScheduleDelayed(1_ms);

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

	case STATE::FIFO_RESET:

		_state = STATE::FIFO_READ;
		FIFOReset();

		ScheduleOnInterval(_fifo_empty_interval_us, _fifo_empty_interval_us);

		break;

	case STATE::FIFO_READ: {
			hrt_abstime timestamp_sample = now;

			bool success = false;

			if (FIFORead(timestamp_sample)) {
				success = true;

				if (_failure_count > 0) {
					_failure_count--;
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
				if (RegisterCheck(_register_bank0_cfg[_checked_register_bank0])) {
					_last_config_check_timestamp = now;
					_checked_register_bank0 = (_checked_register_bank0 + 1) % size_register_bank0_cfg;

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

void ICM45686::ConfigureSampleRate(int sample_rate)
{
	// round down to the nearest FIFO sample dt
	const float min_interval = FIFO_SAMPLE_DT;
	_fifo_empty_interval_us = math::max(roundf((1e6f / (float)sample_rate) / min_interval) * min_interval, min_interval);

	_fifo_gyro_samples = roundf(math::min((float)_fifo_empty_interval_us / (1e6f / GYRO_RATE), (float)FIFO_MAX_SAMPLES));

	// recompute FIFO empty interval (us) with actual gyro sample limit
	_fifo_empty_interval_us = _fifo_gyro_samples * (1e6f / GYRO_RATE);

	ConfigureFIFOWatermark(_fifo_gyro_samples);
}

void ICM45686::ConfigureFIFOWatermark(uint8_t samples)
{
	// FIFO watermark threshold in number of bytes
	const uint16_t fifo_watermark_threshold = samples * sizeof(FIFO::DATA);

	for (auto &r : _register_bank0_cfg) {
		if (r.reg == Register::BANK_0::FIFO_CONFIG1_0) {
			// FIFO_WM[7:0]  FIFO_CONFIG2
			r.set_bits = fifo_watermark_threshold & 0xFF;

		} else if (r.reg == Register::BANK_0::FIFO_CONFIG1_1) {
			// FIFO_WM[11:8] FIFO_CONFIG3
			r.set_bits = (fifo_watermark_threshold >> 8) & 0xFF;
		}
	}
}

void ICM45686::ConfigureCLKIN()
{
	for (auto &r0 : _register_bank0_cfg) {
		if (r0.reg == Register::BANK_0::RTC_CONFIG) {
			r0.set_bits = RTC_CONFIG_BIT::RTC_MODE;
		}
	}

	for (auto &r0 : _register_bank0_cfg) {
		if (r0.reg == Register::BANK_0::IOC_PAD_SCENARIO_OVRD) {
			r0.set_bits = PADS_INT2_CFG_OVRD | PADS_INT2_CFG_OVRD_CLKIN;
		}
	}
}

bool ICM45686::Configure()
{
	// Set it to little endian first, otherwise the chip doesn't match the manual
	// which is just utterly confusing.
	//uint8_t cmd[3] {
	//    BANK_IPREG_TOP1,
	//        SREG_CTRL,
	//    SREG_CTRL_SREG_DATA_ENDIAN_SEL_BIT::SREG_CTRL_SREG_DATA_ENDIAN_SEL_BIG };
	//transfer(cmd, cmd, sizeof(cmd));

	// first set and clear all configured register bits
	for (const auto &reg_cfg : _register_bank0_cfg) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
	}

	// now check that all are configured
	bool success = true;

	for (const auto &reg_cfg : _register_bank0_cfg) {
		if (!RegisterCheck(reg_cfg)) {
			success = false;
		}
	}

	// 20-bits data format used the only FSR settings that are operational
	// are ±4000dps for gyroscope and ±32 for accelerometer
	_px4_accel.set_range(32.f * CONSTANTS_ONE_G);
	_px4_gyro.set_range(math::radians(4000.f));

	return success;
}

template <typename T>
bool ICM45686::RegisterCheck(const T &reg_cfg)
{
	bool success = true;

	const uint8_t reg_value = RegisterRead(reg_cfg.reg);

	if (reg_cfg.set_bits && ((reg_value & reg_cfg.set_bits) != reg_cfg.set_bits)) {
		PX4_INFO("0x%02hhX: 0x%02hhX (0x%02hhX not set)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.set_bits);
		success = false;
	}

	if (reg_cfg.clear_bits && ((reg_value & reg_cfg.clear_bits) != 0)) {
		PX4_INFO("0x%02hhX: 0x%02hhX (0x%02hhX not cleared)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.clear_bits);
		success = false;
	}

	return success;
}

template <typename T>
uint8_t ICM45686::RegisterRead(T reg)
{
	uint8_t cmd[2] {};
	cmd[0] = static_cast<uint8_t>(reg) | DIR_READ;
	transfer(cmd, cmd, sizeof(cmd));
	return cmd[1];
}

template <typename T>
void ICM45686::RegisterWrite(T reg, uint8_t value)
{
	uint8_t cmd[2] { (uint8_t)reg, value };
	transfer(cmd, cmd, sizeof(cmd));
}

template <typename T>
void ICM45686::RegisterSetAndClearBits(T reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);

	uint8_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}

uint16_t ICM45686::FIFOReadCount()
{
	// read FIFO count
	uint8_t fifo_count_buf[3] {};
	fifo_count_buf[0] = static_cast<uint8_t>(Register::BANK_0::FIFO_COUNT_0) | DIR_READ;

	if (transfer(fifo_count_buf, fifo_count_buf, sizeof(fifo_count_buf)) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return 0;
	}

	// FIFO_COUNT_0 is supposed to contain the high bits and FIFO_COUNT_1 the low bits,
	// according to the manual, however, the device is configured to little endianness
	// which means FIFO and FIFO count are pre-swapped..
	return combine(fifo_count_buf[2], fifo_count_buf[1]);
}

bool ICM45686::FIFORead(const hrt_abstime &timestamp_sample)
{
	const uint16_t fifo_packets = FIFOReadCount();

	if (fifo_packets == 0) {
		perf_count(_fifo_empty_perf);
		return false;
	}

	FIFOTransferBuffer buffer{};
	const size_t transfer_size = math::min(sizeof(FIFOTransferBuffer), fifo_packets * sizeof(FIFO::DATA) + 1);

	if (transfer((uint8_t *)&buffer, (uint8_t *)&buffer, transfer_size) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return false;
	}

	unsigned valid_samples = 0;

	for (unsigned i = 0; i < transfer_size / sizeof(FIFO::DATA); i++) {
		bool valid = true;

		// With FIFO_ACCEL_EN and FIFO_GYRO_EN header should be 8’b_0110_10xx
		const uint8_t FIFO_HEADER = buffer.f[i].FIFO_Header;

		if (FIFO_HEADER & FIFO::FIFO_HEADER_BIT::HEADER_MSG) {
			// FIFO sample empty if HEADER_MSG set
			valid = false;

		} else if (!(FIFO_HEADER & FIFO::FIFO_HEADER_BIT::HEADER_ACCEL)) {
			// accel bit not set
			valid = false;

		} else if (!(FIFO_HEADER & FIFO::FIFO_HEADER_BIT::HEADER_GYRO)) {
			// gyro bit not set
			valid = false;

		} else if (!(FIFO_HEADER & FIFO::FIFO_HEADER_BIT::HEADER_20)) {
			// Packet does not contain a new and valid extended 20-bit data
			valid = false;

		} else if ((FIFO_HEADER & FIFO::FIFO_HEADER_BIT::HEADER_TIMESTAMP_FSYNC) != Bit3) {
			// Packet does not contain ODR timestamp
			valid = false;

		} else if (FIFO_HEADER & FIFO::FIFO_HEADER_BIT::HEADER_ODR_ACCEL) {
			// accel ODR changed
			valid = false;

		} else if (FIFO_HEADER & FIFO::FIFO_HEADER_BIT::HEADER_ODR_GYRO) {
			// gyro ODR changed
			valid = false;
		}

		if (valid) {
			valid_samples++;

		} else {
			perf_count(_bad_transfer_perf);
			break;
		}
	}

	if (valid_samples > 0) {
		if (ProcessTemperature(buffer.f, valid_samples)) {
			ProcessGyro(timestamp_sample, buffer.f, valid_samples);
			ProcessAccel(timestamp_sample, buffer.f, valid_samples);
			return true;
		}
	}

	return false;
}

void ICM45686::FIFOReset()
{
	perf_count(_fifo_reset_perf);

	// Disable FIFO
	RegisterClearBits(Register::BANK_0::FIFO_CONFIG3,
			  FIFO_CONFIG3_BIT::FIFO_ES1_EN |
			  FIFO_CONFIG3_BIT::FIFO_ES0_EN |
			  FIFO_CONFIG3_BIT::FIFO_HIRES_EN |
			  FIFO_CONFIG3_BIT::FIFO_GYRO_EN |
			  FIFO_CONFIG3_BIT::FIFO_ACCEL_EN |
			  FIFO_CONFIG3_BIT::FIFO_IF_EN);

	// Disable FIFO by switching to bypass mode
	RegisterSetAndClearBits(Register::BANK_0::FIFO_CONFIG0,
				FIFO_CONFIG0_BIT::FIFO_MODE_BYPASS_SET,
				FIFO_CONFIG0_BIT::FIFO_MODE_BYPASS_CLEAR);

	// When the FIFO is disabled we can actually set the FIFO depth
	RegisterSetBits(Register::BANK_0::FIFO_CONFIG0, FIFO_CONFIG0_BIT::FIFO_DEPTH_8K_SET);

	// And then enable FIFO again
	RegisterSetAndClearBits(Register::BANK_0::FIFO_CONFIG0, FIFO_CONFIG0_BIT::FIFO_MODE_STOP_ON_FULL_SET,
				FIFO_CONFIG0_BIT::FIFO_MODE_STOP_ON_FULL_CLEAR);

	// And enable again
	RegisterSetBits(Register::BANK_0::FIFO_CONFIG3,
			FIFO_CONFIG3_BIT::FIFO_HIRES_EN |
			FIFO_CONFIG3_BIT::FIFO_GYRO_EN |
			FIFO_CONFIG3_BIT::FIFO_ACCEL_EN |
			FIFO_CONFIG3_BIT::FIFO_IF_EN);
}

void ICM45686::ProcessAccel(const hrt_abstime &timestamp_sample, const FIFO::DATA fifo[], const uint8_t samples)
{
	sensor_accel_fifo_s accel{};
	accel.timestamp_sample = timestamp_sample;
	accel.samples = 0;

	// 19-bits of accelerometer data
	bool scale_20bit = false;

	// first pass
	for (int i = 0; i < samples; i++) {


		if (_enable_clock_input) {
			// Swapped as device is in little endian by default.
			const uint16_t timestamp_fifo = combine_uint(fifo[i].Timestamp_L, fifo[i].Timestamp_H);
			accel.dt = (float)timestamp_fifo * ((1.f / _input_clock_freq) * 1e6f);

		} else {
			accel.dt = FIFO_TIMESTAMP_SCALING;
		}

		// 20 bit hires mode
		// Sign extension + Accel [19:12] + Accel [11:4] + Accel [3:2] (20 bit extension byte)
		// Accel data is 18 bit ()
		int32_t accel_x = reassemble_20bit(
					  fifo[i].ACCEL_DATA_XL,
					  fifo[i].ACCEL_DATA_XH,
					  fifo[i].HIGHRES_X_LSB & 0xF0 >> 4);
		int32_t accel_y = reassemble_20bit(
					  fifo[i].ACCEL_DATA_YL,
					  fifo[i].ACCEL_DATA_YH,
					  fifo[i].HIGHRES_Y_LSB & 0xF0 >> 4);
		int32_t accel_z = reassemble_20bit(
					  fifo[i].ACCEL_DATA_ZL,
					  fifo[i].ACCEL_DATA_ZH,
					  fifo[i].HIGHRES_Z_LSB & 0xF0 >> 4);

		// sample invalid if -524288
		if (accel_x != -524288 && accel_y != -524288 && accel_z != -524288) {

			// It's not enough to check if any values are exceeding the
			// int16 limits because there might be a rotation applied later.
			// If a rotation is 45 degrees, the new component can be up to
			// sqrt(2) longer than one component. This means the number has
			// to be constrained to fit the int16 which then triggers
			// clipping.
			//
			// Therefore, we set the limits at int16_max/min / sqrt(2) plus
			// a bit of margin.
			static constexpr int16_t max_accel = static_cast<int16_t>(INT16_MAX / sqrt(2.f)) - 100;
			static constexpr int16_t min_accel = static_cast<int16_t>(INT16_MIN / sqrt(2.f)) + 100;

			if (accel_x >= max_accel || accel_x <= min_accel) {
				scale_20bit = true;
			}

			if (accel_y >= max_accel || accel_y <= min_accel) {
				scale_20bit = true;
			}

			if (accel_z >= max_accel || accel_z <= min_accel) {
				scale_20bit = true;
			}

			// least significant bit is always 0)
			accel.x[accel.samples] = accel_x / 2;
			accel.y[accel.samples] = accel_y / 2;
			accel.z[accel.samples] = accel_z / 2;
			accel.samples++;
		}
	}

	if (!scale_20bit) {
		// if highres enabled accel data is always 8192 LSB/g
		_px4_accel.set_scale(CONSTANTS_ONE_G / 8192.f);

	} else {
		// 20 bit data scaled to 16 bit (2^4)
		for (int i = 0; i < samples; i++) {
			// 20 bit hires mode
			// Sign extension + Accel [19:12] + Accel [11:4] + Accel [3:2] (20 bit extension byte)
			// Accel data is 18 bit ()
			int16_t accel_x = combine(fifo[i].ACCEL_DATA_XL, fifo[i].ACCEL_DATA_XH);
			int16_t accel_y = combine(fifo[i].ACCEL_DATA_YL, fifo[i].ACCEL_DATA_YH);
			int16_t accel_z = combine(fifo[i].ACCEL_DATA_ZL, fifo[i].ACCEL_DATA_ZH);

			accel.x[i] = accel_x;
			accel.y[i] = accel_y;
			accel.z[i] = accel_z;
		}

		_px4_accel.set_scale(CONSTANTS_ONE_G / 8192.f * 8.0f);
	}

	// correct frame for publication
	for (int i = 0; i < accel.samples; i++) {
		// sensor's frame is +x forward, +y left, +z up
		//  flip y & z to publish right handed with z down (x forward, y right, z down)
		accel.x[i] = accel.x[i];
		accel.y[i] = (accel.y[i] == INT16_MIN) ? INT16_MAX : -accel.y[i];
		accel.z[i] = (accel.z[i] == INT16_MIN) ? INT16_MAX : -accel.z[i];
	}

	_px4_accel.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf) +
				   perf_event_count(_fifo_empty_perf) + perf_event_count(_fifo_overflow_perf));

	if (accel.samples > 0) {
		_px4_accel.updateFIFO(accel);
	}
}

void ICM45686::ProcessGyro(const hrt_abstime &timestamp_sample, const FIFO::DATA fifo[], const uint8_t samples)
{
	sensor_gyro_fifo_s gyro{};
	gyro.timestamp_sample = timestamp_sample;
	gyro.samples = 0;

	// 20-bits of gyroscope data
	bool scale_20bit = false;

	// first pass
	for (int i = 0; i < samples; i++) {


		if (_enable_clock_input) {
			// Swapped as device is in little endian by default.
			uint16_t timestamp_fifo = combine_uint(fifo[i].Timestamp_L, fifo[i].Timestamp_H);
			gyro.dt = (float)timestamp_fifo * ((1.f / _input_clock_freq) * 1e6f);

		} else {
			gyro.dt = FIFO_TIMESTAMP_SCALING;
		}

		// 20 bit hires mode
		// Gyro [19:12] + Gyro [11:4] + Gyro [3:0] (bottom 4 bits of 20 bit extension byte)
		int32_t gyro_x = reassemble_20bit(fifo[i].GYRO_DATA_XL, fifo[i].GYRO_DATA_XH, fifo[i].HIGHRES_X_LSB & 0x0F);
		int32_t gyro_y = reassemble_20bit(fifo[i].GYRO_DATA_YL, fifo[i].GYRO_DATA_YH, fifo[i].HIGHRES_Y_LSB & 0x0F);
		int32_t gyro_z = reassemble_20bit(fifo[i].GYRO_DATA_ZL, fifo[i].GYRO_DATA_ZH, fifo[i].HIGHRES_Z_LSB & 0x0F);

		// It's not enough to check if any values are exceeding the
		// int16 limits because there might be a rotation applied later.
		// If a rotation is 45 degrees, the new component can be up to
		// sqrt(2) longer than one component. This means the number has
		// to be constrained to fit the int16 which then triggers
		// clipping.
		//
		// Therefore, we set the limits at int16_max/min / sqrt(2) plus
		// a bit of margin.
		static constexpr int16_t max_gyro = static_cast<int16_t>(INT16_MAX / sqrt(2.f)) - 100;
		static constexpr int16_t min_gyro = static_cast<int16_t>(INT16_MIN / sqrt(2.f)) + 100;

		if (gyro_x >= max_gyro || gyro_x <= min_gyro) {
			scale_20bit = true;
		}

		if (gyro_y >= max_gyro || gyro_y <= min_gyro) {
			scale_20bit = true;
		}

		if (gyro_z >= max_gyro || gyro_z <= min_gyro) {
			scale_20bit = true;
		}

		gyro.x[gyro.samples] = gyro_x;
		gyro.y[gyro.samples] = gyro_y;
		gyro.z[gyro.samples] = gyro_z;
		gyro.samples++;
	}

	if (!scale_20bit) {
		// if highres enabled gyro data is always 131 LSB/dps
		_px4_gyro.set_scale(math::radians(1.f / 131.f));

	} else {
		// 20 bit data scaled to 16 bit (2^4)
		for (int i = 0; i < samples; i++) {
			gyro.x[i] = combine(fifo[i].GYRO_DATA_XL, fifo[i].GYRO_DATA_XH);
			gyro.y[i] = combine(fifo[i].GYRO_DATA_YL, fifo[i].GYRO_DATA_YH);
			gyro.z[i] = combine(fifo[i].GYRO_DATA_ZL, fifo[i].GYRO_DATA_ZH);
		}

		_px4_gyro.set_scale(math::radians(1.f / 131.f * 16.0f));
	}

	// correct frame for publication
	for (int i = 0; i < gyro.samples; i++) {
		// sensor's frame is +x forward, +y left, +z up
		//  flip y & z to publish right handed with z down (x forward, y right, z down)
		gyro.x[i] = gyro.x[i];
		gyro.y[i] = (gyro.y[i] == INT16_MIN) ? INT16_MAX : -gyro.y[i];
		gyro.z[i] = (gyro.z[i] == INT16_MIN) ? INT16_MAX : -gyro.z[i];
	}

	_px4_gyro.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf) +
				  perf_event_count(_fifo_empty_perf) + perf_event_count(_fifo_overflow_perf));

	if (gyro.samples > 0) {
		_px4_gyro.updateFIFO(gyro);
	}
}

bool ICM45686::ProcessTemperature(const FIFO::DATA fifo[], const uint8_t samples)
{
	int16_t temperature[FIFO_MAX_SAMPLES];
	float temperature_sum{0};

	int valid_samples = 0;

	for (int i = 0; i < samples; i++) {
		// Swapped as device is in little endian by default.
		const int16_t t = combine(fifo[i].TEMP_DATA_L, fifo[i].TEMP_DATA_H);

		// sample invalid if -32768
		if (t != -32768) {
			temperature_sum += t;
			temperature[valid_samples] = t;
			valid_samples++;
		}
	}

	if (valid_samples > 0) {
		const float temperature_avg = temperature_sum / valid_samples;

		for (int i = 0; i < valid_samples; i++) {
			// temperature changing wildly is an indication of a transfer error
			if (fabsf(temperature[i] - temperature_avg) > 1000) {
				perf_count(_bad_transfer_perf);
				return false;
			}
		}

		// use average temperature reading
		const float temp_c = (temperature_avg / TEMPERATURE_SENSITIVITY) + TEMPERATURE_OFFSET;

		if (PX4_ISFINITE(temp_c)) {
			_px4_accel.set_temperature(temp_c);
			_px4_gyro.set_temperature(temp_c);
			return true;

		} else {
			perf_count(_bad_transfer_perf);
		}
	}

	return false;
}
