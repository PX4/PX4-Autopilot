/****************************************************************************
 *
 *   Copyright (c) 2020-2022 PX4 Development Team. All rights reserved.
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

#include "MPU9250_I2C.hpp"

#include <lib/parameters/param.h>

using namespace time_literals;

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

MPU9250_I2C::MPU9250_I2C(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_drdy_gpio(config.drdy_gpio),
	_rotation(config.rotation)
{
	if (_drdy_gpio != 0) {
		_drdy_missed_perf = perf_alloc(PC_COUNT, MODULE_NAME": DRDY missed");
	}

	int32_t imu_gyro_rate_max = 400;
	param_get(param_find("IMU_GYRO_RATEMAX"), &imu_gyro_rate_max);

	ConfigureSampleRate(imu_gyro_rate_max);
}

MPU9250_I2C::~MPU9250_I2C()
{
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_fifo_empty_perf);
	perf_free(_fifo_overflow_perf);
	perf_free(_fifo_reset_perf);
	perf_free(_drdy_missed_perf);
}

int MPU9250_I2C::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool MPU9250_I2C::Reset()
{
	_state = STATE::RESET;
	DataReadyInterruptDisable();
	ScheduleClear();
	ScheduleNow();
	return true;
}

void MPU9250_I2C::exit_and_cleanup()
{
	DataReadyInterruptDisable();
	I2CSPIDriverBase::exit_and_cleanup();
}

void MPU9250_I2C::print_status()
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

bool MPU9250_I2C::StoreCheckedRegisterValue(Register reg)
{
	// 3 retries
	for (int i = 0; i < 3; i++) {
		uint8_t read1 = RegisterRead(reg);
		uint8_t read2 = RegisterRead(reg);

		if (read1 == read2) {
			for (auto &r : _register_cfg) {
				if (r.reg == reg) {
					r.set_bits = read1;
					r.clear_bits = ~read1;
					return true;
				}
			}

		} else {
			PX4_ERR("0x%02hhX read 1 != read 2 (0x%02hhX != 0x%02hhX)", static_cast<uint8_t>(reg), read1, read2);
		}
	}

	return false;
}

int MPU9250_I2C::probe()
{
	for (int retry = 0; retry < 3; retry++) {
		const uint8_t whoami = RegisterRead(Register::WHO_AM_I);

		if (whoami == WHOAMI) {
			return PX4_OK;

		} else {
			DEVICE_DEBUG("unexpected WHO_AM_I 0x%02x", whoami);
		}
	}

	return PX4_ERROR;
}

void MPU9250_I2C::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET:
		// PWR_MGMT_1: Device Reset
		RegisterWrite(Register::PWR_MGMT_1, PWR_MGMT_1_BIT::H_RESET);
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;
		ScheduleDelayed(100_ms);
		break;

	case STATE::WAIT_FOR_RESET:

		// The reset value is 0x00 for all registers other than the registers below
		//  Document Number: RM-MPU-9250A-00 Page 9 of 55
		if ((RegisterRead(Register::WHO_AM_I) == WHOAMI)
		    && (RegisterRead(Register::PWR_MGMT_1) == 0x01)) {

			// offset registers (factory calibration) should not change during normal operation
			StoreCheckedRegisterValue(Register::XA_OFFSET_H);
			StoreCheckedRegisterValue(Register::XA_OFFSET_L);
			StoreCheckedRegisterValue(Register::YA_OFFSET_H);
			StoreCheckedRegisterValue(Register::YA_OFFSET_L);
			StoreCheckedRegisterValue(Register::ZA_OFFSET_H);
			StoreCheckedRegisterValue(Register::ZA_OFFSET_L);

			// Wakeup and reset digital signal path
			RegisterWrite(Register::PWR_MGMT_1, PWR_MGMT_1_BIT::CLKSEL_0);
			RegisterWrite(Register::SIGNAL_PATH_RESET,
				      SIGNAL_PATH_RESET_BIT::GYRO_RESET | SIGNAL_PATH_RESET_BIT::ACCEL_RESET | SIGNAL_PATH_RESET_BIT::TEMP_RESET);
			RegisterWrite(Register::USER_CTRL, USER_CTRL_BIT::SIG_COND_RST);

			// if reset succeeded then configure
			_state = STATE::CONFIGURE;
			ScheduleDelayed(100_ms);

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

		break;

	case STATE::CONFIGURE:
		if (Configure()) {
			_temperature = ReadTemperature();

			// if configure succeeded then start reading from FIFO
			_state = STATE::FIFO_READ;

			if (DataReadyInterruptConfigure()) {
				_data_ready_interrupt_enabled = true;

				// backup schedule as a watchdog timeout
				ScheduleDelayed(100_ms);

			} else {
				_data_ready_interrupt_enabled = false;
				ScheduleOnInterval(_fifo_empty_interval_us, _fifo_empty_interval_us);
			}

			FIFOReset();

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
			hrt_abstime timestamp_sample = now;

			if (_data_ready_interrupt_enabled) {
				// scheduled from interrupt if _drdy_timestamp_sample was set as expected
				const hrt_abstime drdy_timestamp_sample = _drdy_timestamp_sample.fetch_and(0);

				if (now < drdy_timestamp_sample + _fifo_empty_interval_us) {
					timestamp_sample = drdy_timestamp_sample;

				} else {
					perf_count(_drdy_missed_perf);
				}

				// push backup schedule back
				ScheduleDelayed(_fifo_empty_interval_us * 2);
			}

			// always check current FIFO count
			bool success = false;
			const uint16_t fifo_count = FIFOReadCount();

			if (fifo_count >= FIFO::SIZE) {
				FIFOReset();
				perf_count(_fifo_overflow_perf);

			} else if (fifo_count == 0) {
				perf_count(_fifo_empty_perf);

			} else {
				// FIFO count (size in bytes) should be a multiple of the FIFO::DATA structure
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

				} else if (samples >= 1) {
					if (FIFORead(timestamp_sample, samples)) {
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
					PX4_DEBUG("Full reset because things are failing consistently");
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
					PX4_DEBUG("Force reset because register 0x%02hhX check failed ", (uint8_t)_register_cfg[_checked_register].reg);
					Reset();
				}

			} else {
				// periodically update temperature (~1 Hz)
				if (hrt_elapsed_time(&_temperature_update_timestamp) >= 1_s) {
					_temperature = ReadTemperature();
					_temperature_update_timestamp = now;
				}
			}
		}

		break;
	}
}

void MPU9250_I2C::ConfigureSampleRate(int sample_rate)
{
	// round down to nearest FIFO sample dt
	const float min_interval = FIFO_SAMPLE_DT;
	_fifo_empty_interval_us = math::max(roundf((1e6f / (float)sample_rate) / min_interval) * min_interval, min_interval);

	_fifo_gyro_samples = roundf(math::min((float)_fifo_empty_interval_us / (1e6f / GYRO_RATE), (float)FIFO_MAX_SAMPLES));

	// recompute FIFO empty interval (us) with actual gyro sample limit
	_fifo_empty_interval_us = _fifo_gyro_samples * (1e6f / GYRO_RATE);
}

bool MPU9250_I2C::Configure()
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

	return success;
}

int MPU9250_I2C::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<MPU9250_I2C *>(arg)->DataReady();
	return 0;
}

void MPU9250_I2C::DataReady()
{
	// at least the required number of samples in the FIFO
	if (++_drdy_count >= _fifo_gyro_samples) {
		_drdy_timestamp_sample.store(hrt_absolute_time());
		_drdy_count -= _fifo_gyro_samples;
		ScheduleNow();
	}
}

bool MPU9250_I2C::DataReadyInterruptConfigure()
{
	// TODO
	return false;

	if (_drdy_gpio == 0) {
		return false;
	}

	// Setup data ready on falling edge
	return (px4_arch_gpiosetevent(_drdy_gpio, false, true, false, &DataReadyInterruptCallback, this) == 0);
}

bool MPU9250_I2C::DataReadyInterruptDisable()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	return (px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0);
}

bool MPU9250_I2C::RegisterCheck(const register_config_t &reg_cfg)
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

uint8_t MPU9250_I2C::RegisterRead(Register reg)
{
	uint8_t cmd = static_cast<uint8_t>(reg);
	uint8_t value = 0;

	if (transfer(&cmd, 1, &value, 1) != 0) {
		perf_count(_bad_transfer_perf);
		return 0;
	}

	return value;
}

void MPU9250_I2C::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t cmd[2];
	cmd[0] = static_cast<uint8_t>(reg);
	cmd[1] = value;

	if (transfer(cmd, sizeof(cmd), nullptr, 0) != 0) {
		perf_count(_bad_transfer_perf);
	}
}

void MPU9250_I2C::RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);

	uint8_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}

uint16_t MPU9250_I2C::FIFOReadCount()
{
	// read FIFO count
	uint8_t cmd = static_cast<uint8_t>(Register::FIFO_COUNTH);

	// transfer buffer
	struct TransferBuffer {
		uint8_t FIFO_COUNTH{0};
		uint8_t FIFO_COUNTL{0};
	} buffer{};

	// read FIFO count
	if (transfer(&cmd, 1, (uint8_t *)&buffer, sizeof(buffer)) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return 0;
	}

	return (buffer.FIFO_COUNTH << 8) + buffer.FIFO_COUNTL;
}

bool MPU9250_I2C::FIFORead(const hrt_abstime &timestamp_sample, uint8_t samples)
{
	uint8_t cmd = static_cast<uint8_t>(Register::FIFO_R_W);

	// FIFO transfer buffer
	struct FIFOTransferBuffer {
		FIFO::DATA f[FIFO_MAX_SAMPLES] {};
	} buffer{};

	// samples (FIFO::DATA)
	const size_t transfer_size = math::min(samples * sizeof(FIFO::DATA), FIFO::SIZE);

	if (transfer(&cmd, 1, (uint8_t *)&buffer, transfer_size) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return false;
	}

	const uint8_t valid_samples = samples;

	if (valid_samples > 0) {
		sensor_imu_fifo_s sensor_imu_fifo{};
		sensor_imu_fifo.timestamp_sample = timestamp_sample;
		sensor_imu_fifo.device_id = get_device_id();
		sensor_imu_fifo.dt = FIFO_SAMPLE_DT;
		sensor_imu_fifo.samples = valid_samples;
		sensor_imu_fifo.accel_scale = ACCEL_SCALE;
		sensor_imu_fifo.gyro_scale = GYRO_SCALE;

		for (int i = 0; i < valid_samples; i++) {
			// sensor's frame is +x forward, +y left, +z up
			//  flip y & z to publish right handed with z down (x forward, y right, z down)
			sensor_imu_fifo.accel_x[i] =              combine(buffer.f[i].ACCEL_XOUT_H, buffer.f[i].ACCEL_XOUT_L);
			sensor_imu_fifo.accel_y[i] = math::negate(combine(buffer.f[i].ACCEL_YOUT_H, buffer.f[i].ACCEL_YOUT_L));
			sensor_imu_fifo.accel_z[i] = math::negate(combine(buffer.f[i].ACCEL_ZOUT_H, buffer.f[i].ACCEL_ZOUT_L));
			rotate_3i(_rotation, sensor_imu_fifo.accel_x[i], sensor_imu_fifo.accel_y[i], sensor_imu_fifo.accel_z[i]);

			sensor_imu_fifo.gyro_x[i] =              combine(buffer.f[i].GYRO_XOUT_H, buffer.f[i].GYRO_XOUT_L);
			sensor_imu_fifo.gyro_y[i] = math::negate(combine(buffer.f[i].GYRO_YOUT_H, buffer.f[i].GYRO_YOUT_L));
			sensor_imu_fifo.gyro_z[i] = math::negate(combine(buffer.f[i].GYRO_ZOUT_H, buffer.f[i].GYRO_ZOUT_L));
			rotate_3i(_rotation, sensor_imu_fifo.gyro_x[i], sensor_imu_fifo.gyro_y[i], sensor_imu_fifo.gyro_z[i]);
		}

		sensor_imu_fifo.temperature = (hrt_elapsed_time(&_temperature_update_timestamp) < 5_s) ? _temperature : NAN;

		sensor_imu_fifo.error_count = perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf)
					      + perf_event_count(_fifo_empty_perf) + perf_event_count(_fifo_overflow_perf);

		sensor_imu_fifo.timestamp = hrt_absolute_time();
		_sensor_imu_fifo_pub.publish(sensor_imu_fifo);

		return true;
	}

	return false;
}

void MPU9250_I2C::FIFOReset()
{
	perf_count(_fifo_reset_perf);

	// FIFO_EN: disable FIFO
	RegisterWrite(Register::FIFO_EN, 0);

	// USER_CTRL: reset FIFO
	RegisterSetAndClearBits(Register::USER_CTRL, USER_CTRL_BIT::FIFO_RST, USER_CTRL_BIT::FIFO_EN);

	// reset while FIFO is disabled
	_drdy_count = 0;
	_drdy_timestamp_sample.store(0);

	// FIFO_EN: enable both gyro and accel
	// USER_CTRL: re-enable FIFO
	for (const auto &r : _register_cfg) {
		if ((r.reg == Register::FIFO_EN) || (r.reg == Register::USER_CTRL)) {
			RegisterSetAndClearBits(r.reg, r.set_bits, r.clear_bits);
		}
	}
}

float MPU9250_I2C::ReadTemperature()
{
	uint8_t cmd = static_cast<uint8_t>(Register::TEMP_OUT_H);

	// transfer buffer
	struct TransferBuffer {
		uint8_t TEMP_OUT_H{0};
		uint8_t TEMP_OUT_L{0};
	} buffer{};

	// read current temperature
	if (transfer(&cmd, 1, (uint8_t *)&buffer, sizeof(buffer)) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return NAN;
	}

	const int16_t TEMP_OUT = combine(buffer.TEMP_OUT_H, buffer.TEMP_OUT_L);
	const float TEMP_degC = (TEMP_OUT / TEMPERATURE_SENSITIVITY) + TEMPERATURE_OFFSET;

	if (PX4_ISFINITE(TEMP_degC)
	    && (TEMP_degC >= TEMPERATURE_SENSOR_MIN)
	    && (TEMP_degC <= TEMPERATURE_SENSOR_MAX)) {

		return TEMP_degC;
	}

	return NAN;
}
