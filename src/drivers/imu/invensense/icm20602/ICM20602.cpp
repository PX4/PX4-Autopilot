/****************************************************************************
 *
 *   Copyright (c) 2019-2022 PX4 Development Team. All rights reserved.
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

#include "ICM20602.hpp"

#include <lib/parameters/param.h>

using namespace time_literals;

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

ICM20602::ICM20602(const I2CSPIDriverConfig &config) :
	SPI(config),
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

ICM20602::~ICM20602()
{
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_fifo_empty_perf);
	perf_free(_fifo_overflow_perf);
	perf_free(_fifo_reset_perf);
	perf_free(_drdy_missed_perf);
}

int ICM20602::init()
{
	int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool ICM20602::Reset()
{
	_state = STATE::RESET;
	DataReadyInterruptDisable();
	ScheduleClear();
	ScheduleNow();
	return true;
}

void ICM20602::exit_and_cleanup()
{
	DataReadyInterruptDisable();
	I2CSPIDriverBase::exit_and_cleanup();
}

void ICM20602::print_status()
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

bool ICM20602::StoreCheckedRegisterValue(Register reg)
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

int ICM20602::probe()
{
	// 3 retries
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

void ICM20602::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET:
		// PWR_MGMT_1: Device Reset
		RegisterWrite(Register::PWR_MGMT_1, PWR_MGMT_1_BIT::DEVICE_RESET);
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;
		ScheduleDelayed(2_ms); // From power-up 2 ms start-up time for register read/write
		break;

	case STATE::WAIT_FOR_RESET:

		// The reset value is 0x00 for all registers other than the registers below
		//  Document Number: DS-000176 Page 31 of 57
		if ((RegisterRead(Register::WHO_AM_I) == WHOAMI)
		    && (RegisterRead(Register::PWR_MGMT_1) == 0x41)
		    && (RegisterRead(Register::CONFIG) == 0x80)) {

			// offset registers (factory calibration) should not change during normal operation
			StoreCheckedRegisterValue(Register::XG_OFFS_TC_H);
			StoreCheckedRegisterValue(Register::XG_OFFS_TC_L);
			StoreCheckedRegisterValue(Register::YG_OFFS_TC_H);
			StoreCheckedRegisterValue(Register::YG_OFFS_TC_L);
			StoreCheckedRegisterValue(Register::ZG_OFFS_TC_H);
			StoreCheckedRegisterValue(Register::ZG_OFFS_TC_L);

			StoreCheckedRegisterValue(Register::XA_OFFSET_H);
			StoreCheckedRegisterValue(Register::XA_OFFSET_L);
			StoreCheckedRegisterValue(Register::YA_OFFSET_H);
			StoreCheckedRegisterValue(Register::YA_OFFSET_L);
			StoreCheckedRegisterValue(Register::ZA_OFFSET_H);
			StoreCheckedRegisterValue(Register::ZA_OFFSET_L);

			// Disable I2C, wakeup, and reset digital signal path
			RegisterWrite(Register::I2C_IF, I2C_IF_BIT::I2C_IF_DIS); // set immediately to prevent switching into I2C mode
			RegisterWrite(Register::PWR_MGMT_1, PWR_MGMT_1_BIT::CLKSEL_0);
			RegisterWrite(Register::SIGNAL_PATH_RESET, SIGNAL_PATH_RESET_BIT::ACCEL_RST | SIGNAL_PATH_RESET_BIT::TEMP_RST);
			RegisterSetAndClearBits(Register::USER_CTRL, USER_CTRL_BIT::SIG_COND_RST, 0);

			// if reset succeeded then configure
			_state = STATE::CONFIGURE;
			ScheduleDelayed(35_ms); // max 35 ms start-up time from sleep

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
			uint8_t samples = 0;

			if (_data_ready_interrupt_enabled) {
				// scheduled from interrupt if _drdy_timestamp_sample was set as expected
				const hrt_abstime drdy_timestamp_sample = _drdy_timestamp_sample.fetch_and(0);

				if (now < drdy_timestamp_sample + _fifo_empty_interval_us) {
					timestamp_sample = drdy_timestamp_sample;
					samples = _fifo_gyro_samples;

				} else {
					perf_count(_drdy_missed_perf);
				}

				// push backup schedule back
				ScheduleDelayed(_fifo_empty_interval_us * 2);
			}

			if (samples == 0) {
				// check current FIFO count
				const uint16_t fifo_count = FIFOReadCount();

				if (fifo_count >= FIFO::SIZE) {
					FIFOReset();
					perf_count(_fifo_overflow_perf);

				} else if (fifo_count == 0) {
					perf_count(_fifo_empty_perf);

				} else {
					// FIFO count (size in bytes) should be a multiple of the FIFO::DATA structure
					samples = fifo_count / sizeof(FIFO::DATA);

					if (samples > _fifo_gyro_samples) {
						// grab desired number of samples, but reschedule next cycle sooner
						int extra_samples = samples - _fifo_gyro_samples;
						samples = _fifo_gyro_samples;

						if (_fifo_gyro_samples > extra_samples) {
							// reschedule to run when a total of _fifo_gyro_samples should be available in the FIFO
							const uint32_t reschedule_delay_us = (_fifo_gyro_samples - extra_samples) * static_cast<int>(FIFO_SAMPLE_DT);
							ScheduleOnInterval(_fifo_empty_interval_us, reschedule_delay_us);

						} else {
							// otherwise reschedule to run immediately
							ScheduleOnInterval(_fifo_empty_interval_us);
						}

					} else if (samples < _fifo_gyro_samples) {
						// reschedule next cycle to catch the desired number of samples
						ScheduleOnInterval(_fifo_empty_interval_us, (_fifo_gyro_samples - samples) * static_cast<int>(FIFO_SAMPLE_DT));
					}
				}
			}

			bool success = false;

			if (samples == _fifo_gyro_samples) {
				if (FIFORead(timestamp_sample, samples)) {
					success = true;

					if (_failure_count > 0) {
						_failure_count--;
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

			// check configuration registers periodically or immediately following any failure
			if (!success || hrt_elapsed_time(&_last_config_check_timestamp) > 100_ms) {
				if (RegisterCheck(_register_cfg[_checked_register])) {
					_last_config_check_timestamp = now;
					_checked_register = (_checked_register + 1) % size_register_cfg;

				} else {
					// register check failed, force reset
					perf_count(_bad_register_perf);
					PX4_DEBUG("Force reset because register 0x%02hhX check failed ", (uint8_t)_register_cfg[_checked_register].reg);
					Reset();
				}
			}
		}

		break;
	}
}

void ICM20602::ConfigureSampleRate(int sample_rate)
{
	// round down to nearest FIFO sample dt
	const float min_interval = FIFO_SAMPLE_DT;
	_fifo_empty_interval_us = math::max(roundf((1e6f / (float)sample_rate) / min_interval) * min_interval, min_interval);

	_fifo_gyro_samples = roundf(math::min((float)_fifo_empty_interval_us / (1e6f / GYRO_RATE), (float)FIFO_MAX_SAMPLES));

	// recompute FIFO empty interval (us) with actual gyro sample limit
	_fifo_empty_interval_us = _fifo_gyro_samples * (1e6f / GYRO_RATE);

	ConfigureFIFOWatermark(_fifo_gyro_samples);
}

void ICM20602::ConfigureFIFOWatermark(uint8_t samples)
{
	// FIFO watermark threshold in number of bytes
	const uint16_t fifo_watermark_threshold = samples * sizeof(FIFO::DATA);

	for (auto &r : _register_cfg) {
		if (r.reg == Register::CONFIG) {
			// Document Number: DS-000176 Page 45 of 57
			// User should ensure that bit 7 of register 0x1A is set to 0 before using FIFO watermark threshold feature
			r.clear_bits = Bit7;

		} else if (r.reg == Register::FIFO_WM_TH1) {
			r.set_bits = (fifo_watermark_threshold >> 8) & 0b00000011;

		} else if (r.reg == Register::FIFO_WM_TH2) {
			r.set_bits = fifo_watermark_threshold & 0xFF;
		}
	}
}

bool ICM20602::Configure()
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

int ICM20602::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<ICM20602 *>(arg)->DataReady();
	return 0;
}

void ICM20602::DataReady()
{
	_drdy_timestamp_sample.store(hrt_absolute_time());
	ScheduleNow();
}

bool ICM20602::DataReadyInterruptConfigure()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	// Setup data ready on falling edge
	return (px4_arch_gpiosetevent(_drdy_gpio, false, true, false, &DataReadyInterruptCallback, this) == 0);
}

bool ICM20602::DataReadyInterruptDisable()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	return (px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0);
}

bool ICM20602::RegisterCheck(const register_config_t &reg_cfg)
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

uint8_t ICM20602::RegisterRead(Register reg)
{
	uint8_t cmd[2] {};
	cmd[0] = static_cast<uint8_t>(reg) | DIR_READ;
	transfer(cmd, cmd, sizeof(cmd));
	return cmd[1];
}

void ICM20602::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t cmd[2] { (uint8_t)reg, value };
	transfer(cmd, cmd, sizeof(cmd));
}

void ICM20602::RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);

	uint8_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}

uint16_t ICM20602::FIFOReadCount()
{
	// transfer buffer
	struct TransferBuffer {
		uint8_t cmd{static_cast<uint8_t>(Register::FIFO_COUNTH) | DIR_READ};
		uint8_t FIFO_COUNTH{0};
		uint8_t FIFO_COUNTL{0};
	} buffer{};

	// read FIFO count
	if (transfer((uint8_t *)&buffer, (uint8_t *)&buffer, sizeof(buffer)) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return 0;
	}

	return (buffer.FIFO_COUNTH << 8) + buffer.FIFO_COUNTL;
}

bool ICM20602::FIFORead(const hrt_abstime &timestamp_sample, uint8_t samples)
{
	// FIFO transfer buffer
	struct TransferBuffer {
		uint8_t cmd{static_cast<uint8_t>(Register::FIFO_COUNTH) | DIR_READ};
		uint8_t FIFO_COUNTH{0};
		uint8_t FIFO_COUNTL{0};
		FIFO::DATA f[FIFO_MAX_SAMPLES] {};
	} buffer{};

	// cmd + FIFO_COUNTH + FIFO_COUNTL + samples (FIFO::DATA)
	const size_t transfer_size = 3 + math::min(samples * sizeof(FIFO::DATA), FIFO::SIZE);

	if (transfer((uint8_t *)&buffer, (uint8_t *)&buffer, transfer_size) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return false;
	}

	const uint16_t fifo_count_bytes = (buffer.FIFO_COUNTH << 8) + buffer.FIFO_COUNTL;
	const uint8_t fifo_count_samples = fifo_count_bytes / sizeof(FIFO::DATA);

	const uint8_t valid_samples = math::min(samples, fifo_count_samples);

	if ((fifo_count_bytes >= FIFO::SIZE) || (valid_samples > FIFO_MAX_SAMPLES)) {
		perf_count(_fifo_overflow_perf);
		FIFOReset();
		return false;

	} else if (fifo_count_samples == 0) {
		perf_count(_fifo_empty_perf);
		return false;
	}

	if (valid_samples > 0) {
		sensor_imu_fifo_s sensor_imu_fifo{};
		sensor_imu_fifo.timestamp_sample = timestamp_sample;
		sensor_imu_fifo.device_id = get_device_id();
		sensor_imu_fifo.dt = FIFO_SAMPLE_DT;
		sensor_imu_fifo.samples = valid_samples;
		sensor_imu_fifo.accel_scale = ACCEL_SCALE;
		sensor_imu_fifo.gyro_scale = GYRO_SCALE;

		float temperature_sum{0};

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

			temperature_sum += combine(buffer.f[i].TEMP_OUT_H, buffer.f[i].TEMP_OUT_L);
		}

		// use average temperature reading
		const float temperature_avg = temperature_sum / valid_samples;
		const float temperature_C = (temperature_avg / TEMPERATURE_SENSITIVITY) + TEMPERATURE_OFFSET;
		sensor_imu_fifo.temperature = temperature_C;

		sensor_imu_fifo.error_count = perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf)
					      + perf_event_count(_fifo_empty_perf) + perf_event_count(_fifo_overflow_perf);

		sensor_imu_fifo.timestamp = hrt_absolute_time();
		_sensor_imu_fifo_pub.publish(sensor_imu_fifo);

		return true;
	}

	return false;
}

void ICM20602::FIFOReset()
{
	perf_count(_fifo_reset_perf);

	// FIFO_EN: disable FIFO
	RegisterWrite(Register::FIFO_EN, 0);

	// USER_CTRL: reset FIFO
	RegisterSetAndClearBits(Register::USER_CTRL, USER_CTRL_BIT::FIFO_RST, USER_CTRL_BIT::FIFO_EN);

	// reset while FIFO is disabled
	_drdy_timestamp_sample.store(0);

	// FIFO_EN: enable both gyro and accel
	// USER_CTRL: re-enable FIFO
	for (const auto &r : _register_cfg) {
		if ((r.reg == Register::FIFO_EN) || (r.reg == Register::USER_CTRL)) {
			RegisterSetAndClearBits(r.reg, r.set_bits, r.clear_bits);
		}
	}
}

float ICM20602::ReadTemperature()
{
	// transfer buffer
	struct TransferBuffer {
		uint8_t cmd{static_cast<uint8_t>(Register::TEMP_OUT_H) | DIR_READ};
		uint8_t TEMP_OUT_H{0};
		uint8_t TEMP_OUT_L{0};
	} buffer{};

	if (transfer((uint8_t *)&buffer, (uint8_t *)&buffer, sizeof(buffer)) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return NAN;;
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
