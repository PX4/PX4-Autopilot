/****************************************************************************
 *
 *   Copyright (c) 2021-2022 PX4 Development Team. All rights reserved.
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

#include "ICM42670P.hpp"

#include <lib/parameters/param.h>

using namespace time_literals;

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

ICM42670P::ICM42670P(const I2CSPIDriverConfig &config):
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

ICM42670P::~ICM42670P()
{
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_fifo_empty_perf);
	perf_free(_fifo_overflow_perf);
	perf_free(_fifo_reset_perf);
	perf_free(_fifo_timestamp_error_perf);
	perf_free(_drdy_missed_perf);
}

int ICM42670P::init()
{
	int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool ICM42670P::Reset()
{
	_state = STATE::RESET;
	DataReadyInterruptDisable();
	ScheduleClear();
	ScheduleNow();
	return true;
}

void ICM42670P::exit_and_cleanup()
{
	DataReadyInterruptDisable();
	I2CSPIDriverBase::exit_and_cleanup();
}

void ICM42670P::print_status()
{
	I2CSPIDriverBase::print_status();

	PX4_INFO("FIFO empty interval: %d us (%.1f Hz)", _fifo_empty_interval_us, 1e6 / _fifo_empty_interval_us);

	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_fifo_empty_perf);
	perf_print_counter(_fifo_overflow_perf);
	perf_print_counter(_fifo_reset_perf);
	perf_print_counter(_fifo_timestamp_error_perf);
	perf_print_counter(_drdy_missed_perf);
}

int ICM42670P::probe()
{
	// 3 retries
	for (int retry = 0; retry < 3; retry++) {
		uint8_t whoami = RegisterRead(Register::BANK_0::WHO_AM_I);

		if (whoami == WHOAMI) {
			return PX4_OK;

		} else {
			DEVICE_DEBUG("unexpected WHO_AM_I 0x%02x", whoami);
		}
	}

	return PX4_ERROR;
}

void ICM42670P::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET:
		// SIGNAL_PATH_RESET: Software Reset (auto clear bit)
		RegisterWrite(Register::BANK_0::SIGNAL_PATH_RESET, SIGNAL_PATH_RESET_BIT::SOFT_RESET_DEVICE_CONFIG);
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;
		ScheduleDelayed(1_ms); // wait 1 ms for soft reset to be effective
		break;

	case STATE::WAIT_FOR_RESET:
		if ((RegisterRead(Register::BANK_0::WHO_AM_I) == WHOAMI)
		    && (RegisterRead(Register::BANK_0::SIGNAL_PATH_RESET) == 0x00)
		    && (RegisterRead(Register::BANK_0::INT_STATUS) & INT_STATUS_BIT::RESET_DONE_INT)) {

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
				PX4_DEBUG("Reset not complete, check again in 100 ms");
				ScheduleDelayed(100_ms);
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

		if (DataReadyInterruptConfigure()) {
			_data_ready_interrupt_enabled = true;

			// backup schedule as a watchdog timeout
			ScheduleDelayed(100_ms);

		} else {
			_data_ready_interrupt_enabled = false;
			ScheduleOnInterval(_fifo_empty_interval_us, _fifo_empty_interval_us);
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

			bool success = false;

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
						const int extra_samples = samples - _fifo_gyro_samples;
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

					if (samples == _fifo_gyro_samples) {
						if (FIFORead(now, samples)) {
							success = true;

							if (_failure_count > 0) {
								_failure_count--;
							}
						}
					}
				}
			}

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
				if (RegisterCheck(_register_bank0_cfg[_checked_register_bank0])
				    && RegisterCheck(_register_mreg1_cfg[_checked_register_mreg1])
				   ) {
					_last_config_check_timestamp = now;
					_checked_register_bank0 = (_checked_register_bank0 + 1) % size_register_bank0_cfg;
					_checked_register_mreg1 = (_checked_register_mreg1 + 1) % size_register_mreg1_cfg;

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

void ICM42670P::ConfigureSampleRate(int sample_rate)
{
	// round down to nearest FIFO sample dt
	const float min_interval = FIFO_SAMPLE_DT;
	_fifo_empty_interval_us = math::max(roundf((1e6f / (float)sample_rate) / min_interval) * min_interval, min_interval);

	_fifo_gyro_samples = roundf(math::min((float)_fifo_empty_interval_us / (1e6f / RATE), (float)FIFO_MAX_SAMPLES));

	// recompute FIFO empty interval (us) with actual gyro sample limit
	_fifo_empty_interval_us = _fifo_gyro_samples * (1e6f / RATE);

	ConfigureFIFOWatermark(_fifo_gyro_samples);
}

void ICM42670P::ConfigureFIFOWatermark(uint8_t samples)
{
	// FIFO watermark threshold in number of bytes
	const uint16_t fifo_watermark_threshold = samples * sizeof(FIFO::DATA);

	for (auto &r : _register_bank0_cfg) {
		if (r.reg == Register::BANK_0::FIFO_CONFIG2) {
			// FIFO_WM[7:0]  FIFO_CONFIG2
			r.set_bits = fifo_watermark_threshold & 0xFF;

		} else if (r.reg == Register::BANK_0::FIFO_CONFIG3) {
			// FIFO_WM[11:8] FIFO_CONFIG3
			r.set_bits = (fifo_watermark_threshold >> 8) & 0x0F;
		}
	}
}

bool ICM42670P::Configure()
{
	// first set and clear all configured register bits
	for (const auto &reg_cfg : _register_bank0_cfg) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
	}

	for (const auto &reg_cfg : _register_mreg1_cfg) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
	}

	// now check that all are configured
	bool success = true;

	for (const auto &reg_cfg : _register_bank0_cfg) {
		if (!RegisterCheck(reg_cfg)) {
			success = false;
		}
	}

	for (const auto &reg_cfg : _register_mreg1_cfg) {
		if (!RegisterCheck(reg_cfg)) {
			success = false;
		}
	}

	return success;
}

int ICM42670P::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<ICM42670P *>(arg)->DataReady();
	return 0;
}

void ICM42670P::DataReady()
{
	_drdy_timestamp_sample.store(hrt_absolute_time());
	ScheduleNow();
}

bool ICM42670P::DataReadyInterruptConfigure()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	// Setup data ready on falling edge
	return (px4_arch_gpiosetevent(_drdy_gpio, false, true, false, &DataReadyInterruptCallback, this) == 0);
}

bool ICM42670P::DataReadyInterruptDisable()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	return (px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0);
}

template <typename T>
bool ICM42670P::RegisterCheck(const T &reg_cfg)
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

uint8_t ICM42670P::RegisterRead(Register::BANK_0 reg)
{
	uint8_t cmd[2] {};
	cmd[0] = static_cast<uint8_t>(reg) | DIR_READ;
	transfer(cmd, cmd, sizeof(cmd));
	return cmd[1];
}

uint8_t ICM42670P::RegisterRead(Register::MREG1 reg)
{
	// BLK_SEL_R must be set to 0
	RegisterWrite(Register::BANK_0::BLK_SEL_R, 0x00);

	// MADDR_R must be set to 0x14 (address of the MREG1 register being accessed)
	RegisterWrite(Register::BANK_0::MADDR_R, (uint8_t)reg);

	// Wait for 10 µs
	px4_udelay(10);

	// Read register M_R to access the value in MREG1 register 0x14
	uint8_t value = RegisterRead(Register::BANK_0::M_R);

	// Wait for 10 µs
	// Host must not access any other register for 10 µs once MREG1, MREG2 or MREG3 access is kicked off.
	px4_udelay(10);

	return value;
}

void ICM42670P::RegisterWrite(Register::BANK_0 reg, uint8_t value)
{
	uint8_t cmd[2] { (uint8_t)reg, value };
	transfer(cmd, cmd, sizeof(cmd));
}

void ICM42670P::RegisterWrite(Register::MREG1 reg, uint8_t value)
{
	// BLK_SEL_W must be set to 0
	RegisterWrite(Register::BANK_0::BLK_SEL_W, 0x00);

	// MADDR_W must be set to address of the MREG1 register being accessed
	RegisterWrite(Register::BANK_0::MADDR_W, (uint8_t)reg);

	// M_W must be set to the desired value
	RegisterWrite(Register::BANK_0::M_W, value);

	// Wait for 10 µs
	// Host must not access any other register for 10 µs once MREG1, MREG2 or MREG3 access is kicked off.
	px4_udelay(10);
}

template <typename T>
void ICM42670P::RegisterSetAndClearBits(T reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);

	uint8_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}

uint16_t ICM42670P::FIFOReadCount()
{
	// transfer buffer
	struct TransferBuffer {
		uint8_t cmd{static_cast<uint8_t>(Register::BANK_0::FIFO_COUNTH) | DIR_READ};
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

bool ICM42670P::FIFORead(const hrt_abstime &timestamp_sample, uint8_t samples)
{
	// FIFO transfer buffer
	struct FIFOTransferBuffer {
		uint8_t cmd{static_cast<uint8_t>(Register::BANK_0::INT_STATUS) | DIR_READ};
		uint8_t INT_STATUS{0};
		uint8_t FIFO_COUNTH{0};
		uint8_t FIFO_COUNTL{0};
		FIFO::DATA f[FIFO_MAX_SAMPLES] {};
	} buffer{};

	// cmd + INT_STATUS + FIFO_COUNTH + FIFO_COUNTL + samples (FIFO::DATA)
	const size_t transfer_size = 4 + math::min(samples * sizeof(FIFO::DATA), FIFO::SIZE);

	if (transfer((uint8_t *)&buffer, (uint8_t *)&buffer, transfer_size) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return false;
	}

	if (buffer.INT_STATUS & INT_STATUS_BIT::FIFO_FULL_INT) {
		perf_count(_fifo_overflow_perf);
		FIFOReset();
		return false;
	}

	const uint16_t fifo_count_bytes = (buffer.FIFO_COUNTH << 8) + buffer.FIFO_COUNTL;

	if (fifo_count_bytes >= FIFO::SIZE) {
		perf_count(_fifo_overflow_perf);
		FIFOReset();
		return false;
	}

	const uint8_t fifo_count_samples = fifo_count_bytes / sizeof(FIFO::DATA);

	if (fifo_count_samples == 0) {
		perf_count(_fifo_empty_perf);
		return false;
	}

	sensor_imu_fifo_s sensor_imu_fifo{};

	sensor_imu_fifo.timestamp_sample = timestamp_sample;
	sensor_imu_fifo.device_id = get_device_id();
	sensor_imu_fifo.dt = FIFO_SAMPLE_DT;
	sensor_imu_fifo.samples = 0;
	sensor_imu_fifo.accel_scale = CONSTANTS_ONE_G / 8192.f; // highres accel data 8192 LSB/g
	sensor_imu_fifo.gyro_scale = math::radians(1.f / 131.f); // highres gyro data 131 LSB/dps

	// check FIFO header in every sample
	uint8_t valid_samples = 0;

	float temperature_sum = 0;

	float timestamp_interval_sum = 0;
	int timestamp_interval_sum_count = 0;

	bool accel_scale_16bit = false; // 18-bits of accelerometer data
	bool gyro_scale_16bit = false; // 20-bits of gyroscope data

	for (int i = 0; i < math::min(samples, fifo_count_samples); i++) {
		const FIFO::DATA &fifo = buffer.f[i];

		// With FIFO_ACCEL_EN and FIFO_GYRO_EN header should be 8’b_0110_10xx
		const uint8_t FIFO_HEADER = fifo.FIFO_Header;

		const bool HEADER_MSG       = FIFO_HEADER & FIFO::FIFO_HEADER_BIT::HEADER_MSG; // FIFO is empty
		const bool HEADER_ACCEL     = FIFO_HEADER & FIFO::FIFO_HEADER_BIT::HEADER_ACCEL;
		const bool HEADER_GYRO      = FIFO_HEADER & FIFO::FIFO_HEADER_BIT::HEADER_GYRO;
		// 3:2 HEADER_TIMESTAMP_FSYNC
		const bool HEADER_ODR_ACCEL = FIFO_HEADER & FIFO::FIFO_HEADER_BIT::HEADER_ODR_ACCEL; // ODR for accel is different
		const bool HEADER_ODR_GYRO  = FIFO_HEADER & FIFO::FIFO_HEADER_BIT::HEADER_ODR_GYRO;  // ODR for gyro is different

		if (!HEADER_MSG && HEADER_ACCEL && HEADER_GYRO && !HEADER_ODR_ACCEL && !HEADER_ODR_GYRO) {

			// 20 bit data scaled to 16 bit
			sensor_imu_fifo.accel_scale = CONSTANTS_ONE_G / 2048.f;

			sensor_imu_fifo.accel_x[valid_samples] = combine(fifo.ACCEL_DATA_X1, fifo.ACCEL_DATA_X0);
			sensor_imu_fifo.accel_y[valid_samples] = combine(fifo.ACCEL_DATA_Y1, fifo.ACCEL_DATA_Y0);
			sensor_imu_fifo.accel_z[valid_samples] = combine(fifo.ACCEL_DATA_Z1, fifo.ACCEL_DATA_Z0);

			sensor_imu_fifo.gyro_x[i] = combine(fifo.GYRO_DATA_X1, fifo.GYRO_DATA_X0);
			sensor_imu_fifo.gyro_y[i] = combine(fifo.GYRO_DATA_Y1, fifo.GYRO_DATA_Y0);
			sensor_imu_fifo.gyro_z[i] = combine(fifo.GYRO_DATA_Z1, fifo.GYRO_DATA_Z0);

			// temperature
			const int16_t TEMP_DATA = combine(fifo.TEMP_DATA1, fifo.TEMP_DATA0);

			// sample invalid if -32768
			if (TEMP_DATA != -32768) {
				temperature_sum += TEMP_DATA;

			} else {
				break;
			}


			// HEADER_TIMESTAMP_FSYNC - 0b10: Packet contains ODR Timestamp
			if (FIFO_HEADER & Bit3) {
				const uint16_t timestamp = (fifo.TimeStamp_h << 8) + fifo.TimeStamp_l;

				if (_timestamp_prev != 0) {
					// If TMST_RES = 0 (corresponding to timestamp resolution of 1µs), timestamp interval reported in FIFO requires scaling by a factor of 32/30.
					// Document Number: DS-000347 Revision: 1.5 Page 59 of 110
					static constexpr float FIFO_DT_SCALE = 32.f / 30.f;

					float dt = 0;

					if (timestamp > _timestamp_prev) {
						dt = static_cast<float>(timestamp - _timestamp_prev) * FIFO_DT_SCALE;

					} else if (timestamp < _timestamp_prev) {
						// uint16_t rollover
						uint32_t timestamp_new = UINT16_MAX + timestamp;
						dt = static_cast<float>(timestamp_new - _timestamp_prev) * FIFO_DT_SCALE;
					}

					timestamp_interval_sum += dt;
					timestamp_interval_sum_count++;

					// check dt is within +=2% of expected value
					if ((dt < (FIFO_SAMPLE_DT * 0.98f)) || (dt > (FIFO_SAMPLE_DT * 1.02f))) {
						perf_count(_fifo_timestamp_error_perf);
					}
				}

				_timestamp_prev = timestamp;
			}

			valid_samples++;
		}
	}

	if (valid_samples > 0) {

		sensor_imu_fifo.samples = valid_samples;

		for (int i = 0; i < sensor_imu_fifo.samples; i++) {
			// sensor's frame is +x forward, +y left, +z up
			//  flip y & z to publish right handed with z down (x forward, y right, z down)

			// sensor_imu_fifo.accel_x[i]
			sensor_imu_fifo.accel_y[i] = math::negate(sensor_imu_fifo.accel_y[i]);
			sensor_imu_fifo.accel_z[i] = math::negate(sensor_imu_fifo.accel_z[i]);
			rotate_3i(_rotation, sensor_imu_fifo.accel_x[i], sensor_imu_fifo.accel_y[i], sensor_imu_fifo.accel_z[i]);

			// sensor_imu_fifo.gyro_x[i]
			sensor_imu_fifo.gyro_y[i] = math::negate(sensor_imu_fifo.gyro_y[i]);
			sensor_imu_fifo.gyro_z[i] = math::negate(sensor_imu_fifo.gyro_z[i]);
			rotate_3i(_rotation, sensor_imu_fifo.gyro_x[i], sensor_imu_fifo.gyro_y[i], sensor_imu_fifo.gyro_z[i]);
		}

		const float temperature_avg = temperature_sum / valid_samples;

		// use average temperature reading
		const float TEMP_degC = (temperature_avg / TEMPERATURE_SENSITIVITY) + TEMPERATURE_OFFSET;

		if (PX4_ISFINITE(TEMP_degC)) {
			sensor_imu_fifo.temperature = TEMP_degC;

		} else {
			perf_count(_bad_transfer_perf);
			return false;
		}

		if (timestamp_interval_sum > 0) {
			const float dt_avg = (timestamp_interval_sum / timestamp_interval_sum_count);

			// check dt is within +=1% of expected value
			if ((dt_avg < (FIFO_SAMPLE_DT * 0.99f)) || (dt_avg > (FIFO_SAMPLE_DT * 1.01f))) {
				PX4_ERR("DT error %.6f", (double)dt_avg);
				perf_count(_fifo_timestamp_error_perf);

			} else {
				sensor_imu_fifo.dt = dt_avg;
			}
		}

		sensor_imu_fifo.error_count = perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf) +
					      perf_event_count(_fifo_empty_perf) + perf_event_count(_fifo_overflow_perf);

		sensor_imu_fifo.timestamp = hrt_absolute_time();
		_sensor_imu_fifo_pub.publish(sensor_imu_fifo);

		return true;
	}

	return false;
}

void ICM42670P::FIFOReset()
{
	perf_count(_fifo_reset_perf);

	// SIGNAL_PATH_RESET: FIFO flush
	RegisterSetBits(Register::BANK_0::SIGNAL_PATH_RESET, SIGNAL_PATH_RESET_BIT::FIFO_FLUSH);

	// reset while FIFO is disabled
	_drdy_timestamp_sample.store(0);

	_timestamp_prev = 0;
}
