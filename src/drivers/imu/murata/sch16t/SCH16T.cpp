/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#include "SCH16T.hpp"

using namespace time_literals;

#define SPI48_DATA_INT32(a)  	(((int32_t)(((a) << 4)  & 0xfffff000UL)) >> 12)
#define SPI48_DATA_UINT32(a)	((uint32_t)(((a) >> 8)  & 0x000fffffUL))
#define SPI48_DATA_UINT16(a) 	((uint16_t)(((a) >> 8)  & 0x0000ffffUL))

static constexpr uint32_t POWER_ON_TIME = 250_ms;

SCH16T::SCH16T(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_px4_accel(get_device_id(), config.rotation),
	_px4_gyro(get_device_id(), config.rotation),
	_drdy_gpio(config.drdy_gpio)
{
	if (_drdy_gpio != 0) {
		_drdy_missed_perf = perf_alloc(PC_COUNT, MODULE_NAME": DRDY missed");
	}

#if defined(SPI6_nRESET_EXTERNAL1)
	_hardware_reset_available = true;
#endif
}

SCH16T::~SCH16T()
{
	perf_free(_reset_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_perf_crc_bad);
	perf_free(_perf_frame_bad);
	perf_free(_drdy_missed_perf);
}

int SCH16T::init()
{
	px4_usleep(POWER_ON_TIME);

	int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);
		return ret;
	}

	Reset();

	return PX4_OK;
}

int SCH16T::probe()
{
	if (hrt_absolute_time() < POWER_ON_TIME) {
		PX4_WARN("Required Power-On Start-Up Time %" PRIu32 " ms", POWER_ON_TIME);
	}

	RegisterRead(COMP_ID);
	uint16_t comp_id = SPI48_DATA_UINT16(RegisterRead(ASIC_ID));
	uint16_t asic_id = SPI48_DATA_UINT16(RegisterRead(ASIC_ID));

	RegisterRead(SN_ID1);
	uint16_t sn_id1 = SPI48_DATA_UINT16(RegisterRead(SN_ID2));
	uint16_t sn_id2 = SPI48_DATA_UINT16(RegisterRead(SN_ID3));
	uint16_t sn_id3 = SPI48_DATA_UINT16(RegisterRead(SN_ID3));

	char serial_str[14];
	snprintf(serial_str, 14, "%05d%01X%04X", sn_id2, sn_id1 & 0x000F, sn_id3);

	PX4_INFO("Serial:\t %s", serial_str);
	PX4_INFO("COMP_ID:\t 0x%0x", comp_id);
	PX4_INFO("ASIC_ID:\t 0x%0x", asic_id);

	// SCH16T-K01 	- 	ID hex = 0x0020
	// SCH1633-B13 	- 	ID hex = 0x0017
	bool success = asic_id == 0x20 && comp_id == 0x17;

	return success ? PX4_OK : PX4_ERROR;
}

void SCH16T::Reset()
{
	if (_drdy_gpio) {
		DataReadyInterruptDisable();
	}

	ScheduleClear();

	_state = STATE::RESET_INIT;
	ScheduleNow();
}

void SCH16T::ResetSpi6(bool reset)
{
#if defined(SPI6_RESET)
	SPI6_RESET(reset);
#endif
}

void SCH16T::exit_and_cleanup()
{
	if (_drdy_gpio) {
		DataReadyInterruptDisable();
	}

	I2CSPIDriverBase::exit_and_cleanup();
}

void SCH16T::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_reset_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_perf_crc_bad);
	perf_print_counter(_perf_frame_bad);
	perf_print_counter(_drdy_missed_perf);
}

void SCH16T::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET_INIT: {
			perf_count(_reset_perf);

			_failure_count = 0;

			if (_hardware_reset_available) {
				PX4_INFO("Resetting (hard)");
				ResetSpi6(true);
				_state = STATE::RESET_HARD;
				ScheduleDelayed(2_ms);

			} else {
				PX4_INFO("Resetting (soft)");
				SoftwareReset();
				_state = STATE::CONFIGURE;
				ScheduleDelayed(POWER_ON_TIME);
			}

			break;
		}

	case STATE::RESET_HARD: {
			if (_hardware_reset_available) {
				ResetSpi6(false);
			}

			_state = STATE::CONFIGURE;
			ScheduleDelayed(POWER_ON_TIME);
			break;
		}

	case STATE::CONFIGURE: {
			Configure();

			_state = STATE::LOCK_CONFIGURATION;
			ScheduleDelayed(POWER_ON_TIME);
			break;
		}

	case STATE::LOCK_CONFIGURATION: {
			ReadStatusRegisters(); // Read all status registers once
			RegisterWrite(CTRL_MODE, (EOI | EN_SENSOR)); // Write EOI and EN_SENSOR

			_state = STATE::VALIDATE;
			ScheduleDelayed(5_ms);
			break;
		}

	case STATE::VALIDATE: {
			ReadStatusRegisters(); // Read all status registers twice
			ReadStatusRegisters();

			// Check that registers are configured properly and that the sensor status is OK
			if (ValidateSensorStatus() && ValidateRegisterConfiguration()) {
				_state = STATE::READ;

				if (_drdy_gpio) {
					DataReadyInterruptConfigure();
					ScheduleDelayed(100_ms); // backup schedule as a watchdog timeout

				} else {
					ScheduleOnInterval(SAMPLE_INTERVAL_US, SAMPLE_INTERVAL_US);
				}

			} else {
				_state = STATE::RESET_INIT;
				ScheduleDelayed(100_ms);
			}

			break;
		}

	case STATE::READ: {
			hrt_abstime timestamp_sample = now;

			if (_drdy_gpio) {
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

			// Collect the data
			SensorData data = {};

			if (ReadData(&data)) {
				_px4_accel.set_temperature(float(data.temp) / 100.f); // Temperature signal sensitivity is 100 LSB/°C
				_px4_gyro.set_temperature(float(data.temp) / 100.f);
				_px4_accel.update(timestamp_sample, data.acc_x, data.acc_y, data.acc_z);
				_px4_gyro.update(timestamp_sample, data.gyro_x, data.gyro_y, data.gyro_z);

				if (_failure_count > 0) {
					_failure_count--;
				}

			} else {
				perf_count(_bad_transfer_perf);
				_failure_count++;
			}

			// Reset if successive failures
			if (_failure_count > 10) {
				PX4_INFO("Failure count high, resetting");
				Reset();
				return;
			}

			break;
		}

	default:
		break;
	} // end switch/case
}

bool SCH16T::ReadData(SensorData *data)
{
	// Data registers are 20bit 2s complement
	RegisterRead(RATE_X2);
	uint64_t gyro_x = RegisterRead(RATE_Y2);
	uint64_t gyro_y = RegisterRead(RATE_Z2);
	uint64_t gyro_z = RegisterRead(ACC_X3);
	uint64_t acc_x  = RegisterRead(ACC_Y3);
	uint64_t acc_y  = RegisterRead(ACC_Z3);
	uint64_t acc_z  = RegisterRead(TEMP);
	uint64_t temp   = RegisterRead(TEMP);

	static constexpr uint64_t MASK48_ERROR = 0x001E00000000UL;
	uint64_t values[] = { gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, temp };

	for (auto v : values) {
		// Check for frame errors
		if (v & MASK48_ERROR) {
			perf_count(_perf_frame_bad);
			return false;
		}

		// Validate the CRC
		if (uint8_t(v & 0xff) != CalculateCRC8(v)) {
			perf_count(_perf_crc_bad);
			return false;
		}
	}

	// Data registers are 20bit 2s complement
	data->acc_x    = SPI48_DATA_INT32(acc_x);
	data->acc_y    = SPI48_DATA_INT32(acc_y);
	data->acc_z    = SPI48_DATA_INT32(acc_z);
	data->gyro_x   = SPI48_DATA_INT32(gyro_x);
	data->gyro_y   = SPI48_DATA_INT32(gyro_y);
	data->gyro_z   = SPI48_DATA_INT32(gyro_z);
	// Temperature data is always 16 bits wide. Drop 4 LSBs as they are not used.
	data->temp 	  = SPI48_DATA_INT32(temp) >> 4;

	// Conver to PX4 coordinate system (FLU to FRD)
	data->acc_x = data->acc_x;
	data->acc_y = -data->acc_y;
	data->acc_z = -data->acc_z;
	data->gyro_x = data->gyro_x;
	data->gyro_y = -data->gyro_y;
	data->gyro_z = -data->gyro_z;

	return true;
}

void SCH16T::Configure()
{
	for (auto &r : _registers) {
		RegisterWrite(r.addr, r.value);
	}

	RegisterWrite(CTRL_USER_IF, DRY_DRV_EN); // Enable data ready
	RegisterWrite(CTRL_MODE, EN_SENSOR); // Enable the sensor

	// NOTE: we use ACC3 for the higher range. The DRDY frequency adjusts to whichever register bank is
	// being sampled from (decimated vs interpolated outputs). RATE_XYZ2 is decimated and RATE_XYZ1 is interpolated.
	_px4_gyro.set_range(math::radians(327.68f)); // +-/ 300°/sec calibrated range, 327.68°/sec electrical headroom (20bit)
	_px4_gyro.set_scale(math::radians(1.f / 1600.f)); // scaling 1600 LSB/°/sec -> rad/s per LSB
	_px4_accel.set_range(260.f);
	_px4_accel.set_scale(1.f / 1600.f);
}

bool SCH16T::ValidateRegisterConfiguration()
{
	bool success = true;

	for (auto &r : _registers) {
		RegisterRead(r.addr); // double read, wasteful but makes the code cleaner, not high rate so doesn't matter anyway
		auto value = SPI48_DATA_UINT16(RegisterRead(r.addr));

		if (value != r.value) {
			PX4_INFO("Register 0x%0x misconfigured: 0x%0x", r.addr, value);
			success = false;
		}
	}

	return success;
}

void SCH16T::ReadStatusRegisters()
{
	RegisterRead(STAT_SUM);
	_sensor_status.summary 		= SPI48_DATA_UINT16(RegisterRead(STAT_SUM_SAT));
	_sensor_status.saturation 	= SPI48_DATA_UINT16(RegisterRead(STAT_COM));
	_sensor_status.common 		= SPI48_DATA_UINT16(RegisterRead(STAT_RATE_COM));
	_sensor_status.rate_common 	= SPI48_DATA_UINT16(RegisterRead(STAT_RATE_X));
	_sensor_status.rate_x 		= SPI48_DATA_UINT16(RegisterRead(STAT_RATE_Y));
	_sensor_status.rate_y 		= SPI48_DATA_UINT16(RegisterRead(STAT_RATE_Z));
	_sensor_status.rate_z 		= SPI48_DATA_UINT16(RegisterRead(STAT_ACC_X));
	_sensor_status.acc_x 		= SPI48_DATA_UINT16(RegisterRead(STAT_ACC_Y));
	_sensor_status.acc_y 		= SPI48_DATA_UINT16(RegisterRead(STAT_ACC_Z));
	_sensor_status.acc_z 		= SPI48_DATA_UINT16(RegisterRead(STAT_ACC_Z));
}

bool SCH16T::ValidateSensorStatus()
{
	auto &s = _sensor_status;
	uint16_t values[] = { s.summary, s.saturation, s.common, s.rate_common, s.rate_x, s.rate_y, s.rate_z, s.acc_x, s.acc_y, s.acc_z };

	for (auto v : values) {
		if (v != 0xFFFF) {
			PX4_INFO("Sensor status failed");
			return false;
		}
	}

	return true;
}

void SCH16T::SoftwareReset()
{
	RegisterWrite(CTRL_RESET, SPI_SOFT_RESET);
}

uint64_t SCH16T::RegisterRead(uint8_t addr)
{
	uint64_t frame = {};
	frame |= uint64_t(addr) << 38; // Target address offset
	frame |= uint64_t(1) << 35; // FrameType: SPI48BF
	frame |= uint64_t(CalculateCRC8(frame));

	return TransferSpiFrame(frame);
}

// Non-data registers are the only writable ones and are 16 bit or less
void SCH16T::RegisterWrite(uint8_t addr, uint16_t value)
{
	uint64_t frame = {};
	frame |= uint64_t(1) << 37; // Write bit
	frame |= uint64_t(addr) << 38; // Target address offset
	frame |= uint64_t(1) << 35; // FrameType: SPI48BF
	frame |= uint64_t(value) << 8;
	frame |= uint64_t(CalculateCRC8(frame));

	// We don't care about the return frame on a write
	(void)TransferSpiFrame(frame);
}

// The SPI protocol (SafeSPI) is 48bit out-of-frame. This means read return frames will be received on the next transfer.
uint64_t SCH16T::TransferSpiFrame(uint64_t frame)
{
	set_frequency(SPI_SPEED);

	uint16_t buf[3];

	for (int index = 0; index < 3; index++) {
		buf[3 - index - 1] = (frame >> (index << 4)) & 0xFFFF;
	}

	transferhword(buf, buf, 3);

#if defined(DEBUG_BUILD)
	PX4_INFO("TransferSpiFrame: 0x%llx", frame);

	PX4_INFO("RECEIVED");

	for (auto r : buf) {
		PX4_INFO("%u", r);
	}

#endif

	uint64_t value = {};

	for (int index = 0; index < 3; index++) {
		value |= (uint64_t)buf[index] << ((3 - index - 1) << 4);
	}

	return value;
}

int SCH16T::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<SCH16T *>(arg)->DataReady();
	return 0;
}

void SCH16T::DataReady()
{
	_drdy_timestamp_sample.store(hrt_absolute_time());
	ScheduleNow();
}

bool SCH16T::DataReadyInterruptConfigure()
{
	// Setup data ready on falling edge
	return px4_arch_gpiosetevent(_drdy_gpio, true, false, false, &DataReadyInterruptCallback, this) == 0;
}

bool SCH16T::DataReadyInterruptDisable()
{
	return px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0;
}

uint8_t SCH16T::CalculateCRC8(uint64_t frame)
{
	uint64_t data = frame & 0xFFFFFFFFFF00LL;
	uint8_t crc = 0xFF;

	for (int i = 47; i >= 0; i--) {
		uint8_t data_bit = data >> i & 0x01;
		crc = crc & 0x80 ? (uint8_t)((crc << 1) ^ 0x2F) ^ data_bit : (uint8_t)(crc << 1) | data_bit;
	}

	return crc;
}
