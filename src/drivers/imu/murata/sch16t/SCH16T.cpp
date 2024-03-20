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

#include "SCH16T.hpp"

using namespace time_literals;

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

SCH16T::SCH16T(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_drdy_gpio(config.drdy_gpio),
	_px4_accel(get_device_id(), config.rotation),
	_px4_gyro(get_device_id(), config.rotation)
{
	if (_drdy_gpio != 0) {
		_drdy_missed_perf = perf_alloc(PC_COUNT, MODULE_NAME": DRDY missed");
	}

#ifdef SPI6_nRESET_EXTERNAL1
	_hardware_reset_available = true;
#endif
}

SCH16T::~SCH16T()
{
	perf_free(_reset_perf);
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_perf_crc_bad);
	perf_free(_perf_frame_bad);
	perf_free(_drdy_missed_perf);
}

int SCH16T::init()
{
	PX4_INFO("init");

	px4_usleep(250_ms); // wait for power-on time

	int ret = SPI::init(); // this calls probe()

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);
		return ret;
	}

	Reset();

	return PX4_OK;
}

int SCH16T::probe()
{
	PX4_INFO("probe");

	// Power-On Start-Up Time 310 ms
	if (hrt_absolute_time() < 250_ms) {
		PX4_WARN("required Power-On Start-Up Time 250 ms");
	}

	// SCH16 has COMP_ID-register that can be used to identify between versions and e.g. the measurement ranges.
	RegisterRead(COMP_ID);
	uint16_t comp_id = SPI48_DATA_UINT16(RegisterRead(ASIC_ID));
	uint16_t asic_id = SPI48_DATA_UINT16(RegisterRead(ASIC_ID));

	RegisterRead(SN_ID1);
	uint16_t sn_id1 = SPI48_DATA_UINT16(RegisterRead(SN_ID2));
	uint16_t sn_id2 = SPI48_DATA_UINT16(RegisterRead(SN_ID3));
	uint16_t sn_id3 = SPI48_DATA_UINT16(RegisterRead(SN_ID3));

	char serial_num[14];
	snprintf(serial_num, 14, "%05d%01X%04X", sn_id2, sn_id1 & 0x000F, sn_id3);

	PX4_INFO("ASIC_ID: %u\tCOMP_ID: %u", asic_id, comp_id);
	PX4_INFO("Serial number: %s", serial_num);

	// TODO: check if asic/comp IDs match the expected version?
	bool fail = asic_id == 0 || comp_id == 0;

	return fail ? PX4_ERROR : PX4_OK;
}

void SCH16T::Reset()
{
	PX4_INFO("Reset()");

	if (_drdy_gpio) {
		DataReadyInterruptDisable();
	}

	ScheduleClear();

	_state = STATE::RESET_INIT;
	ScheduleNow();
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
	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_perf_crc_bad);
	perf_print_counter(_perf_frame_bad);
	perf_print_counter(_drdy_missed_perf);
}

////////////////////////////////////
// From the datasheet flowchart
//
// - set user controls
// - enable sensor
// - wait 250_ms
// - read all status registers once
// - set EOI
// - wait 3_ms
// - read all status registers twice
// - validate user control registers
// - validate status registers are OK
////////////////////////////////////
void SCH16T::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET_INIT: {
			perf_count(_reset_perf);

			_reset_timestamp = now;
			_failure_count = 0;

			if (_hardware_reset_available) {
				PX4_INFO("SPI6_RESET true");
				SPI6_RESET(true);
				_state = STATE::RESET_HARD;
				ScheduleDelayed(2_ms);

			} else {
				SoftwareReset();
				_state = STATE::CONFIGURE;
				ScheduleDelayed(250_ms); // wait for power-on time
			}

			break;
		}

	case STATE::RESET_HARD: {
			if (_hardware_reset_available) {
				PX4_INFO("SPI6_RESET false");
				SPI6_RESET(false);
			}

			_state = STATE::CONFIGURE;
			ScheduleDelayed(250_ms); // wait for power-on time
			break;
		}

	case STATE::CONFIGURE: {
			// Sets up control registers
			Configure();

			_state = STATE::VALIDATE;
			ScheduleDelayed(250_ms); // wait for power-on time
			break;
		}

	case STATE::VALIDATE: {
			// Read all status registers once
			ReadStatusRegisters();

			// Write EOI and EN_SENSOR
			RegisterWrite(CTRL_MODE, 0b0011);

			// Read all status registers twice
			ReadStatusRegisters();
			ReadStatusRegisters();

			// Check that registers are configured properly and that the sensor status is OK
			bool success = ValidateSensorStatus() && ValidateRegisterConfiguration();

			if (success) {
				_state = STATE::READ;

				if (_drdy_gpio) {
					DataReadyInterruptConfigure();
					ScheduleDelayed(100_ms); // backup schedule as a watchdog timeout

				} else {
					ScheduleOnInterval(SAMPLE_INTERVAL_US, SAMPLE_INTERVAL_US);
				}

			} else {
				PX4_INFO("Configuration validation failed, resetting");
				_state = STATE::RESET_INIT;
				ScheduleDelayed(100_ms);
			}

			break;
		}

	case STATE::READ: {
			hrt_abstime timestamp_sample = now;

			PX4_INFO("STATE::READ");

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
			auto data = ReadData();

			if (data.frame_error) {
				PX4_INFO("frame error");
				perf_count(_bad_transfer_perf);
				_failure_count++;

				if (_failure_count > 10) {
					PX4_INFO("Failure count high, resetting");
					Reset();
					return;
				}

			} else {

				// Publish data
				_px4_accel.set_temperature(data.temp);
				_px4_gyro.set_temperature(data.temp);
				_px4_accel.update(timestamp_sample, data.acc_x, data.acc_y, data.acc_z);
				_px4_gyro.update(timestamp_sample, data.gyro_x, data.gyro_y, data.gyro_z);

				if (_failure_count > 0) {
					_failure_count--;
				}
			}

			break;
		}

	default:
		break;
	} // end switch/case
}

SCH16T::SensorData SCH16T::ReadData()
{
	SensorData data = {};

	// Data registers are 20bit 2s complement
	RegisterRead(RATE_X2);
	uint64_t gyro_x = RegisterRead(RATE_Y2);
	uint64_t gyro_y = RegisterRead(RATE_Z2);
	uint64_t gyro_z = RegisterRead(ACC_X2);
	uint64_t acc_x  = RegisterRead(ACC_Y2);
	uint64_t acc_y  = RegisterRead(ACC_Z2);
	uint64_t acc_z  = RegisterRead(TEMP);
	uint64_t temp   = RegisterRead(TEMP);

	static constexpr uint64_t MASK48_ERROR = 0x001E00000000UL;
	uint64_t values[] = { gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, temp };

	for (auto v : values) {
		// Check for frame errors
		if (v & MASK48_ERROR) {
			PX4_INFO("Frame error");
			data.frame_error = true;
			perf_count(_perf_frame_bad);
		}

		// Validate the CRC
		if (uint8_t(v & 0xff) != CalculateCRC8(v)) {
			PX4_INFO("Invalid CRC");
			data.frame_error = true;;
			perf_count(_perf_crc_bad);
		}
	}

	data.acc_x    = SPI48_DATA_INT32(acc_x);
	data.acc_y    = SPI48_DATA_INT32(acc_y);
	data.acc_z    = SPI48_DATA_INT32(acc_z);
	data.gyro_x   = SPI48_DATA_INT32(gyro_x);
	data.gyro_y   = SPI48_DATA_INT32(gyro_y);
	data.gyro_z   = SPI48_DATA_INT32(gyro_z);
	// Temperature data is always 16 bits wide. Drop 4 LSBs as they are not used.
	data.temp 	  = SPI48_DATA_INT32(temp) >> 4;

	return data;
}

void SCH16T::Configure()
{
	// Filter settings
	RegisterWrite(CTRL_FILT_RATE, 0x0000); 	// default 68Hz
	RegisterWrite(CTRL_FILT_ACC12, 0x0000); // default 68Hz
	RegisterWrite(CTRL_FILT_ACC3, 0x0000); 	// default 68Hz

	// Table 63 CTRL_RATE Register bit description
	// +-------------+-------------------------------------+---------+-------------+
	// | Bit Name    | Bit Description                     | Bits    | Reset Value |
	// +-------------+-------------------------------------+---------+-------------+
	// | DYN_RATE_XYZ1 | Dynamic Range for RATE_X1/Y1/Z1.  | [14:12] | 3b001       |
	// | DYN_RATE_XYZ2 | Dynamic Range for RATE_X2/Y2/Z2.  | [11:9]  | 3b001       |
	// | DEC_RATE_Z2   | Decimation ratio for RATE_Z2.     | [8:6]   | 3b000       |
	// | DEC_RATE_Y2   | Decimation ratio for RATE_Y2.     | [5:3]   | 3b000       |
	// | DEC_RATE_X2   | Decimation ratio for RATE_X2.     | [2:0]   | 3b000       |
	// +-------------+-------------------------------------+---------+-------------+
	uint16_t ctrl_rate = 0x00;
	ctrl_rate |= uint16_t(0b001) << 9; 	// +/- 300 deg/s, 1600 LSB/(deg/s) -- default
	ctrl_rate |= uint16_t(0b001) << 12; // +/- 300 deg/s, 1600 LSB/(deg/s) -- default
	ctrl_rate |= uint16_t(0b011) << 0; 	// Decimation 8, 1475Hz
	ctrl_rate |= uint16_t(0b011) << 3; 	// Decimation 8, 1475Hz
	ctrl_rate |= uint16_t(0b011) << 6; 	// Decimation 8, 1475Hz
	RegisterWrite(CTRL_RATE, ctrl_rate);

	// Set gyro range and scale
	_px4_gyro.set_range(math::radians(300.f));
	_px4_gyro.set_scale(math::radians(1.f / 1600.f)); // scaling 1600 LSB/°/sec -> rad/s per LSB

	// Table 64 ACC12_CTRL Register bit description
	// +--------------+-----------------------------------------+---------+-------------+
	// | Bit Name     | Bit Description                         | Bits    | Reset Value |
	// +--------------+-----------------------------------------+---------+-------------+
	// | DYN_ACC_XYZ1 | Dynamic Range for ACC_X1/Y1/Z1 outputs. | [14:12] | 3b001       |
	// | DYN_ACC_XYZ2 | Dynamic Range for ACC_X2/Y2/Z2 outputs. | [11:9]  | 3b001       |
	// | DEC_ACC_Z2   | Decimation ratio for ACC_Z2 output.     | [8:6]   | 3b000       |
	// | DEC_ACC_Y2   | Decimation ratio for ACC_Y2 output.     | [5:3]   | 3b000       |
	// | DEC_ACC_X2   | Decimation ratio for ACC_X2 output.     | [2:0]   | 3b000       |
	// +--------------+-----------------------------------------+---------+-------------+
	uint16_t ctrl_acc12 = 0x00;
	ctrl_acc12 |= uint16_t(0b001) << 9; 	// +/- 80 m/s^2, 200 LSB/(m/s^2) -- default
	ctrl_acc12 |= uint16_t(0b001) << 12; 	// +/- 80 m/s^2, 200 LSB/(m/s^2) -- default
	ctrl_acc12 |= uint16_t(0b011) << 0; 	// Decimation 8, 1475Hz
	ctrl_acc12 |= uint16_t(0b011) << 3; 	// Decimation 8, 1475Hz
	ctrl_acc12 |= uint16_t(0b011) << 6; 	// Decimation 8, 1475Hz
	RegisterWrite(CTRL_ACC12, ctrl_acc12);

	// Set accel range and scale
	_px4_accel.set_range(80.f);
	_px4_accel.set_scale(1.f / 200.f);

	// Table 65 ACC3_CTRL Register bit description
	// +--------------+----------------------------------+--------+-------------+
	// | Bit Name     | Bit Description                  | Bits   | Reset Value |
	// +--------------+----------------------------------+--------+-------------+
	// | DYN_ACC_XYZ3 | Dynamic Range for ACC_X3/Y3/Z3.  | [2:0]  | 3b000       |
	// +--------------+----------------------------------+--------+-------------+
	uint16_t ctrl_acc3 = 0x00;
	ctrl_acc3 |= uint16_t(0b000) << 0; 	// +/- 80 m/s^2, 100 LSB/(m/s^2) -- default
	RegisterWrite(CTRL_ACC3, ctrl_acc3);

	// - Enable data ready
	RegisterWrite(CTRL_USER_IF, uint16_t(1 << 5));

	// - Enable the sensor
	RegisterWrite(CTRL_MODE, 0b0001);
}

bool SCH16T::ValidateRegisterConfiguration()
{
	bool success = true;

	// TODO: check registers
	PX4_INFO("TODO: validate user control registers");

	if (!success) {
		PX4_INFO("ValidateRegisterConfiguration(): FAIL");
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
			PX4_INFO("ValidateSensorStatus(): FAIL");
			return false;
		}
	}

	return true;
}

void SCH16T::SoftwareReset()
{
	// Writing 4b1010 to this field generates a SPI soft reset. SPI Communication is not allowed during 2ms after SPI SOFTRESET.
	RegisterWrite(CTRL_RESET, 0b1010);
}

uint64_t SCH16T::RegisterRead(uint8_t addr)
{
	uint64_t frame = {};
	frame |= uint64_t(addr) << 38; // Target address offset
	frame |= uint64_t(1) << 35; // FrameType: SPI48BF
	frame |= uint64_t(CalculateCRC8(frame));

	PX4_INFO("RegisterRead: 0x%llx", frame);

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

	// We don't care about the return frame on a write
	(void)TransferSpiFrame(frame);
}

// The SPI protocol (SafeSPI) is 48bit out-of-frame. This means read return frames will be received on the next transfer.
uint64_t SCH16T::TransferSpiFrame(uint64_t frame)
{
	set_frequency(SPI_SPEED);

	uint16_t tx[3];
	uint16_t rx[3];

	// Swap 16-bit word order of data to send. By default the SPI_Transfer() sends
	// the lower 16 bit word first so the word order is swapped here to comply with
	// MSB first requirement.
	tx[0] = (frame >> 32) & 0xffffUL;
	tx[1] = (frame >> 16) & 0xffffUL;
	tx[2] = frame & 0xffffUL;

	transferhword(tx, rx, 3);
	px4_udelay(SPI_STALL_PERIOD);

	PX4_INFO("RECEIVED");
	for (auto r : rx) {
		PX4_INFO("%u", r);
	}

	uint64_t value = ((((uint64_t)rx[0]) << 32) & 0x0000ffff00000000) |
			 ((((uint64_t)rx[1]) << 16) & 0x00000000ffff0000) |
			 (((uint64_t)rx[2]) & 0x000000000000ffff);

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