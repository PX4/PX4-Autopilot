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
	ModuleParams(nullptr),
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
	perf_free(_drdy_missed_perf);
	perf_free(_perf_general_error);
	perf_free(_perf_command_error);
	perf_free(_perf_saturation_error);
	perf_free(_perf_doing_initialization);
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

	bool success = asic_id == 0x21 && comp_id == 0x23;

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
	perf_print_counter(_drdy_missed_perf);
	perf_print_counter(_perf_general_error);
	perf_print_counter(_perf_command_error);
	perf_print_counter(_perf_saturation_error);
	perf_print_counter(_perf_doing_initialization);
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
			ConfigurationFromParameters();
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
					ScheduleOnInterval(_sample_interval_us, _sample_interval_us);
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

				if ((now - drdy_timestamp_sample) < _sample_interval_us) {
					timestamp_sample = drdy_timestamp_sample;

				} else {
					perf_count(_drdy_missed_perf);
				}

				// push backup schedule back
				ScheduleDelayed(_sample_interval_us * 2);
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
	// Register reads return 48bits. See SafeSpi 48bit out-of-frame protocol.
	RegisterRead(RATE_X2);
	uint64_t gyro_x = RegisterRead(RATE_Y2);
	uint64_t gyro_y = RegisterRead(RATE_Z2);
	uint64_t gyro_z = RegisterRead(ACC_X2);
	uint64_t acc_x  = RegisterRead(ACC_Y2);
	uint64_t acc_y  = RegisterRead(ACC_Z2);
	uint64_t acc_z  = RegisterRead(TEMP);
	uint64_t temp   = RegisterRead(TEMP);

	static constexpr uint64_t MASK48_GENERAL_ERROR = 	0b00000000'00010000'00000000'00000000'00000000'00000000;
	static constexpr uint64_t MASK48_COMMAND_ERROR = 	0b00000000'00001000'00000000'00000000'00000000'00000000;
	static constexpr uint64_t MASK48_SATURATION_ERROR = 0b00000000'00000100'00000000'00000000'00000000'00000000;
	static constexpr uint64_t MASK48_DOING_INIT = 		0b00000000'00000110'00000000'00000000'00000000'00000000;

	uint64_t values[] = { gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, temp };

	for (auto v : values) {
		// [1b ][1b][ 2b ]
		// [IDS][CE][S1:0]
		// IDS: Internal Data Status indication. SCH16T uses this field to indicate common cause error. This is redundant, more accurate info
		// 		is seen from sensor status (S1:S0).
		// CE: Command Error indication. SCH16T reports only semantically invalid frame content using this field. SPI protocol
		// 		level errors are indicated with High-Z on MISO pin.
		// S1,0: Sensor status indication.
		// 		00: Normal operation
		// 		01: Error status
		// 		10: Saturation error
		// 		11: Initialization running

		if (v & MASK48_GENERAL_ERROR) {
			perf_count(_perf_general_error);
			return false;

		} else if (v & MASK48_COMMAND_ERROR) {
			perf_count(_perf_command_error);
			return false;

		} else if ((v & MASK48_DOING_INIT) == MASK48_DOING_INIT) {
			perf_count(_perf_doing_initialization);
			return false;

		} else if (v & MASK48_SATURATION_ERROR) {
			perf_count(_perf_saturation_error);
			// Don't consider saturation an error
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

void SCH16T::ConfigurationFromParameters()
{
	// NOTE: We use ACC2 and RATE2 which are both decimated without interpolation
	CTRL_FILT_RATE_Register		filt_rate;
	CTRL_FILT_ACC12_Register	filt_acc12;
	CTRL_FILT_ACC3_Register 	filt_acc3;
	RATE_CTRL_Register			rate_ctrl;
	ACC12_CTRL_Register 		acc12_ctrl;
	ACC3_CTRL_Register 			acc3_ctrl;

	// We always use the maximum dynamic range for gyro and accel
	rate_ctrl.bits.DYN_RATE_XYZ1 = 	RATE_RANGE_300;
	rate_ctrl.bits.DYN_RATE_XYZ2 = 	RATE_RANGE_300;
	acc12_ctrl.bits.DYN_ACC_XYZ1 = 	ACC12_RANGE_80;
	acc12_ctrl.bits.DYN_ACC_XYZ2 = 	ACC12_RANGE_80;
	acc3_ctrl.bits.DYN_ACC_XYZ3 = 	ACC3_RANGE_260;

	_px4_gyro.set_range(math::radians(327.5f)); 		// 327.5 °/sec
	_px4_gyro.set_scale(math::radians(1.f / 1600.f)); 	// 1600 LSB/(°/sec)
	_px4_accel.set_range(163.4f); 		// 163.4 m/s2
	_px4_accel.set_scale(1.f / 3200.f); // 3200 LSB/(m/s2)

	// Gyro filter
	switch (_sch16t_gyro_filt.get()) {
	case 0:
		filt_rate.bits.FILT_SEL_RATE_X = FILTER_13_HZ;
		filt_rate.bits.FILT_SEL_RATE_Y = FILTER_13_HZ;
		filt_rate.bits.FILT_SEL_RATE_Z = FILTER_13_HZ;
		break;

	case 1:
		filt_rate.bits.FILT_SEL_RATE_X = FILTER_30_HZ;
		filt_rate.bits.FILT_SEL_RATE_Y = FILTER_30_HZ;
		filt_rate.bits.FILT_SEL_RATE_Z = FILTER_30_HZ;
		break;

	case 2:
		filt_rate.bits.FILT_SEL_RATE_X = FILTER_68_HZ;
		filt_rate.bits.FILT_SEL_RATE_Y = FILTER_68_HZ;
		filt_rate.bits.FILT_SEL_RATE_Z = FILTER_68_HZ;
		break;

	case 3:
		filt_rate.bits.FILT_SEL_RATE_X = FILTER_235_HZ;
		filt_rate.bits.FILT_SEL_RATE_Y = FILTER_235_HZ;
		filt_rate.bits.FILT_SEL_RATE_Z = FILTER_235_HZ;
		break;

	case 4:
		filt_rate.bits.FILT_SEL_RATE_X = FILTER_280_HZ;
		filt_rate.bits.FILT_SEL_RATE_Y = FILTER_280_HZ;
		filt_rate.bits.FILT_SEL_RATE_Z = FILTER_280_HZ;
		break;

	case 5:
		filt_rate.bits.FILT_SEL_RATE_X = FILTER_370_HZ;
		filt_rate.bits.FILT_SEL_RATE_Y = FILTER_370_HZ;
		filt_rate.bits.FILT_SEL_RATE_Z = FILTER_370_HZ;
		break;

	case 6:
		filt_rate.bits.FILT_SEL_RATE_X = FILTER_BYPASS;
		filt_rate.bits.FILT_SEL_RATE_Y = FILTER_BYPASS;
		filt_rate.bits.FILT_SEL_RATE_Z = FILTER_BYPASS;
		break;
	}

	// ACC12 / ACC3 filter
	switch (_sch16t_acc_filt.get()) {
	case 0:
		filt_acc12.bits.FILT_SEL_ACC_X12 = FILTER_13_HZ;
		filt_acc12.bits.FILT_SEL_ACC_Y12 = FILTER_13_HZ;
		filt_acc12.bits.FILT_SEL_ACC_Z12 = FILTER_13_HZ;

		filt_acc3.bits.FILT_SEL_ACC_X3 = FILTER_13_HZ;
		filt_acc3.bits.FILT_SEL_ACC_Y3 = FILTER_13_HZ;
		filt_acc3.bits.FILT_SEL_ACC_Z3 = FILTER_13_HZ;
		break;

	case 1:
		filt_acc12.bits.FILT_SEL_ACC_X12 = FILTER_30_HZ;
		filt_acc12.bits.FILT_SEL_ACC_Y12 = FILTER_30_HZ;
		filt_acc12.bits.FILT_SEL_ACC_Z12 = FILTER_30_HZ;

		filt_acc3.bits.FILT_SEL_ACC_X3 = FILTER_30_HZ;
		filt_acc3.bits.FILT_SEL_ACC_Y3 = FILTER_30_HZ;
		filt_acc3.bits.FILT_SEL_ACC_Z3 = FILTER_30_HZ;
		break;

	case 2:
		filt_acc12.bits.FILT_SEL_ACC_X12 = FILTER_68_HZ;
		filt_acc12.bits.FILT_SEL_ACC_Y12 = FILTER_68_HZ;
		filt_acc12.bits.FILT_SEL_ACC_Z12 = FILTER_68_HZ;

		filt_acc3.bits.FILT_SEL_ACC_X3 = FILTER_68_HZ;
		filt_acc3.bits.FILT_SEL_ACC_Y3 = FILTER_68_HZ;
		filt_acc3.bits.FILT_SEL_ACC_Z3 = FILTER_68_HZ;
		break;

	case 3:
		filt_acc12.bits.FILT_SEL_ACC_X12 = FILTER_235_HZ;
		filt_acc12.bits.FILT_SEL_ACC_Y12 = FILTER_235_HZ;
		filt_acc12.bits.FILT_SEL_ACC_Z12 = FILTER_235_HZ;

		filt_acc3.bits.FILT_SEL_ACC_X3 = FILTER_235_HZ;
		filt_acc3.bits.FILT_SEL_ACC_Y3 = FILTER_235_HZ;
		filt_acc3.bits.FILT_SEL_ACC_Z3 = FILTER_235_HZ;
		break;

	case 4:
		filt_acc12.bits.FILT_SEL_ACC_X12 = FILTER_280_HZ;
		filt_acc12.bits.FILT_SEL_ACC_Y12 = FILTER_280_HZ;
		filt_acc12.bits.FILT_SEL_ACC_Z12 = FILTER_280_HZ;

		filt_acc3.bits.FILT_SEL_ACC_X3 = FILTER_280_HZ;
		filt_acc3.bits.FILT_SEL_ACC_Y3 = FILTER_280_HZ;
		filt_acc3.bits.FILT_SEL_ACC_Z3 = FILTER_280_HZ;
		break;

	case 5:
		filt_acc12.bits.FILT_SEL_ACC_X12 = FILTER_370_HZ;
		filt_acc12.bits.FILT_SEL_ACC_Y12 = FILTER_370_HZ;
		filt_acc12.bits.FILT_SEL_ACC_Z12 = FILTER_370_HZ;

		filt_acc3.bits.FILT_SEL_ACC_X3 = FILTER_370_HZ;
		filt_acc3.bits.FILT_SEL_ACC_Y3 = FILTER_370_HZ;
		filt_acc3.bits.FILT_SEL_ACC_Z3 = FILTER_370_HZ;
		break;

	case 6:
		filt_acc12.bits.FILT_SEL_ACC_X12 = FILTER_BYPASS;
		filt_acc12.bits.FILT_SEL_ACC_Y12 = FILTER_BYPASS;
		filt_acc12.bits.FILT_SEL_ACC_Z12 = FILTER_BYPASS;

		filt_acc3.bits.FILT_SEL_ACC_X3 = FILTER_BYPASS;
		filt_acc3.bits.FILT_SEL_ACC_Y3 = FILTER_BYPASS;
		filt_acc3.bits.FILT_SEL_ACC_Z3 = FILTER_BYPASS;
		break;
	}

	// Gyro decimation (only affects channel 2, ie RATE2)
	switch (_sch16t_decim.get()) {
	case 0:
		_sample_interval_us = 85;
		rate_ctrl.bits.DEC_RATE_X2 = DECIMATION_NONE;
		rate_ctrl.bits.DEC_RATE_Y2 = DECIMATION_NONE;
		rate_ctrl.bits.DEC_RATE_Z2 = DECIMATION_NONE;
		acc12_ctrl.bits.DEC_ACC_X2 = DECIMATION_NONE;
		acc12_ctrl.bits.DEC_ACC_Y2 = DECIMATION_NONE;
		acc12_ctrl.bits.DEC_ACC_Z2 = DECIMATION_NONE;
		break;

	case 1:
		_sample_interval_us = 169;
		rate_ctrl.bits.DEC_RATE_X2 = DECIMATION_5900_HZ;
		rate_ctrl.bits.DEC_RATE_Y2 = DECIMATION_5900_HZ;
		rate_ctrl.bits.DEC_RATE_Z2 = DECIMATION_5900_HZ;
		acc12_ctrl.bits.DEC_ACC_X2 = DECIMATION_5900_HZ;
		acc12_ctrl.bits.DEC_ACC_Y2 = DECIMATION_5900_HZ;
		acc12_ctrl.bits.DEC_ACC_Z2 = DECIMATION_5900_HZ;
		break;

	case 2:
		_sample_interval_us = 338;
		rate_ctrl.bits.DEC_RATE_X2 = DECIMATION_2950_HZ;
		rate_ctrl.bits.DEC_RATE_Y2 = DECIMATION_2950_HZ;
		rate_ctrl.bits.DEC_RATE_Z2 = DECIMATION_2950_HZ;
		acc12_ctrl.bits.DEC_ACC_X2 = DECIMATION_2950_HZ;
		acc12_ctrl.bits.DEC_ACC_Y2 = DECIMATION_2950_HZ;
		acc12_ctrl.bits.DEC_ACC_Z2 = DECIMATION_2950_HZ;
		break;

	case 3:
		_sample_interval_us = 678;
		rate_ctrl.bits.DEC_RATE_X2 = DECIMATION_1475_HZ;
		rate_ctrl.bits.DEC_RATE_Y2 = DECIMATION_1475_HZ;
		rate_ctrl.bits.DEC_RATE_Z2 = DECIMATION_1475_HZ;
		acc12_ctrl.bits.DEC_ACC_X2 = DECIMATION_1475_HZ;
		acc12_ctrl.bits.DEC_ACC_Y2 = DECIMATION_1475_HZ;
		acc12_ctrl.bits.DEC_ACC_Z2 = DECIMATION_1475_HZ;
		break;

	case 4:
		_sample_interval_us = 1355;
		rate_ctrl.bits.DEC_RATE_X2 = DECIMATION_738_HZ;
		rate_ctrl.bits.DEC_RATE_Y2 = DECIMATION_738_HZ;
		rate_ctrl.bits.DEC_RATE_Z2 = DECIMATION_738_HZ;
		acc12_ctrl.bits.DEC_ACC_X2 = DECIMATION_738_HZ;
		acc12_ctrl.bits.DEC_ACC_Y2 = DECIMATION_738_HZ;
		acc12_ctrl.bits.DEC_ACC_Z2 = DECIMATION_738_HZ;
		break;
	}

	_registers[0] = RegisterConfig(CTRL_FILT_RATE,  filt_rate.value);
	_registers[1] = RegisterConfig(CTRL_FILT_ACC12, filt_acc12.value);
	_registers[2] = RegisterConfig(CTRL_FILT_ACC3,  filt_acc3.value);
	_registers[3] = RegisterConfig(CTRL_RATE,       rate_ctrl.value);
	_registers[4] = RegisterConfig(CTRL_ACC12,      acc12_ctrl.value);
	_registers[5] = RegisterConfig(CTRL_ACC3,       acc3_ctrl.value);
}

void SCH16T::Configure()
{
	for (auto &r : _registers) {
		RegisterWrite(r.addr, r.value);
	}

	RegisterWrite(CTRL_USER_IF, DRY_DRV_EN); // Enable data ready
	RegisterWrite(CTRL_MODE, EN_SENSOR); // Enable the sensor
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
