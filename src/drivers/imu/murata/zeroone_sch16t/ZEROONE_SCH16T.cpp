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

#include "ZEROONE_SCH16T.hpp"
#include <cstdint>
#include <cstring>

using namespace time_literals;

ZEROONE_SCH16T::ZEROONE_SCH16T(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	ModuleParams(nullptr),
	_px4_accel(get_device_id(), config.rotation),
	_px4_gyro(get_device_id(), config.rotation)
{
	_fpga_driver = new SCH16T_FPGA_driver(this);
	_px4_accel.set_range(163.4f);// 163.4 m/s2
	_px4_gyro.set_range(math::radians(5000.f));// 5000 °/sec

	_registers[0] = RegisterConfig(CTRL_FILT_RATE,  FILTER_235HZ);
	/* 68Hz -- default FILTER_235HZ */
	_registers[1] = RegisterConfig(CTRL_FILT_ACC12, FILTER_235HZ);
	/* 68Hz -- default FILTER_235HZ */
	_registers[2] = RegisterConfig(CTRL_FILT_ACC3,  FILTER_235HZ);
	/* 68Hz -- default FILTER_235HZ */
	_registers[3] = RegisterConfig(CTRL_RATE, RATE_300DPS_1518HZ);
	/* +/- 300 deg/s, 1600 LSB/(deg/s) -- default, Decimation 8, 1475Hz RATE_300DPS_1475HZ */
	_registers[4] = RegisterConfig(CTRL_ACC12, ACC12_8G_1518HZ);
	/* +/- 80 m/s^2, 3200 LSB/(m/s^2) -- default, Decimation 8, 1475Hz ACC12_8G_1475HZ */
	_registers[5] = RegisterConfig(CTRL_ACC3, ACC3_26G);
	/* +/- 260 m/s^2, 1600 LSB/(m/s^2) -- default ACC3_26G */
}

ZEROONE_SCH16T::~ZEROONE_SCH16T()
{
	delete _fpga_driver;
	perf_free(_reset_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_perf_crc_bad);
	perf_free(_perf_general_error);
	perf_free(_perf_command_error);
	perf_free(_perf_saturation_error);
	perf_free(_perf_doing_initialization);
	perf_free(_fifo_empty_perf);
}

int ZEROONE_SCH16T::init()
{
	px4_usleep(250000);//wait for the fpga to initialize

	int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);
		return ret;
	}

	ScheduleClear();

	_state = State::PowerOn;

	ScheduleNow();

	return PX4_OK;
}

int ZEROONE_SCH16T::probe()
{
	reset_chip();
	px4_usleep(400000);//wait for the fpga to reset

	if (!read_product_id()) {
		return PX4_ERROR;
	}

	return PX4_OK;
}

void ZEROONE_SCH16T::exit_and_cleanup()
{
	I2CSPIDriverBase::exit_and_cleanup();
}

void ZEROONE_SCH16T::transfer(uint8_t *send, uint8_t *recv, unsigned int len)
{
	device::SPI::transfer(send, recv, len);
}

void ZEROONE_SCH16T::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_reset_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_perf_crc_bad);
	perf_print_counter(_perf_general_error);
	perf_print_counter(_perf_command_error);
	perf_print_counter(_perf_saturation_error);
	perf_print_counter(_perf_doing_initialization);
	perf_print_counter(_fifo_empty_perf);
}

void ZEROONE_SCH16T::RunImpl()
{
	_drdy_timestamp_sample.store(hrt_absolute_time());

	switch (_state) {
	case State::PowerOn: {
			_state = State::Reset;
			ScheduleDelayed(250000);
			break;
		}

	case State::Reset: {
			reset_chip();
			_state = State::Configure;
			ScheduleDelayed(400000);
			break;
		}

	case State::Configure: {
			_failure_count = 0;
			configure_registers();
			_state = State::LockConfiguration;
			ScheduleDelayed(250000);
			break;
		}

	case State::LockConfiguration: {
			read_status_registers(); // Read all status registers once
			register_write(CTRL_MODE, EOI | EN_SENSOR);// Write EOI and EN_SENSOR
			_state = State::Validate;
			ScheduleDelayed(50000);
			break;
		}

	case State::Validate: {
			read_status_registers(); // Read all status registers twice
			read_status_registers();

			// Check that registers are configured properly and that the sensor status is OK
			if (validate_sensor_status() && validate_register_configuration()) {
				_state = State::Read;
				fpga_init();
				ScheduleOnInterval(_fifo_empty_interval_us, _fifo_empty_interval_us);

			} else {
				_state = State::Reset;
				ScheduleDelayed(100000);
			}

			break;
		}

	case State::Read: {
			if (read_data()) {
				if (_failure_count > 0) {
					_failure_count--;
				}

			} else {
				_failure_count++;
			}

			// Reset if successive failures
			if (_failure_count > 10) {
				_state = State::Reset;
				return;
			}

			break;
		}

	default:
		break;
	} // end switch/case
}

void ZEROONE_SCH16T::fpga_init(void)
{
	_fpga_driver->fpga_init(_fifo_enable, _fifo_cmd_num, _direct_mode);
}

bool ZEROONE_SCH16T::read_data()
{
	hrt_abstime timestamp_sample = _drdy_timestamp_sample.load();
	uint8_t pkt_num{};
	uint8_t buff[32];

	pkt_num = _fpga_driver->fpga_read_reg8(Addr_RW8_SensorFifoCtrl);

	if (pkt_num == 0) {
		perf_count(_fifo_empty_perf);

	} else {

		if (pkt_num > _fifo_gyro_samples) {
			// grab desired number of samples, but reschedule next cycle sooner
			int extra_samples = pkt_num - _fifo_gyro_samples;
			pkt_num = _fifo_gyro_samples;

			if (_fifo_gyro_samples > extra_samples) {
				// reschedule to run when a total of _fifo_gyro_samples should be available in the FIFO
				const uint32_t reschedule_delay_us = (_fifo_gyro_samples - extra_samples) * static_cast<int>(FIFO_SAMPLE_DT);
				ScheduleOnInterval(_fifo_empty_interval_us, reschedule_delay_us);

			} else {
				// otherwise reschedule to run immediately
				ScheduleOnInterval(_fifo_empty_interval_us);
			}

		} else if (pkt_num < _fifo_gyro_samples) {
			// reschedule next cycle to catch the desired number of samples
			ScheduleOnInterval(_fifo_empty_interval_us, (_fifo_gyro_samples - pkt_num) * static_cast<int>(FIFO_SAMPLE_DT));
		}
	}


	if (pkt_num == _fifo_gyro_samples) {
		sensor_gyro_fifo_s gyro{};
		gyro.timestamp_sample = timestamp_sample;
		gyro.samples = pkt_num;
		gyro.dt = FIFO_SAMPLE_DT;

		sensor_accel_fifo_s accel{};
		accel.timestamp_sample = timestamp_sample;
		accel.samples = pkt_num;
		accel.dt = FIFO_SAMPLE_DT;
		// 18-bits of accelerometer data
		bool acc_scale_20bit = false;

		// 18-bits of 	// 20-bits of gyroscope data data
		bool gyro_scale_20bit = false;

		float temperature_sum{0};

		RawFifoData raw_data[pkt_num];

		// First pass: read all data and check for 20-bit scaling
		for (int i = 0; i < pkt_num; i++) {
			_fpga_driver->fpga_read_fifo8(Addr_W8_SensorValueFifo8, buff, 24);
			_fpga_driver->fpga_write_reg8(Addr_RW8_SensorFifoCtrl, 0x01);

			// Store raw buffer for later 20-bit rescale
			memcpy(raw_data[i].buff, buff, 24);

			/*Using timestamps to ensure data packet integrity is a temporary solution,
			and it's clearly not as effective as CRC checks. We will update the CRC check function to guarantee SAFESPI transmission in the future.*/
			raw_data[i].time_stamp = buff[23] & 0xf;
			static uint8_t local_time_stamp{0};

			if (raw_data[i].time_stamp != local_time_stamp) {
				local_time_stamp = raw_data[i].time_stamp;
				return false;

			} else {
				local_time_stamp = (local_time_stamp + 1) & 0xf;
			}

			if (buff[0] & 0x80) { raw_data[i].gyro_x = SPI32BITCONVERT2_20BIT(((buff[1] & 0xf) << 16) | (buff[2] << 8) | buff[3]); }

			if (buff[0] & 0x40) { raw_data[i].gyro_y = -SPI32BITCONVERT2_20BIT(((buff[4] & 0xf) << 16) | (buff[5] << 8) | buff[6]); }

			if (buff[0] & 0x20) { raw_data[i].gyro_z = -SPI32BITCONVERT2_20BIT(((buff[7] & 0xf) << 16) | (buff[8] << 8) | buff[9]); }

			if (buff[0] & 0x10) { raw_data[i].acc_x = SPI32BITCONVERT2_20BIT(((buff[10] & 0xf) << 16) | (buff[11] << 8) | buff[12]); }

			if (buff[0] & 0x08) { raw_data[i].acc_y = -SPI32BITCONVERT2_20BIT(((buff[13] & 0xf) << 16) | (buff[14] << 8) | buff[15]); }

			if (buff[0] & 0x04) { raw_data[i].acc_z = -SPI32BITCONVERT2_20BIT(((buff[16] & 0xf) << 16) | (buff[17] << 8) | buff[18]); }

			if (buff[0] & 0x02) { raw_data[i].temp = SPI32BITCONVERT2_20BIT(((buff[19] & 0xf) << 16) | (buff[20] << 8) | buff[21]) >> 4; }

			temperature_sum += raw_data[i].temp;

			// check if any values are going to exceed int16 limits
			static constexpr int16_t max_accel = INT16_MAX;
			static constexpr int16_t min_accel = INT16_MIN;

			if (raw_data[i].acc_x >= max_accel || raw_data[i].acc_x <= min_accel) {
				acc_scale_20bit = true;
			}

			if (raw_data[i].acc_y >= max_accel || raw_data[i].acc_y <= min_accel) {
				acc_scale_20bit = true;
			}

			if (raw_data[i].acc_z >= max_accel || raw_data[i].acc_z <= min_accel) {
				acc_scale_20bit = true;
			}

			static constexpr int16_t max_gyro = INT16_MAX;
			static constexpr int16_t min_gyro = INT16_MIN;

			if (raw_data[i].gyro_x >= max_gyro || raw_data[i].gyro_x <= min_gyro) {
				gyro_scale_20bit = true;
			}

			if (raw_data[i].gyro_y >= max_gyro || raw_data[i].gyro_y <= min_gyro) {
				gyro_scale_20bit = true;
			}

			if (raw_data[i].gyro_z >= max_gyro || raw_data[i].gyro_z <= min_gyro) {
				gyro_scale_20bit = true;
			}
		}

		// Second pass: process all samples with consistent scaling
		if (!acc_scale_20bit) {
			_px4_accel.set_scale(1.f / 3200.f); // 3200 LSB/(m/s2)

			for (int i = 0; i < pkt_num; i++) {
				accel.x[i] = raw_data[i].acc_x;
				accel.y[i] = raw_data[i].acc_y;
				accel.z[i] = raw_data[i].acc_z;
			}

		} else {
			_px4_accel.set_scale(1.f / 200.f); // 200 LSB/(m/s2)

			for (int i = 0; i < pkt_num; i++) {
				// 20 bit data scaled to 16 bit (2^4)
				// Sign extension + Accel [19:12] + Accel [11:4] + Accel [3:2] (20 bit extension byte)
				const uint8_t *b = raw_data[i].buff;
				accel.x[i] = (((b[10] & 0xf) << 12) | (b[11] << 4) | (b[12] >> 4));
				accel.y[i] = -(((b[13] & 0xf) << 12) | (b[14] << 4) | (b[15] >> 4));
				accel.z[i] = -(((b[16] & 0xf) << 12) | (b[17] << 4) | (b[18] >> 4));
			}
		}

		if (!gyro_scale_20bit) {
			_px4_gyro.set_scale(math::radians(1.f / 100.f));     // 100 LSB/(°/sec)

			for (int i = 0; i < pkt_num; i++) {
				gyro.x[i] = raw_data[i].gyro_x;
				gyro.y[i] = raw_data[i].gyro_y;
				gyro.z[i] = raw_data[i].gyro_z;
			}

		} else {
			_px4_gyro.set_scale(math::radians(16.f / 100.f));

			for (int i = 0; i < pkt_num; i++) {
				// 20 bit data scaled to 16 bit (2^4)
				const uint8_t *b = raw_data[i].buff;
				gyro.x[i] = (((b[1] & 0xf) << 12) | (b[2] << 4) | (b[3] >> 4));
				gyro.y[i] = -(((b[4] & 0xf) << 12) | (b[5] << 4) | (b[6] >> 4));
				gyro.z[i] = -(((b[7] & 0xf) << 12) | (b[8] << 4) | (b[9] >> 4));
			}
		}

		const float temperature_avg = temperature_sum / pkt_num;

		_px4_gyro.updateFIFO(gyro);

		_px4_accel.updateFIFO(accel);

		_px4_accel.set_temperature(temperature_avg / 100.f); // Temperature signal sensitivity is 100 LSB/°C
		_px4_gyro.set_temperature(temperature_avg / 100.f);

	}

	return true;
}

void ZEROONE_SCH16T::reset_chip()
{
	register_write(CTRL_RESET, SPI_SOFT_RESET);
#if defined(SPI6_RESET)
	SPI6_RESET(reset);
#endif
}

bool ZEROONE_SCH16T::read_product_id()
{
	uint32_t comp_id = 0, asic_id = 0;
	/*the command is to notice the fpga to load next register to read,and the response is the previous register value.*/
	register_read(COMP_ID, 0);
	register_read(ASIC_ID, &comp_id);
	register_read(ASIC_ID, &asic_id);
	bool success = asic_id == 0x21 && comp_id == 0x21;

	if (!success) {
		PX4_ERR("Unsupported COMP_ID and ASIC_ID combination");
		PX4_ERR("COMP_ID: 0x%04lx, ASIC_ID: 0x%04lx", (unsigned long)comp_id, (unsigned long)asic_id);
		return false;
	}

	return success;
}

void ZEROONE_SCH16T::configure_registers()
{
	uint32_t reg_value;

	for (auto &r : _registers) {
		register_write(r.addr, r.value);
	}

	register_read(CTRL_USER_IF, 0);
	register_read(CTRL_USER_IF, &reg_value);
	reg_value |= DRY_DRV_EN;
	register_write(CTRL_USER_IF, reg_value);
	register_write(CTRL_MODE, EN_SENSOR);
}

bool ZEROONE_SCH16T::validate_sensor_status()
{
	auto &s = _sensor_status;
	uint16_t values[] = { s.summary, s.saturation, s.common, s.rate_common, s.rate_x, s.rate_y, s.rate_z, s.acc_x, s.acc_y, s.acc_z };

	for (auto v : values) {
		if (v != 0xFFFF) {
			return false;
		}
	}

	return true;
}

bool ZEROONE_SCH16T::validate_register_configuration()
{
	uint32_t value;

	for (auto &r : _registers) {
		register_read(r.addr, 0); // double read, wasteful but makes the code cleaner, not high rate so doesn't matter anyway
		register_read(r.addr, &value);

		if (value != r.value) {
			return false;
		}
	}

	return true;
}

uint8_t ZEROONE_SCH16T::wait_direct_busy(void)
{
	uint16_t timeout = 0x200;

	while (timeout) {
		if (!(_fpga_driver->fpga_read_reg16(Addr_RW16_DirectSpiCtrl) & 1)) { return 1; }

		timeout--;
	}

	return 0;
}

uint16_t ZEROONE_SCH16T::_sch16t_read_status(uint16_t addr)
{
	uint32_t value;

	if (register_read(addr, &value) == 0) {
		return 0;
	}

	return value & 0xffff;
}

void ZEROONE_SCH16T::read_status_registers()
{
	/*the FPGA doesn't process the first instruction,it only starts processing after receiving the second instruction,resulting in the same command being sent twice.*/
	/*the command is to notice the fpga to load next status to read,and the response is the previous status.*/
	_sch16t_read_status(STAT_SUM);
	_sensor_status.summary      = _sch16t_read_status(STAT_SUM);
	_sensor_status.summary      = _sch16t_read_status(STAT_SUM_SAT);
	_sensor_status.saturation   = _sch16t_read_status(STAT_COM);
	_sensor_status.common       = _sch16t_read_status(STAT_RATE_COM);
	_sensor_status.rate_common  = _sch16t_read_status(STAT_RATE_X);
	_sensor_status.rate_x       = _sch16t_read_status(STAT_RATE_Y);
	_sensor_status.rate_y       = _sch16t_read_status(STAT_RATE_Z);
	_sensor_status.rate_z       = _sch16t_read_status(STAT_ACC_X);
	_sensor_status.acc_x        = _sch16t_read_status(STAT_ACC_Y);
	_sensor_status.acc_y        = _sch16t_read_status(STAT_ACC_Z);
	_sensor_status.acc_z        = _sch16t_read_status(STAT_ACC_Z);
}

uint8_t ZEROONE_SCH16T::register_read(uint16_t addr, uint32_t *value)
{
	uint8_t tbuf[8];
	uint8_t rbuf[8];

	_direct_mode = CtrlMode_Direct;
	_fpga_driver->sensor_ctrl(FIFO_NRESET, _direct_mode, _fifo_enable, _fifo_cmd_num, _fifo_baudrate);

	uint8_t buff[6];

	buff[0] = (addr >> 2) & 0xff;
	buff[1] = ((addr & 3) << 6);
	buff[1] |= 1 << 3;
	buff[2] = 0;
	buff[3] = 0;
	buff[4] = 0;
	buff[5] = SCH16T_FPGA_driver::gen_crc8(buff);
	memcpy(tbuf, buff, 6);

	wait_direct_busy();
	_fpga_driver->fpga_write_reg16(Addr_RW16_DirectSpiCtrl, 0x01);
	_fpga_driver->fpga_write_ram8(Addr_RW8_DirectRam8, 0, tbuf, 6);
	_fpga_driver->fpga_write_reg16(Addr_RW16_DirectSpiCtrl, 0x02 | (5 << 8));
	wait_direct_busy();
	_fpga_driver->fpga_read_ram8(Addr_RW8_DirectRam8, 0, rbuf, 6);

	if (value == 0) {
		return 0;
	}

	if (SCH16T_FPGA_driver::gen_crc8(rbuf) != rbuf[5]) {
		return 0;
	}

	*value = ((rbuf[2] & 0xf) << 16) | (rbuf[3] << 8) | (rbuf[4]);
	return 1;
}

// Non-data registers are the only writable ones and are 16 bit or less
void ZEROONE_SCH16T::register_write(uint16_t addr, uint32_t value)
{
	uint8_t tbuf[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	_direct_mode = CtrlMode_Direct;
	_fpga_driver->sensor_ctrl(FIFO_NRESET, _direct_mode, _fifo_enable, _fifo_cmd_num, _fifo_baudrate);

	uint8_t buff[6];
	buff[0] = (addr >> 2) & 0xff;
	buff[1] = ((addr & 3) << 6);
	buff[1] |= 1 << 5;
	buff[1] |= 1 << 3;
	buff[2] = (value >> 16) & 0xf;
	buff[3] = (value >> 8) & 0xff;
	buff[4] = value & 0xff;
	buff[5] = SCH16T_FPGA_driver::gen_crc8(buff);
	memcpy(tbuf, buff, 6);

	wait_direct_busy();
	_fpga_driver->fpga_write_reg16(Addr_RW16_DirectSpiCtrl, 0x01);
	_fpga_driver->fpga_write_ram8(Addr_RW8_DirectRam8, 0, tbuf, 6);
	_fpga_driver->fpga_write_reg16(Addr_RW16_DirectSpiCtrl, 0x02 | (5 << 8));
	wait_direct_busy();
}

#if defined(DEBUG_BUILD)
uint8_t ZEROONE_SCH16T::fpga_test()
{
	return _fpga_driver->fpga_test();
}
#endif
