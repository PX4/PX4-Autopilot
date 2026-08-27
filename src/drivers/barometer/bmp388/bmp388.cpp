/****************************************************************************
 *
 *   Copyright (c) 2019-2026 PX4 Development Team. All rights reserved.
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

#include "bmp388.h"

#include <drivers/drv_sensor.h>

#include <errno.h>
#include <inttypes.h>

using namespace time_literals;

BMP388::BMP388(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_px4_baro(get_device_id())
{
	_retries = 1;
}

BMP388::~BMP388()
{
	perf_free(_bad_transfer_perf);
	perf_free(_conversion_timeout_perf);
	perf_free(_reset_perf);
}

int BMP388::init()
{
	const int ret = I2C::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	// probe() may have refined the device type to BMP390
	_px4_baro.set_device_id(get_device_id());

	_state = STATE::RESET;
	ScheduleNow();

	return PX4_OK;
}

int BMP388::probe()
{
	for (int i = 0; i < 3; i++) {
		const uint8_t chip_id = RegisterRead(Register::CHIP_ID);

		if ((chip_id == BMP388_CHIP_ID) || (chip_id == BMP390_CHIP_ID)) {
			_chip_id = chip_id;
			_rev_id = RegisterRead(Register::REV_ID);

			if (chip_id == BMP390_CHIP_ID) {
				set_device_type(DRV_BARO_DEVTYPE_BMP390);
				_item_name = "bmp390";
			}

			return PX4_OK;
		}

		DEVICE_DEBUG("unexpected CHIP_ID 0x%02x", chip_id);
	}

	return PX4_ERROR;
}

void BMP388::RunImpl()
{
	switch (_state) {
	case STATE::RESET:

		// the command decoder is not ready until tstartup (2 ms) after POR. Give up on the
		// gate after 10 ms so a wedged sensor falls into the 100 ms retry below instead of
		// polling this bus at 500 Hz forever.
		if ((RegisterRead(Register::STATUS) & STATUS_BIT::CMD_RDY) || (_cmd_rdy_polls++ >= 5)) {
			RegisterWrite(Register::CMD, CMD_VALUE::SOFTRESET);
			_cmd_rdy_polls = 0;
			_failure_count = 0;
			_state = STATE::WAIT_FOR_RESET;
			perf_count(_reset_perf);
		}

		ScheduleDelayed(2_ms);
		break;

	case STATE::WAIT_FOR_RESET:

		// a softreset issued while the decoder was busy is dropped with cmd_err
		if ((RegisterRead(Register::CHIP_ID) == _chip_id)
		    && (RegisterRead(Register::STATUS) & STATUS_BIT::CMD_RDY)
		    && !(RegisterRead(Register::ERR_REG) & ERR_REG_BIT::CMD_ERR)) {
			_state = STATE::CONFIGURE;
			ScheduleNow();

		} else {
			PX4_DEBUG("reset failed, retrying");
			_state = STATE::RESET;
			ScheduleDelayed(100_ms);
		}

		break;

	case STATE::CONFIGURE:
		if (Configure()) {
			_configure_failures = 0;
			_state = STATE::MEASURE;
			ScheduleNow();

		} else {
			// init() returns before the first configure, so this is the only place a bad
			// calibration CRC or a dead sensor can be reported
			if (++_configure_failures == 10) {
				PX4_ERR("configure failed, not publishing");
			}

			_state = STATE::RESET;
			ScheduleDelayed(100_ms);
		}

		break;

	case STATE::MEASURE:
		if (Measure()) {
			_state = STATE::COLLECT;
			ScheduleDelayed(_conversion_time_typ_us);

		} else {
			Failure();
		}

		break;

	case STATE::COLLECT:
		switch (Collect()) {
		case PX4_OK:
			if (_failure_count > 0) {
				_failure_count--;
			}

			_state = STATE::MEASURE;
			ScheduleNow();
			break;

		case -EAGAIN:
			// conversion still running (DS typical < max)
			ScheduleDelayed(2_ms);
			break;

		default:
			Failure();
			break;
		}

		break;
	}
}

bool BMP388::Measure()
{
	// forced mode runs one conversion, then the chip returns to sleep
	if (!RegisterWrite(Register::PWR_CTRL, PWR_CTRL_BIT::PRESS_EN | PWR_CTRL_BIT::TEMP_EN | PWR_CTRL_BIT::MODE_FORCED)) {
		return false;
	}

	// conversion starts when the write lands, not when this work item started
	_measure_timestamp = hrt_absolute_time();

	return true;
}

int BMP388::Collect()
{
	// poll STATUS on its own: DS table 29, drdy is cleared by reading a DATA register,
	// so bundling the data into the poll would drop a conversion that lands mid-burst
	uint8_t status = 0;

	if (!RegisterRead(Register::STATUS, &status, sizeof(status))) {
		return -EIO;
	}

	if ((status & (STATUS_BIT::DRDY_PRESS | STATUS_BIT::DRDY_TEMP)) != (STATUS_BIT::DRDY_PRESS | STATUS_BIT::DRDY_TEMP)) {
		if (hrt_elapsed_time(&_measure_timestamp) < _conversion_time_max_us) {
			return -EAGAIN;
		}

		perf_count(_conversion_timeout_perf);
		return -ETIMEDOUT;
	}

	sensor_data data{};

	if (!RegisterRead(Register::DATA_0, &data, sizeof(data))) {
		return -EIO;
	}

	const uint32_t uncomp_press = ((uint32_t)data.press_23_16 << 16) | ((uint32_t)data.press_15_8 << 8) | data.press_7_0;
	const uint32_t uncomp_temp = ((uint32_t)data.temp_23_16 << 16) | ((uint32_t)data.temp_15_8 << 8) | data.temp_7_0;

	int64_t t_lin = 0;
	const int64_t temperature = CompensateTemperature(uncomp_temp, t_lin); // 0.01 degC
	const uint64_t pressure = CompensatePressure(uncomp_press, t_lin); // 0.01 Pa

	// pressure is averaged over the conversion window, so timestamp its midpoint
	const hrt_abstime timestamp_sample = _measure_timestamp + (_conversion_time_typ_us / 2);

	_px4_baro.set_error_count(perf_event_count(_bad_transfer_perf) + perf_event_count(_conversion_timeout_perf));
	_px4_baro.set_temperature(temperature / 100.f);
	_px4_baro.update(timestamp_sample, pressure / 100.f);

	return PX4_OK;
}

void BMP388::Failure()
{
	// skip the sample; reset once failures are persistent
	if (++_failure_count > 10) {
		_state = STATE::RESET;
		ScheduleDelayed(100_ms);

	} else {
		_state = STATE::MEASURE;
		ScheduleDelayed(10_ms);
	}
}

bool BMP388::Configure()
{
	if (!ReadCalibration()) {
		return false;
	}

	// ultra high resolution: osr_p x16, osr_t x2. IIR bypass and ODR stay at their reset defaults (ODR is unused in forced mode).
	static constexpr uint8_t kOsr = OSR_BIT::OSR_P_X16 | OSR_BIT::OSR_T_X2;

	if (!RegisterWrite(Register::OSR, kOsr) || (RegisterRead(Register::OSR) != kOsr)) {
		return false;
	}

	// forced-mode conversion time for this OSR: BMP388 DS table 21, BMP390 DS table 23
	static constexpr uint32_t kConversionTypUsBmp388 = 36900;
	static constexpr uint32_t kConversionMaxUsBmp388 = 43300;
	static constexpr uint32_t kConversionTypUsBmp390 = 37140;
	static constexpr uint32_t kConversionMaxUsBmp390 = 41890;

	if (_chip_id == BMP390_CHIP_ID) {
		_conversion_time_typ_us = kConversionTypUsBmp390;
		_conversion_time_max_us = kConversionMaxUsBmp390;

	} else {
		_conversion_time_typ_us = kConversionTypUsBmp388;
		_conversion_time_max_us = kConversionMaxUsBmp388;
	}

	return true;
}

// bmp3_selftest.c
static uint8_t Crc8(uint8_t seed, uint8_t data)
{
	static constexpr uint8_t kPoly = 0x1D;

	for (int i = 0; i < 8; i++) {
		const bool feedback = (seed ^ data) & 0x80;
		seed <<= 1;
		data <<= 1;

		if (feedback) {
			seed ^= kPoly;
		}
	}

	return seed;
}

bool BMP388::ReadCalibration()
{
	if (!RegisterRead(Register::NVM_PAR_T1, &_calib, sizeof(_calib))) {
		return false;
	}

	uint8_t crc = 0xFF;

	for (const uint8_t byte : reinterpret_cast<const uint8_t (&)[sizeof(_calib)]>(_calib)) {
		crc = Crc8(crc, byte);
	}

	crc ^= 0xFF;

	if (RegisterRead(Register::TRIM_CRC) != crc) {
		PX4_DEBUG("calibration CRC mismatch");
		return false;
	}

	return true;
}

// Integer compensation from the Bosch BMP3-Sensor-API (bmp3.c). t_lin carries the
// temperature into the pressure formula.
int64_t BMP388::CompensateTemperature(uint32_t uncomp_temp, int64_t &t_lin) const
{
	const int64_t partial_data1 = (int64_t)uncomp_temp - (256 * _calib.par_t1);
	const int64_t partial_data2 = _calib.par_t2 * partial_data1;
	const int64_t partial_data3 = partial_data1 * partial_data1;
	const int64_t partial_data4 = partial_data3 * _calib.par_t3;
	const int64_t partial_data5 = (partial_data2 * 262144) + partial_data4;

	t_lin = partial_data5 / 4294967296;

	return (t_lin * 25) / 16384;
}

uint64_t BMP388::CompensatePressure(uint32_t uncomp_press, int64_t t_lin) const
{
	int64_t partial_data1 = t_lin * t_lin;
	int64_t partial_data2 = partial_data1 / 64;
	int64_t partial_data3 = (partial_data2 * t_lin) / 256;
	int64_t partial_data4 = (_calib.par_p8 * partial_data3) / 32;
	int64_t partial_data5 = (_calib.par_p7 * partial_data1) * 16;
	int64_t partial_data6 = (_calib.par_p6 * t_lin) * 4194304;
	const int64_t offset = (_calib.par_p5 * 140737488355328) + partial_data4 + partial_data5 + partial_data6;

	partial_data2 = (_calib.par_p4 * partial_data3) / 32;
	partial_data4 = (_calib.par_p3 * partial_data1) * 4;
	partial_data5 = (_calib.par_p2 - 16384) * t_lin * 2097152;
	const int64_t sensitivity = ((_calib.par_p1 - 16384) * 70368744177664) + partial_data2 + partial_data4 + partial_data5;

	partial_data1 = (sensitivity / 16777216) * uncomp_press;
	partial_data2 = _calib.par_p10 * t_lin;
	partial_data3 = partial_data2 + (65536 * _calib.par_p9);
	partial_data4 = (partial_data3 * uncomp_press) / 8192;
	// divide by 10 first so uncomp_press * partial_data4 cannot overflow
	partial_data5 = (uncomp_press * (partial_data4 / 10)) / 512;
	partial_data5 = partial_data5 * 10;
	partial_data6 = (int64_t)((uint64_t)uncomp_press * (uint64_t)uncomp_press);
	partial_data2 = (_calib.par_p11 * partial_data6) / 65536;
	partial_data3 = (partial_data2 * uncomp_press) / 128;
	partial_data4 = (offset / 4) + partial_data1 + partial_data5 + partial_data3;

	return ((uint64_t)partial_data4 * 25) / (uint64_t)1099511627776;
}

bool BMP388::RegisterRead(Register reg, void *buf, size_t len)
{
	const uint8_t cmd = static_cast<uint8_t>(reg);

	if (transfer(&cmd, 1, static_cast<uint8_t *>(buf), len) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return false;
	}

	return true;
}

uint8_t BMP388::RegisterRead(Register reg)
{
	uint8_t value = 0;
	RegisterRead(reg, &value, 1);
	return value;
}

bool BMP388::RegisterWrite(Register reg, uint8_t value)
{
	const uint8_t buf[2] {static_cast<uint8_t>(reg), value};

	if (transfer(buf, sizeof(buf), nullptr, 0) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return false;
	}

	return true;
}

void BMP388::print_status()
{
	I2CSPIDriverBase::print_status();

	PX4_INFO("chip id: 0x%02x rev id: 0x%02x", _chip_id, _rev_id);
	PX4_INFO("conversion time: %" PRIu32 " us typ, %" PRIu32 " us max", _conversion_time_typ_us,
		 _conversion_time_max_us);

	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_conversion_timeout_perf);
	perf_print_counter(_reset_perf);
}
