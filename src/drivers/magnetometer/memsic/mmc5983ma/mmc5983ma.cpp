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

#include "mmc5983ma.h"

using namespace time_literals;

MMC5983MA::MMC5983MA(device::Device *interface, const I2CSPIDriverConfig &config) :
	I2CSPIDriver(config),
	_interface(interface),
	_px4_mag(interface->get_device_id(), config.rotation),
	_sample_perf(perf_alloc(PC_ELAPSED, "mmc5983ma_read")),
	_comms_errors(perf_alloc(PC_COUNT, "mmc5983ma_comms_errors"))
{}

MMC5983MA::~MMC5983MA()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	delete _interface;
}

int MMC5983MA::init()
{
	// Start with a reset of the chip
	write_register(MMC5983MA_ADDR_CTRL_REG2, MMC5983MA_CTRL_REG1_SW_RESET);
	px4_usleep(20_ms);

	// Triggered mode with degaussing (averaging two samples), 18-bit resolution, automatic set/reset
	write_register(MMC5983MA_ADDR_CTRL_REG0, MMC5983MA_CTRL_REG0_AUTO_SR_EN);
	write_register(MMC5983MA_ADDR_CTRL_REG1, MMC5983MA_CTRL_REG1_BW_200HZ);
	write_register(MMC5983MA_ADDR_CTRL_REG2, MMC5983MA_CTRL_REG2_EN_PRD_SET);

	ScheduleNow();

	return PX4_OK;
}

void MMC5983MA::RunImpl()
{
	switch (_state) {
	case State::Measure:
	{
		auto start_time = hrt_absolute_time();

		write_register(MMC5983MA_ADDR_CTRL_REG0, MMC5983MA_CTRL_REG0_TM_M);

		_collect_retries = 0;
		_state = State::Collect;

		auto elapsed = hrt_elapsed_time(&start_time);
		if (elapsed > 5_ms) {
			ScheduleNow();
		} else {
			ScheduleDelayed(5_ms - elapsed);
		}

		return;
	}
	case State::Collect:
	{
		auto start_time = hrt_absolute_time();

		uint8_t status = read_register(MMC5983MA_ADDR_STATUS_REG);

		if (status & MMC5983MA_STATUS_REG_MEAS_M_DONE) {
			SensorData data = {};

			if (read_register_block(&data) != PX4_OK) {
				PX4_DEBUG("read failed");
				perf_count(_comms_errors);
				_state = State::Measure;
				_sample_index = 0;
				ScheduleDelayed(100_ms);
				return;
			}

			// Measurement available
			_measurements[_sample_index] = data;
			_sample_index++;

			if (_sample_index > 1) {
				publish_data();
				_sample_index = 0;
				perf_count(_sample_perf);
			}

			_state = State::Measure;

			auto elapsed = hrt_elapsed_time(&start_time);
			if (elapsed > 5_ms) {
				ScheduleNow();
			} else {
				ScheduleDelayed(5_ms - elapsed);
			}

			return;

		} else {
			PX4_DEBUG("not ready");
			perf_count(_comms_errors);
			_collect_retries++;
			_state = _collect_retries > 3 ? State::Measure : State::Collect;
			ScheduleDelayed(5_ms);
			return;
		}
	}
	} // end switch/case
}

void MMC5983MA::publish_data()
{
	uint32_t xraw_1 = (_measurements[0].xout0 << 10) | (_measurements[0].xout1 << 2) | ((
				  _measurements[0].xyzout2 & 0b11000000) >> 6);
	uint32_t yraw_1 = (_measurements[0].yout0 << 10) | (_measurements[0].yout1 << 2) | ((
				  _measurements[0].xyzout2 & 0b00110000) >> 4);
	uint32_t zraw_1 = (_measurements[0].zout0 << 10) | (_measurements[0].zout1 << 2) | ((
				  _measurements[0].xyzout2 & 0b00001100) >> 2);

	uint32_t xraw_2 = (_measurements[1].xout0 << 10) | (_measurements[1].xout1 << 2) | ((
				  _measurements[1].xyzout2 & 0b11000000) >> 6);
	uint32_t yraw_2 = (_measurements[1].yout0 << 10) | (_measurements[1].yout1 << 2) | ((
				  _measurements[1].xyzout2 & 0b00110000) >> 4);
	uint32_t zraw_2 = (_measurements[1].zout0 << 10) | (_measurements[1].zout1 << 2) | ((
				  _measurements[1].xyzout2 & 0b00001100) >> 2);

	float xrawf = float(xraw_1 + xraw_2) / 2.f;
	float yrawf = float(yraw_1 + yraw_2) / 2.f;
	float zrawf = float(zraw_1 + zraw_2) / 2.f;

	// NOTE: Temperature conversions did not work
	// float trawf = float(_measurements[0].tout + _measurements[1].tout) / 2.f;
	// float temp_c = trawf * 0.8f - 75.f;
	// _px4_mag.set_temperature(temp_c);

	// +/- 8 Gauss full scale range
	// 18-bit mode scaling factor: 0.0625 mG/LSB
	float x = 8.f - (xrawf * 0.0625f) / 1e3f;
	float y = 8.f - (yrawf * 0.0625f) / 1e3f;
	float z = 8.f - (zrawf * 0.0625f) / 1e3f;

	_px4_mag.update(hrt_absolute_time(), x, y, z);
	_px4_mag.set_error_count(perf_event_count(_comms_errors));
}

uint8_t MMC5983MA::read_register_block(SensorData *data)
{
	uint8_t reg = MMC5983MA_ADDR_XOUT_0;

	if (_interface->read(reg, data, sizeof(SensorData)) != PX4_OK) {
		perf_count(_comms_errors);

		return PX4_ERROR;
	}

	return PX4_OK;
}

uint8_t MMC5983MA::read_register(uint8_t reg)
{
	uint8_t value = 0;

	if (_interface->read(reg, &value, sizeof(value)) != PX4_OK) {
		perf_count(_comms_errors);
	}

	return value;
}

void MMC5983MA::write_register(uint8_t reg, uint8_t value)
{
	if (_interface->write(reg, &value, sizeof(value)) != PX4_OK) {
		perf_count(_comms_errors);
	}
}

void MMC5983MA::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}
