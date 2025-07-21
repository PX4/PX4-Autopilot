/****************************************************************************
 *
 *   Copyright (c) 2017-2022 PX4 Development Team. All rights reserved.
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

#include "SDP3X.hpp"
#include <parameters/param.h>

using namespace time_literals;

SDP3X::SDP3X(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_keep_retrying(config.keep_running)
{
}

SDP3X::~SDP3X()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int SDP3X::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	if (ret == PX4_OK) {
		ScheduleNow();
	}

	return ret;
}

void SDP3X::print_status()
{
	I2CSPIDriverBase::print_status();

	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}

int SDP3X::probe()
{
	_retries = 1;
	bool require_initialization = !init_sdp3x();

	if (require_initialization && _keep_retrying) {
		PX4_INFO("no sensor found, but will keep retrying");
		return 0;
	}

	return require_initialization ? -1 : 0;
}

int SDP3X::write_command(uint16_t command)
{
	uint8_t cmd[2];
	cmd[0] = static_cast<uint8_t>(command >> 8);
	cmd[1] = static_cast<uint8_t>(command & 0xff);
	return transfer(&cmd[0], 2, nullptr, 0);
}

bool SDP3X::init_sdp3x()
{
	return configure() == 0;
}

int SDP3X::configure()
{
	int ret = write_command(SDP3X_CONT_MODE_STOP);

	if (ret == PX4_OK) {
		px4_udelay(500); // SDP3X is unresponsive for 500us after stop continuous measurement command
		ret = write_command(SDP3X_CONT_MEAS_AVG_MODE);
	}

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		DEVICE_DEBUG("config failed");
		_state = State::RequireConfig;
		return ret;
	}

	_state = State::Configuring;

	return ret;
}

int SDP3X::read_scale()
{
	// get scale
	uint8_t val[9];
	int ret = transfer(nullptr, 0, &val[0], sizeof(val));

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		PX4_ERR("get scale failed");
		return ret;
	}

	// Check the CRC
	if (!crc(&val[0], 2, val[2]) || !crc(&val[3], 2, val[5]) || !crc(&val[6], 2, val[8])) {
		perf_count(_comms_errors);
		return PX4_ERROR;
	}

	_scale = (((uint16_t)val[6]) << 8) | val[7];

	switch (_scale) {
	case SDP3X_SCALE_PRESSURE_SDP31:
		set_device_type(DRV_DIFF_PRESS_DEVTYPE_SDP31);
		break;

	case SDP3X_SCALE_PRESSURE_SDP32:
		set_device_type(DRV_DIFF_PRESS_DEVTYPE_SDP32);
		break;

	case SDP3X_SCALE_PRESSURE_SDP33:
		set_device_type(DRV_DIFF_PRESS_DEVTYPE_SDP33);
		break;
	}

	return PX4_OK;
}

int SDP3X::collect()
{
	perf_begin(_sample_perf);

	const hrt_abstime timestamp_sample = hrt_absolute_time();

	// read 6 bytes from the sensor
	uint8_t val[6] {};
	int ret = transfer(nullptr, 0, &val[0], sizeof(val));

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return ret;
	}

	// Check the CRC
	if (!crc(&val[0], 2, val[2]) || !crc(&val[3], 2, val[5])) {
		perf_count(_comms_errors);
		return -EAGAIN;
	}

	int16_t P = (((int16_t)val[0]) << 8) | val[1];
	int16_t temp = (((int16_t)val[3]) << 8) | val[4];

	float diff_press_pa = static_cast<float>(P) / static_cast<float>(_scale);
	float temperature_c = temp / static_cast<float>(SDP3X_SCALE_TEMPERATURE);

	differential_pressure_s differential_pressure{};
	differential_pressure.timestamp_sample = timestamp_sample;
	differential_pressure.device_id = get_device_id();
	differential_pressure.differential_pressure_pa = diff_press_pa;
	int32_t differential_press_rev = 0;
	param_get(param_find("SENS_DPRES_REV"), &differential_press_rev);

	//If differential pressure reverse param set, swap positive and negative
	if (differential_press_rev == 1) {
		differential_pressure.differential_pressure_pa = -1.0f * diff_press_pa;
	}

	differential_pressure.temperature = temperature_c;
	differential_pressure.error_count = perf_event_count(_comms_errors);
	differential_pressure.timestamp = hrt_absolute_time();
	_differential_pressure_pub.publish(differential_pressure);

	perf_end(_sample_perf);

	return ret;
}

void SDP3X::RunImpl()
{
	switch (_state) {
	case State::RequireConfig:
		if (configure() == PX4_OK) {
			ScheduleDelayed(10_ms);

		} else {
			// periodically retry to configure
			ScheduleDelayed(300_ms);
		}

		break;

	case State::Configuring:
		if (read_scale() == 0) {
			_state = State::Running;

		} else {
			_state = State::RequireConfig;
		}

		ScheduleDelayed(10_ms);
		break;

	case State::Running:
		int ret = collect();

		if (ret != 0 && ret != -EAGAIN) {
			DEVICE_DEBUG("measure error");
			_state = State::RequireConfig;
		}

		ScheduleDelayed(CONVERSION_INTERVAL);
		break;
	}
}

bool SDP3X::crc(const uint8_t data[], unsigned size, uint8_t checksum)
{
	uint8_t crc_value = 0xff;

	// calculate 8-bit checksum with polynomial 0x31 (x^8 + x^5 + x^4 + 1)
	for (unsigned i = 0; i < size; i++) {
		crc_value ^= (data[i]);

		for (int bit = 8; bit > 0; --bit) {
			if (crc_value & 0x80) {
				crc_value = (crc_value << 1) ^ 0x31;

			} else {
				crc_value = (crc_value << 1);
			}
		}
	}

	// verify checksum
	return (crc_value == checksum);
}
