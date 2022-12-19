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

/**
 * @file SDP6X.hpp
 *
 * Driver for Sensirion SDP6X Differential Pressure Sensor
 *
 */

#include "SDP6X.hpp"

SDP6X::SDP6X(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config)
{
}


SDP6X::~SDP6X()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int SDP6X::init()
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

void SDP6X::print_status()
{
	I2CSPIDriverBase::print_status();

	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}

int
SDP6X::probe()
{
	return !init_sdp6x();
}

bool
SDP6X::init_sdp6x()
{
	uint8_t reset_cmd = SDP6X_RESET_CMD;
	int ret = transfer(&reset_cmd, 1, nullptr, 0);

	if (ret == 0) {
		_device_id.devid_s.devtype = DRV_DIFF_PRESS_DEVTYPE_SDP31;
		// wait until sensor is ready
		px4_usleep(20000);
		ScheduleNow();
	}

	return ret == 0;
}

int
SDP6X::collect()
{
	perf_begin(_sample_perf);

	const hrt_abstime timestamp_sample = hrt_absolute_time();

	// this takes about 5ms. But even if we split the write and read with a delay in between,
	// the device will still block for several ms. So effectively can't run another device on that bus.
	uint8_t measure_cmd = SDP6X_MEASURE_CMD;
	uint8_t data[3];
	int ret = transfer(&measure_cmd, 1, (uint8_t *)&data[0], sizeof(data));

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return ret;
	}

	// Check the CRC
	if (!crc(&data[0], 2, data[2])) {
		perf_count(_comms_errors);
		return EAGAIN;
	}

	int16_t P = (((int16_t)data[0]) << 8) | data[1];

	float diff_press_pa_raw = static_cast<float>(P) / 60.f; // 500Pa -> scale factor=60

	differential_pressure_s differential_pressure{};

	differential_pressure.timestamp = hrt_absolute_time();
	differential_pressure.timestamp_sample = timestamp_sample;
	differential_pressure.error_count = perf_event_count(_comms_errors);
	differential_pressure.temperature = 0.f;
	differential_pressure.differential_pressure_pa = diff_press_pa_raw;
	differential_pressure.device_id = get_device_id();

	_differential_pressure_pub.publish(differential_pressure);

	perf_end(_sample_perf);

	return ret;
}

void
SDP6X::RunImpl()
{
	int ret = PX4_ERROR;

	// measurement phase
	ret = collect();

	if (PX4_OK != ret) {
		DEVICE_DEBUG("measure error");
	}

	// schedule a fresh cycle call when the measurement is done
	ScheduleDelayed(CONVERSION_INTERVAL);
}

bool SDP6X::crc(const uint8_t data[], unsigned size, uint8_t checksum)
{
	uint8_t crc_value = 0;

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
