/****************************************************************************
 *
 *   Copyright (c) 2014-2021 PX4 Development Team. All rights reserved.
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

#include "lightware_laser_serial.hpp"

#include <errno.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

/* Configuration Constants */
#define LW_TAKE_RANGE_REG		'd'
static constexpr uint32_t LW_READ_TIMEOUT_MS {10};

LightwareLaserSerial::LightwareLaserSerial(const char *port, uint8_t rotation) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
	_px4_rangefinder(0, rotation),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": com_err"))
{
	/* store port name */
	strncpy(_port, port, sizeof(_port) - 1);

	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';

	device::Device::DeviceId device_id;
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;

	uint8_t bus_num = atoi(&_port[strlen(_port) - 1]); // Assuming '/dev/ttySx'

	if (bus_num < 10) {
		device_id.devid_s.bus = bus_num;
	}

	_px4_rangefinder.set_device_id(device_id.devid);
	_px4_rangefinder.set_device_type(DRV_DIST_DEVTYPE_LIGHTWARE_LASER);
}

LightwareLaserSerial::~LightwareLaserSerial()
{
	stop();

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
LightwareLaserSerial::init()
{
	int32_t hw_model = 0;
	param_get(param_find("SENS_EN_SF0X"), &hw_model);

	switch (hw_model) {
	case 1: /* SF02 (40m, 12 Hz)*/
		_px4_rangefinder.set_min_distance(0.30f);
		_px4_rangefinder.set_max_distance(40.0f);
		_interval = 83334;
		break;

	case 2:  /* SF10/a (25m 32Hz) */
		_px4_rangefinder.set_min_distance(0.01f);
		_px4_rangefinder.set_max_distance(25.0f);
		_interval = 31250;
		break;

	case 3:  /* SF10/b (50m 32Hz) */
		_px4_rangefinder.set_min_distance(0.01f);
		_px4_rangefinder.set_max_distance(50.0f);
		_interval = 31250;
		break;

	case 4:  /* SF10/c (100m 16Hz) */
		_px4_rangefinder.set_min_distance(0.01f);
		_px4_rangefinder.set_max_distance(100.0f);
		_interval = 62500;
		break;

	case 5:
		/* SF11/c (120m 20Hz) */
		_px4_rangefinder.set_min_distance(0.2f);
		_px4_rangefinder.set_max_distance(120.0f);
		_interval = 50000;
		break;

	case 6:
		/* SF30/B (50m 39Hz) */
		_px4_rangefinder.set_min_distance(0.2f);
		_px4_rangefinder.set_max_distance(50.0f);
		_interval = 1e6 / 39;
		_simple_serial = true;
		break;

	case 7:
		/* SF30/C (100m 39Hz) */
		_px4_rangefinder.set_min_distance(0.2f);
		_px4_rangefinder.set_max_distance(100.0f);
		_interval = 1e6 / 39;
		_simple_serial = true;
		break;

	case 8:
		/* LW20/c (100M 20Hz) */
		_px4_rangefinder.set_min_distance(0.2f);
		_px4_rangefinder.set_max_distance(100.0f);
		_interval = 50000;
		break;

	default:
		PX4_ERR("invalid HW model %" PRIi32 ".", hw_model);
		return -1;
	}

	start();

	return PX4_OK;
}

int LightwareLaserSerial::measure()
{
	// Send the command to begin a measurement.
	char cmd = LW_TAKE_RANGE_REG;
	int ret = _uart.write(&cmd, sizeof(cmd));

	if (ret != sizeof(cmd)) {
		perf_count(_comms_errors);
		PX4_DEBUG("write fail %d", ret);
		return ret;
	}

	return PX4_OK;
}

int LightwareLaserSerial::collect()
{
	perf_begin(_sample_perf);

	/* clear buffer if last read was too long ago */
	int64_t read_elapsed = hrt_elapsed_time(&_last_read);

	/* the buffer for read chars is buflen minus null termination */
	char readbuf[sizeof(_linebuf)];
	unsigned readlen = sizeof(readbuf) - 1;

	/* read from the sensor (uart buffer) */
	const hrt_abstime timestamp_sample = hrt_absolute_time();
	int ret = _uart.readAtLeast(reinterpret_cast<uint8_t *>(&readbuf[0]), readlen, 1, LW_READ_TIMEOUT_MS);

	if (ret < 0) {
		PX4_DEBUG("read err: %d", ret);
		perf_count(_comms_errors);
		perf_end(_sample_perf);

		/* only throw an error if we time out */
		if (read_elapsed > (_interval * 2)) {
			return ret;

		} else {
			return -EAGAIN;
		}

	} else if (ret == 0) {
		return -ETIMEDOUT;
	}

	_last_read = hrt_absolute_time();

	float distance_m = -1.0f;
	bool valid = false;

	if (_simple_serial) {
		// Simplified protocol used by the SF30/B and SF30/C
		// First byte: MSB of byte is set;  remaining bits are high "byte" of reading
		// Second byte: MSB of byte is not set; remaining bits are low "byte" of reading
		// Distance in centimeters = (buf[0] & 0x7F)*128 + buf[0]
		bool have_msb = false;

		for (int i = 0; i < ret; i++) {
			if (have_msb && !(readbuf[i] & 0x80)) {
				distance_m += readbuf[i] * .01f;
				valid = true;
				break;

			} else {
				if (readbuf[i] & 0x80) {
					have_msb = true;
					distance_m = (readbuf[i] & 0x7F) * 1.28f;
				}
			}
		}

	} else {
		for (int i = 0; i < ret; i++) {
			// Check for overflow
			if (_linebuf_index >= sizeof(_linebuf)) {
				_parse_state = LW_PARSE_STATE0_UNSYNC;
			}

			if (OK == lightware_parser(readbuf[i], _linebuf, &_linebuf_index, &_parse_state, &distance_m)) {
				valid = true;
			}
		}
	}


	if (!valid) {
		return -EAGAIN;
	}

	PX4_DEBUG("val (float): %8.4f, raw: %s, valid: %s", (double)distance_m, _linebuf, ((valid) ? "OK" : "NO"));

	_px4_rangefinder.update(timestamp_sample, distance_m);

	perf_end(_sample_perf);

	return PX4_OK;
}

void LightwareLaserSerial::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;

	/* schedule a cycle to start things */
	ScheduleNow();
}

void LightwareLaserSerial::stop()
{
	ScheduleClear();
	_uart.close();
}

void LightwareLaserSerial::Run()
{
	if (!_uart.isOpen()) {
		if (!_uart.setPort(_port)) {
			PX4_ERR("failed to configure port %s", _port);
			return;
		}

		int32_t hw_model = 0;

		param_get(param_find("SENS_EN_SF0X"), &hw_model);

		const uint32_t baudrate = (hw_model >= 5) ? 115200 : 9600;

		if (!_uart.setBaudrate(baudrate)) {
			PX4_ERR("failed to set baudrate %" PRIu32 " on %s", baudrate, _port);
			return;
		}

		if (!_uart.open()) {
			PX4_ERR("failed to open %s", _port);
			_uart.close();
			return;
		}

		// LW20: Enable serial mode by sending some characters
		if (hw_model == 8) {
			const char *data = "www\r\n";
			(void)_uart.write(data, strlen(data));
		}
	}

	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		int collect_ret = collect();

		if (collect_ret == -EAGAIN) {
			/* reschedule to grab the missing bits, time to transmit 8 bytes @ 9600 bps */
			ScheduleDelayed(1042 * 8);

			return;
		}

		if (OK != collect_ret) {

			/* we know the sensor needs about four seconds to initialize */
			if (hrt_absolute_time() > 5 * 1000 * 1000LL && _consecutive_fail_count < 5) {
				PX4_ERR("collection error #%u", _consecutive_fail_count);
			}

			_consecutive_fail_count++;

			/* restart the measurement state machine */
			start();
			return;

		} else {
			/* apparently success */
			_consecutive_fail_count = 0;
		}

		/* next phase is measurement */
		_collect_phase = false;
	}

	/* measurement phase */
	if (OK != measure()) {
		PX4_DEBUG("measure error");
	}

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	ScheduleDelayed(_interval);
}

void LightwareLaserSerial::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}
