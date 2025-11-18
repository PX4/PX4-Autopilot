/****************************************************************************
 *
 *   Copyright (C) 2025 PX4 Development Team. All rights reserved.
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
#include "tla2528.h"
#include <drivers/drv_adc.h>

#define READ 		0x10
#define WRITE 		0x08
#define SET_BIT 	0x18
#define CLEAR_BIT 	0x20

#define SYSTEM_STATUS	0x00
#define GENERAL_CFG	0x01
#define DATA_CFG	0x02
#define OSR_CFG		0x03
#define OPMODE_CFG	0x04
#define PIN_CFG		0x05
#define GPIO_CFG	0x07
#define GPO_DRIVE_CFG	0x09
#define GPO_VALUE	0x0B
#define GPI_VALUE	0x0D
#define SEQUENCE_CFG	0x10
#define CHANNEL_SEL	0x11
#define AUTO_SEQ_CH_SEL	0x12

TLA2528::TLA2528(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	ModuleParams(nullptr),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": single-sample")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": comms errors"))
{
	static_assert(arraySize(adc_report_s::channel_id) >= NUM_CHANNELS, "TLA2528 reports 8 channels");
}

TLA2528::~TLA2528()
{
	ScheduleClear();
	perf_free(_cycle_perf);
	perf_free(_comms_errors);
}

int TLA2528::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		PX4_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	_adc_report.device_id = this->get_device_id();
	_adc_report.v_ref = _adc_tla2528_refv.get();
	_adc_report.resolution = 4096;

	ScheduleClear();
	ScheduleOnInterval(SAMPLE_INTERVAL, SAMPLE_INTERVAL);
	return PX4_OK;
}

int TLA2528::init_calibrate()
{
	uint8_t send_data[3];
	send_data[0] = SET_BIT;
	send_data[1] = GENERAL_CFG;
	send_data[2] = 0x02;
	int ret = transfer(&send_data[0], 3, nullptr, 0);

	if (ret != PX4_OK) {
		PX4_DEBUG("TLA2528::Initializing Calibration failed (%i)", ret);
		perf_count(_comms_errors);
	}

	return ret;
}

int TLA2528::poll_calibrate()
{
	uint8_t send_data[2];
	uint8_t recv_data;
	send_data[0] = READ;
	send_data[1] = GENERAL_CFG;
	int ret = transfer(&send_data[0], 2, nullptr, 0);
	ret |= transfer(nullptr, 0, &recv_data, 1);

	if (recv_data & 2u) {
		PX4_DEBUG("TLA2528::Calibration not yet finished");
		perf_count(_comms_errors);
		return PX4_ERROR;
	}

	return ret;
}

int TLA2528::init_reset()
{
	uint8_t send_data[3];
	send_data[0] = SET_BIT;
	send_data[1] = GENERAL_CFG;
	send_data[2] = 0x01;
	int ret = transfer(&send_data[0], 3, nullptr, 0);

	if (ret != PX4_OK) {
		PX4_DEBUG("TLA2528::Initializing Reset failed (%i)", ret);
		perf_count(_comms_errors);
	}

	return ret;
}

int TLA2528::poll_reset()
{
	uint8_t send_data[2];
	uint8_t recv_data;
	send_data[0] = READ;
	send_data[0] = GENERAL_CFG;
	int ret = transfer(&send_data[0], 2, nullptr, 0);
	ret |= transfer(nullptr, 0, &recv_data, 1);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return ret;
	}

	if (recv_data & 1u) {
		PX4_DEBUG("TLA2528::Reset not finished");
		return PX4_ERROR;
	}

	return ret;
}

void TLA2528::adc_get()
{
	// Start sequential read
	uint8_t send_data[3] = {SET_BIT, SEQUENCE_CFG, 0x10};
	uint8_t recv_data[2];
	int ret = transfer(&send_data[0], 3, nullptr, 0);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return;
	}

	// Read data
	for (int i = 0; i < NUM_CHANNELS; i++) {
		ret = transfer(nullptr, 0, &recv_data[0], 2);

		if (ret == PX4_OK) {
			uint16_t tmp = ((uint16_t)recv_data[0]) << 8;
			uint16_t measurement = (tmp | recv_data[1]) >> 4;
			uint8_t ch_id = recv_data[1] & 0x0F;

			_adc_report.channel_id[i] = ch_id;
			_adc_report.raw_data[i] = measurement;

		} else {
			perf_count(_comms_errors);
		}
	}

	// Stop sequential read
	send_data[0] = CLEAR_BIT;
	send_data[1] = SEQUENCE_CFG;
	send_data[2] = 0x10;
	transfer(&send_data[0], 3, nullptr, 0);
	return;
}

int TLA2528::probe()
{
	for (int i = 0; i < 3; i++) {
		// Set device in debug mode (should respond with 0xA5AX to all reads)
		uint8_t send_data[3] = {SET_BIT, DATA_CFG, 0x80};
		int ret = transfer(&send_data[0], 3, nullptr, 0);

		// Read
		uint8_t recv_data[2];
		send_data[0] = SET_BIT;
		send_data[0] = DATA_CFG;
		ret |= transfer(&send_data[0], 2, nullptr, 0);
		ret |= transfer(nullptr, 0, &recv_data[0], 2);

		// Turn debug mode off
		send_data[0] = CLEAR_BIT;
		send_data[1] = DATA_CFG;
		send_data[2] = 0x80;
		ret |= transfer(&send_data[0], 3, nullptr, 0);

		if (recv_data[0] == 165 && recv_data[1] >= 160 && ret == PX4_OK) {
			return PX4_OK;
		}

		px4_sleep(1);
	}

	return PX4_ERROR;
}

int TLA2528::configure()
{
	uint8_t send_data[3];
	// Configure pins as analog
	send_data[0] = SET_BIT;
	send_data[1] = PIN_CFG;
	send_data[2] = 0x00;
	int ret = transfer(&send_data[0], 3, nullptr, 0);

	// Append channel-id to measurements
	send_data[0] = SET_BIT;
	send_data[1] = DATA_CFG;
	send_data[2] = 0x10;
	ret |= transfer(&send_data[0], 3, nullptr, 0);

	// Activate all pins
	send_data[0] = SET_BIT;
	send_data[1] = AUTO_SEQ_CH_SEL;
	send_data[2] = 0xFF;
	ret |= transfer(&send_data[0], 3, nullptr, 0);

	// Set seq-mode
	send_data[0] = SET_BIT;
	send_data[1] = SEQUENCE_CFG;
	send_data[2] = 0x01;
	ret |= transfer(&send_data[0], 3, nullptr, 0);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		PX4_DEBUG("TLA2528::Configuring failed (%i)", ret);
	}

	return ret;
}

void TLA2528::exit_and_cleanup()
{
	I2CSPIDriverBase::exit_and_cleanup();	// nothing to do
}

void TLA2528::RunImpl()
{
	if (should_exit()) {
		PX4_INFO("stopping");
		return;	// stop and return immediately to avoid unexpected schedule from stopping procedure
	}

	int ret;

	switch (_state) {
	case STATE::RESET:
		ret = init_reset();

		if (ret == PX4_OK) {
			_state = STATE::CONFIGURE;
		}

		break;

	case STATE::CONFIGURE:
		ret = poll_reset();
		ret |= configure();
		ret |= init_calibrate();

		if (ret == PX4_OK) {
			_state = STATE::CALIBRATE;

		} else {
			_state = STATE::RESET;
		}

		break;

	case STATE::CALIBRATE:
		ret = poll_calibrate();

		if (ret == PX4_OK) {
			_state = STATE::WORK;

		} else {
			_state = STATE::RESET;
		}

		break;

	case STATE::WORK:
		perf_begin(_cycle_perf);
		adc_get();
		_adc_report.timestamp = hrt_absolute_time();
		_adc_report_pub.publish(_adc_report);
		perf_end(_cycle_perf);
		break;
	}

	for (unsigned i = 0; i < arraySize(adc_report_s::channel_id); ++i) {
		_adc_report.channel_id[i] = -1;
	}
}
