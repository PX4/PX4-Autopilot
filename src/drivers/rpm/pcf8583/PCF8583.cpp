/****************************************************************************
 *
 *   Copyright (c) 2020, 2021 PX4 Development Team. All rights reserved.
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

#include "PCF8583.hpp"

PCF8583::PCF8583(const I2CSPIDriverConfig &config) :
	I2C(config),
	ModuleParams(nullptr),
	I2CSPIDriver(config)
{
}

int PCF8583::init()
{
	if (I2C::init() != PX4_OK) {
		return PX4_ERROR;
	}

	PX4_DEBUG("addr: %" PRId8 ", pool: %" PRId32 ", reset: %" PRId32 ", magenet: %" PRId32, get_device_address(),
		  _param_pcf8583_pool.get(),
		  _param_pcf8583_reset.get(),
		  _param_pcf8583_magnet.get());


	initCounter();

	ScheduleOnInterval(_param_pcf8583_pool.get());

	_rpm_pub.advertise();

	return PX4_OK;
}

int PCF8583::probe()
{
	uint8_t s = readRegister(0x00);
	PX4_INFO("status register: %" PRId8 " fail_count: %" PRId8, s, _tranfer_fail_count);

	if (_tranfer_fail_count != 0 || (s != 0 && s != 32)) { //extremly poor detection :-(
		return PX4_ERROR;
	}

	return PX4_OK;
}

void PCF8583::initCounter()
{
	// set counter mode
	_tranfer_fail_count = 0;
	setRegister(0x00, 0b00100000);
	resetCounter();

}

uint32_t PCF8583::getCounter()
{
	uint8_t a = readRegister(0x01);
	uint8_t b = readRegister(0x02);
	uint8_t c = readRegister(0x03);

	return uint32_t(
		       hiWord(a) * 1u + loWord(a) * 10u
		       + hiWord(b) * 100u + loWord(b) * 1000u
		       + hiWord(c) * 10000u + loWord(c) * 1000000u);
}

void PCF8583::resetCounter()
{
	_last_measurement_time = hrt_absolute_time();
	setRegister(0x01, 0x00);
	setRegister(0x02, 0x00);
	setRegister(0x03, 0x00);
	_count = 0;
	_last_reset_time = _last_measurement_time;
	_reset_count ++;
}

void PCF8583::setRegister(uint8_t reg, uint8_t value)
{
	uint8_t buff[2];
	buff[0] = reg;
	buff[1] = value;
	int ret = transfer(buff, 2, nullptr, 0);

	if (PX4_OK != ret) {
		PX4_DEBUG("setRegister : i2c::transfer returned %d", ret);
		_tranfer_fail_count++;
	}
}

uint8_t PCF8583::readRegister(uint8_t reg)
{
	uint8_t rcv{};
	int ret = transfer(&reg, 1, &rcv, 1);

	if (PX4_OK != ret) {
		PX4_DEBUG("readRegister : i2c::transfer returned %d", ret);
		_tranfer_fail_count++;
	}

	return rcv;
}

void PCF8583::RunImpl()
{
	// read sensor and compute frequency
	uint32_t oldcount = _count;
	uint64_t oldtime = _last_measurement_time;

	_count = getCounter();
	_last_measurement_time = hrt_absolute_time();

	int diffCount = _count - oldcount;
	uint64_t diffTime = _last_measurement_time - oldtime;

	//check if device failed or reset
	uint8_t s = readRegister(0x00);

	if (_tranfer_fail_count > 0 || s != 0b00100000 || diffCount < 0) {
		PX4_ERR("pcf8583 RPM sensor restart: fail count %" PRId8 ", status: %" PRId8 ", diffCount: %" PRId8,
			_tranfer_fail_count, s, diffCount);
		initCounter();
		return;
	}

	//check counter range
	if (_param_pcf8583_reset.get() < diffCount + (int)_count) {
		resetCounter();
	}

	float indicated_rpm = (float)diffCount / _param_pcf8583_magnet.get() / ((float)diffTime / 1000000) * 60.f;
	float estimated_accurancy = 1 / (float)_param_pcf8583_magnet.get() / ((float)diffTime / 1000000) * 60.f;

	// publish
	rpm_s msg{};
	msg.indicated_frequency_rpm = indicated_rpm;
	msg.estimated_accurancy_rpm = estimated_accurancy;
	msg.timestamp = hrt_absolute_time();
	_rpm_pub.publish(msg);
}

void PCF8583::print_status()
{
	I2CSPIDriverBase::print_status();
	PX4_INFO("poll interval:  %" PRId32 " us", _param_pcf8583_pool.get());
	PX4_INFO("Last reset %.3fs ago, Count of resets: %d", (double)(hrt_absolute_time() - _last_reset_time) / 1000000.0,
		 _reset_count);
	PX4_INFO("Last count %ld", _count);
}
