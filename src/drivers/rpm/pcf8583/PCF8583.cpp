/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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
	setRegister(0x00, 0b00100000);

	uint8_t s = readRegister(0x00);
	PX4_DEBUG("status register: %" PRId8 " fail_count: %" PRId8, s, _tranfer_fail_count);

	// PCF8583 contains free RAM registers
	// This checks if I2C devices contains this RAM memory registers
	// Some values are stored into this registers
	// then it is vertified that the entered values fit.
	setRegister(0x04, 10);
	setRegister(0x05, 10);
	setRegister(0x06, 10);
	setRegister(0x0c, 5);
	setRegister(0x0d, 5);
	setRegister(0x0e, 5);
	uint32_t tmp{0};

	// check values stored in free RAM parts
	tmp += readRegister(0x04);
	tmp += readRegister(0x05);
	tmp += readRegister(0x06);
	tmp += readRegister(0x0c);
	tmp += readRegister(0x0d);
	tmp += readRegister(0x0e);

	if (tmp != 45) {
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
	// Counter value is stored in 9 words
	// in 3 register as BCD value
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


// Configure PCF8583 driver into counting mode
void PCF8583::setRegister(uint8_t reg, uint8_t value)
{
	uint8_t buff[2];
	buff[0] = reg;
	buff[1] = value;
	int ret = transfer(buff, 2, nullptr, 0);

	if (reg == 0x00) {
		_last_config_register_content = value;
	}

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
	int32_t oldcount = _count;

	int32_t diffTime = hrt_elapsed_time(&_last_measurement_time);

	// check if delay is enought
	if (diffTime < _param_pcf8583_pool.get() / 2) {
		PX4_ERR("pcf8583 loop called too early");
		return;
	}

	_count = getCounter();
	_last_measurement_time = hrt_absolute_time();

	int32_t diffCount = _count - oldcount;

	// check if there is enought space in counter
	// Otherwise, reset counter
	if (diffCount > (999999 - oldcount)) {
		PX4_ERR("pcf8583 RPM register overflow");
		resetCounter();
		return;
	}

	//check if device failed or reset
	uint8_t s = readRegister(0x00);

	if (_tranfer_fail_count > 0 || s != 0b00100000 || diffCount < 0) {
		PX4_ERR("pcf8583 RPM sensor restart: fail count %d, status: %d, diffCount: %ld",
			_tranfer_fail_count, s, diffCount);
		initCounter();
		return;
	}

	// Calculate RPM and accuracy estimation
	float indicated_rpm = (((float)diffCount / _param_pcf8583_magnet.get()) / ((float)diffTime / 1e6f)) * 60.f;

	// publish data to uorb
	rpm_s msg{};
	msg.rpm_estimate = indicated_rpm;
	msg.timestamp = hrt_absolute_time();
	_rpm_pub.publish(msg);

	//check counter range
	if (_param_pcf8583_reset.get() < diffCount + (int)_count) {
		resetCounter();
	}
}

void PCF8583::print_status()
{
	I2CSPIDriverBase::print_status();
	PX4_INFO("poll interval:  %" PRId32 " us", _param_pcf8583_pool.get());
	PX4_INFO("Last reset %.3fs ago, Count of resets: %d", (double)(hrt_absolute_time() - _last_reset_time) / 1000000.0,
		 _reset_count);
	PX4_INFO("Last count %ld", _count);
}
