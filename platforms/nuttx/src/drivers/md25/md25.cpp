/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 * @file md25.cpp
 *
 * Driver for MD25 I2C Motor Driver
 *
 * references:
 * http://www.robot-electronics.co.uk/htm/md25tech.htm
 * http://www.robot-electronics.co.uk/files/rpi_md25.c
 *
 */

#include "md25.hpp"
#include <poll.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#include <systemlib/err.h>
#include <arch/board/board.h>
#include <systemlib/mavlink_log.h>

#include <uORB/Publication.hpp>
#include <uORB/topics/debug_key_value.h>
#include <drivers/drv_hrt.h>

// registers
enum {
	// RW: read/write
	// R: read
	REG_SPEED1_RW = 0,
	REG_SPEED2_RW,
	REG_ENC1A_R,
	REG_ENC1B_R,
	REG_ENC1C_R,
	REG_ENC1D_R,
	REG_ENC2A_R,
	REG_ENC2B_R,
	REG_ENC2C_R,
	REG_ENC2D_R,
	REG_BATTERY_VOLTS_R,
	REG_MOTOR1_CURRENT_R,
	REG_MOTOR2_CURRENT_R,
	REG_SW_VERSION_R,
	REG_ACCEL_RATE_RW,
	REG_MODE_RW,
	REG_COMMAND_RW,
};

// File descriptors
static orb_advert_t mavlink_log_pub;

MD25::MD25(const char *deviceName, int bus,
	   uint16_t address, uint32_t speed) :
	I2C("MD25", deviceName, bus, address, speed),
	_controlPoll(),
	_actuators(NULL, ORB_ID(actuator_controls_0), 20),
	_version(0),
	_motor1Speed(0),
	_motor2Speed(0),
	_revolutions1(0),
	_revolutions2(0),
	_batteryVoltage(0),
	_motor1Current(0),
	_motor2Current(0),
	_motorAccel(0),
	_mode(MODE_UNSIGNED_SPEED),
	_command(CMD_RESET_ENCODERS)
{
	// setup control polling
	_controlPoll.fd = _actuators.getHandle();
	_controlPoll.events = POLLIN;

	// if initialization fails raise an error, unless
	// probing
	int ret = I2C::init();

	if (ret != OK) {
		warnc(ret, "I2C::init failed for bus: %d address: %d\n", bus, address);
	}

	// setup default settings, reset encoders
	setMotor1Speed(0);
	setMotor2Speed(0);
	resetEncoders();
	_setMode(MD25::MODE_UNSIGNED_SPEED);
	setSpeedRegulation(false);
	setMotorAccel(10);
	setTimeout(true);
}

MD25::~MD25()
{
}

int MD25::readData()
{
	uint8_t sendBuf[1];
	sendBuf[0] = REG_SPEED1_RW;
	uint8_t recvBuf[17];
	int ret = transfer(sendBuf, sizeof(sendBuf),
			   recvBuf, sizeof(recvBuf));

	if (ret == OK) {
		_version = recvBuf[REG_SW_VERSION_R];
		_motor1Speed = _uint8ToNorm(recvBuf[REG_SPEED1_RW]);
		_motor2Speed = _uint8ToNorm(recvBuf[REG_SPEED2_RW]);
		_revolutions1 = -int32_t((recvBuf[REG_ENC1A_R] << 24) +
					 (recvBuf[REG_ENC1B_R] << 16) +
					 (recvBuf[REG_ENC1C_R] << 8)  +
					 recvBuf[REG_ENC1D_R]) / 360.0;
		_revolutions2 = -int32_t((recvBuf[REG_ENC2A_R] << 24) +
					 (recvBuf[REG_ENC2B_R] << 16) +
					 (recvBuf[REG_ENC2C_R] << 8)  +
					 recvBuf[REG_ENC2D_R]) / 360.0;
		_batteryVoltage = recvBuf[REG_BATTERY_VOLTS_R] / 10.0;
		_motor1Current = recvBuf[REG_MOTOR1_CURRENT_R] / 10.0;
		_motor2Current = recvBuf[REG_MOTOR2_CURRENT_R] / 10.0;
		_motorAccel = recvBuf[REG_ACCEL_RATE_RW];
		_mode = e_mode(recvBuf[REG_MODE_RW]);
		_command = e_cmd(recvBuf[REG_COMMAND_RW]);
	}

	return ret;
}

void MD25::status(char *string, size_t n)
{
	snprintf(string, n,
		 "version:\t%10d\n" \
		 "motor 1 speed:\t%10.2f\n" \
		 "motor 2 speed:\t%10.2f\n" \
		 "revolutions 1:\t%10.2f\n" \
		 "revolutions 2:\t%10.2f\n" \
		 "battery volts :\t%10.2f\n" \
		 "motor 1 current :\t%10.2f\n" \
		 "motor 2 current :\t%10.2f\n" \
		 "motor accel :\t%10d\n" \
		 "mode :\t%10d\n" \
		 "command :\t%10d\n",
		 getVersion(),
		 double(getMotor1Speed()),
		 double(getMotor2Speed()),
		 double(getRevolutions1()),
		 double(getRevolutions2()),
		 double(getBatteryVolts()),
		 double(getMotor1Current()),
		 double(getMotor2Current()),
		 getMotorAccel(),
		 getMode(),
		 getCommand());
}

uint8_t MD25::getVersion()
{
	return _version;
}

float MD25::getMotor1Speed()
{
	return _motor1Speed;
}

float MD25::getMotor2Speed()
{
	return _motor2Speed;
}

float MD25::getRevolutions1()
{
	return _revolutions1;
}

float MD25::getRevolutions2()
{
	return _revolutions2;
}

float MD25::getBatteryVolts()
{
	return _batteryVoltage;
}

float MD25::getMotor1Current()
{
	return _motor1Current;
}
float MD25::getMotor2Current()
{
	return _motor2Current;
}

uint8_t MD25::getMotorAccel()
{
	return _motorAccel;
}

MD25::e_mode MD25::getMode()
{
	return _mode;
}

MD25::e_cmd MD25::getCommand()
{
	return _command;
}

int MD25::resetEncoders()
{
	return _writeUint8(REG_COMMAND_RW,
			   CMD_RESET_ENCODERS);
}

int MD25::_setMode(e_mode mode)
{
	return _writeUint8(REG_MODE_RW,
			   mode);
}

int MD25::setSpeedRegulation(bool enable)
{
	if (enable) {
		return _writeUint8(REG_COMMAND_RW,
				   CMD_ENABLE_SPEED_REGULATION);

	} else {
		return _writeUint8(REG_COMMAND_RW,
				   CMD_DISABLE_SPEED_REGULATION);
	}
}

int MD25::setTimeout(bool enable)
{
	if (enable) {
		return _writeUint8(REG_COMMAND_RW,
				   CMD_ENABLE_TIMEOUT);

	} else {
		return _writeUint8(REG_COMMAND_RW,
				   CMD_DISABLE_TIMEOUT);
	}
}

int MD25::setDeviceAddress(uint8_t address)
{
	uint8_t sendBuf[1];
	sendBuf[0] = CMD_CHANGE_I2C_SEQ_0;
	int ret = OK;
	ret = transfer(sendBuf, sizeof(sendBuf),
		       nullptr, 0);

	if (ret != OK) {
		warnc(ret, "MD25::setDeviceAddress");
		return ret;
	}

	usleep(5000);
	sendBuf[0] = CMD_CHANGE_I2C_SEQ_1;
	ret = transfer(sendBuf, sizeof(sendBuf),
		       nullptr, 0);

	if (ret != OK) {
		warnc(ret, "MD25::setDeviceAddress");
		return ret;
	}

	usleep(5000);
	sendBuf[0] = CMD_CHANGE_I2C_SEQ_2;
	ret = transfer(sendBuf, sizeof(sendBuf),
		       nullptr, 0);

	if (ret != OK) {
		warnc(ret, "MD25::setDeviceAddress");
		return ret;
	}

	return OK;
}

int MD25::setMotorAccel(uint8_t accel)
{
	return _writeUint8(REG_ACCEL_RATE_RW,
			   accel);
}

int MD25::setMotor1Speed(float value)
{
	return _writeUint8(REG_SPEED1_RW,
			   _normToUint8(value));
}

int MD25::setMotor2Speed(float value)
{
	return _writeUint8(REG_SPEED2_RW,
			   _normToUint8(value));
}

void MD25::update()
{
	// wait for an actuator publication,
	// check for exit condition every second
	// note "::poll" is required to distinguish global
	// poll from member function for driver
	if (::poll(&_controlPoll, 1, 1000) < 0) { return; } // poll error

	// if new data, send to motors
	if (_actuators.updated()) {
		_actuators.update();
		setMotor1Speed(_actuators.control[CH_SPEED_LEFT]);
		setMotor2Speed(_actuators.control[CH_SPEED_RIGHT]);
	}
}

int MD25::probe()
{
	uint8_t goodAddress = 0;
	bool found = false;
	int ret = OK;

	// try initial address first, if good, then done
	if (readData() == OK) { return ret; }

	// try all other addresses
	uint8_t testAddress = 0;

	//printf("searching for MD25 address\n");
	while (true) {
		set_address(testAddress);
		ret = readData();

		if (ret == OK && !found) {
			//printf("device found at address: 0x%X\n", testAddress);
			if (!found) {
				found = true;
				goodAddress = testAddress;
			}
		}

		if (testAddress > 254) {
			break;
		}

		testAddress++;
	}

	if (found) {
		set_address(goodAddress);
		return OK;

	} else {
		set_address(0);
		return ret;
	}
}

int MD25::search()
{
	uint8_t goodAddress = 0;
	bool found = false;
	int ret = OK;
	// try all other addresses
	uint8_t testAddress = 0;

	//printf("searching for MD25 address\n");
	while (true) {
		set_address(testAddress);
		ret = readData();

		if (ret == OK && !found) {
			printf("device found at address: 0x%X\n", testAddress);

			if (!found) {
				found = true;
				goodAddress = testAddress;
			}
		}

		if (testAddress > 254) {
			break;
		}

		testAddress++;
	}

	if (found) {
		set_address(goodAddress);
		return OK;

	} else {
		set_address(0);
		return ret;
	}
}

int MD25::_writeUint8(uint8_t reg, uint8_t value)
{
	uint8_t sendBuf[2];
	sendBuf[0] = reg;
	sendBuf[1] = value;
	return transfer(sendBuf, sizeof(sendBuf),
			nullptr, 0);
}

int MD25::_writeInt8(uint8_t reg, int8_t value)
{
	uint8_t sendBuf[2];
	sendBuf[0] = reg;
	sendBuf[1] = value;
	return transfer(sendBuf, sizeof(sendBuf),
			nullptr, 0);
}

float MD25::_uint8ToNorm(uint8_t value)
{
	// TODO, should go from 0 to 255
	// possibly should handle this differently
	return (value - 128) / 127.0;
}

uint8_t MD25::_normToUint8(float value)
{
	if (value > 1) { value = 1; }

	if (value < -1) { value = -1; }

	// TODO, should go from 0 to 255
	// possibly should handle this differently
	return 127 * value + 128;
}

int md25Test(const char *deviceName, uint8_t bus, uint8_t address)
{
	printf("md25 test: starting\n");

	// setup
	MD25 md25("/dev/md25", bus, address);

	// print status
	char buf[400];
	md25.status(buf, sizeof(buf));
	printf("%s\n", buf);

	// setup for test
	md25.setSpeedRegulation(false);
	md25.setTimeout(true);
	float dt = 0.1;
	float speed = 0.2;
	float t = 0;

	// motor 1 test
	printf("md25 test: spinning motor 1 forward for 1 rev at 0.1 speed\n");
	t = 0;

	while (true) {
		t += dt;
		md25.setMotor1Speed(speed);
		md25.readData();
		usleep(1000000 * dt);

		if (md25.getRevolutions1() > 1) {
			printf("finished 1 revolution fwd\n");
			break;
		}

		if (t > 2.0f) { break; }
	}

	md25.setMotor1Speed(0);
	printf("revolution of wheel 1: %8.4f\n", double(md25.getRevolutions1()));
	md25.resetEncoders();

	t = 0;

	while (true) {
		t += dt;
		md25.setMotor1Speed(-speed);
		md25.readData();
		usleep(1000000 * dt);

		if (md25.getRevolutions1() < -1) {
			printf("finished 1 revolution rev\n");
			break;
		}

		if (t > 2.0f) { break; }
	}

	md25.setMotor1Speed(0);
	printf("revolution of wheel 1: %8.4f\n", double(md25.getRevolutions1()));
	md25.resetEncoders();

	// motor 2 test
	printf("md25 test: spinning motor 2 forward for 1 rev at 0.1 speed\n");
	t = 0;

	while (true) {
		t += dt;
		md25.setMotor2Speed(speed);
		md25.readData();
		usleep(1000000 * dt);

		if (md25.getRevolutions2() > 1) {
			printf("finished 1 revolution fwd\n");
			break;
		}

		if (t > 2.0f) { break; }
	}

	md25.setMotor2Speed(0);
	printf("revolution of wheel 2: %8.4f\n", double(md25.getRevolutions2()));
	md25.resetEncoders();

	t = 0;

	while (true) {
		t += dt;
		md25.setMotor2Speed(-speed);
		md25.readData();
		usleep(1000000 * dt);

		if (md25.getRevolutions2() < -1) {
			printf("finished 1 revolution rev\n");
			break;
		}

		if (t > 2.0f) { break; }
	}

	md25.setMotor2Speed(0);
	printf("revolution of wheel 2: %8.4f\n", double(md25.getRevolutions2()));
	md25.resetEncoders();

	printf("Test complete\n");
	return 0;
}

int md25Sine(const char *deviceName, uint8_t bus, uint8_t address, float amplitude, float frequency)
{
	printf("md25 sine: starting\n");

	// setup
	MD25 md25("/dev/md25", bus, address);

	// print status
	char buf[400];
	md25.status(buf, sizeof(buf));
	printf("%s\n", buf);

	// setup for test
	md25.setSpeedRegulation(false);
	md25.setTimeout(true);
	float dt = 0.01;
	float t_final = 60.0;
	float prev_revolution = md25.getRevolutions1();

	// debug publication
	uORB::Publication<debug_key_value_s> debug_msg(NULL,
			ORB_ID(debug_key_value));

	// sine wave for motor 1
	md25.resetEncoders();

	while (true) {

		// input
		uint64_t timestamp = hrt_absolute_time();
		float t = timestamp / 1000000.0f;

		float input_value = amplitude * sinf(2 * M_PI * frequency * t);
		md25.setMotor1Speed(input_value);

		// output
		md25.readData();
		float current_revolution = md25.getRevolutions1();

		// send input message
		//strncpy(debug_msg.key, "md25 in   ", 10);
		//debug_msg.timestamp_ms = 1000*timestamp;
		//debug_msg.value = input_value;
		//debug_msg.update();

		// send output message
		strncpy(debug_msg.key, "md25 out  ", 10);
		debug_msg.timestamp_ms = 1000 * timestamp;
		debug_msg.value = current_revolution;
		debug_msg.update();

		if (t > t_final) { break; }

		// update for next step
		prev_revolution = current_revolution;

		// sleep
		usleep(1000000 * dt);
	}

	md25.setMotor1Speed(0);

	printf("md25 sine complete\n");
	return 0;
}

// vi:noet:smarttab:autoindent:ts=4:sw=4:tw=78
