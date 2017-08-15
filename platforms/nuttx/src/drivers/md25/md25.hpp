/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
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

#pragma once

#include <poll.h>
#include <stdio.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_controls.h>
#include <drivers/device/i2c.h>

/**
 * This is a driver for the MD25 motor controller utilizing the I2C interface.
 */
class MD25 : public device::I2C
{
public:

	/**
	* modes
	*
	* NOTE: this driver assumes we are always
	* in mode 0!
	*
	* seprate speed mode:
	*  motor speed1 = speed1
	*  motor speed2 = speed2
	* turn speed mode:
	*	motor speed1 = speed1 + speed2
	*  motor speed2 = speed2 - speed2
	* unsigned:
	*  full rev (0), stop(128), full fwd (255)
	* signed:
	*  full rev (-127), stop(0), full fwd (128)
	*
	* modes numbers:
	* 0 : unsigned separate (default)
	* 1 : signed separate
	* 2 : unsigned turn
	* 3 : signed turn
	*/
	enum e_mode {
		MODE_UNSIGNED_SPEED = 0,
		MODE_SIGNED_SPEED,
		MODE_UNSIGNED_SPEED_TURN,
		MODE_SIGNED_SPEED_TURN,
	};

	/** commands */
	enum e_cmd {
		CMD_RESET_ENCODERS = 32,
		CMD_DISABLE_SPEED_REGULATION = 48,
		CMD_ENABLE_SPEED_REGULATION = 49,
		CMD_DISABLE_TIMEOUT = 50,
		CMD_ENABLE_TIMEOUT = 51,
		CMD_CHANGE_I2C_SEQ_0 = 160,
		CMD_CHANGE_I2C_SEQ_1 = 170,
		CMD_CHANGE_I2C_SEQ_2 = 165,
	};

	/** control channels */
	enum e_channels {
		CH_SPEED_LEFT = 0,
		CH_SPEED_RIGHT
	};

	/**
	 * constructor
	 * @param deviceName the name of the device e.g. "/dev/md25"
	 * @param bus the I2C bus
	 * @param address the adddress on the I2C bus
	 * @param speed the speed of the I2C communication
	 */
	MD25(const char *deviceName,
	     int bus,
	     uint16_t address,
	     uint32_t speed = 100000);

	/**
	 * deconstructor
	 */
	virtual ~MD25();

	/**
	 * @return software version
	 */
	uint8_t getVersion();

	/**
	 * @return speed of motor, normalized (-1, 1)
	 */
	float getMotor1Speed();

	/**
	 * @return speed of motor 2, normalized (-1, 1)
	 */
	float getMotor2Speed();

	/**
	 * @return number of rotations since reset
	 */
	float getRevolutions1();

	/**
	 * @return number of rotations since reset
	 */
	float getRevolutions2();

	/**
	 * @return battery voltage, volts
	 */
	float getBatteryVolts();

	/**
	 * @return motor 1 current, amps
	 */
	float getMotor1Current();

	/**
	 * @return motor 2 current, amps
	 */
	float getMotor2Current();

	/**
	 * @return the motor acceleration
	 * controls motor speed change (1-10)
	 * accel rate  | time for full fwd. to full rev.
	 *  1          | 6.375 s
	 *  2          | 1.6 s
	 *  3          | 0.675 s
	 *  5(default) | 1.275 s
	 * 10          | 0.65 s
	 */
	uint8_t getMotorAccel();

	/**
	 * @return motor output mode
	 * */
	e_mode getMode();

	/**
	 * @return current command register value
	 */
	e_cmd getCommand();

	/**
	 * resets the encoders
	 * @return non-zero -> error
	 * */
	int resetEncoders();

	/**
	 * enable/disable speed regulation
	 * @return non-zero -> error
	 */
	int setSpeedRegulation(bool enable);

	/**
	 * set the timeout for the motors
	 * enable/disable timeout (motor stop)
	 * after 2 sec of no i2c messages
	 * @return non-zero -> error
	 */
	int setTimeout(bool enable);

	/**
	 * sets the device address
	 * can only be done with one MD25
	 * on the bus
	 * @return non-zero -> error
	 */
	int setDeviceAddress(uint8_t address);

	/**
	 * set motor acceleration
	 * @param accel
	 * controls motor speed change (1-10)
	 * accel rate  | time for full fwd. to full rev.
	 *  1          | 6.375 s
	 *  2          | 1.6 s
	 *  3          | 0.675 s
	 *  5(default) | 1.275 s
	 * 10          | 0.65 s
	 */
	int setMotorAccel(uint8_t accel);

	/**
	 * set motor 1 speed
	 * @param normSpeed normalize speed between -1 and 1
	 * @return non-zero -> error
	*/
	int setMotor1Speed(float normSpeed);

	/**
	 * set motor 2 speed
	 * @param normSpeed normalize speed between -1 and 1
	 * @return non-zero -> error
	 */
	int setMotor2Speed(float normSpeed);

	/**
	 * main update loop that updates MD25 motor
	 * speeds based on actuator publication
	 */
	void update();

	/**
	 * probe for device
	 */
	virtual int probe();

	/**
	 * search for device
	 */
	int search();

	/**
	 * read data from i2c
	 */
	int readData();

	/**
	 * print status
	 */
	void status(char *string, size_t n);

private:
	/** poll structure for control packets */
	struct pollfd _controlPoll;

	/** actuator controls subscription */
	uORB::Subscription<actuator_controls_s> _actuators;

	// local copy of data from i2c device
	uint8_t _version;
	float _motor1Speed;
	float _motor2Speed;
	float _revolutions1;
	float _revolutions2;
	float _batteryVoltage;
	float _motor1Current;
	float _motor2Current;
	uint8_t _motorAccel;
	e_mode _mode;
	e_cmd _command;

	// private methods
	int _writeUint8(uint8_t reg, uint8_t value);
	int _writeInt8(uint8_t reg, int8_t value);
	float _uint8ToNorm(uint8_t value);
	uint8_t _normToUint8(float value);

	/**
	 * set motor control mode,
	 * this driver assumed we are always in mode 0
	 * so we don't let the user change the mode
	 * @return non-zero -> error
	 */
	int _setMode(e_mode);
};

// unit testing
int md25Test(const char *deviceName, uint8_t bus, uint8_t address);

// sine testing
int md25Sine(const char *deviceName, uint8_t bus, uint8_t address, float amplitude, float frequency);

// vi:noet:smarttab:autoindent:ts=4:sw=4:tw=78
