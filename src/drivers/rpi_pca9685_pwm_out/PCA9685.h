/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 *
 * Original Author      : Georgi Todorov
 * Edited by	: Tord Wessman
 * Created on  : Nov 22, 2013
 * Rewrite	   : Fan.zhang  421395590@qq.com
 * Updated on  : Mar 2, 2017
 ****************************************************************************/

#pragma once
#include <inttypes.h>

// Register Definitions
// 寄存器定义
#define MODE1 0x00			//Mode  register  1
#define MODE2 0x01			//Mode  register  2
#define SUBADR1 0x02		//I2C-bus subaddress 1
#define SUBADR2 0x03		//I2C-bus subaddress 2
#define SUBADR3 0x04		//I2C-bus subaddress 3
#define ALLCALLADR 0x05     //LED All Call I2C-bus address
#define LED0 0x6			//LED0 start register
#define LED0_ON_L 0x6		//LED0 output and brightness control byte 0
#define LED0_ON_H 0x7		//LED0 output and brightness control byte 1
#define LED0_OFF_L 0x8		//LED0 output and brightness control byte 2
#define LED0_OFF_H 0x9		//LED0 output and brightness control byte 3
#define LED_MULTIPLYER 4	// For the other 15 channels
#define ALLLED_ON_L 0xFA    //load all the LEDn_ON registers, byte 0 (turn 0-7 channels on)
#define ALLLED_ON_H 0xFB	//load all the LEDn_ON registers, byte 1 (turn 8-15 channels on)
#define ALLLED_OFF_L 0xFC	//load all the LEDn_OFF registers, byte 0 (turn 0-7 channels off)
#define ALLLED_OFF_H 0xFD	//load all the LEDn_OFF registers, byte 1 (turn 8-15 channels off)
#define PRE_SCALE 0xFE		//prescaler for output frequency
#define MAX_PWM_RES 4096        //Resolution 4096=12bit 分辨率，按2的阶乘计算，12bit为4096
#define CLOCK_FREQ 25000000.0 //25MHz default osc clock
#define PCA9685_DEFAULT_I2C_ADDR 0x40  // default i2c address for pca9685 默认i2c地址为0x40
#define PCA9685_DEFAULT_I2C_BUS  1     // default i2c bus for pca9685  默认为1

//! Main class that exports features for PCA9685 chip
class PCA9685
{
public:

	PCA9685();

	/**
	 * Constructor takes bus and address arguments
	 * @param bus the bus to use in /dev/i2c-%d.
	 * @param address the device address on bus
	 */
	PCA9685(int bus, int address);

	void init(int bus, int address);
	~PCA9685();

	/// Sets PCA9685 mode to 00
	void reset();

	/**
	 * Set the frequency of PWM
	 * @param freq desired frequency. 40Hz to 1000Hz using internal 25MHz oscillator.
	 */
	void setPWMFreq(int freq);

	/**
	 * PWM a single channel with custom on time.
	 * send pwm vale to led(channel)
	 * @param channel
	 * @param on_value 0-4095 value to turn on the pulse
	 * @param off_value 0-4095 value to turn off the pulse
	 */
	void setPWM(uint8_t channel, int on_value, int off_value);

	/**
	 * send pwm value to led(channel),
	 * value should be range of 0-4095
	 */
	void setPWM(uint8_t channel, int value);

private:
	int _fd = -1; ///< I2C device file descriptor

	/**
	 * Read a single byte from PCA9685
	 * @param fd file descriptor for I/O
	 * @param address register address to read from
	 * @return read byte
	 */
	uint8_t read_byte(int fd, uint8_t address);

	/**
	 *  Write a single byte to PCA9685
	 * @param fd file descriptor for I/O
	 * @param address register address to write to
	 * @param data 8 bit data to write
	 */
	void write_byte(int fd, uint8_t address, uint8_t data);

	/**
	 * Open device file for PCA9685 I2C bus
	 * @return fd returns the file descriptor number or -1 on error
	 */
	int open_fd(int bus, int address);
};
