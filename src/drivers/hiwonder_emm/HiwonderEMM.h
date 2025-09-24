/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#pragma once
#include <cstdint>
#include <drivers/device/i2c.h>
#include <px4_boardconfig.h>

#define I2C_ADDR 0x34 // I2C address
#define ADC_BAT_ADDR 0 // Voltage address
#define MOTOR_TYPE_ADDR 0x14 // Set the motor type
#define MOTOR_ENCODER_POLARITY_ADDR 21 // Set the encoder direction polarity
#define MOTOR_FIXED_PWM_ADDR 31 // Fixed PWM control, open loop
#define MOTOR_FIXED_SPEED_ADDR 51 // Fixed speed control, closed loop
#define MOTOR_ENCODER_TOTAL_ADDR 60 // Total pulse value of 4 encoder motors
#define MOTOR_TYPE_WITHOUT_ENCODER 0 // Motor without encoder
#define MOTOR_TYPE_TT 1 // TT encoder motor
#define MOTOR_TYPE_N20 2 // N20 encoder motor
#define MOTOR_TYPE_JGB37_520_12V_110RPM 3 // JGB37 encoder motor
#define BUS_FREQUENCY 100000 // I2C bus frequency
#define I2CBUS 1 // I2C bus number
#define CHANNEL_COUNT 4 // Number of output channels
class HiwonderEMM : public device::I2C
{
public:
	HiwonderEMM(int bus, int addr);
	~HiwonderEMM() override = default;

	int init() override;

	int read_adc(int &adc_value);

	int set_motor_speed(const uint8_t speed_values[4]);

protected:
	int probe() override;

};
