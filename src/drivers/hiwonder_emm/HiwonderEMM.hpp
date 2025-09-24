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

/**
 * @file HiwonderEMM.hpp
 *
 * Driver for the Hiwonder 4-channel encoder motor driver over I2C.
 *
 * Product: https://www.hiwonder.com/products/4-channel-encoder-motor-driver
 *
 */

#pragma once

#include <cstdint>
#include <drivers/device/i2c.h>
#include <px4_boardconfig.h>
#include <px4_log.h>
#include <drivers/device/device.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <px4_platform_common/module.h>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/sem.hpp>
#include <px4_defines.h>

static constexpr uint8_t I2C_ADDR = 0x34;                     // I2C address
static constexpr uint8_t ADC_BAT_ADDR = 0;                    // Voltage address
static constexpr uint8_t MOTOR_TYPE_ADDR = 0x14;              // Set the motor type
static constexpr uint8_t MOTOR_ENCODER_POLARITY_ADDR = 21;    // Set the encoder direction polarity
static constexpr uint8_t MOTOR_FIXED_PWM_ADDR = 31;           // Fixed PWM control, open loop
static constexpr uint8_t MOTOR_FIXED_SPEED_ADDR = 51;         // Fixed speed control, closed loop
static constexpr uint8_t MOTOR_ENCODER_TOTAL_ADDR = 60;       // Total pulse value of 4 encoder motors
static constexpr uint8_t MOTOR_TYPE_WITHOUT_ENCODER = 0;      // Motor without encoder
static constexpr uint8_t MOTOR_TYPE_TT = 1; 		      // TT encoder motor
static constexpr uint8_t MOTOR_TYPE_N20 = 2;                  // N20 encoder motor
static constexpr uint8_t MOTOR_TYPE_JGB37_520_12V_110RPM = 3; // JGB37 encoder motor
static constexpr uint8_t I2CBUS = 1;                          // I2C bus number
static constexpr uint8_t CHANNEL_COUNT = 4;                   // Number of output channels

class HiwonderEMM : public ModuleBase<HiwonderEMM>, public OutputModuleInterface, public device::I2C
{
public:
	HiwonderEMM();
	~HiwonderEMM() override = default;

	// I2C
	int init() override;

	// ModuleBase
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	int print_status() override;

	// OutputModuleInterface
	bool updateOutputs(uint16_t *outputs, unsigned num_outputs,
			   unsigned num_control_groups_updated) override;

protected:
	int probe() override;

private:
	MixingOutput _mixing_output {
		"EMM",
		CHANNEL_COUNT,
		*this,
		MixingOutput::SchedulingPolicy::Auto,
		false
	};
	void Run() override;

	/**
	 * @brief Read the input voltage supplied to the motor driver.
	 * @param adc_value [mV] Reference to store the ADC value.
	 * @return OK if the transfer was successful, -errno otherwise.
	 */
	int read_adc(int &adc_value);

	/**
	 * @brief Read the encoder counts for all motors.
	 * @param encoder_counts Array to store the encoder counts for each motor.
	 * @param count Number of encoder counts to read (should be equal to CHANNEL_COUNT).
	 * @return OK if the transfer was successful, -errno otherwise.
	 */
	int read_encoder_counts(int32_t *encoder_counts, const uint8_t count);

	/**
	 * @brief Set the speed values for the motors.
	 * @param speed_values Array of speed values for each motor in the range [0, 255].
	 *                     128 represents stop, values below 128 represent reverse motion,
	 *                     and values above 128 represent forward motion.
	 * @param count Number of speed values provided in the array (should be equal to CHANNEL_COUNT).
	 * @return OK if the transfer was successful, -errno otherwise.
	 */
	int set_motor_speed(const uint8_t *speed_values, const uint8_t count);
};
