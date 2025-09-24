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

#include <px4_log.h>
#include <px4_defines.h>
#include <cmath>
#include "HiwonderEMM.h"

HiwonderEMM::HiwonderEMM(int bus, int addr):
	I2C(DRV_MOTOR_DEVTYPE_HIWONDER_EMM, MODULE_NAME, bus, addr, 400000)
{

}

int HiwonderEMM::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		PX4_ERR("I2C init failed. Error: %d", ret);
		return ret;
	}

	const uint8_t cmd[2] = {MOTOR_TYPE_ADDR, MOTOR_TYPE_JGB37_520_12V_110RPM};
	ret = transfer(cmd, 2, nullptr, 0);

	if (ret != PX4_OK) {
		PX4_ERR("Failed to set motor type. Error: %d", ret);
		return ret;
	}

	const uint8_t cmd2[2] = {MOTOR_ENCODER_POLARITY_ADDR, 0};
	ret = transfer(cmd2, 2, nullptr, 0);

	if (ret != PX4_OK) {
		PX4_ERR("Failed to set encoder polarity. Error: %d", ret);
		return ret;
	}

	PX4_INFO("Hiwonder EMM initialized");

	return PX4_OK;
}

int HiwonderEMM::probe()
{
	int ret = I2C::probe();

	if (ret != PX4_OK) {
		PX4_ERR("I2C probe failed. Error: %d", ret);
		return ret;
	}

	int adc_value{0};
	ret = read_adc(adc_value);

	if (ret != PX4_OK) {
		PX4_ERR("Failed to probe Hiwonder EMM. Error: %d", ret);
		return ret;
	}

	PX4_INFO("Hiwonder EMM found");

	return PX4_OK;
}

int HiwonderEMM::read_adc(int &adc_value)
{
	const uint8_t cmd = ADC_BAT_ADDR;
	uint8_t buf[2] = {};
	const int ret = transfer(&cmd, 1, buf, 2);

	if (ret != PX4_OK) {
		PX4_ERR("Failed to read ADC. Error: %d", ret);
		adc_value = 0;
		return ret;
	}

	adc_value = buf[0] | (buf[1] << 8);
	return ret;
}

int HiwonderEMM::set_motor_speed(const uint8_t speed_values[4])
{
	uint8_t cmd[5];
	cmd[0] = MOTOR_FIXED_SPEED_ADDR;
	cmd[1] = speed_values[0];
	cmd[2] = speed_values[1];
	cmd[3] = speed_values[2];
	cmd[4] = speed_values[3];
	const int ret = transfer(cmd, sizeof(cmd), nullptr, 0);

	if (ret != PX4_OK) {
		PX4_ERR("Failed to set motor speed. Error: %d", ret);
	}

	return ret;
}
