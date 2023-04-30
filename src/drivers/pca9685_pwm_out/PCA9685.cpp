/****************************************************************************
 *
 *   Copyright (c) 2019-2022 PX4 Development Team. All rights reserved.
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
#include "PCA9685.h"


#ifdef CONFIG_PCA9685_USE_EXTERNAL_CRYSTAL
#define PCA9685_CLOCK_REFERENCE CONFIG_PCA9685_EXTERNAL_CRYSTAL_FREQ
#else
#define PCA9685_CLOCK_REFERENCE CONFIG_PCA9685_INTERNAL_CRYSTAL_FREQ
#endif

#define PCA9685_DEFAULT_MODE1_CFG PCA9685_MODE1_AI_MASK
#define PCA9685_DEFAULT_MODE2_CFG PCA9685_MODE2_OUTDRV_MASK

using namespace drv_pca9685_pwm;

PCA9685::PCA9685(int bus, int addr):
	I2C(DRV_PWM_DEVTYPE_PCA9685, MODULE_NAME, bus, addr, 400000),
	currentFreq(0.0)
{

}

int PCA9685::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) { return ret; }

	uint8_t buf[2] = {};

	buf[0] = PCA9685_REG_MODE1;
	buf[1] = PCA9685_DEFAULT_MODE1_CFG | PCA9685_MODE1_SLEEP_MASK;  // put into sleep mode
	ret = transfer(buf, 2, nullptr, 0);

	if (OK != ret) {
		PX4_ERR("init: i2c::transfer returned %d", ret);
		return ret;
	}

#ifdef CONFIG_PCA9685_USE_EXTERNAL_CRYSTAL
	buf[1] = PCA9685_DEFAULT_MODE1_CFG | PCA9685_MODE1_SLEEP_MASK | PCA9685_MODE1_EXTCLK_MASK;
	ret = transfer(buf, 2, nullptr, 0); // enable EXTCLK if possible

	if (OK != ret) {
		PX4_ERR("init: i2c::transfer returned %d", ret);
		return ret;
	}

#endif

	buf[0] = PCA9685_REG_MODE2;
	buf[1] = PCA9685_DEFAULT_MODE2_CFG;
	ret = transfer(buf, 2, nullptr, 0);

	if (OK != ret) {
		PX4_ERR("init: i2c::transfer returned %d", ret);
		return ret;
	}

	return PX4_OK;
}

int PCA9685::updatePWM(const uint16_t *outputs, unsigned num_outputs)
{
	if (num_outputs > PCA9685_PWM_CHANNEL_COUNT) {
		num_outputs = PCA9685_PWM_CHANNEL_COUNT;
		PX4_DEBUG("PCA9685 can only drive up to 16 channels");
	}

	uint16_t out[PCA9685_PWM_CHANNEL_COUNT];
	memcpy(out, outputs, sizeof(uint16_t) * num_outputs);

	for (unsigned i = 0; i < num_outputs; ++i) {
		out[i] = calcRawFromPulse(out[i]);
	}

	return writePWM(0, out, num_outputs);
}

int PCA9685::updateFreq(float freq)
{
	uint16_t divider = (uint16_t)round((float)PCA9685_CLOCK_REFERENCE / freq / PCA9685_PWM_RES) - 1;

	if (divider > 0x00FF) {
		PX4_ERR("frequency too low");
		return PX4_ERROR;
	}

	if (divider < 0x0003) {
		PX4_ERR("frequency too high");
		return PX4_ERROR;
	}

	currentFreq = (float)PCA9685_CLOCK_REFERENCE / (float)((divider + 1) * 4096);
	PX4_INFO("PCA9685 PWM frequency: target=%.2f real=%.2f", (double)freq, (double)currentFreq);

	return setDivider(divider);
}

int PCA9685::updateRAW(const uint16_t *outputs, unsigned int num_outputs)
{
	return writePWM(0, outputs, num_outputs);
}

int PCA9685::setAllPWM(uint16_t output)
{
	uint16_t val = (uint16_t)roundl((output * currentFreq * PCA9685_PWM_RES / (float)1e6));
	uint8_t buf[] = {
		PCA9685_REG_ALLLED_ON_L,
		0x00, 0x00,
		(uint8_t)(val & (uint8_t)0xFF),
		val != 0 ? (uint8_t)(val >> 8) : (uint8_t)PCA9685_LED_ON_FULL_ON_OFF_MASK
	};
	return transfer(buf, sizeof(buf), nullptr, 0);
}

int PCA9685::sleep()
{
	uint8_t buf[2] = {
		PCA9685_REG_MODE1,
		PCA9685_DEFAULT_MODE1_CFG | PCA9685_MODE1_SLEEP_MASK
	};
	return transfer(buf, 2, nullptr, 0);
}

int PCA9685::wake()
{
	uint8_t buf[2] = {
		PCA9685_REG_MODE1,
		PCA9685_DEFAULT_MODE1_CFG
	};
	return transfer(buf, 2, nullptr, 0);
}

int PCA9685::doRestart()
{
	uint8_t buf[2] = {
		PCA9685_REG_MODE1,
		PCA9685_DEFAULT_MODE1_CFG | PCA9685_MODE1_RESTART_MASK
	};
	return transfer(buf, 2, nullptr, 0);
}

int PCA9685::probe()
{
	int ret = I2C::probe();

	if (ret != PX4_OK) { return ret; }

	uint8_t buf[2] = {0x00};
	return transfer(buf, 2, buf, 1);
}

int PCA9685::writePWM(uint8_t idx, const uint16_t *value, uint8_t num)
{
	uint8_t buf[PCA9685_PWM_CHANNEL_COUNT * PCA9685_REG_LED_INCREMENT + 1] = {};
	buf[0] = PCA9685_REG_LED0 + PCA9685_REG_LED_INCREMENT * idx;

	for (int i = 0; i < num; ++i) {
		buf[1 + i * PCA9685_REG_LED_INCREMENT] = 0x00;

		if (value[i] == 0) {
			buf[2 + i * PCA9685_REG_LED_INCREMENT] = 0x00;
			buf[3 + i * PCA9685_REG_LED_INCREMENT] = 0x00;
			buf[4 + i * PCA9685_REG_LED_INCREMENT] = PCA9685_LED_ON_FULL_ON_OFF_MASK;

		} else if (value[i] == 4096) {
			buf[2 + i * PCA9685_REG_LED_INCREMENT] = PCA9685_LED_ON_FULL_ON_OFF_MASK;
			buf[3 + i * PCA9685_REG_LED_INCREMENT] = 0x00;
			buf[4 + i * PCA9685_REG_LED_INCREMENT] = 0x00;

		} else {
			buf[2 + i * PCA9685_REG_LED_INCREMENT] = 0x00;
			buf[3 + i * PCA9685_REG_LED_INCREMENT] = (uint8_t)(value[i] & 0xFF);
			buf[4 + i * PCA9685_REG_LED_INCREMENT] = (uint8_t)(value[i] >> 8);
		}
	}

	return transfer(buf, num * PCA9685_REG_LED_INCREMENT + 1, nullptr, 0);
}

int PCA9685::setDivider(uint8_t value)
{
	uint8_t buf[2] = {};
	buf[0] = PCA9685_REG_PRE_SCALE;
	buf[1] = value;
	return transfer(buf, 2, nullptr, 0);
}
