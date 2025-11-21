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
	return I2C::init();
}

int PCA9685::configure()
{
	uint8_t buf[2] = {};

	buf[0] = PCA9685_REG_MODE1;
	buf[1] = PCA9685_DEFAULT_MODE1_CFG | PCA9685_MODE1_SLEEP_MASK;  // put into sleep mode
	int ret = transfer(buf, 2, nullptr, 0);

#ifdef CONFIG_PCA9685_USE_EXTERNAL_CRYSTAL
	/* EXTCLK is sticky, so writing it once is enough. Its not a problem when its written to 0 later. */
	buf[1] = PCA9685_DEFAULT_MODE1_CFG | PCA9685_MODE1_SLEEP_MASK | PCA9685_MODE1_EXTCLK_MASK;
	ret |= transfer(buf, 2, nullptr, 0);
#endif

	buf[0] = PCA9685_REG_MODE2;
	buf[1] = PCA9685_DEFAULT_MODE2_CFG;
	ret |= transfer(buf, 2, nullptr, 0);

	if (ret != PX4_OK) {
		PX4_ERR("PCA9685 configure fail");
	}

	return ret;
}

void PCA9685::registers_check(bool *transfer_ok, bool *registers_ok)
{
	*transfer_ok = true;
	*registers_ok = true;

	/* Check MODE1 register */
	uint8_t send_buf = PCA9685_REG_MODE1;
	uint8_t recv_buf;

	int ret = transfer(&send_buf, 1, &recv_buf, 1);
	uint8_t ignore_extclk_mask = ~PCA9685_MODE1_EXTCLK_MASK;

	if (ret != PX4_OK) {
		*transfer_ok = false;
		return;
	}

	if ((recv_buf & ignore_extclk_mask) != (PCA9685_DEFAULT_MODE1_CFG & ignore_extclk_mask)) {
		*registers_ok = false;
		return;
	}

	/* Check MODE2 register */
	send_buf = PCA9685_REG_MODE2;
	ret = transfer(&send_buf, 1, &recv_buf, 1);

	if (ret != PX4_OK) {
		*transfer_ok = false;
		return;
	}

	if (recv_buf != PCA9685_DEFAULT_MODE2_CFG) {
		*registers_ok = false;
		return;
	}
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
	uint8_t send_buf[2];
	uint8_t recv_buf;

	send_buf[0] = PCA9685_REG_MODE1;
	int ret = transfer(&send_buf[0], 1, &recv_buf, 1);

	if (ret != PX4_OK) {
		return PX4_ERROR;
	}

	send_buf[1] = recv_buf & ~PCA9685_MODE1_SLEEP_MASK; // Clear sleep bit
	ret |= transfer(&send_buf[0], 2, nullptr, 0);
	px4_usleep(500); // wait for oscillator to stabilize

	if (recv_buf & PCA9685_MODE1_RESTART_MASK) { // Check if reset bit is set
		send_buf[1] |= PCA9685_MODE1_RESTART_MASK; // Set restart bit
		ret |= transfer(&send_buf[0], 2, nullptr, 0);
	}

	ret |= transfer(&send_buf[0], 1, &recv_buf, 1);

	if (ret != PX4_OK || recv_buf & PCA9685_MODE1_RESTART_MASK || recv_buf & PCA9685_MODE1_SLEEP_MASK) {
		PX4_ERR("PCA9685 wake failed: Ret: %d MODE1 REG after wake: %d", ret, recv_buf);
		return PX4_ERROR;
	}

	return ret;
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
