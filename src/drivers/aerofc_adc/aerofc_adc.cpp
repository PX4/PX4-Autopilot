/****************************************************************************
 *
 *   Copyright (C) 2016  Intel Corporation. All rights reserved.
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

#include <px4_config.h>
#include <px4_defines.h>

#include <arch/board/board.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>

#include <board_config.h>

#include <drivers/device/i2c.h>
#include <drivers/drv_adc.h>

#include <systemlib/err.h>

#define SLAVE_ADDR 0x50
#define ENABLE_ADC_REG 0x00
#define ADC_CHANNEL_1_REG 0x01
#define ADC_CHANNEL_2_REG 0x02
#define ADC_CHANNEL_3_REG 0x03
#define ADC_CHANNEL_4_REG 0x04
#define ADC_CHANNEL_5_REG 0x05
#define MAX_CHANNEL 5

// 10Hz
#define CYCLE_TICKS_DELAY MSEC2TICK(100)

extern "C" { __EXPORT int aerofc_adc_main(int argc, char *argv[]); }

class AEROFC_ADC : public device::I2C
{
public:
	AEROFC_ADC(uint8_t bus);
	virtual ~AEROFC_ADC();

	virtual int init();
	virtual int ioctl(file *filp, int cmd, unsigned long arg);
	virtual ssize_t read(file *filp, char *buffer, size_t len);

private:
	virtual int probe();

	void cycle();
	static void cycle_trampoline(void *arg);

	struct work_s _work;
	adc_msg_s _samples[MAX_CHANNEL];
	pthread_mutex_t _samples_mutex;
};

static AEROFC_ADC *instance = nullptr;

int aerofc_adc_main(int argc, char *argv[])
{
	if (argc < 2) {
		warn("Missing action <start>");
		return PX4_OK;
	}

	if (!strcmp(argv[1], "start")) {
		if (instance) {
			warn("AEROFC_ADC was already started");
		}

		instance = new AEROFC_ADC(PX4_I2C_BUS_EXPANSION);

		if (!instance) {
			warn("No memory to instance AEROFC_ADC");
			return PX4_OK;
		}

		if (instance->init() != PX4_OK) {
			delete instance;
			instance = nullptr;
		}

	} else {
		warn("Action not supported");
	}

	return PX4_OK;
}

AEROFC_ADC::AEROFC_ADC(uint8_t bus) :
	I2C("AEROFC_ADC", ADC0_DEVICE_PATH, bus, 0, 400000)
{
	for (unsigned i = 0; i < MAX_CHANNEL; i++) {
		_samples[i].am_channel = i + 1;
	}

	pthread_mutex_init(&_samples_mutex, NULL);
}

AEROFC_ADC::~AEROFC_ADC()
{
	work_cancel(HPWORK, &_work);
}

int AEROFC_ADC::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		return ret;
	}

	work_queue(HPWORK, &_work, (worker_t)&AEROFC_ADC::cycle_trampoline, this,
		   CYCLE_TICKS_DELAY);

	return PX4_OK;
}

int AEROFC_ADC::probe()
{
	uint8_t buffer[1];
	int ret;

	_retries = 3;

	set_address(SLAVE_ADDR | ENABLE_ADC_REG);
	buffer[0] = 1;
	ret = transfer(buffer, sizeof(buffer), NULL, 0);

	if (ret != PX4_OK) {
		goto error;
	}

	buffer[0] = 0;
	ret = transfer(0, 0, buffer, sizeof(buffer));

	if (ret != PX4_OK || buffer[0] != 1) {
		goto error;
	}

	return PX4_OK;

error:
	warn("AEROFC_ADC not found");
	return -EIO;
}

int
AEROFC_ADC::ioctl(file *filp, int cmd, unsigned long arg)
{
	return -ENOTTY;
}

ssize_t
AEROFC_ADC::read(file *filp, char *buffer, size_t len)
{
	if (len > sizeof(_samples)) {
		len = sizeof(_samples);
	}

	pthread_mutex_lock(&_samples_mutex);
	memcpy(buffer, _samples, len);
	pthread_mutex_unlock(&_samples_mutex);

	return len;
}

void
AEROFC_ADC::cycle_trampoline(void *arg)
{
	AEROFC_ADC *dev = reinterpret_cast<AEROFC_ADC * >(arg);
	dev->cycle();
}

void
AEROFC_ADC::cycle()
{
	int32_t value[MAX_CHANNEL];

	for (uint8_t i = 0; i < MAX_CHANNEL; i++) {
		int ret;
		uint8_t buffer[2];

		set_address(SLAVE_ADDR | (ADC_CHANNEL_1_REG + i));
		ret = transfer(0, 0, buffer, sizeof(buffer));

		if (ret != PX4_OK) {
			warn("Error reading channel %u", i + 1);
		}

		value[i] = (int32_t)(buffer[0] | (buffer[1] << 8));
	}

	pthread_mutex_lock(&_samples_mutex);

	for (uint8_t i = 0; i < MAX_CHANNEL; i++) {
		_samples[i].am_data = value[i];
	}

	pthread_mutex_unlock(&_samples_mutex);

	work_queue(HPWORK, &_work, (worker_t)&AEROFC_ADC::cycle_trampoline, this,
		   CYCLE_TICKS_DELAY);
}
