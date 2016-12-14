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
#define ADC_ENABLE_REG 0x00
#define ADC_CHANNEL_REG 0x05
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
	adc_msg_s _sample;
	pthread_mutex_t _sample_mutex;
};

static AEROFC_ADC *instance = nullptr;

static void test()
{
	int fd = open(ADC0_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "can't open ADC device");
	}

	adc_msg_s data[MAX_CHANNEL];
	ssize_t count = read(fd, data, sizeof(data));

	if (count < 0) {
		errx(1, "read error");
	}

	unsigned channels = count / sizeof(data[0]);

	for (unsigned j = 0; j < channels; j++) {
		printf("%d: %u  ", data[j].am_channel, data[j].am_data);
	}

	printf("\n");

	exit(0);
}

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

	} else if (!strcmp(argv[1], "test")) {
		test();

	} else {
		warn("Action not supported");
	}

	return PX4_OK;
}

AEROFC_ADC::AEROFC_ADC(uint8_t bus) :
	I2C("AEROFC_ADC", ADC0_DEVICE_PATH, bus, SLAVE_ADDR, 400000),
	_sample{}
{
	_sample.am_channel = 1;
	pthread_mutex_init(&_sample_mutex, NULL);
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
	uint8_t buffer[2];
	int ret;

	_retries = 3;

	/* Enable ADC */
	buffer[0] = ADC_ENABLE_REG;
	buffer[1] = 0x01;
	ret = transfer(buffer, 2, NULL, 0);

	if (ret != PX4_OK) {
		goto error;
	}

	usleep(10000);

	/* Read ADC value */
	buffer[0] = ADC_CHANNEL_REG;
	ret = transfer(buffer, 1, buffer, 2);

	if (ret != PX4_OK) {
		goto error;
	}

	return PX4_OK;

error:
	warn("AEROFC_ADC not found");
	return -EIO;
}

int AEROFC_ADC::ioctl(file *filp, int cmd, unsigned long arg)
{
	return -ENOTTY;
}

ssize_t AEROFC_ADC::read(file *filp, char *buffer, size_t len)
{
	if (len < sizeof(_sample)) {
		return -ENOSPC;
	}

	if (len > sizeof(_sample)) {
		len = sizeof(_sample);
	}

	pthread_mutex_lock(&_sample_mutex);
	memcpy(buffer, &_sample, len);
	pthread_mutex_unlock(&_sample_mutex);

	return len;
}

void AEROFC_ADC::cycle_trampoline(void *arg)
{
	AEROFC_ADC *dev = reinterpret_cast<AEROFC_ADC * >(arg);
	dev->cycle();
}

void AEROFC_ADC::cycle()
{
	uint8_t buffer[2];
	int ret;

	buffer[0] = ADC_CHANNEL_REG;
	ret = transfer(buffer, 1, buffer, sizeof(buffer));

	if (ret != PX4_OK) {
		warn("Error reading sample");
		return;
	}

	pthread_mutex_lock(&_sample_mutex);

	_sample.am_data = (buffer[0] | (buffer[1] << 8));

	pthread_mutex_unlock(&_sample_mutex);

	work_queue(HPWORK, &_work, (worker_t)&AEROFC_ADC::cycle_trampoline, this,
		   CYCLE_TICKS_DELAY);
}
