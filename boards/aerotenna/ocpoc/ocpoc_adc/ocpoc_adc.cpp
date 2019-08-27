/****************************************************************************
 *
 *   Copyright (c) 2012-2017 PX4 Development Team. All rights reserved.
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
 * @file ocpoc_adc.cpp
 *
 * OcPoC ADC Driver
 *
 * @author Lianying Ji <ji@aerotenna.com>
 * @author Dave Royer <dave@aerotenna.com>
 */

#include <px4_platform_common/config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <drivers/drv_adc.h>

#include <VirtDevObj.hpp>

#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>

#define ADC_BASE_DEV_PATH "/dev/adc"
#define ADC_VOLTAGE_PATH "/sys/bus/iio/devices/iio:device0/in_voltage8_raw"

__BEGIN_DECLS
__EXPORT int ocpoc_adc_main(int argc, char *argv[]);
__END_DECLS

class OcpocADC: public DriverFramework::VirtDevObj
{
public:
	OcpocADC();
	virtual ~OcpocADC();

	virtual int init();

	virtual ssize_t devRead(void *buf, size_t count) override;
	virtual int devIOCTL(unsigned long request, unsigned long arg) override;

protected:
	virtual void _measure() override;

private:
	int read(px4_adc_msg_t(*buf)[PX4_MAX_ADC_CHANNELS], unsigned int len);

	pthread_mutex_t _samples_lock;
	px4_adc_msg_t _samples;
};

OcpocADC::OcpocADC()
	: DriverFramework::VirtDevObj("ocpoc_adc", ADC0_DEVICE_PATH, ADC_BASE_DEV_PATH, 1e6 / 100)
{
	pthread_mutex_init(&_samples_lock, NULL);
}

OcpocADC::~OcpocADC()
{
	pthread_mutex_destroy(&_samples_lock);
}

void OcpocADC::_measure()
{
	px4_adc_msg_t tmp_samples[PX4_MAX_ADC_CHANNELS];

	int ret = read(&tmp_samples, sizeof(tmp_samples));

	if (ret != 0) {
		PX4_ERR("ocpoc_adc_read: %d", ret);
	}

	pthread_mutex_lock(&_samples_lock);
	memcpy(&_samples, &tmp_samples, sizeof(tmp_samples));
	pthread_mutex_unlock(&_samples_lock);
}

int OcpocADC::init()
{
	int ret;

	ret = DriverFramework::VirtDevObj::init();

	if (ret != PX4_OK) {
		PX4_ERR("init failed");
		return ret;
	}

	return PX4_OK;
}

int OcpocADC::devIOCTL(unsigned long request, unsigned long arg)
{
	return -ENOTTY;
}

ssize_t OcpocADC::devRead(void *buf, size_t count)
{
	const size_t maxsize = sizeof(_samples);
	int ret;

	if (count > maxsize) {
		count = maxsize;
	}

	ret = pthread_mutex_trylock(&_samples_lock);

	if (ret != 0) {
		return 0;
	}

	memcpy(buf, &_samples, count);
	pthread_mutex_unlock(&_samples_lock);

	return count;
}

int OcpocADC::read(px4_adc_msg_t(*buf)[PX4_MAX_ADC_CHANNELS], unsigned int len)
{
	uint32_t buff[1];
	int ret = 0;

	FILE *xadc_fd = fopen(ADC_VOLTAGE_PATH, "r");

	if (xadc_fd != NULL) {
		int ret_tmp = fscanf(xadc_fd, "%d", buff);

		if (ret_tmp < 0) {
			ret = ret_tmp;
		}

		fclose(xadc_fd);

		(*buf)[0].am_data = buff[0];

	} else {
		(*buf)[0].am_data = 0;
		ret = -1;
	}

	(*buf)[0].am_channel = ADC_BATTERY_VOLTAGE_CHANNEL;

	return ret;
}

static OcpocADC *instance = nullptr;

int ocpoc_adc_main(int argc, char *argv[])
{
	int ret;

	if (argc < 2) {
		PX4_WARN("usage: {start|stop|test}");
		return PX4_ERROR;
	}

	if (!strcmp(argv[1], "start")) {
		if (instance) {
			PX4_WARN("already started");
			return PX4_OK;
		}

		instance = new OcpocADC;

		if (!instance) {
			PX4_WARN("not enough memory");
			return PX4_ERROR;
		}

		if (instance->init() != PX4_OK) {
			delete instance;
			instance = nullptr;
			PX4_WARN("init failed");
			return PX4_ERROR;
		}

		return PX4_OK;

	} else if (!strcmp(argv[1], "stop")) {
		if (!instance) {
			PX4_WARN("already stopped");
			return PX4_OK;
		}

		delete instance;
		instance = nullptr;
		return PX4_OK;

	} else if (!strcmp(argv[1], "test")) {
		if (!instance) {
			PX4_ERR("start first");
			return PX4_ERROR;
		}

		px4_adc_msg_t adc_msgs[PX4_MAX_ADC_CHANNELS];

		ret = instance->devRead((char *)&adc_msgs, sizeof(adc_msgs));

		if (ret < 0) {
			PX4_ERR("ret: %s (%d)\n", strerror(ret), ret);
			return ret;

		} else {
			PX4_INFO("ADC Data: %d", adc_msgs[0].am_data);
		}

		return PX4_OK;

	} else {
		PX4_WARN("action (%s) not supported", argv[1]);

		return PX4_ERROR;
	}

	return PX4_OK;

}
