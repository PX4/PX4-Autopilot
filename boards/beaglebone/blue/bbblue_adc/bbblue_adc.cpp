/****************************************************************************
 *
 *   Copyright (c) 2012-2018 PX4 Development Team. All rights reserved.
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
 * @file bbblue_adc.cpp
 *
 * BBBlue ADC Driver
 *
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

#ifdef __DF_BBBLUE
#include <robotcontrol.h>
#include <board_config.h>
#endif

#define ADC_BASE_DEV_PATH "/dev/null"

#define BBBLUE_MAX_ADC_CHANNELS 	            (6)
#define BBBLUE_MAX_ADC_USER_CHANNELS 	        (4)
#define BBBLUE_ADC_DC_JACK_VOLTAGE_CHANNEL 		(5)
#define BBBLUE_ADC_LIPO_BATTERY_VOLTAGE_CHANNEL (6)

__BEGIN_DECLS
__EXPORT int bbblue_adc_main(int argc, char *argv[]);
__END_DECLS

class BBBlueADC: public DriverFramework::VirtDevObj
{
public:
	BBBlueADC();
	virtual ~BBBlueADC();

	virtual int init();

	virtual ssize_t devRead(void *buf, size_t count) override;
	virtual int devIOCTL(unsigned long request, unsigned long arg) override;

protected:
	virtual void _measure() override;

private:
	pthread_mutex_t _samples_lock;
	px4_adc_msg_t   _samples[BBBLUE_MAX_ADC_CHANNELS];
};

BBBlueADC::BBBlueADC()
	: DriverFramework::VirtDevObj("bbblue_adc", ADC0_DEVICE_PATH, ADC_BASE_DEV_PATH, 1e6 / 100)
{
	pthread_mutex_init(&_samples_lock, NULL);
}

BBBlueADC::~BBBlueADC()
{
	pthread_mutex_destroy(&_samples_lock);

	rc_cleaning();
}

void BBBlueADC::_measure()
{
#ifdef __DF_BBBLUE
	px4_adc_msg_t tmp_samples[BBBLUE_MAX_ADC_CHANNELS];

	for (int i = 0; i < BBBLUE_MAX_ADC_USER_CHANNELS; ++i) {
		tmp_samples[i].am_channel = i;
		tmp_samples[i].am_data = rc_adc_read_raw(i);
	}

	tmp_samples[BBBLUE_MAX_ADC_USER_CHANNELS].am_channel = BBBLUE_ADC_LIPO_BATTERY_VOLTAGE_CHANNEL;
	tmp_samples[BBBLUE_MAX_ADC_USER_CHANNELS].am_data    = rc_adc_read_raw(BBBLUE_ADC_LIPO_BATTERY_VOLTAGE_CHANNEL);

	tmp_samples[BBBLUE_ADC_DC_JACK_VOLTAGE_CHANNEL].am_channel = BBBLUE_ADC_DC_JACK_VOLTAGE_CHANNEL;
	tmp_samples[BBBLUE_ADC_DC_JACK_VOLTAGE_CHANNEL].am_data    = rc_adc_read_raw(BBBLUE_ADC_DC_JACK_VOLTAGE_CHANNEL);

	pthread_mutex_lock(&_samples_lock);
	memcpy(&_samples, &tmp_samples, sizeof(tmp_samples));
	pthread_mutex_unlock(&_samples_lock);
#endif
}

int BBBlueADC::init()
{
	rc_init();

	int ret = DriverFramework::VirtDevObj::init();

	if (ret != PX4_OK) {
		PX4_ERR("init failed");
		return ret;
	}

	_measure(); // start the initial conversion so that the test command right
	// after the start command can return values
	return PX4_OK;
}

int BBBlueADC::devIOCTL(unsigned long request, unsigned long arg)
{
	return -ENOTTY;
}

ssize_t BBBlueADC::devRead(void *buf, size_t count)
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

static BBBlueADC *instance = nullptr;

int bbblue_adc_main(int argc, char *argv[])
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

		instance = new BBBlueADC;

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

		PX4_INFO("BBBlueADC started");
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

		px4_adc_msg_t adc_msgs[BBBLUE_MAX_ADC_CHANNELS];

		ret = instance->devRead((char *)&adc_msgs, sizeof(adc_msgs));

		if (ret < 0) {
			PX4_ERR("ret: %s (%d)\n", strerror(ret), ret);
			return ret;

		} else if (ret != sizeof(adc_msgs)) {
			PX4_ERR("incomplete read: %d expected %d", ret, sizeof(adc_msgs));
			return ret;
		}

		for (int i = 0; i < BBBLUE_MAX_ADC_USER_CHANNELS; ++i) {
			PX4_INFO("ADC channel: %d; value: %d", (int)adc_msgs[i].am_channel,
				 adc_msgs[i].am_data);
		}

		for (int i = BBBLUE_MAX_ADC_USER_CHANNELS; i < BBBLUE_MAX_ADC_CHANNELS; ++i) {
			PX4_INFO("ADC channel: %d; value: %d, voltage: %6.2f V", (int)adc_msgs[i].am_channel,
				 adc_msgs[i].am_data, adc_msgs[i].am_data * 1.8 / 4095.0 * 11.0);

		}

		return PX4_OK;

	} else {
		PX4_WARN("action (%s) not supported", argv[1]);

		return PX4_ERROR;
	}

	return PX4_OK;

}


