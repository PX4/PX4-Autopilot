/****************************************************************************
 *
 *   Copyright (C) 2012-2019 PX4 Development Team. All rights reserved.
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
 *
 * Navio2 ADC Driver
 *
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <drivers/drv_adc.h>
#include <cdev/CDev.hpp>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/adc_report.h>

#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <stdlib.h>

#define ADC_BASE_DEV_PATH "/dev/adc"
#define ADC_SYSFS_PATH "/sys/kernel/rcio/adc/ch0"
#define ADC_MAX_CHAN 6

/*
 * ADC Channels:
 * A0 - Board voltage (5V)
 * A1 - servo rail voltage
 * A2 - power module voltage (ADC0, POWER port)
 * A3 - power module current (ADC1, POWER port)
 * A4 - ADC2 (ADC port)
 * A5 - ADC3 (ADC port)
 */

#define NAVIO_ADC_BATTERY_VOLTAGE_CHANNEL (2)
#define NAVIO_ADC_BATTERY_CURRENT_CHANNEL (3)

using namespace time_literals;

class NavioADC : public cdev::CDev, public px4::ScheduledWorkItem
{
public:
	NavioADC();
	virtual ~NavioADC();

	int		init() override;

	ssize_t		read(cdev::file_t *filp, char *buffer, size_t len) override;

protected:
	int		open_first(cdev::file_t *filp) override;
	int		close_last(cdev::file_t *filp) override;

private:
	void			Run() override;

	void			update_adc_report(hrt_abstime now);

	void _measure();
	int read_channel(px4_adc_msg_t *adc_msg, int channel);

	static const hrt_abstime	kINTERVAL{10_ms};	/**< 100Hz base rate */
	perf_counter_t			_sample_perf;
	uORB::Publication<adc_report_s>		_to_adc_report{ORB_ID(adc_report)};
	int _fd[ADC_MAX_CHAN];
	px4_adc_msg_t _samples[ADC_MAX_CHAN] {};
};

NavioADC::NavioADC() :
	CDev(ADC0_DEVICE_PATH),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": samples"))
{
	for (int i = 0; i < ADC_MAX_CHAN; i++) {
		_fd[i] = -1;
	}
}

NavioADC::~NavioADC()
{
	for (int i = 0; i < ADC_MAX_CHAN; i++) {
		if (_fd[i] != -1) {
			::close(_fd[i]);
		}
	}
}

int
NavioADC::init()
{
	int ret = CDev::init();

	if (ret != PX4_OK) {
		PX4_ERR("init failed");
		return ret;
	}

	for (int i = 0; i < ADC_MAX_CHAN; i++) {
		char channel_path[sizeof(ADC_SYSFS_PATH)] {};
		strncpy(channel_path, ADC_SYSFS_PATH, sizeof(ADC_SYSFS_PATH));
		channel_path[sizeof(ADC_SYSFS_PATH) - 2] += i;

		_fd[i] = ::open(channel_path, O_RDONLY);

		if (_fd[i] == -1) {
			int err = errno;
			ret = -1;
			PX4_ERR("init: open: %s (%d)", strerror(err), err);
			goto cleanup;
		}
	}

	return PX4_OK;

cleanup:

	for (int i = 0; i < ADC_MAX_CHAN; i++) {
		if (_fd[i] != -1) {
			::close(_fd[i]);
		}
	}

	return ret;
}

ssize_t
NavioADC::read(cdev::file_t *filp, char *buffer, size_t len)
{
	lock();
	const size_t maxsize = sizeof(_samples);

	if (len > maxsize) {
		len = maxsize;
	}

	memcpy(buffer, &_samples, len);
	unlock();

	return len;
}

int
NavioADC::open_first(cdev::file_t *filp)
{
	/* get fresh data */
	Run();

	/* and schedule regular updates */
	ScheduleOnInterval(kINTERVAL, kINTERVAL);

	return 0;
}

int
NavioADC::close_last(cdev::file_t *filp)
{
	ScheduleClear();

	return 0;
}

void
NavioADC::Run()
{
	hrt_abstime now = hrt_absolute_time();
	_measure();
	update_adc_report(now);
}

void
NavioADC::update_adc_report(hrt_abstime now)
{
	adc_report_s adc{};
	adc.timestamp = now;

	unsigned max_num = ADC_MAX_CHAN;

	if (max_num > (sizeof(adc.channel_id) / sizeof(adc.channel_id[0]))) {
		max_num = (sizeof(adc.channel_id) / sizeof(adc.channel_id[0]));
	}

	for (unsigned i = 0; i < max_num; i++) {
		adc.channel_id[i] = _samples[i].am_channel;
		adc.channel_value[i] = _samples[i].am_data * 3.3f / 4096.0f;
	}

	_to_adc_report.publish(adc);
}

void
NavioADC::_measure()
{
	px4_adc_msg_t tmp_samples[ADC_MAX_CHAN] {};

	for (int i = 0; i < ADC_MAX_CHAN; ++i) {
		int ret;

		/* Currently we only use these channels */
		if (i != NAVIO_ADC_BATTERY_VOLTAGE_CHANNEL &&
		    i != NAVIO_ADC_BATTERY_CURRENT_CHANNEL) {
			tmp_samples[i].am_channel = i;
			tmp_samples[i].am_data = 0;

			continue;
		}

		ret = read_channel(&tmp_samples[i], i);

		if (ret < 0) {
			PX4_ERR("read_channel(%d): %d", i, ret);
			tmp_samples[i].am_channel = i;
			tmp_samples[i].am_data = 0;
		}
	}

	tmp_samples[NAVIO_ADC_BATTERY_VOLTAGE_CHANNEL].am_channel = ADC_BATTERY_VOLTAGE_CHANNEL;
	tmp_samples[NAVIO_ADC_BATTERY_CURRENT_CHANNEL].am_channel = ADC_BATTERY_CURRENT_CHANNEL;

	lock();
	memcpy(&_samples, &tmp_samples, sizeof(tmp_samples));
	unlock();
}

int
NavioADC::read_channel(px4_adc_msg_t *adc_msg, int channel)
{
	char buffer[11]; /* 32bit max INT has maximum 10 chars */
	int ret;

	if (channel < 0 || channel >= ADC_MAX_CHAN) {
		return -EINVAL;
	}

	ret = lseek(_fd[channel], 0, SEEK_SET);

	if (ret == -1) {
		ret = errno;
		PX4_ERR("read_channel %d: lseek: %s (%d)", channel, strerror(ret), ret);
		return  ret;
	}

	ret = ::read(_fd[channel], buffer, sizeof(buffer) - 1);

	if (ret == -1) {
		ret = errno;
		PX4_ERR("read_channel %d: read: %s (%d)", channel, strerror(ret), ret);
		return  ret;

	} else if (ret == 0) {
		PX4_ERR("read_channel %d: read empty", channel);
		ret = -EINVAL;
		return ret;
	}

	buffer[ret] = 0;

	adc_msg->am_channel = channel;
	adc_msg->am_data = strtol(buffer, NULL, 10);
	ret = 0;

	return ret;
}

static NavioADC *instance = nullptr;

extern "C" __EXPORT int navio_adc_main(int argc, char *argv[])
{
	if (argc < 2) {
		PX4_WARN("usage: {start|stop}");
		return PX4_ERROR;
	}

	if (!strcmp(argv[1], "start")) {
		if (instance) {
			PX4_WARN("already started");
			return PX4_OK;
		}

		instance = new NavioADC;

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

	} else {
		PX4_WARN("action (%s) not supported", argv[1]);

		return PX4_ERROR;
	}

	return PX4_OK;

}
