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
 * OcPoC ADC Driver
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
#define ADC_VOLTAGE_PATH "/sys/bus/iio/devices/iio:device0/in_voltage8_raw"

using namespace time_literals;

class OcpocADC : public cdev::CDev, public px4::ScheduledWorkItem
{
public:
	OcpocADC();
	virtual ~OcpocADC();

	int		init() override;

	ssize_t		read(cdev::file_t *filp, char *buffer, size_t len) override;

protected:
	int		open_first(cdev::file_t *filp) override;
	int		close_last(cdev::file_t *filp) override;

private:
	void			Run() override;

	void			update_adc_report(hrt_abstime now);

	void _measure();
	int read(px4_adc_msg_t(*buf)[PX4_MAX_ADC_CHANNELS], unsigned int len);

	static const hrt_abstime	kINTERVAL{10_ms};	/**< 100Hz base rate */
	perf_counter_t			_sample_perf;
	uORB::Publication<adc_report_s>		_to_adc_report{ORB_ID(adc_report)};
	px4_adc_msg_t _samples;
};

OcpocADC::OcpocADC() :
	CDev(ADC0_DEVICE_PATH),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": samples"))
{
}

OcpocADC::~OcpocADC()
{
}

int
OcpocADC::init()
{
	int ret = CDev::init();

	if (ret != PX4_OK) {
		PX4_ERR("init failed");
		return ret;
	}

	return PX4_OK;
}

ssize_t
OcpocADC::read(cdev::file_t *filp, char *buffer, size_t len)
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
OcpocADC::open_first(cdev::file_t *filp)
{
	/* get fresh data */
	Run();

	/* and schedule regular updates */
	ScheduleOnInterval(kINTERVAL, kINTERVAL);

	return 0;
}

int
OcpocADC::close_last(cdev::file_t *filp)
{
	ScheduleClear();

	return 0;
}

void
OcpocADC::Run()
{
	hrt_abstime now = hrt_absolute_time();
	_measure();
	update_adc_report(now);
}

void
OcpocADC::update_adc_report(hrt_abstime now)
{
	adc_report_s adc{};
	adc.timestamp = now;

	adc.channel_id[0] = _samples.am_channel;
	adc.channel_value[0] = _samples.am_data * 3.3f / 4096.0f;

	_to_adc_report.publish(adc);
}

int
OcpocADC::read(px4_adc_msg_t(*buf)[PX4_MAX_ADC_CHANNELS], unsigned int len)
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

void
OcpocADC::_measure()
{
	lock();
	px4_adc_msg_t tmp_samples[PX4_MAX_ADC_CHANNELS] {};

	int ret = read(&tmp_samples, sizeof(tmp_samples));

	if (ret != 0) {
		PX4_ERR("ocpoc_adc_read: %d", ret);
	}

	memcpy(&_samples, &tmp_samples, sizeof(tmp_samples));
	unlock();
}

static OcpocADC *instance = nullptr;

extern "C" __EXPORT int ocpoc_adc_main(int argc, char *argv[])
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

	} else {
		PX4_WARN("action (%s) not supported", argv[1]);

		return PX4_ERROR;
	}

	return PX4_OK;

}
// This is a replacement for the hardcoded 4096
uint32_t px4_arch_adc_dn_fullcount(void)
{
	return 1 << 12; // 12 bit ADC
}
