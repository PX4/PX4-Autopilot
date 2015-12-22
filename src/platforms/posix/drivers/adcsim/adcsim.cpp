/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file adcsim.cpp
 *
 * Driver for the ADCSIM.
 *
 * This is a designed for simulating sampling things like voltages
 * and so forth.
 */

#include <px4_config.h>
#include <px4_time.h>
#include <px4_adc.h>
#include <board_config.h>
#include <drivers/device/device.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>

#include <arch/board/board.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_adc.h>

#include <systemlib/err.h>
#include <systemlib/perf_counter.h>

#include <uORB/topics/system_power.h>

#include "SyncObj.hpp"
#include "VirtDevObj.hpp"

using namespace DriverFramework;

#define ADC_BASE_DEV_PATH "/dev/adc"

class ADCSIM : public VirtDevObj
{
public:
	ADCSIM(uint32_t channels);
	virtual ~ADCSIM();

	virtual ssize_t		devRead(void *buffer, size_t len);

private:
	perf_counter_t		_sample_perf;

	unsigned		_channel_count;
	adc_msg_s		*_samples;		/**< sample buffer */

	/** worker function */
	virtual void		_measure();

	/**
	 * Sample a single channel and return the measured value.
	 *
	 * @param channel		The channel to sample.
	 * @return			The sampled value, or 0xffff if
	 *				sampling failed.
	 */
	uint16_t		_sample(unsigned channel);

	SyncObj 		m_lock;
};

ADCSIM::ADCSIM(uint32_t channels) :
	VirtDevObj("adcsim", "/dev/adcsim", ADC_BASE_DEV_PATH, 10000),
	_sample_perf(perf_alloc(PC_ELAPSED, "adc_samples")),
	_channel_count(0),
	_samples(nullptr)
{
	//_debug_enabled = true;

	/* always enable the temperature sensor */
	channels |= 1 << 16;

	/* allocate the sample array */
	for (unsigned i = 0; i < 32; i++) {
		if (channels & (1 << i)) {
			_channel_count++;
		}
	}

	_samples = new adc_msg_s[_channel_count];

	/* prefill the channel numbers in the sample array */
	if (_samples != nullptr) {
		unsigned index = 0;

		for (unsigned i = 0; i < 32; i++) {
			if (channels & (1 << i)) {
				_samples[index].am_channel = i;
				_samples[index].am_data = 0;
				index++;
			}
		}
	}
}

ADCSIM::~ADCSIM()
{
	if (_samples != nullptr) {
		delete _samples;
	}
}

ssize_t
ADCSIM::devRead(void *buffer, size_t len)
{
	const size_t maxsize = sizeof(adc_msg_s) * _channel_count;

	if (len > maxsize) {
		len = maxsize;
	}

	/* block interrupts while copying samples to avoid racing with an update */
	m_lock.lock();
	memcpy(buffer, _samples, len);
	m_lock.unlock();

	return len;
}

void
ADCSIM::_measure()
{
	m_lock.lock();

	/* scan the channel set and sample each */
	for (unsigned i = 0; i < _channel_count; i++) {
		_samples[i].am_data = _sample(_samples[i].am_channel);
	}

	m_lock.unlock();
}

uint16_t
ADCSIM::_sample(unsigned channel)
{
	perf_begin(_sample_perf);

	uint16_t result = 1;

	perf_end(_sample_perf);
	return result;
}

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int adcsim_main(int argc, char *argv[]);

namespace
{
ADCSIM	*g_adc;

int
test(void)
{
	DevHandle h;
	DevMgr::getHandle(ADCSIM0_DEVICE_PATH, h);

	if (!h.isValid()) {
		PX4_ERR("can't open ADCSIM device (%d)", h.getError());
		return 1;
	}

	for (unsigned i = 0; i < 50; i++) {
		adc_msg_s data[12];
		ssize_t count = h.read(data, sizeof(data));

		if (count < 0) {
			PX4_ERR("read error (%d)", h.getError());
			return 1;
		}

		unsigned channels = count / sizeof(data[0]);

		for (unsigned j = 0; j < channels; j++) {
			PX4_INFO("%d: %lu  ", data[j].am_channel, (unsigned long)data[j].am_data);
		}

		usleep(500000);
	}

	DevMgr::releaseHandle(h);
	return 0;
}
}

int
adcsim_main(int argc, char *argv[])
{
	int ret = 0;

	if (g_adc == nullptr) {
		/* FIXME - this hardcodes the default channel set for SITL - should be configurable */
		g_adc = new ADCSIM((1 << 10) | (1 << 11) | (1 << 12) | (1 << 13));

		if (g_adc == nullptr) {
			PX4_ERR("couldn't allocate the ADCSIM driver");
			return 1;
		}

		ret = g_adc->init();

		if (ret != 0) {
			PX4_ERR("ADCSIM init failed (%d)", ret);
			return 1;
		}
	}

	if (argc > 1 && g_adc) {
		if (!strcmp(argv[1], "test")) {
			ret = test();
		}
	}

	return ret;
}
