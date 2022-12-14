/****************************************************************************
 *
 *   Copyright (c) 2015, 2016 Airmind Development Team. All rights reserved.
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
 * 3. Neither the name Airmind nor the names of its contributors may be
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
 * @file test_ppm.cpp
 *
 * @author sudragon
 */

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <perf/perf_counter.h>
#include <systemlib/err.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>

#define TEST_PPM_PIN      (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN15)

class TEST_PPM
{
public:
	TEST_PPM(unsigned channels);
	virtual ~TEST_PPM(void);
	virtual int		init();
	unsigned 		_values[20];
	unsigned 		_gaps[20];
	unsigned 		_channels;
	unsigned 		_plus_width;
protected:

private:
	struct hrt_call		_call;
	unsigned 		_call_times;
	void			start();

	void			stop();
	void 			do_out();
	static void		loops(void *arg);


};


/** driver 'main' command */
extern "C" { __EXPORT int test_ppm_main(int argc, char *argv[]); }

TEST_PPM::TEST_PPM(unsigned channels) :
	_channels(channels),
	_plus_width(400),
	_call{},
	_call_times(0)
{
	memset(&_call, 0, sizeof(_call));

	for (int i = 0; i < 20; i++) {
		_values[i] = 1500;
	}

	_values[0] = 5000;
}

TEST_PPM::~TEST_PPM()
{
	/* make sure we are truly inactive */
	stop();
}

int
TEST_PPM::init()
{
	px4_arch_configgpio(TEST_PPM_PIN);
	start();
	return OK;
}


void
TEST_PPM::start()
{
	stop();
	_call_times = 0;
	hrt_call_after(&_call, 1000, (hrt_callout)&TEST_PPM::loops, this);
}

void
TEST_PPM::stop()
{
	hrt_cancel(&_call);
}

void
TEST_PPM::loops(void *arg)
{
	TEST_PPM *dev = static_cast<TEST_PPM *>(arg);
	dev->do_out();
}
void
TEST_PPM::do_out(void)
{
	if ((_call_times % 2) == 0) {
		px4_arch_gpiowrite(TEST_PPM_PIN, false);
		hrt_call_after(&_call, _values[_call_times / 2] - _plus_width, (hrt_callout)&TEST_PPM::loops, this);

	} else {
		px4_arch_gpiowrite(TEST_PPM_PIN, true);
		hrt_call_after(&_call, _plus_width, (hrt_callout)&TEST_PPM::loops, this);
	}

	if ((_call_times / 2) < _channels + 1) { _call_times++; }

	else { _call_times = 0; }
}

namespace test_ppm
{

TEST_PPM	*g_test = nullptr;

void	start(unsigned channels);
void	stop();
void	usage();
void 	set(unsigned ch, unsigned value);

/**
 * Start the driver.
 *
 * This function only returns if the driver is up and running
 * or failed to detect the sensor.
 */
void
start(unsigned  channels)
{

	if (g_test != nullptr)
		/* if already started, the still command succeeded */
	{
		errx(1, "already started");
	}

	g_test = new TEST_PPM(channels);

	if (g_test == nullptr) {
		goto fail;
	}

	if (OK != g_test->init()) {
		goto fail;
	}

	exit(0);
fail:

	if (g_test != nullptr) {
		delete (g_test);
		g_test = nullptr;
	}

	errx(1, "test_ppm  start failed");
}

void
stop()
{
	if (g_test != nullptr) {
		delete g_test;
		g_test = nullptr;

	} else {
		/* warn, but not an error */
		warnx("test_ppm already stopped.");
	}

	exit(0);
}

void
set(unsigned  ch, unsigned value)
{
	if (ch > 18 || ch < 1) {warnx("channel is not valid.");}

	if (value > 2500 || value < 1) { warnx("value is not valid.");}

	g_test->_values[ch] = value;
	g_test->_gaps[ch] = 2500 - value;

	if (ch == g_test->_channels) { g_test->_gaps[ch] = 5000; }

	return;
}

void
usage()
{
	warnx("missing command: try 'start',  'stop', 'set'\n");
}

} // namespace

int
test_ppm_main(int argc, char *argv[])
{
	if (argc < 2) {
		test_ppm::usage();
		return -1;
	}

	const char *verb = argv[1];
	unsigned  channels = 7;

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		test_ppm::start(channels);
		return 0;
	}

	if (!strcmp(verb, "stop")) {

		test_ppm::stop();
		return 0;
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "set")) {
		if (argc < 4) {
			errx(1, "Usage: test_ppm  set  <channel> <value>");
		}

		unsigned channel  = strtol(argv[2], NULL, 0);
		unsigned value	= strtol(argv[3], NULL, 0);

		test_ppm::set(channel, value);
		return 0;
	}

	test_ppm::usage();
	return -1;
}
