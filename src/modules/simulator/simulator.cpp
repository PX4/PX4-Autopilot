/****************************************************************************
 *
 *   Copyright (c) 2015 Mark Charlebois. All rights reserved.
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
 * @file simulator.cpp
 * A device simulator
 */

#include <px4_debug.h>
#include <px4_tasks.h>
#include <systemlib/err.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h> 
#include <drivers/drv_led.h>
#ifndef __PX4_QURT
#include <sys/socket.h>
#include <netinet/in.h>
#endif
#include "simulator.h"

using namespace simulator;

static px4_task_t g_sim_task = -1;

Simulator *Simulator::_instance = NULL;

Simulator *Simulator::getInstance()
{
	return _instance;
}

bool Simulator::getMPUReport(uint8_t *buf, int len)
{
	return _mpu.copyData(buf, len);
}

bool Simulator::getRawAccelReport(uint8_t *buf, int len)
{
	return _accel.copyData(buf, len);
}

bool Simulator::getBaroSample(uint8_t *buf, int len)
{
	return _baro.copyData(buf, len);
}

int Simulator::start(int argc, char *argv[])
{
	int ret = 0;
	_instance = new Simulator();
	if (_instance) {
		PX4_INFO("Simulator started\n");
		drv_led_start();
#ifndef __PX4_QURT
		_instance->updateSamples();
#endif
	}
	else {
		PX4_WARN("Simulator creation failed\n");
		ret = 1;
	}
	return ret;
}


#ifndef __PX4_QURT
void Simulator::updateSamples()
{
	// get samples from external provider
	struct sockaddr_in myaddr;
	struct sockaddr_in srcaddr;
	socklen_t addrlen = sizeof(srcaddr);
	int len, fd;
	const int buflen = 200;
	const int port = 9876;
	unsigned char buf[buflen];

	if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		PX4_WARN("create socket failed\n");
		return;
	}

	memset((char *)&myaddr, 0, sizeof(myaddr));
	myaddr.sin_family = AF_INET;
	myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	myaddr.sin_port = htons(port);

	if (bind(fd, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0) {
		PX4_WARN("bind failed\n");
		return;
	}

	for (;;) {
		len = recvfrom(fd, buf, buflen, 0, (struct sockaddr *)&srcaddr, &addrlen);
		if (len > 0) {
			if (len == sizeof(RawMPUData)) {
				PX4_DBG("received: MPU data\n");
				_mpu.writeData(buf);
			}
			else if (len == sizeof(RawAccelData)) {
				PX4_DBG("received: accel data\n");
				_accel.writeData(buf);
			}
			else if (len == sizeof(RawBaroData)) {
				PX4_DBG("received: accel data\n");
				_baro.writeData(buf);
			}
			else {
				PX4_DBG("bad packet: len = %d\n", len);
			}
		}
	}
}
#endif

static void usage()
{
	PX4_WARN("Usage: simulator {start|stop}");
}

extern "C" {

int simulator_main(int argc, char *argv[])
{
	int ret = 0;
	if (argc != 2) {
		usage();
		return 1;
	}
	if (strcmp(argv[1], "start") == 0) {
		if (g_sim_task >= 0) {
			warnx("Simulator already started");
			return 0;
		}
		g_sim_task = px4_task_spawn_cmd("Simulator",
			SCHED_DEFAULT,
			SCHED_PRIORITY_MAX - 5,
			1500,
			Simulator::start,
			nullptr);
	}
	else if (strcmp(argv[1], "stop") == 0) {
		if (g_sim_task < 0) {
			PX4_WARN("Simulator not running");
		}
		else {
			px4_task_delete(g_sim_task);
			g_sim_task = -1;
		}
	}
	else {
		usage();
		ret = -EINVAL;
	}

	return ret;
}

}

__BEGIN_DECLS
extern void led_init(void);
extern void led_on(int led);
extern void led_off(int led);
extern void led_toggle(int led);
__END_DECLS

bool static _led_state[2] = { false , false };

__EXPORT void led_init()
{
	PX4_DBG("LED_INIT\n");
}

__EXPORT void led_on(int led)
{
	if (led == 1 || led == 0)
	{
		PX4_DBG("LED%d_ON", led);
		_led_state[led] = true;
	}
}

__EXPORT void led_off(int led)
{
	if (led == 1 || led == 0)
	{
		PX4_DBG("LED%d_OFF", led);
		_led_state[led] = false;
	}
}

__EXPORT void led_toggle(int led)
{
	if (led == 1 || led == 0)
	{
		_led_state[led] = !_led_state[led];
		PX4_DBG("LED%d_TOGGLE: %s\n", led, _led_state[led] ? "ON" : "OFF");

	}
}

