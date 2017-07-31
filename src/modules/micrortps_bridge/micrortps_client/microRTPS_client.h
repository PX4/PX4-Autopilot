/****************************************************************************
 *
 * Copyright 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "microRTPS_transport.h"

#include <cinttypes>
#include <cstdio>
#include <ctime>
#include <pthread.h>
#include <termios.h>

#include <microcdr/microCdr.h>
#include <px4_config.h>
#include <px4_getopt.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <px4_time.h>
#include <uORB/uORB.h>

#define BUFFER_SIZE 1024
#define UPDATE_TIME_MS 0
#define LOOPS 10000
#define SLEEP_MS 1
#define BAUDRATE 460800
#define DEVICE "/dev/ttyACM0"
#define POLL_MS 1
#define DEFAULT_RECV_PORT 2019
#define DEFAULT_SEND_PORT 2020

void *send(void *data);
void micrortps_start_topics(struct timespec &begin, int &total_read, uint32_t &received, int &loop);

struct options {
	enum class eTransports {
		UART,
		UDP
	};
	eTransports transport = options::eTransports::UART;
	char device[64] = DEVICE;
	int update_time_ms = UPDATE_TIME_MS;
	int loops = LOOPS;
	int sleep_ms = SLEEP_MS;
	uint32_t baudrate = BAUDRATE;
	int poll_ms = POLL_MS;
	uint16_t recv_port = DEFAULT_RECV_PORT;
	uint16_t send_port = DEFAULT_SEND_PORT;
};

extern struct options _options;
extern bool _should_exit_task;
extern Transport_node *transport_node;

