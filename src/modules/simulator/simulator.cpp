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

#include <px4_tasks.h>
#include <err.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include "simulator.h"

static px4_task_t g_sim_task = -1;

Simulator *Simulator::_instance = NULL;

Simulator::Simulator() : _max_readers(3)
{
	sem_init(&_lock, 0, _max_readers);
}

Simulator *Simulator::getInstance()
{
	return _instance;
}

bool Simulator::getMPUReport(uint8_t *buf, int len)
{
	if (len != sizeof(MPUReport)) {
		return false;
	}
	read_lock();
	memcpy(buf, &_mpureport[_readidx], sizeof(MPUReport));
	read_unlock();
	return true;
}

int Simulator::start(int argc, char *argv[])
{
	int ret = 0;
	_instance = new Simulator();
	if (_instance) 
		_instance->updateSamples();
	else {
		printf("Simulator creation failed\n");
		ret = 1;
	}
	return ret;
}


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
	int writeidx;

        if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
                printf("create socket failed\n");
		return;
        }

        memset((char *)&myaddr, 0, sizeof(myaddr));
        myaddr.sin_family = AF_INET;
        myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
        myaddr.sin_port = htons(port);

        if (bind(fd, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0) {
                printf("bind failed\n");
		return;
        }

        for (;;) {
                len = recvfrom(fd, buf, buflen, 0, (struct sockaddr *)&srcaddr, &addrlen);
                if (len > 0) {
			writeidx = !_readidx;
			if (len == sizeof(MPUReport)) {
                        	printf("received: MPU data\n");
				memcpy((void *)&_mpureport[writeidx], (void *)buf, len);
			
				// Swap read and write buffers
				write_lock();
				_readidx = !_readidx;
				write_unlock();
			}
			else {
                        	printf("bad packet: len = %d\n", len);
			}
                }
        }
}

void Simulator::read_lock()
{
	sem_wait(&_lock);
}
void Simulator::read_unlock()
{
	sem_post(&_lock);
}
void Simulator::write_lock()
{
	for (int i=0; i<_max_readers; i++) {
		sem_wait(&_lock);
	}
}
void Simulator::write_unlock()
{
	for (int i=0; i<_max_readers; i++) {
		sem_post(&_lock);
	}
}

static void usage()
{
	warnx("Usage: simulator {start|stop}");
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
			warnx("Simulator not running");
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

