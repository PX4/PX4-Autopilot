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
#include <stdio.h>
#include <string.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include "simulator.h"

Simulator *Simulator::_instance = NULL;

Simulator::Simulator() : _max_readers(3)
{
	sem_init(&_lock, 0, _max_readers);
}

Simulator *Simulator::getInstance()
{
	return _instance;
}

int Simulator::getSample(sim_dev_t dev, sample &val)
{
	int ret;

	switch (dev) {
	case SIM_GYRO:
		read_lock();
		val = _gyro[_readidx];
		read_unlock();
		ret = 0;
		break;
	case SIM_ACCEL:
		read_lock();
		val = _accel[_readidx];
		read_unlock();
		ret = 0;
		break;
	case SIM_MAG:
		read_lock();
		val = _mag[_readidx];
		read_unlock();
		ret = 0;
		break;
	default:
		ret = 1;
	}	
	return ret;
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
	int writeidx, num;

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
                        buf[len] = 0;
                        printf("received: %s\n", buf);
			// FIXME - temp hack to read data, not safe - bad bad bad
			num = sscanf((const char *)buf, "%f,%f,%f,%f,%f,%f,%f,%f,%f",
				&_gyro[writeidx].x, &_gyro[writeidx].y, &_gyro[writeidx].z,
				&_accel[writeidx].x, &_accel[writeidx].y, &_accel[writeidx].z,
				&_mag[writeidx].x, &_mag[writeidx].y, &_mag[writeidx].z);
			if (num != 9) {
				printf("Only read %d items\n", num);
			}
			else {
				// Swap read and write buffers
				write_lock();
				_readidx = !_readidx;
				write_unlock();
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

extern "C" {

int simulator_main(int argc, char *argv[])
{
	return (int)(px4_task_spawn_cmd("Simulator",
			SCHED_DEFAULT,
			SCHED_PRIORITY_MAX - 5,
			1500,
			Simulator::start,
			nullptr) < 0);

	return 0;
}

}

