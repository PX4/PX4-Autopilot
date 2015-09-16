
/****************************************************************************
 *
 *   Copyright (C) 2015 Mark Charlebois. All rights reserved.
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
 * @file vcdevtest_example.cpp
 * Example for Linux
 *
 * @author Mark Charlebois <charlebm@gmail.com>
 */

#include <px4_tasks.h>
#include <px4_time.h>
#include "vcdevtest_example.h"
#include <drivers/drv_device.h>
#include <drivers/device/device.h>
#include <unistd.h>
#include <stdio.h>

px4::AppState VCDevExample::appState;

using namespace device;

#define TESTDEV "/dev/vdevex"

static int writer_main(int argc, char *argv[])
{
	char buf[1];

	int fd = px4_open(TESTDEV, PX4_F_RDONLY);
	if (fd < 0) {
		PX4_INFO("--- Open failed %d %d", fd, px4_errno);
		return -px4_errno;
	}

	int ret;
	int i=0;
	while (i<3) {
		// Wait for 4 seconds
		PX4_INFO("Writer: Sleeping for 4 sec\n");
		ret = sleep(4);
		if (ret < 0) {
			PX4_INFO("Writer: sleep failed %d %d\n", ret, errno);
			return ret;
		}

		PX4_INFO("Writer: writing to fd\n");
		buf[0] = 'A'+(char)(i % 26);
		ret = px4_write(fd, buf, 1);
	++i;
	}
	px4_close(fd);
	return ret;
}

class VCDevNode : public VDev {
public:
	VCDevNode() : VDev("vcdevtest", TESTDEV) {};

	~VCDevNode() {}

	virtual ssize_t write(device::file_t *handlep, const char *buffer, size_t buflen);
	virtual ssize_t read(device::file_t *handlep, const char *buffer, size_t buflen);
private:
	uint32_t _read_offset;
	uint32_t _write_offset;
	char     _buf[1000];
};

ssize_t VCDevNode::write(device::file_t *handlep, const char *buffer, size_t buflen)
{
	// ignore what was written, but let pollers know something was written
	poll_notify(POLLIN);

	for (int i=0; i<buflen && _write_offset<1000; i++) {
		_buf[_write_offset] = buffer[i];
		_write_offset++;
	}

	return buflen;
}

ssize_t VCDevNode::read(device::file_t *handlep, const char *buffer, size_t buflen)
{
	for (int i=0; i<buflen && _read_offset<_write_offset; i++) {
		buffer[i] = _buf[_read_offset];
		_read_offset++;
	}
	
	return buflen;
}

VCDevExample::~VCDevExample() {
	if (_node) {
		delete _node;
		_node = 0;
	}
}

static int test_pub_block(int fd, unsigned long blocked)
{
	int ret = px4_ioctl(fd, DEVIOCSPUBBLOCK, blocked);
	if (ret < 0) {
		PX4_INFO("ioctl PX4_DEVIOCSPUBBLOCK failed %d %d", ret, px4_errno);
		return -px4_errno;
	}

	ret = px4_ioctl(fd, DEVIOCGPUBBLOCK, 0);
	if (ret < 0) {
		PX4_INFO("ioctl PX4_DEVIOCGPUBBLOCK failed %d %d", ret, px4_errno);
		return -px4_errno;
	}
	PX4_INFO("pub_blocked = %d %s\n", ret, (unsigned long)ret == blocked ? "PASS" : "FAIL");

	return 0;
}

int VCDevExample::main()
{
	appState.setRunning(true);

	_node = new VCDevNode();

	if (_node == 0) {
		PX4_INFO("Failed to allocate VCDevNode\n");
		return -ENOMEM;
	}

	if (_node->init() != PX4_OK) {
		PX4_INFO("Failed to init VCDevNode\n");
		return 1;
	}

	int fd = px4_open(TESTDEV, PX4_F_RDONLY);

	if (fd < 0) {
		PX4_INFO("Open failed %d %d", fd, px4_errno);
		return -px4_errno;
	}

	void *p = 0;
	int ret = px4_ioctl(fd, DIOC_GETPRIV, (unsigned long)&p);
	if (ret < 0) {
		PX4_INFO("ioctl DIOC_GETPRIV failed %d %d", ret, px4_errno);
		return -px4_errno;
	}
	PX4_INFO("priv data = %p %s\n", p, p == (void *)_node ? "PASS" : "FAIL");

	ret = test_pub_block(fd, 1);
	if (ret < 0)
		return ret;
	ret = test_pub_block(fd, 0);
	if (ret < 0)
		return ret;

	int i=0;
	px4_pollfd_struct_t fds[1];

	// Start a task that will write something in 4 seconds
	(void)px4_task_spawn_cmd("writer", 
				       SCHED_DEFAULT,
				       SCHED_PRIORITY_MAX - 6,
				       2000,
				       writer_main,
				       (char* const*)NULL);

	while (!appState.exitRequested() && i<13) {
		PX4_INFO("=====================\n");
		PX4_INFO("Reader: sleeping 2 sec\n");
		sleep(2);

		fds[0].fd = fd;
		fds[0].events = POLLIN;
		fds[0].revents = 0;

		PX4_INFO("Reader: Calling px4_poll with 1 sec wait\n");
		ret = px4_poll(fds, 1, 1000);
		PX4_INFO("Reader: Done px4_poll\n");
		if (ret < 0) {
			PX4_INFO("Reader: px4_poll failed %d %d\n", ret, px4_errno);
			px4_close(fd);
		} 
		else if (i > 0) {
			if (ret == 0) {
				PX4_INFO("Reader: Nothing to read - PASS\n");
			}
			else {
				PX4_INFO("Reader: poll returned %d\n", ret);
			}
		}
		else if (i == 0) {
			if (ret == 1) {
				PX4_INFO("Reader: %d to read - %s\n", ret, fds[0].revents & POLLIN ? "PASS" : "FAIL");
			}
			else {
				PX4_INFO("Reader: %d to read - FAIL\n", ret);
			}
		
		}
		++i;
	}
	i=0;
	while (!appState.exitRequested() && i<13) {
		PX4_INFO("=====================\n");
		PX4_INFO("Reader:  sleeping 2 sec ====\n");
		sleep(2);

		fds[0].fd = fd;
		fds[0].events = POLLIN;
		fds[0].revents = 0;

		PX4_INFO("Reader: Calling px4_poll with 0 timeout\n");
		ret = px4_poll(fds, 1, 0);
		PX4_INFO("Reader: Done px4_poll\n");
		if (ret < 0) {
			PX4_INFO("Reader: px4_poll failed %d %d\n", ret, px4_errno);
			px4_close(fd);
		} 
		if (ret == 0) {
			PX4_INFO("Reader: Nothing to read - PASS\n");
		}
		if (ret == 1) {
			PX4_INFO("Reader: %d to read - %s\n", ret, fds[0].revents & POLLIN ? "PASS" : "FAIL");
		}
		else {
			PX4_INFO("Reader: %d to read - FAIL\n", ret);
		}
		
		++i;
	}
	px4_close(fd);

	return 0;
}
