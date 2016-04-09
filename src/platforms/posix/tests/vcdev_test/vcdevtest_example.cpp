
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

#define TESTDEV "/dev/vdevtest"

static bool g_exit = false;

static int writer_main(int argc, char *argv[])
{
	char buf[1];

	int fd = px4_open(TESTDEV, PX4_F_WRONLY);

	if (fd < 0) {
		PX4_INFO("Writer: Open failed %d %d", fd, px4_errno);
		return -px4_errno;
	}

	int ret;
	int i = 0;

	while (!g_exit) {
		// Wait for 2 seconds
		PX4_INFO("Writer: Sleeping for 2 sec");
		ret = sleep(2);

		if (ret < 0) {
			PX4_INFO("Writer: sleep failed %d %d", ret, errno);
			return ret;
		}

		buf[0] = 'A' + (char)(i % 26);
		PX4_INFO("Writer: writing char '%c'", buf[0]);
		ret = px4_write(fd, buf, 1);
		++i;
	}

	px4_close(fd);
	PX4_INFO("Writer: stopped");
	return ret;
}

class PrivData
{
public:
	PrivData() : _read_offset(0) {}
	~PrivData() {}

	size_t _read_offset;
};

class VCDevNode : public VDev
{
public:
	VCDevNode() :
		VDev("vcdevtest", TESTDEV),
		_is_open_for_write(false),
		_write_offset(0) {};

	~VCDevNode() {}

	virtual int open(device::file_t *handlep);
	virtual int close(device::file_t *handlep);
	virtual ssize_t write(device::file_t *handlep, const char *buffer, size_t buflen);
	virtual ssize_t read(device::file_t *handlep, char *buffer, size_t buflen);
private:
	bool _is_open_for_write;
	size_t _write_offset;
	char     _buf[1000];
};

int VCDevNode::open(device::file_t *handlep)
{
	// Only allow one writer
	if (_is_open_for_write && (handlep->flags & PX4_F_WRONLY)) {
		errno = EBUSY;
		return -1;
	}

	int ret = VDev::open(handlep);

	if (ret != 0) {
		return ret;
	}

	handlep->priv = new PrivData;

	if (_is_open_for_write && (handlep->flags & PX4_F_WRONLY)) {
		_is_open_for_write = true;
	}

	return 0;
}

int VCDevNode::close(device::file_t *handlep)
{
	delete(PrivData *)handlep->priv;
	handlep->priv = nullptr;
	VDev::close(handlep);

	// Enable a new writer of the device is re-opened for write
	if ((handlep->flags & PX4_F_WRONLY) && _is_open_for_write) {
		_is_open_for_write = false;
	}

	return 0;
}

ssize_t VCDevNode::write(device::file_t *handlep, const char *buffer, size_t buflen)
{
	for (size_t i = 0; i < buflen && _write_offset < 1000; i++) {
		_buf[_write_offset] = buffer[i];
		_write_offset++;
	}

	// ignore what was written, but let pollers know something was written
	poll_notify(POLLIN);

	return buflen;
}

ssize_t VCDevNode::read(device::file_t *handlep, char *buffer, size_t buflen)
{
	PrivData *p = (PrivData *)handlep->priv;
	ssize_t chars_read = 0;
	PX4_INFO("read %zu write %zu", p->_read_offset, _write_offset);

	for (size_t i = 0; i < buflen && (p->_read_offset < _write_offset); i++) {
		buffer[i] = _buf[p->_read_offset];
		p->_read_offset++;
		chars_read++;
	}

	return chars_read;
}

VCDevExample::~VCDevExample()
{
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

	PX4_INFO("pub_blocked = %d %s", ret, (unsigned long)ret == blocked ? "PASS" : "FAIL");

	return 0;
}

int VCDevExample::do_poll(int fd, int timeout, int iterations, int delayms_after_poll)
{
	int pollret, readret;
	int loop_count = 0;
	char readbuf[10];
	px4_pollfd_struct_t fds[1];

	fds[0].fd = fd;
	fds[0].events = POLLIN;
	fds[0].revents = 0;

	bool mustblock = (timeout < 0);

	// Test indefinte blocking poll
	while ((!appState.exitRequested()) && (loop_count < iterations)) {
		pollret = px4_poll(fds, 1, timeout);

		if (pollret < 0) {
			PX4_ERR("Reader: px4_poll failed %d %d FAIL", pollret, px4_errno);
			goto fail;
		}

		PX4_INFO("Reader: px4_poll returned %d", pollret);

		if (pollret) {
			readret = px4_read(fd, readbuf, 10);

			if (readret != 1) {
				if (mustblock) {
					PX4_ERR("Reader:     read failed %d FAIL", readret);
					goto fail;

				} else {
					PX4_INFO("Reader:     read failed %d FAIL", readret);
				}

			} else {
				readbuf[readret] = '\0';
				PX4_INFO("Reader: px4_poll     returned %d, read '%s' PASS", pollret, readbuf);
			}
		}

		if (delayms_after_poll) {
			usleep(delayms_after_poll * 1000);
		}

		loop_count++;
	}

	return 0;
fail:
	return 1;
}
int VCDevExample::main()
{
	appState.setRunning(true);

	_node = new VCDevNode();

	if (_node == 0) {
		PX4_INFO("Failed to allocate VCDevNode");
		return -ENOMEM;
	}

	if (_node->init() != PX4_OK) {
		PX4_INFO("Failed to init VCDevNode");
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

	PX4_INFO("priv data = %p %s", p, p == (void *)_node ? "PASS" : "FAIL");

	ret = test_pub_block(fd, 1);

	if (ret < 0) {
		return ret;
	}

	ret = test_pub_block(fd, 0);

	if (ret < 0) {
		return ret;
	}


	// Start a task that will write something in 4 seconds
	(void)px4_task_spawn_cmd("writer",
				 SCHED_DEFAULT,
				 SCHED_PRIORITY_MAX - 6,
				 2000,
				 writer_main,
				 (char *const *)NULL);

	ret = 0;

	PX4_INFO("TEST: BLOCKING POLL ---------------");

	if (do_poll(fd, -1, 3, 0)) {
		ret = 1;
		goto fail2;
	}

	PX4_INFO("TEST: ZERO TIMEOUT POLL -----------");

	if (do_poll(fd, 0, 3, 0)) {
		ret = 1;
		goto fail2;
		goto fail2;
	}

	PX4_INFO("TEST: ZERO TIMEOUT POLL -----------");

	if (do_poll(fd, 0, 3, 0)) {
		ret = 1;
		goto fail2;
		goto fail2;
	}

	PX4_INFO("TEST: ZERO TIMEOUT POLL -----------");

	if (do_poll(fd, 0, 3, 0)) {
		ret = 1;
		goto fail2;
	}

	PX4_INFO("TEST: 100ms TIMEOUT POLL -----------");

	if (do_poll(fd, 0, 30, 100)) {
		ret = 1;
		goto fail2;
	}

	PX4_INFO("TEST: 1 SEC TIMOUT POLL ------------");

	if (do_poll(fd, 1000, 3, 0)) {
		ret = 1;
		goto fail2;
	}

	PX4_INFO("TEST: waiting for writer to stop");
fail2:
	g_exit = true;
	px4_close(fd);
	PX4_INFO("TEST: waiting for writer to stop");
	sleep(3);
	return ret;
}
