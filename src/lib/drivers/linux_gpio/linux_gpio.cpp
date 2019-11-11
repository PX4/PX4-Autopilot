/****************************************************************************
 *
 *   Copyright (c) 2016 - 2017 PX4 Development Team. All rights reserved.
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

#include "linux_gpio.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>

#include <px4_platform_common/posix.h>

#define PIN_INDEX_BUFFER_MAX (16)
#define PIN_DIRECTION_BUFFER_MAX (30 + PIN_INDEX_BUFFER_MAX)
#define PIN_VALUE_BUFFER_MAX (26 + PIN_INDEX_BUFFER_MAX)

LinuxGPIO::LinuxGPIO(unsigned int pin)
	: _pin(pin)
	, _fd(-1)
{
}

LinuxGPIO::~LinuxGPIO()
{
	if (_fd != -1) {
		close(_fd);
	}
}

int LinuxGPIO::exportPin()
{
	int ret;
	char pinIndex[PIN_INDEX_BUFFER_MAX];
	char valuePath[PIN_VALUE_BUFFER_MAX];
	int fd = -1;
	int bytes_to_write;

	struct stat statbuf;

	/* If GPIO was already opened, close it */
	if (_fd != -1) {
		close(_fd);
		_fd = -1;
	}

	/* Check if GPIO is already exported */
	snprintf(valuePath, PIN_VALUE_BUFFER_MAX, "/sys/class/gpio/gpio%d/value", _pin);
	ret = stat(valuePath, &statbuf);

	if (ret == -1) {
		/* GPIO is not exported */
		fd = open("/sys/class/gpio/export", O_WRONLY);

		if (fd == -1) {
			int err = errno;
			PX4_ERR("export %u: open: %s (%d)", _pin, strerror(err), err);
			return -1;
		}

		bytes_to_write = snprintf(pinIndex, PIN_INDEX_BUFFER_MAX, "%u", _pin);
		ret = write(fd, pinIndex, bytes_to_write);

		if (ret == -1) {
			int err = errno;
			PX4_ERR("export %u: write: %s (%d)", _pin, strerror(err), err);
			goto cleanup;

		} else if (ret != bytes_to_write) {
			PX4_ERR("export %u: write incomplete %d != %d", _pin, ret, bytes_to_write);
			goto cleanup;
		}
	}

	_fd = open(valuePath, O_RDWR);

	if (_fd == -1) {
		int err = errno;
		ret = -1;
		PX4_ERR("export %u: open: %s (%d)", _pin, strerror(err), err);
		goto cleanup;
	}

	ret = 0;

cleanup:

	if (fd != -1) {
		close(fd);
	}

	return ret;
}

int LinuxGPIO::unexportPin()
{
	char pinIndex[PIN_INDEX_BUFFER_MAX];
	int fd;
	int ret;
	int bytes_to_write;

	if (_fd != -1) {
		close(_fd);
		_fd = -1;
	}

	fd = open("/sys/class/gpio/unexport", O_WRONLY);

	if (fd == -1) {
		int err = errno;
		PX4_ERR("unexport %u: open: %s (%d)", _pin, strerror(err), err);
		return -1;
	}

	bytes_to_write = snprintf(pinIndex, PIN_INDEX_BUFFER_MAX, "%u", _pin);
	ret = write(fd, pinIndex, bytes_to_write);

	if (ret == -1) {
		int err = errno;
		PX4_ERR("unexport %u: write: %s (%d)", _pin, strerror(err), err);
		goto cleanup;

	} else if (ret != bytes_to_write) {
		PX4_ERR("unexport %u: write incomplete %d != %d", _pin, ret, bytes_to_write);
		goto cleanup;
	}

	ret = 0;

cleanup:
	close(fd);

	return ret;
}

int LinuxGPIO::setDirection(LinuxGPIO::Direction dir)
{
	char path[PIN_DIRECTION_BUFFER_MAX];
	int fd;
	int ret;

	snprintf(path, PIN_DIRECTION_BUFFER_MAX, "/sys/class/gpio/gpio%d/direction", _pin);
	fd = open(path, O_WRONLY);

	if (fd == -1) {
		int err = errno;
		PX4_ERR("dir %u: open: %s (%d)", _pin, strerror(err), err);
		return -1;
	}

	if (dir == Direction::IN) {
		ret = write(fd, "in", 2);

	} else {
		ret = write(fd, "out", 3);
	}

	if (ret == -1) {
		int err = errno;
		PX4_ERR("dir %u: write: %s (%d)", _pin, strerror(err), err);
		goto cleanup;
	}

	ret = 0;

cleanup:
	close(fd);

	return ret;
}

int LinuxGPIO::readValue()
{
	char buf[2];
	int ret;

	ret = ::read(_fd, buf, sizeof(buf));

	if (ret == -1) {
		int err = errno;
		PX4_ERR("readValue %u: read: %s (%d)", _pin, strerror(err), err);
		return ret;
	}

	ret = strtol(buf, nullptr, 10);

	return ret;
}

int LinuxGPIO::writeValue(LinuxGPIO::Value value)
{
	char buf[2];
	int ret;

	if (value != Value::LOW && value != Value::HIGH) {
		return -EINVAL;
	}

	int buflen = snprintf(buf, sizeof(buf), "%u", (unsigned int)value);

	ret = ::write(_fd, buf, buflen);

	if (ret == -1) {
		int err = errno;
		PX4_ERR("writeValue %u: write: %s (%d)", _pin, strerror(err), err);
		return ret;
	}

	return 0;
}
