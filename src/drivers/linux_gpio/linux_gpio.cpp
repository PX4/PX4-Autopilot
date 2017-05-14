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

#include <drivers/linux_gpio/linux_gpio.h>
#include <px4_posix.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#define PIN_INDEX_BUFFER_MAX (16)
#define PIN_DIRECTION_BUFFER_MAX (30 + PIN_INDEX_BUFFER_MAX)
#define PIN_VALUE_BUFFER_MAX (26 + PIN_INDEX_BUFFER_MAX)

LinuxGPIO::LinuxGPIO(unsigned int pin)
	: _pin(pin)
{
}

int LinuxGPIO::exportPin()
{
	return LinuxGPIO::_exportPin(_pin);
}

int LinuxGPIO::unexportPin()
{
	return LinuxGPIO::_unexportPin(_pin);
}

int LinuxGPIO::setDirection(LinuxGPIO::Direction dir)
{
	return LinuxGPIO::_setDirection(_pin, (int)dir);
}

int LinuxGPIO::readValue()
{
	return LinuxGPIO::_readValue(_pin);
}

int LinuxGPIO::writeValue(LinuxGPIO::Value value)
{
	return LinuxGPIO::_writeValue(_pin, (unsigned int)value);
}

int LinuxGPIO::_exportPin(unsigned int pin)
{
	char pinIndex[PIN_INDEX_BUFFER_MAX];
	int fd;
	int ret;
	int bytes_to_write;

	fd = open("/sys/class/gpio/export", O_WRONLY);

	if (fd == -1) {
		int err = errno;
		PX4_ERR("export failed: open: %s (%d)", strerror(err), err);
		return -1;
	}

	bytes_to_write = snprintf(pinIndex, PIN_INDEX_BUFFER_MAX, "%u", pin);
	ret = write(fd, pinIndex, bytes_to_write);

	if (ret == -1) {
		int err = errno;
		PX4_ERR("export failed: write: %s (%d)", strerror(err), err);
		goto cleanup;

	} else if (ret != bytes_to_write) {
		PX4_ERR("failed to write: incomplete %d != %d", ret, bytes_to_write);
		goto cleanup;
	}

	ret = 0;

cleanup:
	close(fd);

	return ret;
}

int LinuxGPIO::_unexportPin(unsigned int pin)
{
	char pinIndex[PIN_INDEX_BUFFER_MAX];
	int fd;
	int ret;
	int bytes_to_write;

	fd = open("/sys/class/gpio/unexport", O_WRONLY);

	if (fd == -1) {
		int err = errno;
		PX4_ERR("unexport %u: open: %s (%d)", pin, strerror(err), err);
		return -1;
	}

	bytes_to_write = snprintf(pinIndex, PIN_INDEX_BUFFER_MAX, "%u", pin);
	ret = write(fd, pinIndex, bytes_to_write);

	if (ret == -1) {
		int err = errno;
		PX4_ERR("unexport %u: write: %s (%d)", pin, strerror(err), err);
		goto cleanup;

	} else if (ret != bytes_to_write) {
		PX4_ERR("unexport %u: write incomplete %d != %d", pin, ret, bytes_to_write);
		goto cleanup;
	}

	ret = 0;

cleanup:
	close(fd);

	return ret;
}

int LinuxGPIO::_setDirection(unsigned int pin, int dir)
{
	char path[PIN_DIRECTION_BUFFER_MAX];
	int fd;
	int ret;

	snprintf(path, PIN_DIRECTION_BUFFER_MAX, "/sys/class/gpio/gpio%d/direction", pin);
	fd = open(path, O_WRONLY);

	if (fd == -1) {
		int err = errno;
		PX4_ERR("dir %u: open: %s (%d)", pin, strerror(err), err);
		return -1;
	}

	if (dir == 0) {
		ret = write(fd, "in", 2);

	} else {
		ret = write(fd, "out", 3);
	}

	if (ret == -1) {
		int err = errno;
		PX4_ERR("dir %u: write: %s (%d)", pin, strerror(err), err);
		goto cleanup;
	}

	ret = 0;

cleanup:
	close(fd);

	return ret;
}

int LinuxGPIO::_readValue(int pin)
{
	char path[PIN_VALUE_BUFFER_MAX];
	char buf[2];
	int fd;
	int ret;

	snprintf(path, PIN_VALUE_BUFFER_MAX, "/sys/class/gpio/gpio%d/value", pin);
	fd = open(path, O_RDONLY);

	if (fd == -1) {
		int err = errno;
		PX4_ERR("read %u: open: %s (%d)", pin, strerror(err), err);
		return -1;
	}

	ret = read(fd, buf, sizeof(buf));

	if (ret == -1) {
		int err = errno;
		PX4_ERR("read %u: write: %s (%d)", pin, strerror(err), err);
		goto cleanup;
	}

	ret = 0;

	ret = strtol(buf, nullptr, 10);

cleanup:
	close(fd);

	return ret;
}

int LinuxGPIO::_writeValue(int pin, unsigned int value)
{
	char path[PIN_VALUE_BUFFER_MAX];
	char buf[2];
	int fd;
	int ret;

	if (value != (unsigned int)Value::LOW && value != (unsigned int)Value::HIGH) {
		return -EINVAL;
	}

	snprintf(path, PIN_VALUE_BUFFER_MAX, "/sys/class/gpio/gpio%d/value", pin);
	fd = open(path, O_WRONLY);

	if (fd == -1) {
		int err = errno;
		PX4_ERR("set %u: open: %s (%d)", pin, strerror(err), err);
		return -1;
	}

	int buflen = snprintf(buf, sizeof(buf), "%u", value);

	ret = write(fd, buf, buflen);

	if (ret == -1) {
		int err = errno;
		PX4_ERR("set %u: write: %s (%d)", pin, strerror(err), err);
		goto cleanup;
	}

	ret = 0;

cleanup:
	close(fd);

	return ret;
}

