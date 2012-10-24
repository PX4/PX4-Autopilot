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
 * @file Firmware uploader for PX4IO
 */

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>

#include "uploader.h"

PX4IO_Uploader::PX4IO_Uploader() :
	_io_fd(-1),
	_fw_fd(-1)
{
}

PX4IO_Uploader::~PX4IO_Uploader()
{
}

int
PX4IO_Uploader::upload(const char *filenames[])
{
	int	ret;

	_io_fd = open("/dev/ttyS2", O_RDWR);

	if (_io_fd < 0) {
		log("could not open interface");
		return -errno;
	}

	/* look for the bootloader */
	ret = sync();

	if (ret != OK) {
		/* this is immediately fatal */
		log("bootloader not responding");
		return -EIO;
	}

	for (unsigned i = 0; filenames[i] != nullptr; i++) {
		_fw_fd = open(filenames[i], O_RDONLY);

		if (_fw_fd < 0) {
			log("failed to open %s", filenames[i]);
			continue;
		}

		log("using firmware from %s", filenames[i]);
		break;
	}

	if (_fw_fd == -1)
		return -ENOENT;

	/* do the usual program thing - allow for failure */
	for (unsigned retries = 0; retries < 1; retries++) {
		if (retries > 0) {
			log("retrying update...");
			ret = sync();

			if (ret != OK) {
				/* this is immediately fatal */
				log("bootloader not responding");
				return -EIO;
			}
		}

		ret = erase();

		if (ret != OK) {
			log("erase failed");
			continue;
		}

		ret = program();

		if (ret != OK) {
			log("program failed");
			continue;
		}

		ret = verify();

		if (ret != OK) {
			log("verify failed");
			continue;
		}

		ret = reboot();

		if (ret != OK) {
			log("reboot failed");
			return ret;
		}

		log("update complete");

		ret = OK;
		break;
	}

	close(_fw_fd);
	return ret;
}

int
PX4IO_Uploader::recv(uint8_t &c, unsigned timeout)
{
	struct pollfd fds[1];

	fds[0].fd = _io_fd;
	fds[0].events = POLLIN;

	/* wait 100 ms for a character */
	int ret = ::poll(&fds[0], 1, timeout);

	if (ret < 1) {
		//log("poll timeout %d", ret);
		return -ETIMEDOUT;
	}

	read(_io_fd, &c, 1);
	//log("recv 0x%02x", c);
	return OK;
}

int
PX4IO_Uploader::recv(uint8_t *p, unsigned count)
{
	while (count--) {
		int ret = recv(*p++);

		if (ret != OK)
			return ret;
	}

	return OK;
}

void
PX4IO_Uploader::drain()
{
	uint8_t c;
	int ret;

	do {
		ret = recv(c, 10);
		//log("discard 0x%02x", c);
	} while (ret == OK);
}

int
PX4IO_Uploader::send(uint8_t c)
{
	//log("send 0x%02x", c);
	if (write(_io_fd, &c, 1) != 1)
		return -errno;

	return OK;
}

int
PX4IO_Uploader::send(uint8_t *p, unsigned count)
{
	while (count--) {
		int ret = send(*p++);

		if (ret != OK)
			return ret;
	}

	return OK;
}

int
PX4IO_Uploader::get_sync(unsigned timeout)
{
	uint8_t c[2];
	int ret;

	ret = recv(c[0], timeout);

	if (ret != OK)
		return ret;

	ret = recv(c[1], timeout);

	if (ret != OK)
		return ret;

	if ((c[0] != PROTO_INSYNC) || (c[1] != PROTO_OK)) {
		log("bad sync 0x%02x,0x%02x", c[0], c[1]);
		return -EIO;
	}

	return OK;
}

int
PX4IO_Uploader::sync()
{
	drain();

	/* complete any pending program operation */
	for (unsigned i = 0; i < (PROG_MULTI_MAX + 6); i++)
		send(0);

	send(PROTO_GET_SYNC);
	send(PROTO_EOC);
	return get_sync();
}

int
PX4IO_Uploader::get_info(int param, uint32_t &val)
{
	int ret;

	send(PROTO_GET_DEVICE);
	send(param);
	send(PROTO_EOC);

	ret = recv((uint8_t *)&val, sizeof(val));

	if (ret != OK)
		return ret;

	return get_sync();
}

int
PX4IO_Uploader::erase()
{
	log("erase...");
	send(PROTO_CHIP_ERASE);
	send(PROTO_EOC);
	return get_sync(10000);		/* allow 10s timeout */
}

int
PX4IO_Uploader::program()
{
	uint8_t	file_buf[PROG_MULTI_MAX];
	ssize_t count;
	int ret;

	log("program...");
	lseek(_fw_fd, 0, SEEK_SET);

	while (true) {
		/* get more bytes to program */
		//log("  %d", (int)lseek(_fw_fd, 0, SEEK_CUR));
		count = read(_fw_fd, file_buf, sizeof(file_buf));

		if (count == 0)
			return OK;

		if (count < 0)
			return -errno;

		ASSERT((count % 4) == 0);

		send(PROTO_PROG_MULTI);
		send(count);
		send(&file_buf[0], count);
		send(PROTO_EOC);

		ret = get_sync(1000);

		if (ret != OK)
			return ret;
	}
}

int
PX4IO_Uploader::verify()
{
	uint8_t	file_buf[PROG_MULTI_MAX];
	ssize_t count;
	int ret;

	log("verify...");
	lseek(_fw_fd, 0, SEEK_SET);

	send(PROTO_CHIP_VERIFY);
	send(PROTO_EOC);
	ret = get_sync();

	if (ret != OK)
		return ret;

	while (true) {
		/* get more bytes to verify */
		int base = (int)lseek(_fw_fd, 0, SEEK_CUR);
		count = read(_fw_fd, file_buf, sizeof(file_buf));

		if (count == 0)
			break;

		if (count < 0)
			return -errno;

		ASSERT((count % 4) == 0);

		send(PROTO_READ_MULTI);
		send(count);
		send(PROTO_EOC);

		for (ssize_t i = 0; i < count; i++) {
			uint8_t c;

			ret = recv(c);

			if (ret != OK) {
				log("%d: got %d waiting for bytes", base + i, ret);
				return ret;
			}

			if (c != file_buf[i]) {
				log("%d: got 0x%02x expected 0x%02x", base + i, c, file_buf[i]);
				return -EINVAL;
			}
		}

		ret = get_sync();

		if (ret != OK) {
			log("timeout waiting for post-verify sync");
			return ret;
		}
	}

	return OK;
}

int
PX4IO_Uploader::reboot()
{
	send(PROTO_REBOOT);
	send(PROTO_EOC);

	return OK;
}

int
PX4IO_Uploader::compare(bool &identical)
{
	uint32_t file_vectors[15];
	uint32_t fw_vectors[15];
	int ret;

	lseek(_fw_fd, 0, SEEK_SET);
	ret = read(_fw_fd, &file_vectors[0], sizeof(file_vectors));

	send(PROTO_CHIP_VERIFY);
	send(PROTO_EOC);
	ret = get_sync();

	if (ret != OK)
		return ret;

	send(PROTO_READ_MULTI);
	send(sizeof(fw_vectors));
	send(PROTO_EOC);
	ret = recv((uint8_t *)&fw_vectors[0], sizeof(fw_vectors));

	if (ret != OK)
		return ret;

	identical = (memcmp(&file_vectors[0], &fw_vectors[0], sizeof(file_vectors))) ? true : false;

	return OK;
}

void
PX4IO_Uploader::log(const char *fmt, ...)
{
	va_list	ap;

	printf("[PX4IO] ");
	va_start(ap, fmt);
	vprintf(fmt, ap);
	va_end(ap);
	printf("\n");
	fflush(stdout);
}