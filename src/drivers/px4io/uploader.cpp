/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
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
 * @file uploader.cpp
 * Firmware uploader for PX4IO
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
#include <termios.h>
#include <sys/stat.h>

#include "uploader.h"

static const uint32_t crctab[] =
{
	0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f, 0xe963a535, 0x9e6495a3,
	0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988, 0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91,
	0x1db71064, 0x6ab020f2, 0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
	0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9, 0xfa0f3d63, 0x8d080df5,
	0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172, 0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b,
	0x35b5a8fa, 0x42b2986c, 0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
	0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423, 0xcfba9599, 0xb8bda50f,
	0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924, 0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d,
	0x76dc4190, 0x01db7106, 0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
	0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d, 0x91646c97, 0xe6635c01,
	0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e, 0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457,
	0x65b0d9c6, 0x12b7e950, 0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
	0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7, 0xa4d1c46d, 0xd3d6f4fb,
	0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0, 0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9,
	0x5005713c, 0x270241aa, 0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
	0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81, 0xb7bd5c3b, 0xc0ba6cad,
	0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a, 0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683,
	0xe3630b12, 0x94643b84, 0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
	0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb, 0x196c3671, 0x6e6b06e7,
	0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc, 0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5,
	0xd6d6a3e8, 0xa1d1937e, 0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
	0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55, 0x316e8eef, 0x4669be79,
	0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236, 0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f,
	0xc5ba3bbe, 0xb2bd0b28, 0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
	0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f, 0x72076785, 0x05005713,
	0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38, 0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21,
	0x86d3d2d4, 0xf1d4e242, 0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
	0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69, 0x616bffd3, 0x166ccf45,
	0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2, 0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db,
	0xaed16a4a, 0xd9d65adc, 0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
	0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693, 0x54de5729, 0x23d967bf,
	0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94, 0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
};

static uint32_t
crc32(const uint8_t *src, unsigned len, unsigned state)
{
	for (unsigned i = 0; i < len; i++)
		state = crctab[(state ^ src[i]) & 0xff] ^ (state >> 8);
	return state;
}

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
	const char *filename = NULL;
	size_t fw_size;

	_io_fd = open("/dev/ttyS2", O_RDWR);

	if (_io_fd < 0) {
		log("could not open interface");
		return -errno;
	}

	/* adjust line speed to match bootloader */
	struct termios t;
	tcgetattr(_io_fd, &t);
	cfsetspeed(&t, 115200);
	tcsetattr(_io_fd, TCSANOW, &t);

	/* look for the bootloader */
	ret = sync();

	if (ret != OK) {
		/* this is immediately fatal */
		log("bootloader not responding");
		close(_io_fd);
		_io_fd = -1;
		return -EIO;
	}

	for (unsigned i = 0; filenames[i] != nullptr; i++) {
		_fw_fd = open(filenames[i], O_RDONLY);

		if (_fw_fd < 0) {
			log("failed to open %s", filenames[i]);
			continue;
		}

		log("using firmware from %s", filenames[i]);
		filename = filenames[i];
		break;
	}

	if (filename == NULL) {
		log("no firmware found");
		close(_io_fd);
		_io_fd = -1;
		return -ENOENT;
	}

	struct stat st;
	if (stat(filename, &st) != 0) {
		log("Failed to stat %s - %d\n", filename, (int)errno);
		close(_io_fd);
		_io_fd = -1;
		return -errno;
	}
	fw_size = st.st_size;

	if (_fw_fd == -1) {
		close(_io_fd);
		_io_fd = -1;
		return -ENOENT;
	}

	/* do the usual program thing - allow for failure */
	for (unsigned retries = 0; retries < 1; retries++) {
		if (retries > 0) {
			log("retrying update...");
			ret = sync();

			if (ret != OK) {
				/* this is immediately fatal */
				log("bootloader not responding");
				close(_io_fd);
				_io_fd = -1;
				return -EIO;
			}
		}

		ret = get_info(INFO_BL_REV, bl_rev);

		if (ret == OK) {
			if (bl_rev <= BL_REV) {
				log("found bootloader revision: %d", bl_rev);
			} else {
				log("found unsupported bootloader revision %d, exiting", bl_rev);
				close(_io_fd);
				_io_fd = -1;
				return OK;
			}
		}

		ret = erase();

		if (ret != OK) {
			log("erase failed");
			continue;
		}

		ret = program(fw_size);

		if (ret != OK) {
			log("program failed");
			continue;
		}

		if (bl_rev <= 2)
			ret = verify_rev2(fw_size);
		else if(bl_rev == 3) {
			ret = verify_rev3(fw_size);
		}

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
	close(_io_fd);
	_io_fd = -1;
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
		int ret = recv(*p++, 5000);

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
		ret = recv(c, 1000);

		if (ret == OK) {
			//log("discard 0x%02x", c);
		}
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


static int read_with_retry(int fd, void *buf, size_t n)
{
	int ret;
	uint8_t retries = 0;
	do {
		ret = read(fd, buf, n);
	} while (ret == -1 && retries++ < 100);
	if (retries != 0) {
		printf("read of %u bytes needed %u retries\n",
		       (unsigned)n,
		       (unsigned)retries);
	}
	return ret;
}

int
PX4IO_Uploader::program(size_t fw_size)
{
	uint8_t	file_buf[PROG_MULTI_MAX];
	ssize_t count;
	int ret;
	size_t sent = 0;

	log("programming %u bytes...", (unsigned)fw_size);

	ret = lseek(_fw_fd, 0, SEEK_SET);

	while (sent < fw_size) {
		/* get more bytes to program */
		size_t n = fw_size - sent;
		if (n > sizeof(file_buf)) {
			n = sizeof(file_buf);
		}
		count = read_with_retry(_fw_fd, file_buf, n);

		if (count != (ssize_t)n) {
			log("firmware read of %u bytes at %u failed -> %d errno %d", 
			    (unsigned)n,
			    (unsigned)sent,
			    (int)count,
			    (int)errno);
		}

		if (count == 0)
			return OK;

		sent += count;

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
	return OK;
}

int
PX4IO_Uploader::verify_rev2(size_t fw_size)
{
	uint8_t	file_buf[4];
	ssize_t count;
	int ret;
	size_t sent = 0;

	log("verify...");
	lseek(_fw_fd, 0, SEEK_SET);

	send(PROTO_CHIP_VERIFY);
	send(PROTO_EOC);
	ret = get_sync();

	if (ret != OK)
		return ret;

	while (sent < fw_size) {
		/* get more bytes to verify */
		size_t n = fw_size - sent;
		if (n > sizeof(file_buf)) {
			n = sizeof(file_buf);
		}
		count = read_with_retry(_fw_fd, file_buf, n);

		if (count != (ssize_t)n) {
			log("firmware read of %u bytes at %u failed -> %d errno %d", 
			    (unsigned)n,
			    (unsigned)sent,
			    (int)count,
			    (int)errno);
		}

		if (count == 0)
			break;

		sent += count;

		if (count < 0)
			return -errno;

		ASSERT((count % 4) == 0);

		send(PROTO_READ_MULTI);
		send(count);
		send(PROTO_EOC);

		for (ssize_t i = 0; i < count; i++) {
			uint8_t c;

			ret = recv(c, 5000);

			if (ret != OK) {
				log("%d: got %d waiting for bytes", sent + i, ret);
				return ret;
			}

			if (c != file_buf[i]) {
				log("%d: got 0x%02x expected 0x%02x", sent + i, c, file_buf[i]);
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
PX4IO_Uploader::verify_rev3(size_t fw_size_local)
{
	int ret;
	uint8_t	file_buf[4];
	ssize_t count;
	uint32_t sum = 0;
	uint32_t bytes_read = 0;
	uint32_t crc = 0;
	uint32_t fw_size_remote;
	uint8_t fill_blank = 0xff;

	log("verify...");
	lseek(_fw_fd, 0, SEEK_SET);

	ret = get_info(INFO_FLASH_SIZE, fw_size_remote);
	send(PROTO_EOC);

	if (ret != OK) {
		log("could not read firmware size");
		return ret;
	}

	/* read through the firmware file again and calculate the checksum*/
	while (bytes_read < fw_size_local) {
		size_t n = fw_size_local - bytes_read;
		if (n > sizeof(file_buf)) {
			n = sizeof(file_buf);
		}
		count = read_with_retry(_fw_fd, file_buf, n);

		if (count != (ssize_t)n) {
			log("firmware read of %u bytes at %u failed -> %d errno %d", 
			    (unsigned)n,
			    (unsigned)bytes_read,
			    (int)count,
			    (int)errno);
		}

		/* set the rest to ff */
		if (count == 0) {
			break;
		}
		/* stop if the file cannot be read */
		if (count < 0)
			return -errno;

		/* calculate crc32 sum */
		sum = crc32((uint8_t *)&file_buf, sizeof(file_buf), sum);

		bytes_read += count;
	}

	/* fill the rest with 0xff */
	while (bytes_read < fw_size_remote) {
		sum = crc32(&fill_blank, sizeof(fill_blank), sum);
		bytes_read += sizeof(fill_blank);
	}

	/* request CRC from IO */
	send(PROTO_GET_CRC);
	send(PROTO_EOC);

	ret = recv((uint8_t*)(&crc), sizeof(crc));

	if (ret != OK) {
		log("did not receive CRC checksum");
		return ret;
	}

	/* compare the CRC sum from the IO with the one calculated */
	if (sum != crc) {
		log("CRC wrong: received: %d, expected: %d", crc, sum);
		return -EINVAL;
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
