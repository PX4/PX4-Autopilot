/****************************************************************************
 *
 *   Copyright (C) 2015 PX4 Development Team. All rights reserved.
 *   Author: @author Kevin Hester <kevinh@geeksville.com>
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
 * @file otp.c
 * Tool for reading/writing/locking OTP
 */

#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <crc32.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <systemlib/otp.h>

__EXPORT int otp_main(int argc, char *argv[]);



static void
usage(const char *reason)
{
	if (reason != NULL) {
		warnx("%s", reason);
	}

	errx(1,
	     "usage:\n"
	     "otp show - show all block contents and lock bits"
	     "otp write <blocknum> <32 hex bytes, LSB first, no spaces> <crc32 for bytes> - will print values after readback\n"
	     "otp read <blocknum> - will print 32 bytes readback from block (and the current CRC)\n"
	     "otp lock <blocknum> <blocknum> - will permanently lock the specified block number\n"
	    );
}

static void otp_lock(int blocknum)
{
	F_unlock();
	sched_lock();

	lock_otp(blocknum);

	F_lock();
	sched_unlock();
}

static int otp_read(int i)
{
	printf("%2d: %c ", i, F_OTP_IS_LOCKED(i) ? 'L' : 'U');
	volatile uint8_t *base = F_OTP_BLOCK_PTR(i);

	for (int j = 0; j < OTP_BLOCK_SIZE; j++) {
		printf("%02x", base[j]);
		// if(j % 4 == 0) printf(" ");
	}

	printf(" %08x\n", crc32((const uint8_t *) base, OTP_BLOCK_SIZE));
	return 0;
}

static int otp_show(void)
{
	for (int i = 0; i < OTP_NUM_BLOCKS; i++) {
		otp_read(i);
	}

	return 0;
}

static int otp_write(int blocknum, const char *hexstr, uint32_t expectedcrc)
{
	if (strlen(hexstr) != OTP_BLOCK_SIZE * 2) {
		errx(1, "Bad hex string length");
		return 1;
	}

	uint8_t bytes[OTP_BLOCK_SIZE];

	for (int i = 0; i < OTP_BLOCK_SIZE; i++) {
		char temp[3];
		temp[0] = hexstr[2 * i];
		temp[1] = hexstr[2 * i + 1];
		temp[2] = '\0';
		bytes[i] = (uint8_t) strtol(temp, NULL, 16);
	}

	if (crc32(bytes, OTP_BLOCK_SIZE) != expectedcrc) {
		errx(1, "CRC does not match bytes");
		return 1;
	}

	F_unlock();
	sched_lock();

	volatile uint8_t *base = F_OTP_BLOCK_PTR(blocknum);

	for (int i = 0; i < OTP_BLOCK_SIZE; i++) {
		if (F_write_byte(ADDR_OTP_START + OTP_BLOCK_SIZE * blocknum + i, bytes[i])) {
			errx(1, "Failed to write");
			return 1;
		}

		if (base[i] != bytes[i]) {
			errx(1, "Write not accepted");
			return 1;
		}
	}

	F_lock();
	sched_unlock();

	printf("WRITTEN\n");
	return 0;
}

int otp_main(int argc, char *argv[])
{
	int getblocknum(int argnum) {
		if (argnum >= argc) {
			errx(1, "must specify a block number");
			return -1;

		} else {
			int bnum = -1;
			sscanf(argv[argnum], "%d", &bnum);

			if (bnum < 0 || bnum >= OTP_NUM_BLOCKS) {
				errx(1, "invalid block number");
				return -1;

			} else {
				return bnum;
			}
		}
	}

	if (argc < 2) {
		usage(NULL);

	} else {
		const char *cmd = argv[1];

		if (!strcmp(cmd, "show")) {
			return otp_show();

		} else if (!strcmp(cmd, "read") && argc == 3) {
			int bnum = getblocknum(2);

			if (bnum >= 0) {
				return otp_read(bnum);
			}

		} else if (!strcmp(cmd, "write") && argc == 5) {
			int bnum = getblocknum(2);
			uint32_t expectedcrc = strtol(argv[4], NULL, 16);

			if (bnum >= 0) {
				return otp_write(bnum, argv[3], expectedcrc);
			}

		} else if (!strcmp(cmd, "lock") && argc == 4) {
			int bnum = getblocknum(2);
			int bnum2 = getblocknum(3);

			if (bnum >= 0 && bnum == bnum2) {
				otp_lock(bnum);
				printf("LOCKED\n");
				return 0;

			} else {
				printf("FAILED\n");
				return 1;
			}

		} else {
			usage("Invalid arguments");
		}
	}

	return 0;
}
