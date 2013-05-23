/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Lorenz Meier <lm@inf.ethz.ch>
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
 * @file reboot.c
 * Tool similar to UNIX reboot command
 */

#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <unistd.h>
#include <stdio.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <libtomcrypt/tomcrypt.h>

/* RM page 75 OTP. Size is 528 bytes total (512 bytes data and 16 bytes locking) */
#define ADDR_OTP_START			0x1FFF7800
#define ADDR_OTP_LOCK_START		0x1FFF7A00

#define OTP_LEN				512
#define OTP_LOCK_LEN			16

#define OTP_LOCK_LOCKED			0x00
#define OTP_LOCK_UNLOCKED		0xFF

__EXPORT int auth_main(int argc, char *argv[]);

#pragma pack(push, 1)

/*
 * The OTP area is divided into 16 OTP data blocks of 32 bytes and one lock OTP block of 16 bytes.
 * The OTP data and lock blocks cannot be erased. The lock block contains 16 bytes LOCKBi (0 ≤ i ≤ 15)
 * to lock the corresponding OTP data block (blocks 0 to 15). Each OTP data block can be programmed
 * until the value 0x00 is programmed in the corresponding OTP lock byte. The lock bytes must only
 * contain 0x00 and 0xFF values, otherwise the OTP bytes might not be taken into account correctly.
 */

	struct otp {
		char		id[4];		///< 'P' 'X' '4' '\n'
		uint8_t		id_type;	///< 0 for USB VID, 1 for generic VID
		uint32_t	vid;
		uint32_t	pid;
		char		signature[128];
		char		public_key[128];
		uint32_t	lock_bytes[4];
	};

	struct otp_lock {
		uint8_t		lock_bytes[16];
	};
#pragma pack(pop)

#define UDID_START		0x1FFF7A10
#define ADDR_FLASH_SIZE		0x1FFF7A22

#pragma pack(push, 1)
	struct udid {
		uint32_t	serial[3];
	};
#pragma pack(pop)

int flash_unlock(void)
{
	/* unlock the control register */
	volatile uint32_t *keyr = (volatile uint32_t *)0x40023c04;
	*keyr = 0x45670123U;
	*keyr = 0xcdef89abU;

	volatile uint32_t *cr = (volatile uint32_t *)0x40023c10;

	/* check the control register */
	if (*cr & 0x80000000) {
		warnx("WARNING: flash unlock failed, flash aborted");
		return 1;
	}

	return 0;
}

int flash_lock(void)
{
	volatile uint32_t *cr = (volatile uint32_t *)0x40023c10;
	/* re-lock the flash control register */
	*cr = 0x80000000;
}

int val_read(void* dest, volatile const void* src, int bytes)
{
	
	int i;
	for (i = 0; i < bytes / 4; i++) {
		*(((volatile uint32_t *)dest) + i) = *(((volatile uint32_t *)src) + i);
	}
	return i*4;
}

int val_write(volatile void* dest, const void* src, int bytes)
{
	flash_unlock();

	volatile uint32_t *sr = (volatile uint32_t *)0x40023c0c;
	volatile uint32_t *cr = (volatile uint32_t *)0x40023c10;

	int i;

	volatile uint32_t *d = (volatile uint32_t *)dest;
	volatile uint32_t *s = (volatile uint32_t *)src;

	for (i = 0; i < bytes / 4; i++) {
		/* program a byte */
		*cr = 1;

		d[i] = s[i];

		/* wait for the operation to complete */
		while (*sr & 0x1000) {
		}

		if (*sr & 0xf2) {
			warnx("WARNING: program error 0x%02x after %d bytes", *sr, i*4);
			goto flash_end;
		}
	}


flash_end:
	flash_lock();

	return i*4;
}

int write_otp(uint8_t id_type, uint32_t vid, uint32_t pid, char* signature, char* public_key)
{
	struct otp otp_mem;
	memset(&otp_mem, 0, sizeof(otp_mem));

	/* fill struct */
	otp_mem.id[0] = 'P';
	otp_mem.id[1] = 'X';
	otp_mem.id[2] = '4';
	otp_mem.id[3] = '\0';

	volatile uint32_t* otp_ptr = ADDR_OTP_START;
	val_write(otp_ptr, &otp_mem, sizeof(struct otp));
}

int lock_otp()
{
	/* determine the required locking size - can only write full lock bytes */
	int size = sizeof(struct otp) / 32;

	struct otp_lock otp_lock_mem;

	memset(&otp_lock_mem, OTP_LOCK_UNLOCKED, sizeof(otp_lock_mem));
	for (int i = 0; i < sizeof(otp_lock_mem) / sizeof(otp_lock_mem.lock_bytes[0]); i++)
		otp_lock_mem.lock_bytes[i] = OTP_LOCK_LOCKED;

	/* XXX add the actual call here to write the OTP_LOCK bytes only at final stage */
	// val_copy(lock_ptr, &otp_lock_mem, sizeof(otp_lock_mem));
}

int auth_main(int argc, char *argv[])
{


	/*
	 * PKCS#1 BASED CHIP ID SIGNING PREPARATION
	 */

	/* XXX this is testing code, the key has to be read from STDINPUT later */
	rsa_key key;
	int     err;
	/* register the system RNG */
	register_prng(&sprng_desc);
	/* make a 1024-bit RSA key with the system RNG */
	if ((err = rsa_make_key(NULL, find_prng("sprng"), 1024/8, 65537, &key))
		!= CRYPT_OK) {
		printf("make_key error: %s\n", error_to_string(err));
		return 1;
	}

	/*
	 * ONE-TIME-PROGRAMMABLE (OTP) MEMORY HANDLING SECTION
	 */

	/* disable scheduling, leave interrupt processing untouched */
	sched_lock();

	if (argc > 1 && !strcmp(argv[1], "-w")) {

		warnx("Writing (but not locking) OTP");
		/* write OTP */
		uint8_t id_type = 0;
		uint32_t vid = 0x26AC;
		uint32_t pid = 0x10;
		char* signature = 0;
		char* public_key = 0;

		write_otp(id_type, vid, pid, signature, public_key);

		return 0;
	}

	if (argc > 1 && !strcmp(argv[1], "-l")) {

		warnx("Locking OTP, no further write operations are permitted");
		lock_otp();
		return 0;
	}

	/* read out unique chip ID */
	const volatile uint32_t* udid_ptr = (const uint32_t*)UDID_START;
	struct udid id;
	val_read(&id, udid_ptr, sizeof(id));

	warnx("Unique serial # [%0X%0X%0X]", id.serial[0], id.serial[1], id.serial[2]);

	uint16_t *fsize = ADDR_FLASH_SIZE;

	warnx("Flash size: %d", (int)*fsize);

	/* get OTP memory */
	struct otp otp_mem;
	const volatile uint32_t* otp_ptr = ADDR_OTP_START;
	val_read(&otp_mem, otp_ptr, sizeof(struct otp));

	/* ID string */
	otp_mem.id[3] = '\0';
	warnx("ID String: %s", (otp_mem.id != 0xFF) ? otp_mem.id : "[not written]");

	/* get OTP lock */
	struct otp_lock otp_lock_mem;
	const volatile uint32_t* otp_lock_ptr = ADDR_OTP_LOCK_START;
	val_read(&otp_lock_mem, otp_lock_ptr, sizeof(struct otp_lock));

	printf("OTP LOCK STATUS: ");
	for (int i = 0; i < sizeof(otp_lock_mem) / sizeof(otp_lock_mem.lock_bytes[0]); i++)
		printf("%0X", otp_lock_mem.lock_bytes[i]);
	printf("\n");

	sched_unlock();
}
