/****************************************************************************
 *
 *   Copyright (C) 2012-2013 PX4 Development Team. All rights reserved.
 *   Authors: 
 *      Lorenz Meier <lm@inf.ethz.ch>
 *      David "Buzz" Bussenschutt <davidbuzz@gmail.com> 
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
 * otp estimation
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author David "Buzz" Bussenschutt <davidbuzz@gmail.com>
 *
 */

#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <string.h>  // memset
#include "conversions.h"
#include "otp.h"
#include "err.h"   // warnx 


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

	int i;
	

	volatile uint32_t *sr = (volatile uint32_t *)0x40023c0c;
	volatile uint32_t *cr = (volatile uint32_t *)0x40023c10;


	volatile uint32_t *d = (volatile uint32_t *)dest;
	volatile uint32_t *s = (volatile uint32_t *)src;

	for (i = 0; i < bytes / 4; i++) {
		/* program a byte */
		*cr = 1;
		
		warnx("%016x",s[i]);

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

int write_otp(uint8_t id_type, uint32_t vid, uint32_t pid, char* signature)
{
	struct otp otp_mem;
	memset(&otp_mem, 0, sizeof(otp_mem));

	/* fill struct */
	otp_mem.id[0] = 'P';
	otp_mem.id[1] = 'X';
	otp_mem.id[2] = '4';
	otp_mem.id[3] = '\0';
	
	memcpy(otp_mem.signature,signature,128); 
	
	warnx("write_otp: %s  / %s ",signature, otp_mem.signature); 

	volatile uint32_t* otp_ptr = ADDR_OTP_START;
	val_write(otp_ptr, &otp_mem, sizeof(struct otp));
}

int lock_otp(void)
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


 