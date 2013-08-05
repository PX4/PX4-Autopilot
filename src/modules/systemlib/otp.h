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
 * @file otp.h
 * One TIme Programmable ( OTP ) Flash routine/s. 
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author David "Buzz" Bussenschutt <davidbuzz@gmail.com>
 *
 */

#ifndef OTP_H_
#define OTP_H_

 __BEGIN_DECLS
 

/* RM page 75 OTP. Size is 528 bytes total (512 bytes data and 16 bytes locking) */
#define ADDR_OTP_START			0x1FFF7800
#define ADDR_OTP_LOCK_START		0x1FFF7A00

#define OTP_LEN				512
#define OTP_LOCK_LEN			16

#define OTP_LOCK_LOCKED			0x00
#define OTP_LOCK_UNLOCKED		0xFF


 #pragma pack(push, 1)

/*
 * The OTP area is divided into 16 OTP data blocks of 32 bytes and one lock OTP block of 16 bytes.
 * The OTP data and lock blocks cannot be erased. The lock block contains 16 bytes LOCKBi (0 ≤ i ≤ 15)
 * to lock the corresponding OTP data block (blocks 0 to 15). Each OTP data block can be programmed
 * until the value 0x00 is programmed in the corresponding OTP lock byte. The lock bytes must only
 * contain 0x00 and 0xFF values, otherwise the OTP bytes might not be taken into account correctly.
 */

	struct otp {
	    // first 32 bytes =  the '0' Block 
		char		id[4];		///4 bytes < 'P' 'X' '4' '\n'
		uint8_t		id_type;	///1 byte < 0 for USB VID, 1 for generic VID
		uint32_t	vid;        ///4 bytes
		uint32_t	pid;        ///4 bytes
		char        unused[19];  ///3 bytes 
		// Cert-of-Auth is next 4 blocks ie 1-4  ( where zero is first block ) 	
		char        signature[128];
        // insert extras here 
		uint32_t	lock_bytes[4];
	};

	struct otp_lock {
		uint8_t		lock_bytes[16];
	};
#pragma pack(pop)

#define UDID_START		0x1FFF7A10
#define ADDR_FLASH_SIZE		0x1FFF7A22

#pragma pack(push, 1)
		union udid {
		uint32_t	serial[3];
		char  data[12];
	};
#pragma pack(pop)

 
 /**
  *   s
  */
 //__EXPORT float calc_indicated_airspeed(float differential_pressure);
 
 __EXPORT int flash_unlock(void);
 __EXPORT int flash_lock(void);
 __EXPORT int val_read(void* dest, volatile const void* src, int bytes);
 __EXPORT int val_write(volatile void* dest, const void* src, int bytes);
 __EXPORT int write_otp(uint8_t id_type, uint32_t vid, uint32_t pid, char* signature);
 __EXPORT int lock_otp(void);

      
__END_DECLS

#endif
