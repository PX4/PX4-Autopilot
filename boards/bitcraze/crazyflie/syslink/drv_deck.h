/****************************************************************************
 *
 *   Copyright (C) 2016 PX4 Development Team. All rights reserved.
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

/* Definitions for crazyflie related drivers */

#ifndef _DRV_CRAZYFLIE_H
#define _DRV_CRAZYFLIE_H

#include <px4_platform_common/defines.h>
#include <stdint.h>
#include <sys/ioctl.h>


#define DECK_DEVICE_PATH	"/dev/deck"



/* structure of the data stored in deck memory */
typedef struct {
	uint8_t header; // Should be 0xEB
	uint32_t pins;
	uint8_t vendorId;
	uint8_t productId;
	uint8_t crc;
	uint8_t data[104];

} __attribute__((packed)) deck_descriptor_t;



/*
 * ioctl() definitions
 */

#define _DECKIOCBASE		(0x4100)
#define _DECKIOC(_n)		(_PX4_IOC(_DECKIOCBASE, _n))

/** get the number of connected deck memory devices */
#define DECKIOGNUM	_DECKIOC(0)

/** set the index of the current deck memory being accessed */
#define DECKIOSNUM	_DECKIOC(1)

#define DECKIOID	_DECKIOC(2)




#endif
