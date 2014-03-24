/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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
 * @file board_serial.h
 * Read off the board serial
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author David "Buzz" Bussenschutt <davidbuzz@gmail.com>
 *
 */

#include "otp.h"
#include "board_config.h"
#include "board_serial.h"

int get_board_serial(uint8_t *serialid)
{
	const volatile uint32_t *udid_ptr = (const uint32_t *)UDID_START;
	union udid id;
	val_read((uint32_t *)&id, udid_ptr, sizeof(id));


	/* Copy the serial from the chips non-write memory and swap endianess */
	serialid[0] = id.data[3];   serialid[1] = id.data[2];  serialid[2] = id.data[1];  serialid[3] = id.data[0];
	serialid[4] = id.data[7];   serialid[5] = id.data[6];  serialid[6] = id.data[5];  serialid[7] = id.data[4];
	serialid[8] = id.data[11];   serialid[9] = id.data[10];  serialid[10] = id.data[9];  serialid[11] = id.data[8];

	return 0;
}
