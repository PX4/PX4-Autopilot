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
 * @file PX4FMU <-> PX4IO messaging protocol.
 *
 * This initial version of the protocol is very simple; each side transmits a
 * complete update with each frame.  This avoids the sending of many small
 * messages and the corresponding complexity involved.
 */

#pragma once

/*
 * XXX MUST BE KEPT IN SYNC WITH THE VERSION IN PX4FMU UNTIL
 * TREES ARE MERGED.
 */

#define PX4IO_OUTPUT_CHANNELS	8
#define PX4IO_INPUT_CHANNELS	12
#define PX4IO_RELAY_CHANNELS	2

#pragma pack(push, 1)

/* command from FMU to IO */
struct px4io_command {
	uint16_t	f2i_magic;
#define F2I_MAGIC	0x636d

	uint16_t	servo_command[PX4IO_OUTPUT_CHANNELS];
	bool		relay_state[PX4IO_RELAY_CHANNELS];
	bool		arm_ok;
};

/* report from IO to FMU */
struct px4io_report {
	uint16_t	i2f_magic;
#define I2F_MAGIC		0x7570

	uint16_t	rc_channel[PX4IO_INPUT_CHANNELS];
	bool		armed;
	uint8_t		channel_count;
};

#pragma pack(pop)