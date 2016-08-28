/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file syslink.c
 *
 * Crazyflie Syslink protocol implementation
 *
 * @author Dennis Shtatnov <densht@gmail.com>
 */


#include <px4_defines.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <systemlib/err.h>
#include <poll.h>
#include <termios.h>


#include "syslink.h"


const char *syslink_magic = "\xbc\xcf";

void syslink_parse_init(syslink_parse_state *state)
{
	state->state = SYSLINK_STATE_START;
	state->index = 0;
}


int syslink_parse_char(syslink_parse_state *state, char c, syslink_message_t *msg)
{

	switch (state->state) {
	case SYSLINK_STATE_START:
		if (c == syslink_magic[state->index]) {
			state->index++;

		} else {
			state->index = 0;
		}

		if (syslink_magic[state->index] == '\x00') {
			state->state = SYSLINK_STATE_TYPE;
		}

		break;

	case SYSLINK_STATE_TYPE:
		msg->type = c;
		state->state = SYSLINK_STATE_LENGTH;
		break;

	case SYSLINK_STATE_LENGTH:
		msg->length = c;

		if (c > SYSLINK_MAX_DATA_LEN) { // Too long
			state->state = SYSLINK_STATE_START;

		} else {
			state->state = c > 0 ? SYSLINK_STATE_DATA : SYSLINK_STATE_CKSUM;
		}

		state->index = 0;
		break;

	case SYSLINK_STATE_DATA:
		msg->data[state->index++] = c;

		if (state->index >= msg->length) {
			state->state = SYSLINK_STATE_CKSUM;
			state->index = 0;
			syslink_compute_cksum(msg);
		}

		break;

	case SYSLINK_STATE_CKSUM:
		if (c != msg->cksum[state->index]) {
			PX4_INFO("Bad checksum");
			state->state = SYSLINK_STATE_START;
			state->index = 0;
			break;
		}

		state->index++;

		if (state->index >= sizeof(msg->cksum)) {
			state->state = SYSLINK_STATE_START;
			state->index = 0;
			return 1;
		}


		break;
	}

	return 0;

}

/*
	Computes Fletcher 8bit checksum per RFC1146
	A := A + D[i]
	B := B + A
*/
void syslink_compute_cksum(syslink_message_t *msg)
{
	uint8_t a = 0, b = 0;
	uint8_t *Di = (uint8_t *)msg, *end = Di + (2 + msg->length) * sizeof(uint8_t);

	while (Di < end) {
		a = a + *Di;
		b = b + a;
		++Di;
	}

	msg->cksum[0] = a;
	msg->cksum[1] = b;
}
