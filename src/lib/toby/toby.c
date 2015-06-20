/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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

/*
 * @file toby.c
 *
 * u-blox TOBY module AT command library
 *
 * AT Command Manual:
 * http://www.u-blox.com/images/downloads/Product_Docs/u-blox-ATCommands_Manual_%28UBX-13002752%29.pdf
 *
 * AT Command Examples for u-blox LEON-G, SARA-G, LISA-U, SARA-U, and TOBY-L series wireless modules:
 * http://www.u-blox.com/images/downloads/Product_Docs/AT-CommandsExamples_ApplicationNote_(UBX-13001820).pdf
 *
 * TOBY-L2 Networking Modes: Describes the two operational modes of the TOBY-L2 module and how to provide connectivity to customer modems.
 * http://www.u-blox.com/images/downloads/Product_Docs/TOBY-L2-NetworkingModes_ApplicationNote_(UBX-14000479).pdf
 *
 * Datasheet:
 * http://www.u-blox.com/images/downloads/Product_Docs/MPCI-L2_DataSheet_(UBX-13004749).pdf
 *
 * System Integration Manual: Describes Hardware and System Design Aspects
 * http://www.u-blox.com/images/downloads/Product_Docs/TOBY-L2-MPCI-L2_SysIntegrManual_%28UBX-13004618%29.pdf
 *
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include <stdbool.h>
#include <stdio.h>
#include "toby.h"

enum TOBY_DECODE_STATE {
	TOBY_DECODE_STATE_UNSYNCED,
	TOBY_DECODE_STATE_AT,
	TOBY_DECODE_STATE_BINARY
};

const char *decode_states[] = {"UNSYNCED",
			       "AT_RCV",
			       "BIN_RCV"
			      };

int _decode_state = TOBY_DECODE_STATE_UNSYNCED;

int toby_init(struct toby_state *state)
{
	return 0;
}

int toby_decode(const uint8_t *buf, unsigned len,
	struct toby_state *state, uint8_t *buf_r, unsigned consumed)
{

	int ret = 1;

	switch (_decode_state) {
	case TOBY_DECODE_STATE_UNSYNCED:
		break;

	}

	return ret;
}

int toby_encode_udp()
{
	return 0;
}

int toby_setup_udp_connection(const char* address, unsigned port, struct toby_state *state)
{
	return 0;
}
