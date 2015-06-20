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
 * @file toby.h
 *
 * u-blox TOBY module AT command library
 *
 * @author Lorenz Meier <lorenz@px4.io>
 */

#pragma once

#include <stdint.h>

__BEGIN_DECLS

enum TOBY_CONN_STATE {
	TOBY_CONN_STATE_UNINIT = 0,
	TOBY_CONN_STATE_MODEM_OK,
	TOBY_CONN_STATE_NETWORK_OK,
	TOBY_CONN_STATE_IP_OK,
	TOBY_CONN_STATE_ERR_MODEM_FAIL,
	TOBY_CONN_STATE_ERR_PIN_FAIL,
	TOBY_CONN_STATE_ERR_NETWORK_FAIL,
	TOBY_CONN_STATE_ERR_IP_FAIL
};

struct toby_state {
	enum TOBY_CONN_STATE connection;
	char host[50];
	unsigned port;
};

__EXPORT int toby_decode(const uint8_t *buf, unsigned len,
	struct toby_state *state, uint8_t *buf_r, unsigned consumed);

__EXPORT int toby_encode_udp();

__EXPORT int toby_setup_udp_connection(const char* address, unsigned port, struct toby_state *state);

__EXPORT int toby_init(struct toby_state *state);

__END_DECLS
