/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * @file zenoh_subscriber.cpp
 *
 * Zenoh subscriber
 *
 * @author Peter van der Perk <peter.vanderperk@nxp.com>
 */

#include "zenoh_subscriber.hpp"

static void data_handler_cb(z_loaned_sample_t *sample, void *arg)
{
	static_cast<Zenoh_Subscriber *>(arg)->data_handler(sample);
}

void Zenoh_Subscriber::data_handler(const z_loaned_sample_t *sample)
{
	z_view_string_t keystr;
	z_keyexpr_as_view_string(z_sample_keyexpr(sample), &keystr);
	printf(">> [Subscriber] Received ('%s' size '%d')\n", z_string_data(z_loan(keystr)),
	       (int)z_bytes_len(z_sample_payload(sample)));
}

Zenoh_Subscriber::Zenoh_Subscriber()
{
}

Zenoh_Subscriber::~Zenoh_Subscriber()
{
	undeclare_subscriber();
}

int Zenoh_Subscriber::undeclare_subscriber()
{
	z_undeclare_subscriber(z_subscriber_move(&_sub));
	return 0;
}

int Zenoh_Subscriber::declare_subscriber(z_owned_session_t s, const char *keyexpr)
{
	z_owned_closure_sample_t callback;
	z_closure(&callback, data_handler_cb, NULL, this);

	z_view_keyexpr_t ke;
	z_view_keyexpr_from_str(&ke, keyexpr);

	if (z_declare_subscriber(z_loan(s), &_sub, z_loan(ke), z_closure_sample_move(&callback), NULL) < 0) {
		printf("Unable to declare subscriber.\n");
		exit(-1);
	}

	return 0;
}

void Zenoh_Subscriber::print()
{
	z_view_string_t keystr;
	z_keyexpr_as_view_string(z_subscriber_keyexpr(z_loan(_sub)), &keystr);
	printf("Topic: %s\n", z_string_data(z_loan(keystr)));
}

void Zenoh_Subscriber::print(const char *type_string, const char *topic_string)
{
	z_view_string_t keystr;
	z_keyexpr_as_view_string(z_subscriber_keyexpr(z_loan(_sub)), &keystr);
	printf("Topic: %.*s -> %s %s \n", (int)z_string_len(z_loan(keystr)), z_string_data(z_loan(keystr)), type_string,
	       topic_string);
}
