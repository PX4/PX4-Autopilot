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
 * @file zenoh_publisher.cpp
 *
 * Zenoh publisher
 *
 * @author Peter van der Perk <peter.vanderperk@nxp.com>
 */

#include "zenoh_publisher.hpp"


Zenoh_Publisher::Zenoh_Publisher()
{
	this->_topic[0] = 0x0;
}

Zenoh_Publisher::~Zenoh_Publisher()
{
	undeclare_publisher();
}

int Zenoh_Publisher::undeclare_publisher()
{
	z_undeclare_publisher(z_publisher_move(&_pub));
	return 0;
}

int Zenoh_Publisher::declare_publisher(z_owned_session_t s, const char *keyexpr)
{
	strncpy(this->_topic, keyexpr, sizeof(this->_topic));

	z_view_keyexpr_t ke;
	z_view_keyexpr_from_str(&ke, this->_topic);

	if (z_declare_publisher(&_pub, z_loan(s), z_loan(ke), NULL) < 0) {
		printf("Unable to declare publisher for key expression!\n");
		return -1;
	}

	if (!z_publisher_check(&_pub)) {
		printf("Unable to declare publisher for key expression!\n");
		return -1;
	}

	return 0;
}

int8_t Zenoh_Publisher::publish(const uint8_t *buf, int size)
{
	z_publisher_put_options_t options;
	z_publisher_put_options_default(&options);
	options.encoding = NULL;

	z_owned_bytes_t payload;
	z_bytes_serialize_from_slice(&payload, buf, size);
	return z_publisher_put(z_loan(_pub), z_move(payload), &options);
}

void Zenoh_Publisher::print()
{
	printf("Topic: %s\n", this->_topic);
}
