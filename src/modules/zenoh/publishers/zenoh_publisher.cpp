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
	z_view_keyexpr_t ke;

	if (z_view_keyexpr_from_str(&ke, keyexpr) < 0) {
		printf("%s is not a valid key expression\n", keyexpr);
		return -1;
	}

	if (z_declare_publisher(z_loan(s), &_pub, z_loan(ke), NULL) < 0) {
		printf("Unable to declare publisher for key expression!\n");
		return -1;
	}

	z_timestamp_new(&ts, z_loan(s));

	return 0;
}

int8_t Zenoh_Publisher::publish(const uint8_t *buf, int size)
{
	z_publisher_put_options_t options;
	z_publisher_put_options_default(&options);

	z_owned_bytes_t attachment;
	z_bytes_empty(&attachment);

	ze_owned_serializer_t serializer;
	ze_serializer_empty(&serializer);

	ze_serializer_serialize_str(z_loan_mut(serializer), "sequence_number");
	ze_serializer_serialize_int64(z_loan_mut(serializer), this->sequence_number++);

	ze_serializer_serialize_str(z_loan_mut(serializer), "source_timestamp");
	ze_serializer_serialize_int64(z_loan_mut(serializer), hrt_absolute_time());

	px4_guid_t px4_guid;
	board_get_px4_guid(px4_guid);

	ze_serializer_serialize_str(z_loan_mut(serializer), "source_gid");
	ze_serializer_serialize_buf(z_loan_mut(serializer), px4_guid, 16);

	ze_serializer_finish(z_move(serializer), &attachment);
	options.attachment = z_move(attachment);

	// Add timestamp
	options.timestamp = &ts;

	z_owned_bytes_t payload;
	z_bytes_copy_from_buf(&payload, buf, size);
	return z_publisher_put(z_loan(_pub), z_move(payload), &options);
}

void Zenoh_Publisher::print()
{
	z_view_string_t keystr;
	z_keyexpr_as_view_string(z_publisher_keyexpr(z_loan(_pub)), &keystr);
	printf("Topic: %s\n", z_string_data(z_loan(keystr)));
}
