/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Lorenz Meier <lm@inf.ethz.ch>
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
 * @file tests_bson.c
 *
 * Tests for the bson en/decoder
 */

#include <stdio.h>
#include <systemlib/err.h>

#include <systemlib/bson/tinybson.h>
#include "tests.h"

static const bool test_bool = true;
static const int32_t test_int = 32;
static const double test_double = 2.5;
static const char *test_string = "this is a test";
static const uint8_t test_data[256];
static const char *test_filename = "/fs/microsd/bson.test";

static int
encode(bson_encoder_t encoder)
{

	if (bson_encoder_append_int(encoder, "thisisanillegalbsonpropertynamebecauseitistoolong", 0) == 0)
		warnx("FAIL: encoder: too-long node name not rejected");

	if (bson_encoder_append_int(encoder, "bool1", test_bool) != 0)
		warnx("FAIL: encoder: append bool failed");
	if (bson_encoder_append_int(encoder, "int1", test_int) != 0)
		warnx("FAIL: encoder: append int failed");
	if (bson_encoder_append_double(encoder, "double1", test_double) != 0)
		warnx("FAIL: encoder: append double failed");
	if (bson_encoder_append_string(encoder, "string1", test_string) != 0)
		warnx("FAIL: encoder: append string failed");
	if (bson_encoder_append_binary(encoder, "data1", test_data) != 0)
		warnx("FAIL: encoder: append data failed");

	bson_encoder_fini(encoder);

	return 0;
}

static int
decode_callback(bson_decoder_t decoder, void *private, bson_node_t node)
{
	if (!strcmp(node->name, "bool1")) {
		if (node->b != test_bool)
			warnx("FAIL: decoder: bool1 value %s, expected %s", 
				(node->b ? "true" : "false"),
				(test_bool ? "true" : "false"));
		return 1;
	}
	if (!strcmp(node->name, "int1")) {
		if (node->i != test_int)
			warnx("FAIL: decoder: int1 value %d, expected %d", node->i, test_int);
		return 1;
	}
	if (!strcmp(node->name, "double1")) {
		if (node->d != test_double)
			warnx("FAIL: decoder: double1 value %f, expected %f", node->d, test_double);
		return 1;
	}
	if (!strcmp(node->name, "string1")) {
		unsigned len = bson_decoder_data_pending(decoder);

		if (len != (strlen(test_string) + 1)) {
			warnx("FAIL: decoder: string1 length %d wrong, expected %d", len, strlen(test_string) + 1);
			return 1;

		char sbuf[len];

		if (bson_decoder_copy_data(decoder, sbuf)) {
			warnx("FAIL: decoder: string1 copy failed");
			return 1;
		}
		if (bson_decoder_data_pending(decoder) != 0) {
			warnx("FAIL: decoder: string1 copy did not exhaust all data");
			return 1;
		}
		if (sbuf[len - 1] != '\0') {
			warnx("FAIL: decoder: string1 not 0-terminated");
			return 1;
		}
		if (strcmp(sbuf, test_string)) {
			warnx("FAIL: decoder: string1 value '%s', expected '%s'", sbuf, test_string);
			return 1;
		}
		return 1;
	}
	if (!strcmp(node->name, "data1")) {
		unsigned len = bson_decoder_data_pending(decoder);

		if (len != sizeof(test_data)) {
			warnx("FAIL: decoder: data1 length %d, expected %d", len, sizeof(test_data));
			return 1;
		}
		
		uint8_t dbuf[len];

		if (bson_decoder_copy_data(decoder, dbuf)) {
			warnx("FAIL: decoder: data1 copy failed");
			return 1;
		}
		if (bson_decoder_data_pending(decoder) != 0) {
			warnx("FAIL: decoder: data1 copy did not exhaust all data");
			return 1;
		}
		if (memcmp(test_data, dbuf, len)) {
			warnx("FAIL: decoder: data1 compare fail");
			return 1;
		}
		return 1;
	}

	warnx("FAIL: decoder: unexpected node name '%s'", node->name);
	return 1;
}

static int
decode(bson_decoder_t decoder)
{
	int result;

	do {
		result = bson_decoder_next(decoder);
	} while (result > 0);
}

int
test_bson(int argc, char *argv[])
{
	bson_encoder_t encoder;
	bson_decoder_t decoder;
	void *buf;
	int len, fd;

	/* encode data to a memory buffer */
	if (bson_encoder_init_buf(&encoder, NULL, 0))
		errx("FAIL: bson_encoder_init_buf");
	encode(encoder);
	len = bson_encoder_buf_size(encoder);
	if (len <= 0)
		errx("FAIL: bson_encoder_buf_len");
	buf = bson_encoder_buf_data(encoder);

	/* now test-decode it */
	if (bson_decoder_init_buf(&decoder, buf, len))
		errx("FAIL: bson_decoder_init_buf");
	decode(decoder);
	free(buf);

	exit(0);
}