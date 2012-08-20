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
 * @file tinybson.h
 *
 * A simple subset SAX-style BSON parser and generator.
 *
 * Some types and defines taken from the standalone BSON parser/generator 
 * in the Mongo C connector.
 */

#ifndef _TINYBSON_H
#define _TINYBSON_H

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

/** subset of the BSON node types we might care about */
typedef enum {
	BSON_EOO = 0,
	BSON_DOUBLE = 1,
	BSON_STRING = 2,
	BSON_OBJECT = 3,
	BSON_ARRAY = 4,
	BSON_BINDATA = 5,
	BSON_UNDEFINED = 6,
	BSON_BOOL = 8,
	BSON_DATE = 9,
	BSON_NULL = 10,
	BSON_INT = 16,
	BSON_TIMESTAMP = 17,
	BSON_LONG = 18
} bson_type_t;

typedef enum bson_binary_subtype {
	BSON_BIN_BINARY = 0,
	BSON_BIN_USER = 128
} bson_binary_subtype_t;

/**
 * Maximum node name length.
 */
#define BSON_MAXNAME		32

/**
 * Node structure passed to the callback.
 */
typedef struct bson_node_s
{
	char			name[BSON_MAXNAME];
	bson_type_t		type;
	bson_binary_subtype_t	subtype;
	union {
		int32_t		i;
		double		d;
		bool		b;
	};
} *bson_node_t;

typedef struct bson_decoder_s *bson_decoder_t;

/**
 * Node callback.
 */
typedef int	(* bson_decoder_callback)(bson_decoder_t decoder, void *private, bson_node_t node);

struct bson_decoder_s
{
	int			fd;
	bson_decoder_callback	callback;
	void			*private;
	unsigned		nesting;
	struct bson_node_s	node;
	int32_t			pending;
};

/**
 * Initialise the decoder.
 */
__EXPORT int bson_decoder_init(bson_decoder_t decoder, int fd, bson_decoder_callback callback, void *private);

/**
 * Process the next node from the stream and invoke the callback.
 */
__EXPORT int bson_decoder_next(bson_decoder_t decoder);

/**
 * Copy node data.
 */
__EXPORT int bson_decoder_copy_data(bson_decoder_t decoder, void *buf);

/**
 * Report copyable data size.
 */
__EXPORT size_t bson_decoder_data_pending(bson_decoder_t decoder);

/**
 * Encoder state structure.
 */
typedef struct bson_encoder_s
{
	int		fd;

} *bson_encoder_t;

/**
 * Initialze the encoder.
 */
__EXPORT int bson_encoder_init(bson_encoder_t encoder, int fd);

/**
 * Finalise the encoded stream.
 */
__EXPORT int bson_encoder_fini(bson_encoder_t encoder);

/**
 * Append an integer to the encoded stream.
 */
__EXPORT int bson_encoder_append_int(bson_encoder_t encoder, const char *name, int32_t value);

/**
 * Append a double to the encoded stream
 */
__EXPORT int bson_encoder_append_double(bson_encoder_t encoder, const char *name, double value);

/** 
 * Append a string to the encoded stream.
 */
__EXPORT int bson_encoder_append_string(bson_encoder_t encoder, const char *name, const char *string);

/**
 * Append a binary blob to the encoded stream.
 */
__EXPORT int bson_encoder_append_binary(bson_encoder_t encoder, const char *name, bson_binary_subtype_t subtype, size_t size, const void *data);


#endif
