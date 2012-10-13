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
 * @file tinybson.c
 *
 * A simple subset SAX-style BSON parser and generator.
 */

#include <unistd.h>
#include <string.h>
#include <err.h>

#include "tinybson.h"


#if 0
# define debug(fmt, args...)		do { warnx("BSON: " fmt, ##args); } while(0)
#else
# define debug(fmt, args...)		do { } while(0)
#endif

#define CODER_CHECK(_c)		do { if (_c->fd == -1) return -1; } while(0)
#define CODER_KILL(_c, _reason)	do { debug("killed: %s", _reason); _c->fd = -1; return -1; } while(0)

static int
read_int8(bson_decoder_t decoder, int8_t *b)
{
	return (read(decoder->fd, b, sizeof(*b)) == sizeof(*b)) ? 0 : -1;	
}

static int
read_int32(bson_decoder_t decoder, int32_t *i)
{
	return (read(decoder->fd, i, sizeof(*i)) == sizeof(*i)) ? 0 : -1;
}

static int
read_double(bson_decoder_t decoder, double *d)
{
	return (read(decoder->fd, d, sizeof(*d)) == sizeof(*d)) ? 0 : -1;
}

int
bson_decoder_init(bson_decoder_t decoder, int fd, bson_decoder_callback callback, void *private)
{
	int32_t	junk;

	decoder->fd = fd;
	decoder->callback = callback;
	decoder->private = private;
	decoder->nesting = 1;
	decoder->pending = 0;
	decoder->node.type = BSON_UNDEFINED;

	/* read and discard document size */
	if (read_int32(decoder, &junk))
		CODER_KILL(decoder, "failed reading length");

	/* ready for decoding */
	return 0;
}

int
bson_decoder_next(bson_decoder_t decoder)
{
	int8_t	tbyte;
	unsigned nlen;

	CODER_CHECK(decoder);

	/* if the previous node was EOO, pop a nesting level */
	if (decoder->node.type == BSON_EOO) {
		if (decoder->nesting > 0)
			decoder->nesting--;

		/* if the nesting level is now zero, the top-level document is done */
		if (decoder->nesting == 0) {
			/* like kill but not an error */
			debug("nesting is zero, document is done");
			decoder->fd = -1;

			/* return end-of-file to the caller */
			return 0;
		}
	}

	/* if there are unread bytes pending in the stream, discard them */
	while (decoder->pending > 0) {
		if (read_int8(decoder, &tbyte))
			CODER_KILL(decoder, "read error discarding pending bytes");
		decoder->pending--;
	}

	/* get the type byte */
	if (read_int8(decoder, &tbyte))
		CODER_KILL(decoder, "read error on type byte");
	decoder->node.type = tbyte;
	decoder->pending = 0;

	debug("got type byte 0x%02x", decoder->node.type);

	/* EOO is special; it has no name/data following */
	if (decoder->node.type != BSON_EOO) {

		/* get the node name */
		nlen = 0;
		for (;;) {
			if (nlen >= BSON_MAXNAME)
				CODER_KILL(decoder, "node name overflow");
			if (read_int8(decoder, (int8_t *)&decoder->node.name[nlen]))
				CODER_KILL(decoder, "read error on node name");
			if (decoder->node.name[nlen] == '\0')
				break;
			nlen++;
		}

		debug("got name '%s'", decoder->node.name);

		switch (decoder->node.type) {
		case BSON_INT:
			if (read_int32(decoder, &decoder->node.i))
				CODER_KILL(decoder, "read error on BSON_INT");
			break;
		case BSON_DOUBLE:
			if (read_double(decoder, &decoder->node.d))
				CODER_KILL(decoder, "read error on BSON_DOUBLE");
			break;
		case BSON_STRING:
			if (read_int32(decoder, &decoder->pending))
				CODER_KILL(decoder, "read error on BSON_STRING length");
			break;
		case BSON_BINDATA:
			if (read_int32(decoder, &decoder->pending))
				CODER_KILL(decoder, "read error on BSON_BINDATA size");
			if (read_int8(decoder, &tbyte))
				CODER_KILL(decoder, "read error on BSON_BINDATA subtype");
			decoder->node.subtype = tbyte;
			break;

			/* XXX currently not supporting other types */
		default:
			CODER_KILL(decoder, "unsupported node type");
		}
	}

	/* call the callback and pass its results back */
	return decoder->callback(decoder, decoder->private, &decoder->node);
}

int
bson_decoder_copy_data(bson_decoder_t decoder, void *buf)
{
	int result;

	CODER_CHECK(decoder);

	/* if data already copied, return zero bytes */
	if (decoder->pending == 0) 
		return 0;

	/* copy bytes per the node size */
	result = read(decoder->fd, buf, decoder->pending);
	if (result != decoder->pending)
		CODER_KILL(decoder, "read error on copy_data");

	/* pending count is discharged */
	decoder->pending = 0;
	return 0;
}

size_t
bson_decoder_data_pending(bson_decoder_t decoder)
{
	return decoder->pending;
}

static int
write_int8(bson_encoder_t encoder, int8_t b)
{
	debug("write_int8 %d", b);
	return (write(encoder->fd, &b, sizeof(b)) == sizeof(b)) ? 0 : -1;	
}

static int
write_int32(bson_encoder_t encoder, int32_t i)
{
	debug("write_int32 %d", i);
	return (write(encoder->fd, &i, sizeof(i)) == sizeof(i)) ? 0 : -1;
}

static int
write_double(bson_encoder_t encoder, double d)
{
	debug("write_double");
	return (write(encoder->fd, &d, sizeof(d)) == sizeof(d)) ? 0 : -1;
}

static int
write_name(bson_encoder_t encoder, const char *name)
{
	size_t len = strlen(name);

	if (len > BSON_MAXNAME)
		return -1;
	debug("write name '%s' len %d", name, len);
	return (write(encoder->fd, name, len + 1) == (int)(len + 1)) ? 0 : -1;
}

int
bson_encoder_init(bson_encoder_t encoder, int fd)
{
	encoder->fd = fd;

	if (write_int32(encoder, 0))
		CODER_KILL(encoder, "write error on document length");

	return 0;
}

int
bson_encoder_fini(bson_encoder_t encoder)
{
	CODER_CHECK(encoder);

	if (write_int8(encoder, BSON_EOO))
		CODER_KILL(encoder, "write error on document terminator");

	return 0;
}

int
bson_encoder_append_int(bson_encoder_t encoder, const char *name, int32_t value)
{
	CODER_CHECK(encoder);

	if (write_int8(encoder, BSON_INT) ||
	    write_name(encoder, name) ||
	    write_int32(encoder, value))
		CODER_KILL(encoder, "write error on BSON_INT");

	return 0;
}

int
bson_encoder_append_double(bson_encoder_t encoder, const char *name, double value)
{
	CODER_CHECK(encoder);

	if (write_int8(encoder, BSON_DOUBLE) ||
	    write_name(encoder, name) ||
	    write_double(encoder, value))
		CODER_KILL(encoder, "write error on BSON_DOUBLE");


	return 0;
}

int
bson_encoder_append_string(bson_encoder_t encoder, const char *name, const char *string)
{
	size_t len;

	CODER_CHECK(encoder);

	len = strlen(string);

	if (write_int8(encoder, BSON_DOUBLE) ||
	    write_name(encoder, name) ||
	    write_int32(encoder, len) ||
	    write(encoder->fd, name, len + 1) != (int)(len + 1))
		CODER_KILL(encoder, "write error on BSON_STRING");
	return 0;
}

int
bson_encoder_append_binary(bson_encoder_t encoder, const char *name, bson_binary_subtype_t subtype, size_t size, const void *data)
{
	CODER_CHECK(encoder);

	if (write_int8(encoder, BSON_BINDATA) ||
	    write_name(encoder, name) ||
	    write_int32(encoder, size) ||
	    write_int8(encoder, subtype) ||
	    write(encoder->fd, data, size) != (int)(size))
		CODER_KILL(encoder, "write error on BSON_BINDATA");
	return 0;
}
