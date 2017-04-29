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

/**
 * @file flashparam.c
 *
 * Global flash based parameter store.
 *
 * This provides the mechanisms to interface to the PX4
 * parameter system but replace the IO with non file based flash
 * i/o routines. So that the code my be implemented on a SMALL memory
 * foot print device.
 */

#include <px4_defines.h>
#include <px4_posix.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "systemlib/param/param.h"
#include "systemlib/uthash/utarray.h"
#include "systemlib/bson/tinybson.h"
#include "flashparams.h"
#include "flashfs.h"

#if 0
# define debug(fmt, args...)            do { warnx(fmt, ##args); } while(0)
#else
# define debug(fmt, args...)            do { } while(0)
#endif


/**
 * Storage for modified parameters.
 */
struct param_wbuf_s {
	union param_value_u     val;
	param_t                 param;
	bool                    unsaved;
};

static int
param_export_internal(bool only_unsaved)
{
	struct param_wbuf_s *s = NULL;
	struct bson_encoder_s encoder;
	int     result = -1;

	/* Use realloc */

	bson_encoder_init_buf(&encoder, NULL, 0);

	/* no modified parameters -> we are done */
	if (param_values == NULL) {
		result = 0;
		goto out;
	}

	while ((s = (struct param_wbuf_s *)utarray_next(param_values, s)) != NULL) {

		int32_t i;
		float   f;

		/*
		 * If we are only saving values changed since last save, and this
		 * one hasn't, then skip it
		 */
		if (only_unsaved && !s->unsaved) {
			continue;
		}

		s->unsaved = false;

		/* append the appropriate BSON type object */

		switch (param_type(s->param)) {

		case PARAM_TYPE_INT32:
			i = s->val.i;

			if (bson_encoder_append_int(&encoder, param_name(s->param), i)) {
				debug("BSON append failed for '%s'", param_name(s->param));
				goto out;
			}

			break;

		case PARAM_TYPE_FLOAT:
			f = s->val.f;

			if (bson_encoder_append_double(&encoder, param_name(s->param), f)) {
				debug("BSON append failed for '%s'", param_name(s->param));
				goto out;
			}

			break;

		case PARAM_TYPE_STRUCT ... PARAM_TYPE_STRUCT_MAX:
			if (bson_encoder_append_binary(&encoder,
						       param_name(s->param),
						       BSON_BIN_BINARY,
						       param_size(s->param),
						       param_get_value_ptr_external(s->param))) {
				debug("BSON append failed for '%s'", param_name(s->param));
				goto out;
			}

			break;

		default:
			debug("unrecognized parameter type");
			goto out;
		}
	}

	result = 0;

out:

	if (result == 0) {

		/* Finalize the bison encoding*/

		bson_encoder_fini(&encoder);

		/* Get requiered space */

		size_t buf_size = bson_encoder_buf_size(&encoder);

		/* Get a buffer from the flash driver with enough space */

		uint8_t *buffer;
		result = parameter_flashfs_alloc(parameters_token, &buffer, &buf_size);

		if (result == OK) {

			/* Check for a write that has no changes */

			uint8_t *was_buffer;
			size_t was_buf_size;
			int was_result = parameter_flashfs_read(parameters_token, &was_buffer, &was_buf_size);

			void *enc_buff = bson_encoder_buf_data(&encoder);

			bool commit = was_result < OK || was_buf_size != buf_size || 0 != memcmp(was_buffer, enc_buff, was_buf_size);

			if (commit) {

				memcpy(buffer, enc_buff, buf_size);
				result = parameter_flashfs_write(parameters_token, buffer, buf_size);
				result = result == buf_size ? OK : -EFBIG;

			}

			free(enc_buff);
			parameter_flashfs_free();
		}
	}

	return result;
}

struct param_import_state {
	bool mark_saved;
};

static int
param_import_callback(bson_decoder_t decoder, void *private, bson_node_t node)
{
	float f;
	int32_t i;
	void *v, *tmp = NULL;
	int result = -1;
	struct param_import_state *state = (struct param_import_state *)private;

	/*
	 * EOO means the end of the parameter object. (Currently not supporting
	 * nested BSON objects).
	 */
	if (node->type == BSON_EOO) {
		debug("end of parameters");
		return 0;
	}

	/*
	 * Find the parameter this node represents.  If we don't know it,
	 * ignore the node.
	 */
	param_t param = param_find_no_notification(node->name);

	if (param == PARAM_INVALID) {
		debug("ignoring unrecognised parameter '%s'", node->name);
		return 1;
	}

	/*
	 * Handle setting the parameter from the node
	 */

	switch (node->type) {
	case BSON_INT32:
		if (param_type(param) != PARAM_TYPE_INT32) {
			debug("unexpected type for '%s", node->name);
			goto out;
		}

		i = node->i;
		v = &i;
		break;

	case BSON_DOUBLE:
		if (param_type(param) != PARAM_TYPE_FLOAT) {
			debug("unexpected type for '%s", node->name);
			goto out;
		}

		f = node->d;
		v = &f;
		break;

	case BSON_BINDATA:
		if (node->subtype != BSON_BIN_BINARY) {
			debug("unexpected subtype for '%s", node->name);
			goto out;
		}

		if (bson_decoder_data_pending(decoder) != param_size(param)) {
			debug("bad size for '%s'", node->name);
			goto out;
		}

		/* XXX check actual file data size? */
		tmp = malloc(param_size(param));

		if (tmp == NULL) {
			debug("failed allocating for '%s'", node->name);
			goto out;
		}

		if (bson_decoder_copy_data(decoder, tmp)) {
			debug("failed copying data for '%s'", node->name);
			goto out;
		}

		v = tmp;
		break;

	default:
		debug("unrecognised node type");
		goto out;
	}

	if (param_set_external(param, v, state->mark_saved, true)) {

		debug("error setting value for '%s'", node->name);
		goto out;
	}

	if (tmp != NULL) {
		free(tmp);
		tmp = NULL;
	}

	/* don't return zero, that means EOF */
	result = 1;

out:

	if (tmp != NULL) {
		free(tmp);
	}

	return result;
}

static int
param_import_internal(bool mark_saved)
{
	struct bson_decoder_s decoder;
	int result = -1;
	struct param_import_state state;

	uint8_t *buffer = 0;
	size_t buf_size;
	parameter_flashfs_read(parameters_token, &buffer, &buf_size);

	if (bson_decoder_init_buf(&decoder, buffer, buf_size, param_import_callback, &state)) {
		debug("decoder init failed");
		goto out;
	}

	state.mark_saved = mark_saved;

	do {
		result = bson_decoder_next(&decoder);

	} while (result > 0);

out:

	if (result < 0) {
		debug("BSON error decoding parameters");
	}

	return result;
}

int flash_param_save(void)
{
	return param_export_internal(false);
}



int flash_param_load(void)
{
	param_reset_all();
	return param_import_internal(true);
}

int flash_param_import(void)
{
	return -1;
}
