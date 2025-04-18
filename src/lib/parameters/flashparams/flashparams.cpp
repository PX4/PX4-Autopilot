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
 * @file flashparams.cpp
 *
 * Global flash based parameter store.
 *
 * This provides the mechanisms to interface to the PX4
 * parameter system but replace the IO with non file based flash
 * i/o routines. So that the code my be implemented on a SMALL memory
 * foot print device.
 */

#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/shutdown.h>
#include <parameters/px4_parameters.hpp>

#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <errno.h>

#include <parameters/param.h>

#include <lib/tinybson/tinybson.h>
#include "flashparams.h"
#include "flashfs.h"
#include "../param_translation.h"

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
};

static int
param_export_internal(param_filter_func filter)
{
	bson_encoder_s encoder{};
	int     result = -1;

	/* Use realloc */

	bson_encoder_init_buf(&encoder, nullptr, 0);
	auto changed_params = user_config.containedAsBitset();

	for (param_t param = 0; param < user_config.PARAM_COUNT; param++) {

		if (!changed_params[param] || (filter && !filter(param))) {
			continue;
		}

		int32_t i;
		float   f;

		/* append the appropriate BSON type object */

		switch (param_type(param)) {
		case PARAM_TYPE_INT32:
			i = user_config.get(param).i;

			if (bson_encoder_append_int32(&encoder, param_name(param), i)) {
				//debug("BSON append failed for '%s'", param_name(s->param));
				goto out;
			}

			break;

		case PARAM_TYPE_FLOAT:
			f = user_config.get(param).f;

			if (bson_encoder_append_double(&encoder, param_name(param), (double)f)) {
				//debug("BSON append failed for '%s'", param_name(s->param));
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


static int
param_import_callback(bson_decoder_t decoder, bson_node_t node)
{
	float f;
	int32_t i;
	void *v = nullptr;
	int result = -1;

	/*
	 * EOO means the end of the parameter object. (Currently not supporting
	 * nested BSON objects).
	 */
	if (node->type == BSON_EOO) {
		debug("end of parameters");
		return 0;
	}

	if (param_modify_on_import(node) == param_modify_on_import_ret::PARAM_SKIP_IMPORT) {
		return 1;
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
			PX4_WARN("unexpected type for %s", node->name);
			result = 1; // just skip this entry
			goto out;
		}

		i = node->i32;
		v = &i;
		break;

	case BSON_DOUBLE:
		if (param_type(param) != PARAM_TYPE_FLOAT) {
			PX4_WARN("unexpected type for %s", node->name);
			result = 1; // just skip this entry
			goto out;
		}

		f = node->d;
		v = &f;
		break;

	default:
		PX4_ERR("%s unrecognised node type %d", node->name, node->type);
		result = 1; // just skip this entry
		goto out;
	}

	if (param_set_external(param, v, true, true)) {
		debug("error setting value for '%s'", node->name);
		goto out;
	}

	/* don't return zero, that means EOF */
	result = 1;

out:
	return result;
}

static int
param_import_internal()
{
	bson_decoder_s decoder{};
	int result = -1;

	uint8_t *buffer = 0;
	size_t buf_size;
	parameter_flashfs_read(parameters_token, &buffer, &buf_size);

	if (bson_decoder_init_buf(&decoder, buffer, buf_size, param_import_callback)) {
		debug("decoder init failed");
		goto out;
	}

	do {
		result = bson_decoder_next(&decoder);

	} while (result > 0);

out:

	if (result < 0) {
		debug("BSON error decoding parameters");
	}

	return result;
}

int flash_param_save(param_filter_func filter)
{
	return param_export_internal(filter);
}

int flash_param_load()
{
	param_reset_all();
	return param_import_internal();
}

int flash_param_import()
{
	return param_import_internal();
}
