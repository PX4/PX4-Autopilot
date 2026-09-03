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

/*
 * Same-bank program/erase stalls instruction fetch on STM32H7. A full
 * snapshot rewrite is tens of ms; a 128 KB sector erase is ~1 s. In-service
 * saves therefore append a delta of unsaved params (a few 32-byte rows).
 * The sector is compacted back to one snapshot at boot, before sensors
 * or DShot start.
 */

static bool
any_unsaved(param_filter_func filter)
{
	for (param_t param = 0; param < user_config.PARAM_COUNT; param++) {
		if ((filter == nullptr || filter(param)) && param_value_unsaved(param)) {
			return true;
		}
	}

	return false;
}

static int
encode_params(bson_encoder_s *encoder, param_filter_func filter, bool snapshot)
{
	for (param_t param = 0; param < user_config.PARAM_COUNT; param++) {
		if (filter && !filter(param)) {
			continue;
		}

		if (snapshot) {
			if (!user_config.contains(param) || param_value_is_default(param)) {
				continue;
			}

		} else if (!param_value_unsaved(param)) {
			continue;
		}

		switch (param_type(param)) {
		case PARAM_TYPE_INT32:
			if (bson_encoder_append_int32(encoder, param_name(param), user_config.get(param).i)) {
				debug("BSON append failed for '%s'", param_name(param));
				return -1;
			}

			break;

		case PARAM_TYPE_FLOAT:
			if (bson_encoder_append_double(encoder, param_name(param), (double)user_config.get(param).f)) {
				debug("BSON append failed for '%s'", param_name(param));
				return -1;
			}

			break;

		default:
			debug("unrecognized parameter type");
			return -1;
		}
	}

	return 0;
}

static int
write_encoded(bson_encoder_s *encoder)
{
	void *enc_buff = bson_encoder_buf_data(encoder);
	size_t enc_size = bson_encoder_buf_size(encoder);

	if (enc_buff == nullptr) {
		return -ENOMEM;
	}

	size_t alloc_size = enc_size;
	uint8_t *buffer;
	int result = parameter_flashfs_alloc(parameters_token, &buffer, &alloc_size);

	if (result != OK) {
		return result;
	}

	memcpy(buffer, enc_buff, enc_size);
	result = parameter_flashfs_write(parameters_token, buffer, enc_size);
	parameter_flashfs_free();

	if (result == (int)enc_size) {
		return OK;
	}

	return result < 0 ? result : -EFBIG;
}

static int
export_and_write(param_filter_func filter, bool snapshot)
{
	bson_encoder_s encoder{};
	bson_encoder_init_buf(&encoder, nullptr, 0);

	int result = encode_params(&encoder, filter, snapshot);

	if (result != 0) {
		free(bson_encoder_buf_data(&encoder));
		return result;
	}

	bson_encoder_fini(&encoder);
	result = write_encoded(&encoder);
	free(bson_encoder_buf_data(&encoder));
	return result;
}

static int
compact_to_snapshot(param_filter_func filter)
{
	if (parameter_flashfs_blank() != 1) {
		int rv = parameter_flashfs_erase();

		if (rv < 0) {
			return rv;
		}
	}

	return export_and_write(filter, true);
}

static int
param_export_internal(param_filter_func filter)
{
	if (any_unsaved(filter)) {
		int result = export_and_write(filter, false);

		if (result != -ENOSPC) {
			return result;
		}

		/* Same-bank erase stalls the CPU for ~1 s; only take that path when
		 * the log no longer fits. Compact at boot is meant to keep this rare. */
		PX4_WARN("flashparams: param sector full, compacting");
		return compact_to_snapshot(filter);
	}

	/* Nothing unsaved. Still persist a snapshot if the store is empty so
	 * load-or-init does not treat a first-boot save as blank. */
	uint8_t *existing = nullptr;
	size_t existing_size = 0;
	int read_result = parameter_flashfs_read(parameters_token, &existing, &existing_size);

	if (read_result >= 0) {
		return 0;
	}

	return compact_to_snapshot(filter);
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

	/* A delta tombstone stores the default so replay overrides an earlier
	 * snapshot. Drop it from user_config so the next snapshot omits it. */
	if (param_value_is_default(param)) {
		user_config.reset(param);
	}

	/* don't return zero, that means EOF */
	result = 1;

out:
	return result;
}

static int
import_one_entry(uint8_t *buffer, size_t buf_size, void *arg)
{
	int *result = static_cast<int *>(arg);
	bson_decoder_s decoder{};

	if (bson_decoder_init_buf(&decoder, buffer, buf_size, param_import_callback)) {
		debug("decoder init failed");
		*result = -1;
		return -1;
	}

	do {
		*result = bson_decoder_next(&decoder);

	} while (*result > 0);

	return *result < 0 ? *result : 0;
}

static int
param_import_internal()
{
	int result = 0;
	int n = parameter_flashfs_walk(parameters_token, import_one_entry, &result);

	if (n < 0) {
		debug("flash walk failed (%d)", n);
		return n;
	}

	if (n == 0) {
		/* No valid entry found. A fully erased store is blank (first boot, or
		 * after switching firmware): report "not yet stored" (1), matching the
		 * convention used by param_load_default(). A store that holds data but
		 * no valid entry (torn write, bit-rot, or foreign data) is corrupt and
		 * must not be silently reseeded - report it so the boot recovery runs. */
		if (parameter_flashfs_blank() == 1) {
			return 1;
		}

		debug("flash holds data but no valid entry");
		return -EILSEQ;
	}

	if (result < 0) {
		debug("BSON error decoding parameters");
		return result;
	}

	return 0;
}

static int
compact_after_import()
{
	int need = parameter_flashfs_needs_compact();

	if (need <= 0) {
		return need;
	}

	/* Same-bank erase stalls instruction fetch for ~1 s on H7. Do it here
	 * at boot, before sensors or DShot start, so in-service saves stay
	 * append-only (a few 32-byte rows for a delta of unsaved params). */
	PX4_INFO("flashparams: compacting param sector");
	int rv = compact_to_snapshot(nullptr);

	if (rv != 0) {
		PX4_ERR("flashparams: compact failed (%d)", rv);
	}

	return rv;
}

int flash_param_save(param_filter_func filter)
{
	return param_export_internal(filter);
}

int flash_param_load()
{
	param_reset_all();
	int result = param_import_internal();

	if (result == 0) {
		compact_after_import();
	}

	return result;
}

int flash_param_import()
{
	int result = param_import_internal();

	if (result == 0) {
		compact_after_import();
	}

	return result;
}
