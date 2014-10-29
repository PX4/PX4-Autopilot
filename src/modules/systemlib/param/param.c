/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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
 * @file param.c
 *
 * Global parameter store.
 *
 * Note that it might make sense to convert this into a driver.  That would
 * offer some interesting options regarding state for e.g. ORB advertisements
 * and background parameter saving.
 */

#include <debug.h>
#include <string.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <systemlib/err.h>
#include <errno.h>
#include <semaphore.h>

#include <sys/stat.h>

#include <drivers/drv_hrt.h>

#include "systemlib/param/param.h"
#include "systemlib/uthash/utarray.h"
#include "systemlib/bson/tinybson.h"

#include "uORB/uORB.h"
#include "uORB/topics/parameter_update.h"

#if 0
# define debug(fmt, args...)		do { warnx(fmt, ##args); } while(0)
#else
# define debug(fmt, args...)		do { } while(0)
#endif

/**
 * Array of static parameter info.
 */
extern char __param_start, __param_end;
static const struct param_info_s	*param_info_base = (struct param_info_s *) &__param_start;
static const struct param_info_s	*param_info_limit = (struct param_info_s *) &__param_end;
#define	param_info_count		((unsigned)(param_info_limit - param_info_base))

/**
 * Storage for modified parameters.
 */
struct param_wbuf_s {
	param_t			param;
	union param_value_u	val;
	bool			unsaved;
};

/** flexible array holding modified parameter values */
UT_array	*param_values;

/** array info for the modified parameters array */
const UT_icd	param_icd = {sizeof(struct param_wbuf_s), NULL, NULL, NULL};

/** parameter update topic */
ORB_DEFINE(parameter_update, struct parameter_update_s);

/** parameter update topic handle */
static orb_advert_t param_topic = -1;

/** lock the parameter store */
static void
param_lock(void)
{
	//do {} while (sem_wait(&param_sem) != 0);
}

/** unlock the parameter store */
static void
param_unlock(void)
{
	//sem_post(&param_sem);
}

/** assert that the parameter store is locked */
static void
param_assert_locked(void)
{
	/* XXX */
}

/**
 * Test whether a param_t is value.
 *
 * @param param			The parameter handle to test.
 * @return			True if the handle is valid.
 */
static bool
handle_in_range(param_t param)
{
	return (param < param_info_count);
}

/**
 * Compare two modifid parameter structures to determine ordering.
 *
 * This function is suitable for passing to qsort or bsearch.
 */
static int
param_compare_values(const void *a, const void *b)
{
	struct param_wbuf_s *pa = (struct param_wbuf_s *)a;
	struct param_wbuf_s *pb = (struct param_wbuf_s *)b;

	if (pa->param < pb->param)
		return -1;

	if (pa->param > pb->param)
		return 1;

	return 0;
}

/**
 * Locate the modified parameter structure for a parameter, if it exists.
 *
 * @param param			The parameter being searched.
 * @return			The structure holding the modified value, or
 *				NULL if the parameter has not been modified.
 */
static struct param_wbuf_s *
param_find_changed(param_t param) {
	struct param_wbuf_s	*s = NULL;

	param_assert_locked();

	if (param_values != NULL) {
#if 0	/* utarray_find requires bsearch, not available */
		struct param_wbuf_s key;
		key.param = param;
		s = utarray_find(param_values, &key, param_compare_values);
#else

		while ((s = (struct param_wbuf_s *)utarray_next(param_values, s)) != NULL) {
			if (s->param == param)
				break;
		}

#endif
	}

	return s;
}

static void
param_notify_changes(void)
{
	struct parameter_update_s pup = { .timestamp = hrt_absolute_time() };

	/*
	 * If we don't have a handle to our topic, create one now; otherwise
	 * just publish.
	 */
	if (param_topic == -1) {
		param_topic = orb_advertise(ORB_ID(parameter_update), &pup);

	} else {
		orb_publish(ORB_ID(parameter_update), param_topic, &pup);
	}
}

param_t
param_find(const char *name)
{
	param_t param;

	/* perform a linear search of the known parameters */
	for (param = 0; handle_in_range(param); param++) {
		if (!strcmp(param_info_base[param].name, name))
			return param;
	}

	/* not found */
	return PARAM_INVALID;
}

unsigned
param_count(void)
{
	return param_info_count;
}

param_t
param_for_index(unsigned index)
{
	if (index < param_info_count)
		return (param_t)index;

	return PARAM_INVALID;
}

int
param_get_index(param_t param)
{
	if (handle_in_range(param))
		return (unsigned)param;

	return -1;
}

const char *
param_name(param_t param)
{
	if (handle_in_range(param))
		return param_info_base[param].name;

	return NULL;
}

bool
param_value_is_default(param_t param)
{
	return param_find_changed(param) ? false : true;
}

bool
param_value_unsaved(param_t param)
{
	static struct param_wbuf_s *s;

	s = param_find_changed(param);

	if (s && s->unsaved)
		return true;

	return false;
}

enum param_type_e
param_type(param_t param)
{
	if (handle_in_range(param))
		return param_info_base[param].type;

	return PARAM_TYPE_UNKNOWN;
}

size_t
param_size(param_t param)
{
	if (handle_in_range(param)) {
		switch (param_type(param)) {
		case PARAM_TYPE_INT32:
		case PARAM_TYPE_FLOAT:
			return 4;

		case PARAM_TYPE_STRUCT ... PARAM_TYPE_STRUCT_MAX:
			/* decode structure size from type value */
			return param_type(param) - PARAM_TYPE_STRUCT;

		default:
			return 0;
		}
	}

	return 0;
}

/**
 * Obtain a pointer to the storage allocated for a parameter.
 *
 * @param param			The parameter whose storage is sought.
 * @return			A pointer to the parameter value, or NULL
 *				if the parameter does not exist.
 */
static const void *
param_get_value_ptr(param_t param)
{
	const void *result = NULL;

	param_assert_locked();

	if (handle_in_range(param)) {

		const union param_value_u *v;

		/* work out whether we're fetching the default or a written value */
		struct param_wbuf_s *s = param_find_changed(param);

		if (s != NULL) {
			v = &s->val;

		} else {
			v = &param_info_base[param].val;
		}

		if (param_type(param) >= PARAM_TYPE_STRUCT
				&& param_type(param) <= PARAM_TYPE_STRUCT_MAX) {
			result = v->p;

		} else {
			result = v;
		}
	}

	return result;
}

int
param_get(param_t param, void *val)
{
	int result = -1;

	param_lock();

	const void *v = param_get_value_ptr(param);

	if (val != NULL) {
		memcpy(val, v, param_size(param));
		result = 0;
	}

	param_unlock();

	return result;
}

static int
param_set_internal(param_t param, const void *val, bool mark_saved)
{
	int result = -1;
	bool params_changed = false;

	param_lock();

	if (param_values == NULL)
		utarray_new(param_values, &param_icd);

	if (param_values == NULL) {
		debug("failed to allocate modified values array");
		goto out;
	}

	if (handle_in_range(param)) {

		struct param_wbuf_s *s = param_find_changed(param);

		if (s == NULL) {

			/* construct a new parameter */
			struct param_wbuf_s buf = {
				.param = param,
				.val.p = NULL,
				.unsaved = false
			};

			/* add it to the array and sort */
			utarray_push_back(param_values, &buf);
			utarray_sort(param_values, param_compare_values);

			/* find it after sorting */
			s = param_find_changed(param);
		}

		/* update the changed value */
		switch (param_type(param)) {
		case PARAM_TYPE_INT32:
			s->val.i = *(int32_t *)val;
			break;

		case PARAM_TYPE_FLOAT:
			s->val.f = *(float *)val;
			break;

		case PARAM_TYPE_STRUCT ... PARAM_TYPE_STRUCT_MAX:
			if (s->val.p == NULL) {
				s->val.p = malloc(param_size(param));

				if (s->val.p == NULL) {
					debug("failed to allocate parameter storage");
					goto out;
				}
			}

			memcpy(s->val.p, val, param_size(param));
			break;

		default:
			goto out;
		}

		s->unsaved = !mark_saved;
		params_changed = true;
		result = 0;
	}

out:
	param_unlock();

	/*
	 * If we set something, now that we have unlocked, go ahead and advertise that
	 * a thing has been set.
	 */
	if (params_changed)
		param_notify_changes();

	return result;
}

int
param_set(param_t param, const void *val)
{
	return param_set_internal(param, val, false);
}

void
param_reset(param_t param)
{
	struct param_wbuf_s *s = NULL;

	param_lock();

	if (handle_in_range(param)) {

		/* look for a saved value */
		s = param_find_changed(param);

		/* if we found one, erase it */
		if (s != NULL) {
			int pos = utarray_eltidx(param_values, s);
			utarray_erase(param_values, pos, 1);
		}
	}

	param_unlock();

	if (s != NULL)
		param_notify_changes();
}

void
param_reset_all(void)
{
	param_lock();

	if (param_values != NULL) {
		utarray_free(param_values);
	}

	/* mark as reset / deleted */
	param_values = NULL;

	param_unlock();

	param_notify_changes();
}

static const char *param_default_file = "/eeprom/parameters";
static char *param_user_file = NULL;

int
param_set_default_file(const char* filename)
{
	if (param_user_file != NULL) {
		free(param_user_file);
		param_user_file = NULL;
	}
	if (filename)
		param_user_file = strdup(filename);
	return 0;
}

const char *
param_get_default_file(void)
{
	return (param_user_file != NULL) ? param_user_file : param_default_file;
}

int
param_save_default(void)
{
	int res;
	int fd;

	const char *filename = param_get_default_file();

	/* write parameters to temp file */
	fd = open(filename, O_WRONLY | O_CREAT);

	if (fd < 0) {
		warn("failed to open param file: %s", filename);
		return ERROR;
	}

	res = param_export(fd, false);

	if (res != OK) {
		warnx("failed to write parameters to file: %s", filename);
	}

	close(fd);

	return res;
}

/**
 * @return 0 on success, 1 if all params have not yet been stored, -1 if device open failed, -2 if writing parameters failed
 */
int
param_load_default(void)
{
	int fd_load = open(param_get_default_file(), O_RDONLY);

	if (fd_load < 0) {
		/* no parameter file is OK, otherwise this is an error */
		if (errno != ENOENT) {
			warn("open '%s' for reading failed", param_get_default_file());
			return -1;
		}
		return 1;
	}

	int result = param_load(fd_load);
	close(fd_load);

	if (result != 0) {
		warn("error reading parameters from '%s'", param_get_default_file());
		return -2;
	}

	return 0;
}

int
param_export(int fd, bool only_unsaved)
{
	struct param_wbuf_s *s = NULL;
	struct bson_encoder_s encoder;
	int	result = -1;

	param_lock();

	bson_encoder_init_file(&encoder, fd);

	/* no modified parameters -> we are done */
	if (param_values == NULL) {
		result = 0;
		goto out;
	}

	while ((s = (struct param_wbuf_s *)utarray_next(param_values, s)) != NULL) {

		int32_t	i;
		float	f;

		/*
		 * If we are only saving values changed since last save, and this
		 * one hasn't, then skip it
		 */
		if (only_unsaved && !s->unsaved)
			continue;

		s->unsaved = false;

		/* append the appropriate BSON type object */
		switch (param_type(s->param)) {
		case PARAM_TYPE_INT32:
			param_get(s->param, &i);

			if (bson_encoder_append_int(&encoder, param_name(s->param), i)) {
				debug("BSON append failed for '%s'", param_name(s->param));
				goto out;
			}

			break;

		case PARAM_TYPE_FLOAT:
			param_get(s->param, &f);

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
						       param_get_value_ptr(s->param))) {
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
	param_unlock();

	if (result == 0)
		result = bson_encoder_fini(&encoder);

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
	param_t param = param_find(node->name);

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

	if (param_set_internal(param, v, state->mark_saved)) {
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

	if (tmp != NULL)
		free(tmp);

	return result;
}

static int
param_import_internal(int fd, bool mark_saved)
{
	struct bson_decoder_s decoder;
	int result = -1;
	struct param_import_state state;

	if (bson_decoder_init_file(&decoder, fd, param_import_callback, &state)) {
		debug("decoder init failed");
		goto out;
	}

	state.mark_saved = mark_saved;

	do {
		result = bson_decoder_next(&decoder);

	} while (result > 0);

out:

	if (result < 0)
		debug("BSON error decoding parameters");

	return result;
}

int
param_import(int fd)
{
	return param_import_internal(fd, false);
}

int
param_load(int fd)
{
	param_reset_all();
	return param_import_internal(fd, true);
}

void
param_foreach(void (*func)(void *arg, param_t param), void *arg, bool only_changed)
{
	param_t	param;

	for (param = 0; handle_in_range(param); param++) {

		/* if requested, skip unchanged values */
		if (only_changed && (param_find_changed(param) == NULL))
			continue;

		func(arg, param);
	}
}
