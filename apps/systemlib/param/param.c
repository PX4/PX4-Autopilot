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
 * @file param.c
 *
 * Global parameter store.
 */

#include <debug.h>
#include <string.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <err.h>

#include <sys/stat.h>

#include "systemlib/param/param.h"
#include "systemlib/uthash/utarray.h"
#include "systemlib/bson/bson.h"

#if 1
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
UT_icd		param_icd = {sizeof(struct param_wbuf_s), NULL, NULL, NULL};

/** lock the parameter store */
static void
param_lock(void)
{
	/* XXX */
}

/** unlock the parameter store */
static void
param_unlock(void)
{
	/* XXX */
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

const char *
param_name(param_t param)
{
	if (handle_in_range(param))
		return param_info_base[param].name;

	return NULL;
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

		if (param_type(param) == PARAM_TYPE_STRUCT) {
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

int
param_set(param_t param, const void *val)
{
	int result = -1;

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

		s->unsaved = true;

		result = 0;
	}

out:
	param_unlock();
	return result;
}

int
param_export(int fd, bool only_unsaved)
{
	struct param_wbuf_s *s = NULL;
	bson	b[1];
	int	result = -1;

	param_lock();

	bson_init(b);

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
		if (only_unsaved & !s->unsaved)
			continue;

		s->unsaved = false;

		/* append the appripriate BSON type object */
		switch (param_type(s->param)) {
		case PARAM_TYPE_INT32:
			param_get(s->param, &i);

			if (bson_append_int(b, param_name(s->param), i) != BSON_OK) {
				debug("BSON append failed for '%s'", param_name(s->param));
				goto out;
			}

			break;

		case PARAM_TYPE_FLOAT:
			param_get(s->param, &f);

			if (bson_append_double(b, param_name(s->param), (double)f) != BSON_OK) {
				debug("BSON append failed for '%s'", param_name(s->param));
				goto out;
			}

			break;

		case PARAM_TYPE_STRUCT ... PARAM_TYPE_STRUCT_MAX:
			if (bson_append_binary(b,
					       param_name(s->param),
					       param_type(s->param),
					       param_get_value_ptr(s->param),
					       param_size(s->param)) != BSON_OK) {
				debug("BSON append failed for '%s'", param_name(s->param));
				goto out;
			}

			break;

		default:
			debug("unrecognised parameter type");
			goto out;
		}
	}

	result = 0;

out:
	param_unlock();

	if (result == 0) {
		/* finalize the object ready for saving */
		bson_finish(b);

		/* print it for debugging purposes only */
		bson_print(b);

		if (bson_buffer_size(b) > PARAM_FILE_MAXSIZE) {
			debug("parameter export file too large");
			result = -1;

		} else if (fd >= 0) {
			int len = write(fd, bson_data(b), bson_buffer_size(b));

			if (len != bson_buffer_size(b)) {
				debug("wriet size mismatch");
				result = -1;
			}
		}
	}

	bson_destroy(b);

	return result;
}

int
param_import(int fd)
{
	bson	b[1];
	bson_iterator iter[1];
	int	result = -1;
	char	*raw = NULL;
	bson_type type;
	const char *name;
	param_t	param;
	ssize_t size;

	/* load the bson object from the parameter file */
	raw = malloc(PARAM_FILE_MAXSIZE);
	size = read(fd, raw, PARAM_FILE_MAXSIZE);
	if (size < 0) {
		debug("read failed for parameter data");
		goto out;
	}

	if (bson_init_finished_data(b, raw) != BSON_OK) {
		debug("BSON init error");
		goto out;
	}

	bson_iterator_init(iter, b);

	while ((type = bson_iterator_next(iter)) != BSON_EOO) {

		int32_t i;
		float f;
		const void *v;

		/* get the parameter name */
		name = bson_iterator_key(iter);
		param = param_find(name);

		/* ignore unknown parameters */
		if (param == PARAM_INVALID) {
			debug("ignoring unknown parameter '%s'", name);
			continue;
		}

		/* get a pointer to the data we are importing */
		switch (type) {
		case BSON_INT:
			if (param_type(param) != PARAM_TYPE_INT32) {
				debug("unexpected BSON_INT for '%s'", name);
				goto out;
			}

			i = bson_iterator_int(iter);
			v = &i;

			break;

		case BSON_DOUBLE:
			if (param_type(param) != PARAM_TYPE_FLOAT) {
				debug("unexpected BSON_FLOAT for '%s'", name);
				goto out;
			}

			f = bson_iterator_double(iter);
			v = &f;

			break;

		case BSON_BINDATA:
			if (bson_iterator_bin_type(iter) != param_type(param)) {
				debug("BSON_BINDATA type does not match '%s'", name);
				goto out;
			}

			if (bson_iterator_bin_len(iter) != param_size(param)) {
				debug("BSON_BINDATA size does not match '%s'", name);
				goto out;
			}

			v = bson_iterator_bin_data(iter);

			break;

		default:
			goto out;
		}

		/* set the parameter */
		if (param_set(param, v) != 0) {
			debug("param_set failed for '%s'", name);
			goto out;
		}
	}
	result = 0;
out:

	if (fd >= 0)
		close(fd);

	if (raw != NULL)
		free(raw);

	bson_destroy(b);	/* XXX safe if bson_init was not called? */
	return result;
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