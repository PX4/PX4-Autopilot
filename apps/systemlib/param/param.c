/**
 * @file param.c
 *
 * Global parameter store.
 */

#include <string.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <err.h>

#include "systemlib/param/param.h"
#include "systemlib/uthash/utarray.h"
#include "systemlib/bson/bson.h"

/**
 * Array of static parameter info.
 */
extern char __param_start, __param_end;
static const struct param_info_s	*param_info_base = (struct param_info_s *)&__param_start;
static const struct param_info_s	*param_info_limit = (struct param_info_s *)&__param_end;
#define	param_info_count		((unsigned)(param_info_limit - param_info_base))

/**
 * Storage for modified parameters.
 */
struct param_wbuf_s
{
	param_t			param;
	union param_value_u	val;
	bool			unsaved;
};

UT_icd		param_icd = {sizeof(struct param_wbuf_s), NULL, NULL, NULL};
UT_array	*param_values;

static void
param_lock(void)
{
	/* XXX */
}

static void
param_unlock(void)
{
	/* XXX */
}

static void
param_assert_locked(void)
{
	/* XXX */
}

static bool
handle_in_range(param_t handle) 
{
	return (handle < param_info_count);
}

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

static struct param_wbuf_s *
param_find_changed(param_t param)
{
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

	for (param = 0; handle_in_range(param); param++) {
		if (!strcmp(param_info_base[param].name, name))
			return param;
	}

	/* not found */
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
		switch (param_info_base[param].type) {
		case PARAM_TYPE_INT32:
		case PARAM_TYPE_FLOAT:
			return 4;

		case PARAM_TYPE_STRUCT ... PARAM_TYPE_STRUCT_MAX:
			return param_info_base[param].type - PARAM_TYPE_STRUCT;

		default:
			return 0;
		}
	}

	return 0;
}

int
param_get(param_t param, void *val)
{
	int result = -1;

	param_lock();

	if (handle_in_range(param)) {

		struct param_wbuf_s *s = param_find_changed(param);
		const union param_value_u *v;

		/* work out whether we're fetching the default or a written value */
		if (s != NULL) {
			v = &s->val;
		} else {
			v = &param_info_base[param].val;
		}

		/* XXX look for updated value stored elsewhere */

		switch (param_info_base[param].type) {
		case PARAM_TYPE_INT32:
			*(int32_t *)val = v->i;
			result = 0;
			break;

		case PARAM_TYPE_FLOAT:
			*(float *)val = v->f;
			result = 0;
			break;

		case PARAM_TYPE_STRUCT ... PARAM_TYPE_STRUCT_MAX:
			*(void **)val = v->p;
			result = 0;
			break;

		default:
			break;
		}

	}

	param_unlock();

	return result;
}

int
param_set(param_t param, void *val)
{
	int result = -1;

	param_lock();

	if (param_values == NULL)
		utarray_new(param_values, &param_icd);
	if (param_values == NULL)
		goto out;

	if (handle_in_range(param)) {

		struct param_wbuf_s *s = param_find_changed(param);

		if (s == NULL) {

			/* construct a new parameter */
			struct param_wbuf_s buf = { .param = param };

			/* add it to the array and sort */
			utarray_push_back(param_values, &buf);
			utarray_sort(param_values, param_compare_values);

			/* find it after sorting */
			s = param_find_changed(param);
		}

		/* update the changed value */
		switch (param_info_base[param].type) {
		case PARAM_TYPE_INT32:
			s->val.i = *(int32_t *)val;
			break;

		case PARAM_TYPE_FLOAT:
			s->val.f = *(float *)val;
			break;

		case PARAM_TYPE_STRUCT ... PARAM_TYPE_STRUCT_MAX:
			s->val.p = *(void **)val;
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
param_export(const char *filename, bool only_unsaved)
{
	struct param_wbuf_s *s;
	bson	b[1];
	int	result = -1;

	param_lock();

	bson_init(b);

	while ((s = (struct param_wbuf_s *)utarray_next(param_values, s)) != NULL) {
		const struct param_info_s *is = &param_info_base[s->param];

		/* 
		 * if we are only saving values changed since last save, and this
		 * one hasn't, then skip it
		 */
		if (only_unsaved & !s->unsaved)
			continue;
		s->unsaved = false;

		switch (is->type) {
		case PARAM_TYPE_INT32:
			if (bson_append_int(b, is->name, s->val.i) != BSON_OK)
				goto out;
			break;

		case PARAM_TYPE_FLOAT:
			if (bson_append_double(b, is->name, s->val.f) != BSON_OK)
				goto out;
			break;

		case PARAM_TYPE_STRUCT ... PARAM_TYPE_STRUCT_MAX:
			if (bson_append_binary(b, 
					       is->name,
					       is->type,
					       is->val.p,
					       is->type - PARAM_TYPE_STRUCT) != BSON_OK)
				goto out;
			break;
		default:
			goto out;
		}
	}
	result = 0;

out:
	param_unlock();

	if (result == 0) {
		bson_finish(b);
		bson_print(b);

		warnx("object is %d bytes", bson_buffer_size(b));

		if (filename != NULL) {
			int fd = open(filename, O_CREAT | O_WRONLY | O_TRUNC, 0666);
			int siz = bson_buffer_size(b);
			int len = write(fd, bson_data(b), siz);
			close(fd);
			if (len != siz)
				result = -1;
		}
	}
	bson_destroy(b);

	return result;
}

int
param_import(const char *filename)
{
	return -1;
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