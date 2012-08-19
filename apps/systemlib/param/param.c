/**
 * @file param.c
 *
 * Global parameter store.
 */

#include <string.h>
#include <stdbool.h>

#include <unistd.h>

#include "param.h"

/**
 * Array of static parameter info.
 */
extern char __param_start, __param_end;
static const struct param_info_s	*param_info_base = (struct param_info_s *)&__param_start;
static const struct param_info_s	*param_info_limit = (struct param_info_s *)&__param_end;
#define	param_info_count		((unsigned)(param_info_limit - param_info_base))

static bool
handle_in_range(param_t handle) 
{
	return (handle < param_info_count);
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

enum
param_type_e param_type(param_t param)
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
	if (handle_in_range(param)) {

		/* XXX look for updated value stored elsewhere */

		switch (param_info_base[param].type) {
		case PARAM_TYPE_INT32:
			*(int32_t *)val = param_info_base[param].i;
			return 0;

		case PARAM_TYPE_FLOAT:
			*(float *)val = param_info_base[param].f;
			return 0;

		case PARAM_TYPE_STRUCT ... PARAM_TYPE_STRUCT_MAX:
			memcpy(val, param_info_base[param].p, param_size(param));
			return 0;

		default:
			return -1;
		}
	}
	return -1;
}

int
param_set(param_t param, void *val)
{
	if (handle_in_range(param)) {

		/* XXX maintain list of changed values */

	}
	return -1;

}

