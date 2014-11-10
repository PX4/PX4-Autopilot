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
 * @file param.h
 *
 * Global parameter store.
 *
 * Note that a number of API members are marked const or pure; these
 * assume that the set of parameters cannot change, or that a parameter
 * cannot change type or size over its lifetime.  If any of these assumptions
 * are invalidated, the attributes should be re-evaluated.
 */

#ifndef _SYSTEMLIB_PARAM_PARAM_H
#define _SYSTEMLIB_PARAM_PARAM_H

#include <stdint.h>
#include <stdbool.h>
#include <sys/types.h>

/** Maximum size of the parameter backing file */
#define PARAM_FILE_MAXSIZE	4096

__BEGIN_DECLS

/**
 * Parameter types.
 */
typedef enum param_type_e {
	/* globally-known parameter types */
	PARAM_TYPE_INT32 = 0,
	PARAM_TYPE_FLOAT,

	/* structure parameters; size is encoded in the type value */
	PARAM_TYPE_STRUCT = 100,
	PARAM_TYPE_STRUCT_MAX = 16384 + PARAM_TYPE_STRUCT,

	PARAM_TYPE_UNKNOWN = 0xffff
} param_type_t;

/**
 * Parameter handle.
 *
 * Parameters are represented by parameter handles, which can
 * be obtained by looking up (or creating?) parameters.
 */
typedef uintptr_t	param_t;

/**
 * Handle returned when a parameter cannot be found.
 */
#define PARAM_INVALID	((uintptr_t)0xffffffff)

/**
 * Look up a parameter by name.
 *
 * @param name		The canonical name of the parameter being looked up.
 * @return		A handle to the parameter, or PARAM_INVALID if the parameter does not exist.
 */
__EXPORT param_t	param_find(const char *name);

/**
 * Return the total number of parameters.
 *
 * @return		The number of parameters.
 */
__EXPORT unsigned	param_count(void);

/**
 * Look up a parameter by index.
 *
 * @param index		An index from 0 to n, where n is param_count()-1.
 * @return		A handle to the parameter, or PARAM_INVALID if the index is out of range.
 */
__EXPORT param_t	param_for_index(unsigned index);

/**
 * Look up the index of a parameter.
 *
 * @param param		The parameter to obtain the index for.
 * @return		The index, or -1 if the parameter does not exist.
 */
__EXPORT int		param_get_index(param_t param);

/**
 * Obtain the name of a parameter.
 *
 * @param param		A handle returned by param_find or passed by param_foreach.
 * @return		The name assigned to the parameter, or NULL if the handle is invalid.
 */
__EXPORT const char	*param_name(param_t param);

/**
 * Test whether a parameter's value has changed from the default.
 *
 * @return		If true, the parameter's value has not been changed from the default.
 */
__EXPORT bool		param_value_is_default(param_t param);

/**
 * Test whether a parameter's value has been changed but not saved.
 *
 * @return		If true, the parameter's value has not been saved.
 */
__EXPORT bool		param_value_unsaved(param_t param);

/**
 * Obtain the type of a parameter.
 *
 * @param param		A handle returned by param_find or passed by param_foreach.
 * @return		The type assigned to the parameter.
 */
__EXPORT param_type_t	param_type(param_t param);

/**
 * Determine the size of a parameter.
 *
 * @param param		A handle returned by param_find or passed by param_foreach.
 * @return		The size of the parameter's value.
 */
__EXPORT size_t		param_size(param_t param);

/**
 * Copy the value of a parameter.
 *
 * @param param		A handle returned by param_find or passed by param_foreach.
 * @param val		Where to return the value, assumed to point to suitable storage for the parameter type.
 *			For structures, a bitwise copy of the structure is performed to this address.
 * @return		Zero if the parameter's value could be returned, nonzero otherwise.
 */
__EXPORT int		param_get(param_t param, void *val);

/**
 * Set the value of a parameter.
 *
 * @param param		A handle returned by param_find or passed by param_foreach.
 * @param val		The value to set; assumed to point to a variable of the parameter type.
 *			For structures, the pointer is assumed to point to a structure to be copied.
 * @return		Zero if the parameter's value could be set from a scalar, nonzero otherwise.
 */
__EXPORT int		param_set(param_t param, const void *val);

/**
 * Reset a parameter to its default value.
 *
 * This function frees any storage used by struct parameters, and returns the parameter
 * to its default value.
 *
 * @param param		A handle returned by param_find or passed by param_foreach.
 */
__EXPORT void		param_reset(param_t param);

/**
 * Reset all parameters to their default values.
 *
 * This function also releases the storage used by struct parameters.
 */
__EXPORT void		param_reset_all(void);

/**
 * Export changed parameters to a file.
 *
 * @param fd		File descriptor to export to.
 * @param only_unsaved	Only export changed parameters that have not yet been exported.
 * @return		Zero on success, nonzero on failure.
 */
__EXPORT int		param_export(int fd, bool only_unsaved);

/**
 * Import parameters from a file, discarding any unrecognized parameters.
 *
 * This function merges the imported parameters with the current parameter set.
 *
 * @param fd		File descriptor to import from.  (Currently expected to be a file.)
 * @return		Zero on success, nonzero if an error occurred during import.
 *			Note that in the failure case, parameters may be inconsistent.
 */
__EXPORT int		param_import(int fd);

/**
 * Load parameters from a file.
 *
 * This function resets all parameters to their default values, then loads new
 * values from a file.
 *
 * @param fd		File descriptor to import from.  (Currently expected to be a file.)
 * @return		Zero on success, nonzero if an error occurred during import.
 *			Note that in the failure case, parameters may be inconsistent.
 */
__EXPORT int		param_load(int fd);

/**
 * Apply a function to each parameter.
 *
 * Note that the parameter set is not locked during the traversal. It also does
 * not hold an internal state, so the callback function can block or sleep between
 * parameter callbacks.
 *
 * @param func		The function to invoke for each parameter.
 * @param arg		Argument passed to the function.
 * @param only_changed	If true, the function is only called for parameters whose values have
 *			been changed from the default.
 */
__EXPORT void		param_foreach(void (*func)(void *arg, param_t param), void *arg, bool only_changed);

/**
 * Set the default parameter file name.
 *
 * @param filename	Path to the default parameter file.  The file is not require to
 *			exist.
 * @return		Zero on success.
 */
__EXPORT int 		param_set_default_file(const char* filename);

/**
 * Get the default parameter file name.
 *
 * @return		The path to the current default parameter file; either as
 *			a result of a call to param_set_default_file, or the 
 *			built-in default.
 */
__EXPORT const char*	param_get_default_file(void);

/**
 * Save parameters to the default file.
 *
 * This function saves all parameters with non-default values.
 *
 * @return		Zero on success.
 */
__EXPORT int 		param_save_default(void);

/**
 * Load parameters from the default parameter file.
 *
 * @return		Zero on success.
 */
__EXPORT int 		param_load_default(void);

/*
 * Macros creating static parameter definitions.
 *
 * Note that these structures are not known by name; they are
 * collected into a section that is iterated by the parameter
 * code.
 *
 * Note that these macros cannot be used in C++ code due to
 * their use of designated initializers.  They should probably
 * be refactored to avoid the use of a union for param_value_u.
 */

/** define an int32 parameter */
#define PARAM_DEFINE_INT32(_name, _default)		\
	static const					\
	__attribute__((used, section("__param")))	\
	struct param_info_s __param__##_name = {	\
		#_name,					\
		PARAM_TYPE_INT32,			\
		.val.i = _default			\
	}

/** define a float parameter */
#define PARAM_DEFINE_FLOAT(_name, _default)		\
	static const					\
	__attribute__((used, section("__param")))	\
	struct param_info_s __param__##_name = {	\
		#_name,					\
		PARAM_TYPE_FLOAT,			\
		.val.f = _default			\
	}

/** define a parameter that points to a structure */
#define PARAM_DEFINE_STRUCT(_name, _default)		\
	static const					\
	__attribute__((used, section("__param")))	\
	struct param_info_s __param__##_name = {	\
		#_name,					\
		PARAM_TYPE_STRUCT + sizeof(_default),	\
		.val.p = &_default			\
	}

/**
 * Parameter value union.
 */
union param_value_u {
	void		*p;
	int32_t		i;
	float		f;
};

/**
 * Static parameter definition structure.
 *
 * This is normally not used by user code; see the PARAM_DEFINE macros
 * instead.
 */
struct param_info_s {
	const char	*name;
	param_type_t	type;
	union param_value_u val;
};

__END_DECLS

#endif
