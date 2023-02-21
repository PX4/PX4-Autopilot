#ifndef VNBOOL_H_INCLUDED
#define VNBOOL_H_INCLUDED

/** \brief Boolean definition. */

#include "vn/util/compiler.h"
#include "vn/int.h"

#if !defined(__cplusplus)

	#if VN_HAVE_STDBOOL_H
		#include <stdbool.h>
	#else
		#if !defined(__GNUC__)
			/* _Bool builtin type is included in GCC. */
			/* ISO C Standard: 5.2.5 An object declared as type _Bool is large
			* enough to store the values 0 and 1. */
			typedef int8_t _Bool;
		#endif

		/* ISO C Standard: 7.16 Boolean type */

		#if defined(__STDC__) && defined(__GNUC__)
			/* Avoid warning "ISO C90/89 does not support boolean types" */
			#define bool int8_t
		#else
			#define bool _Bool
		#endif

		#define true 1
		#define false 0
		#define __bool_true_false_are_defined 1
	#endif

#endif

#endif
