#ifndef _VN_UTIL_COMPILER_H
#define _VN_UTIL_COMPILER_H

/* This header provides some simple checks for various features supported by the
* current compiler. */

/* Determine the level of standard C support. */
#if __STDC__
	#if defined (__STDC_VERSION__)
		#if (__STDC_VERSION__ >= 199901L)
			#define C99
		#endif
	#endif
#endif

/* Determine if the compiler has stdbool.h. */
#if defined(C99) || _MSC_VER >= 1800
	#define VN_HAVE_STDBOOL_H 1
#else
	#define VN_HAVE_STDBOOL_H 0
#endif

/* Determine if the secure CRT is available. */
#if defined(_MSC_VER)
	#define VN_HAVE_SECURE_CRT 1
#else
	#define VN_HAVE_SECURE_CRT 0
#endif

#endif

/* Determine if the generic type math library (tgmath.h) is available. */
#if defined(C99)
	#define VN_HAVE_GENERIC_TYPE_MATH 1
#else
	#define VN_HAVE_GENERIC_TYPE_MATH 0
#endif
