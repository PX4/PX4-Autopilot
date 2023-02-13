#ifndef VNPORT_H_INCLUDED
#define VNPORT_H_INCLUDED

/** Basic portability measures. */

/** VNAPI - DLL linkage specifier. */
#ifdef _MSC_VER
	#if VN_LINKED_AS_SHARED_LIBRARY
		#define VNAPI __declspec(dllimport)
	#elif VN_CREATE_SHARED_LIBRARY
		#define VNAPI __declspec(dllexport)
	#endif
#endif

#ifndef VNAPI
	#define VNAPI
#endif

#endif
