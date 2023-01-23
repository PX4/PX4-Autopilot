#ifndef VNEXPORT_H_INCLUDED
#define VNEXPORT_H_INCLUDED

/* Not only does this have to be windows to use __declspec */
/* it also needs to actually be outputting a DLL */
#if defined _WINDOWS && defined _WINDLL
	#if proglib_c_EXPORTS
		#define DllExport __declspec(dllexport)
	#else
		#define DllExport __declspec(dllimport)
	#endif
#else
	#define DllExport
#endif

#endif
