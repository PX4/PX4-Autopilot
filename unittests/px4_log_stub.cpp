#include <px4_log.h>


__EXPORT void px4_log_modulename(int level, const char *moduleName, const char *fmt, ...)
{
	// Don't log Debug
	if (level >= 1) {
		va_list argptr;
		va_start(argptr, fmt);
		vprintf(fmt, argptr);
		va_end(argptr);
		printf("\n");
	}
}
