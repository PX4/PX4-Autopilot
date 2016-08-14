#include <stdlib.h>
#include <px4_log.h>
#ifdef __PX4_POSIX
#include <execinfo.h>
#endif

__EXPORT int __px4_log_level_current = PX4_LOG_LEVEL_AT_RUN_TIME;

__EXPORT const char *__px4_log_level_str[_PX4_LOG_LEVEL_PANIC + 1] = { "INFO", "DEBUG", "WARN", "ERROR", "PANIC" };
__EXPORT const char *__px4_log_level_color[_PX4_LOG_LEVEL_PANIC + 1] =
{ PX4_ANSI_COLOR_RESET, PX4_ANSI_COLOR_GREEN, PX4_ANSI_COLOR_YELLOW, PX4_ANSI_COLOR_RED, PX4_ANSI_COLOR_RED };

void px4_backtrace()
{
#ifdef __PX4_POSIX
	void *buffer[10];
	char **callstack;
	int bt_size;
	int idx;

	bt_size = backtrace(buffer, 10);
	callstack = backtrace_symbols(buffer, bt_size);

	PX4_INFO("Backtrace: %d", bt_size);

	for (idx = 0; idx < bt_size; idx++) {
		PX4_INFO("%s", callstack[idx]);
	}

	free(callstack);
#endif
}

__EXPORT void px4_log_modulename(int level, const char *moduleName, const char *fmt, ...)
{
	if (level <= __px4_log_level_current) {
		PX4_LOG_COLOR_START
		printf(__px4__log_level_fmt __px4__log_level_arg(level));
		PX4_LOG_COLOR_MODULE
		printf(__px4__log_modulename_pfmt, moduleName);
		PX4_LOG_COLOR_MESSAGE
		va_list argptr;
		va_start(argptr, fmt);
		vprintf(fmt, argptr);
		va_end(argptr);
		PX4_LOG_COLOR_END
		printf("\n");
	}
}

