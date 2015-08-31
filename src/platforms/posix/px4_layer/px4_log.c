#include <px4_log.h>

__EXPORT int __px4_log_level_current = PX4_LOG_LEVEL_AT_RUN_TIME;

__EXPORT const char *__px4_log_level_str[_PX4_LOG_LEVEL_PANIC+1] = { "INFO", "DEBUG", "WARN", "ERROR", "PANIC" };
