#pragma once

#ifdef __PX4_QURT
#include <types.h>
size_t strnlen(const char *s, size_t maxlen);

//inline bool isfinite(int x) { return true; }
#endif
