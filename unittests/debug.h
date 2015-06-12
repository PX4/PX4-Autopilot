
#pragma once

#include <systemlib/err.h>
#define lowsyslog warnx
#define dbg warnx

#if !defined(ASSERT)
# define ASSERT(x) assert((x))
#endif
