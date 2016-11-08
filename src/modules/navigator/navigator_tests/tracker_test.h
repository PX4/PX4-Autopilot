
#pragma once

// Usually we want progress on every single return path query.
// We can relax this constraint for debugging.
#define TRACKER_MAX_NO_PROGRESS 3

#define TRACKER_TEST_LOOKAHEAD

#if defined(_POSIX_VERSION)
// Omit some tests on small systems to save flash memory
#define TRACKER_TEST_LONG_PATHS
#endif

bool trackerTest(void);
