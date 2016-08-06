
#pragma once

// Usually we want progress on every single return path query.
// We can relax this constraint for debugging.
#define TRACKER_MAX_NO_PROGRESS 3

#define TRACKER_TEST_LOOKAHEAD


bool trackerTest(void);
