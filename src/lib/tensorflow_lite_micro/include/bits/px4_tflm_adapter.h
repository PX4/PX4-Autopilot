#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Stub for system() function required by C++ standard library
// but not needed/implemented in embedded environment
inline int system(const char *) { return -1; }

#ifdef __cplusplus
}
#endif
