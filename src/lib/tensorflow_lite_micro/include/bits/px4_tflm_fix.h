#pragma once

// # include <stdlib.h>

// // Fix for missing system() function
// #ifdef __cplusplus
// extern "C" {
// #endif

// // Provide a stub implementation for system()
// inline int system(const char *) { return -1; }

// #ifdef __cplusplus
// }
// #endif

// namespace std
// {
// #ifndef __KERNEL__ //From NuttX, this is defined in nuttx/NuttX/include/cxx/cstdlib
// // System command
// using ::system;
// #endif
// }

// Fix for ctype_base.h character class masks
#define _U 0x01
#define _L 0x02
#define _N 0x04
#define _X 0x08
#define _S 0x10
#define _P 0x20
#define _B 0x40
#define _C 0x80

// // Fix for abs() function ambiguity
// #include <cmath>
// // Explicitly provide an overload for time_t type
// // TODO: Remove the static cast to long in gps and gnss drivers
// #ifdef __cplusplus
// namespace std
// {
// inline long abs(time_t t) { return ::abs(static_cast<long>(t)); }
// }
// #endif
