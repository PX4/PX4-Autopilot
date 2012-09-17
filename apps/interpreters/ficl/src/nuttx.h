#include <stdint.h>

typedef int8_t ficlInteger8;
typedef uint8_t ficlUnsigned8;
typedef int16_t ficlInteger16;
typedef uint16_t ficlUnsigned16;
typedef int32_t ficlInteger32;
typedef uint32_t ficlUnsigned32;

typedef intptr_t ficlInteger;
typedef uintptr_t ficlUnsigned;
typedef float ficlFloat;

#define FICL_PLATFORM_BASIC_TYPES   (1)
#define FICL_PLATFORM_HAS_2INTEGER  (0)
#define FICL_PLATFORM_HAS_FTRUNCATE (0)

#define FICL_PLATFORM_OS            "ansi"
#define FICL_PLATFORM_ARCHITECTURE  "unknown"
