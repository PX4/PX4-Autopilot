#pragma once

#ifdef __PX4_NUTTX
#include "i2c_nuttx.h"
#else
#include "i2c_linux.h"
#endif
