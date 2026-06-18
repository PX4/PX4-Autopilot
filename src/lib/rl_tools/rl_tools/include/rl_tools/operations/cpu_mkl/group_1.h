#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_OPERATIONS_CPU_MKL_GROUP_1_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_OPERATIONS_CPU_MKL_GROUP_1_H
#if defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_OPERATIONS_CPU_MKL_GROUP_1)
    #define RL_TOOLS_OPERATIONS_CPU_MKL_GROUP_1
    #include <mkl.h>
    #include "../../devices/cpu_mkl.h"
    #include "../../numeric_types/policy.h"
    #include "../../utils/assert/declarations_cpu.h"
    #include "../../math/operations_cpu.h"
    #include "../../logging/operations_cpu.h"
    #include "../../utils/env/operations_cpu.h"
#else
    #error "Group 1 already imported"
#endif
#endif
