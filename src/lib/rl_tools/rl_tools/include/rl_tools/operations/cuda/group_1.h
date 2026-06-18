#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_OPERATIONS_CUDA_GROUP_1_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_OPERATIONS_CUDA_GROUP_1_H
#if defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_OPERATIONS_CUDA_GROUP_1)
    #define RL_TOOLS_OPERATIONS_CUDA_GROUP_1
    #define RL_TOOLS_FUNCTION_PLACEMENT __device__ __host__
    #define RL_TOOLS_DEVICES_CUDA_CEIL(A, B) (A / B + (A % B == 0 ? 0 : 1))

    #include "../../devices/cuda.h"
    #include "../../numeric_types/policy.h"
    #include "../../math/operations_cuda.h"
    #include "../../logging/operations_cuda.h"
    #include "../../utils/env/operations_generic.h"
#else
    #error "Group 1 already imported"
#endif
#endif
