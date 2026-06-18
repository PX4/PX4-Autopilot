#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_OPERATIONS_CPU_OPENBLAS_GROUP_3_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_OPERATIONS_CPU_OPENBLAS_GROUP_3_H
#if defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_OPERATIONS_CPU_OPENBLAS_GROUP_3)
    #define RL_TOOLS_OPERATIONS_CPU_OPENBLAS_GROUP_3
    #include "../../containers/matrix/operations_cpu_openblas.h"
    #include "../../containers/tensor/operations_cpu_openblas.h"
    #ifdef RL_TOOLS_ENABLE_HDF5
        #include "../../persist/backends/hdf5/operations_cpu.h"
    #endif
#else
    #error "Group 3 already imported"
#endif
#endif
