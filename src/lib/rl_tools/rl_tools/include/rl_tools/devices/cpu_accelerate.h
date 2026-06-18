#include "../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_DEVICES_CPU_ACCELERATE_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_DEVICES_CPU_ACCELERATE_H

#include "../utils/generic/typing.h"
#include "devices.h"

#define ACCELERATE_NEW_LAPACK
#define ACCELERATE_LAPACK_ILP64
#include "cpu_blas.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::devices{
    template <typename T_SPEC>
    struct CPU_ACCELERATE: CPU_BLAS<T_SPEC>{
        static constexpr DeviceId DEVICE_ID = DeviceId::CPU_ACCELERATE;
    };
    using DefaultCPU_ACCELERATE = CPU_ACCELERATE<DefaultCPUSpecification>;
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif
