#include "../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_OPERATIONS_CPU_MUX_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_OPERATIONS_CPU_MUX_H

#include "../rl_tools.h"
// ------------ Groups 1 ------------
#if defined(RL_TOOLS_ENABLE_TENSORBOARD) && !defined(RL_TOOLS_DISABLE_TENSORBOARD)
#ifdef RL_TOOLS_ENABLE_WARNINGS
#pragma message("RLtools: Enabling Tensorboard")
#endif
#include "../operations/cpu_tensorboard/group_1.h"
#endif
#if defined(RL_TOOLS_BACKEND_ENABLE_MKL) && !defined(RL_TOOLS_BACKEND_DISABLE_BLAS)
#ifdef RL_TOOLS_ENABLE_WARNINGS
#pragma message("RLtools: Using MKL backend")
#endif
#include "../operations/cpu_mkl/group_1.h"
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::devices{
    template <typename DEV_SPEC>
    using DEVICE_FACTORY_INTERNAL = rl_tools::devices::CPU_MKL<DEV_SPEC>;
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#else
#if defined(RL_TOOLS_BACKEND_ENABLE_ACCELERATE) && !defined(RL_TOOLS_BACKEND_DISABLE_BLAS)
#ifdef RL_TOOLS_ENABLE_WARNINGS
#pragma message("RLtools: Using Apple Accelerate backend")
#endif
#include "../operations/cpu_accelerate/group_1.h"
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::devices{
    template <typename DEV_SPEC>
    using DEVICE_FACTORY_INTERNAL = rl_tools::devices::CPU_ACCELERATE<DEV_SPEC>;
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#else
#if defined(RL_TOOLS_BACKEND_ENABLE_OPENBLAS) && !defined(RL_TOOLS_BACKEND_DISABLE_BLAS)
#ifdef RL_TOOLS_ENABLE_WARNINGS
#pragma message("RLtools: Using OpenBLAS backend")
#endif
#include "../operations/cpu_openblas/group_1.h"
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::devices{
    template <typename DEV_SPEC>
    using DEVICE_FACTORY_INTERNAL = rl_tools::devices::CPU_OPENBLAS<DEV_SPEC>;
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#else
#ifdef RL_TOOLS_ENABLE_WARNINGS
#pragma message("RLtools: Using Generic Backend")
#endif
#include "../operations/cpu/group_1.h"
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::devices{
    template <typename DEV_SPEC>
    using DEVICE_FACTORY_INTERNAL = rl_tools::devices::CPU<DEV_SPEC>;
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
#endif
#endif
#if defined(RL_TOOLS_BACKEND_ENABLE_CUDA) && defined(RL_TOOLS_OPERATIONS_CPU_MUX_INCLUDE_CUDA)
#ifdef RL_TOOLS_ENABLE_WARNINGS
#pragma message("RLtools: Enabling CUDA")
#endif
#include "../operations/cuda/group_1.h"
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::devices{
    template<typename DEV_SPEC>
    using DEVICE_FACTORY_INTERNAL_CUDA = rl_tools::devices::CUDA<DEV_SPEC>;
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif

#if defined(RL_TOOLS_ENABLE_TENSORBOARD) && !defined(RL_TOOLS_DISABLE_TENSORBOARD)
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template <typename... ARGS>
    using LOGGER_FACTORY = devices::logging::CPU_TENSORBOARD<devices::logging::CPU_TENSORBOARD_FREQUENCY_EXTENSION>;
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#else
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template <typename... ARGS>
    using LOGGER_FACTORY = devices::logging::CPU;
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
// ------------ Groups 2 ------------
#if defined(RL_TOOLS_ENABLE_TENSORBOARD) && !defined(RL_TOOLS_DISABLE_TENSORBOARD)
#include "../operations/cpu_tensorboard/group_2.h"
#endif
#if defined(RL_TOOLS_BACKEND_ENABLE_MKL) && !defined(RL_TOOLS_BACKEND_DISABLE_BLAS)
#include "../operations/cpu_mkl/group_2.h"
#else
#if defined(RL_TOOLS_BACKEND_ENABLE_ACCELERATE) && !defined(RL_TOOLS_BACKEND_DISABLE_BLAS)
#include "../operations/cpu_accelerate/group_2.h"
#else
#if defined(RL_TOOLS_BACKEND_ENABLE_OPENBLAS) && !defined(RL_TOOLS_BACKEND_DISABLE_BLAS)
#include "../operations/cpu_openblas/group_2.h"
#else
#include "../operations/cpu/group_2.h"
#endif
#endif
#endif
#if defined(RL_TOOLS_BACKEND_ENABLE_CUDA) && defined(RL_TOOLS_OPERATIONS_CPU_MUX_INCLUDE_CUDA)
#include "../operations/cuda/group_2.h"
#endif
// ------------ Groups 3 ------------
#if defined(RL_TOOLS_ENABLE_TENSORBOARD) && !defined(RL_TOOLS_DISABLE_TENSORBOARD)
#include "../operations/cpu_tensorboard/group_3.h"
#endif
#if defined(RL_TOOLS_BACKEND_ENABLE_MKL) && !defined(RL_TOOLS_BACKEND_DISABLE_BLAS)
#include "../operations/cpu_mkl/group_3.h"
#else
#if defined(RL_TOOLS_BACKEND_ENABLE_ACCELERATE) && !defined(RL_TOOLS_BACKEND_DISABLE_BLAS)
#include "../operations/cpu_accelerate/group_3.h"
#else
#if defined(RL_TOOLS_BACKEND_ENABLE_OPENBLAS) && !defined(RL_TOOLS_BACKEND_DISABLE_BLAS)
#include "../operations/cpu_openblas/group_3.h"
#else
#include "../operations/cpu/group_3.h"
#endif
#endif
#endif
#if defined(RL_TOOLS_BACKEND_ENABLE_CUDA) && defined(RL_TOOLS_OPERATIONS_CPU_MUX_INCLUDE_CUDA)
#include "../operations/cuda/group_3.h"
#endif

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::devices{
    template<typename DEV_SPEC=devices::cpu::Specification<devices::math::CPU, devices::random::CPU, LOGGER_FACTORY<>>>
    using DEVICE_FACTORY = DEVICE_FACTORY_INTERNAL<DEV_SPEC>;
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#if defined(RL_TOOLS_BACKEND_ENABLE_CUDA) && defined(RL_TOOLS_OPERATIONS_CPU_MUX_INCLUDE_CUDA)
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::devices{
    template<typename DEV_SPEC=devices::DefaultCUDASpecification>
    using DEVICE_FACTORY_CUDA = DEVICE_FACTORY_INTERNAL_CUDA<DEV_SPEC>;
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif

#endif