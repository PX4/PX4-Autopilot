#include "../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_DEVICES_CPU_MKL_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_DEVICES_CPU_MKL_H

#include "../utils/generic/typing.h"
#include "devices.h"

#include "cpu_blas.h"
#include <iostream>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    namespace devices{
        template <typename T_SPEC>
        struct CPU_MKL: CPU_BLAS<T_SPEC>{
            static constexpr DeviceId DEVICE_ID = DeviceId::CPU_MKL;
        };
        using DefaultCPU_MKL = CPU_MKL<DefaultCPUSpecification>;
    }
    template <typename T_SPEC>
    void init(devices::CPU_MKL<T_SPEC>& device){
        init(static_cast<devices::CPU_BLAS<T_SPEC>&>(device));
        using DEVICE = devices::CPU_MKL<T_SPEC>;
        const char *env_var_name = "MKL_NUM_THREADS";
        const char *value = getenv(env_var_name);
        bool warn = true;
        if (value != NULL) {
            char *endptr;
            typename DEVICE::index_t num_threads = strtol(value, &endptr, 10);
            if (*endptr == '\0') {
                warn = num_threads != 1;
            }
        }
        if(warn){
            std::cerr << "Warning: " << env_var_name << " is not set to 1. This may degrade performance." << std::endl;
        }

    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif
