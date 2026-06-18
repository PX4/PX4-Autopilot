#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_UTILS_ENV_OPERATIONS_CPU_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_UTILS_ENV_OPERATIONS_CPU_H

#include <cstdlib>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template <typename DEV_SPEC>
    const char* environment_variable(const devices::CPU<DEV_SPEC>& device, const char* name){
        return std::getenv(name);
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif