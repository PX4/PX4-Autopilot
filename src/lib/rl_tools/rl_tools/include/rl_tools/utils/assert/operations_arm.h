#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_UTILS_ASSERT_OPERATIONS_ARM_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_UTILS_ASSERT_OPERATIONS_ARM_H

#ifdef RL_TOOLS_ARM_TEST
#include <cstdlib>
#include <stdexcept>
#endif
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::utils{
    template <typename DEV_SPEC, typename T>
    bool assert_exit(devices::ARM<DEV_SPEC>& device, bool condition, T message){
        if(!condition){
            log(device, device.logger, message);
#ifdef RL_TOOLS_ARM_TEST
            throw std::runtime_error(message);
#endif
        }
        return condition;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif