#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_UTILS_ASSERT_OPERATIONS_CUDA_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_UTILS_ASSERT_OPERATIONS_CUDA_H
#include <cassert>
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::utils{
    template <typename DEV_SPEC, typename T, typename utils::typing::enable_if<!DEV_SPEC::KERNEL>::type* = nullptr>
    bool assert_exit(devices::CUDA<DEV_SPEC>& dev, bool condition, T message){
        using DEVICE = devices::CUDA<DEV_SPEC>;
        if(!condition){
            log(dev, dev.logger, message);
            assert(condition);
        }
        return condition;
    }
    template <typename DEV_SPEC, typename T, typename utils::typing::enable_if<DEV_SPEC::KERNEL>::type* = nullptr>
    bool assert_exit(devices::CUDA<DEV_SPEC>& dev, bool condition, T message){
        //noop
        return condition;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif