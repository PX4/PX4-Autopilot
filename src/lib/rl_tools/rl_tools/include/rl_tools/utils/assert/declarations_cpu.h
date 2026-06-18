#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_UTILS_ASSERT_DECLARATIONS_CPU_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_UTILS_ASSERT_DECLARATIONS_CPU_H

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::utils{
    template <typename DEV_SPEC, typename T>
    bool assert_exit(devices::CPU<DEV_SPEC>& device, bool condition, T message);
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif