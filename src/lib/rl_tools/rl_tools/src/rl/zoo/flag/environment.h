#include <rl_tools/version.h>
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ZOO_FLAG_ENVIRONMENT_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ZOO_FLAG_ENVIRONMENT_H

#include <rl_tools/rl/environments/flag/operations_cpu.h>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::zoo::flag{
    namespace rlt = rl_tools;
    template <typename DEVICE, typename TYPE_POLICY, typename TI, TI MAX_EPISODE_LENGTH = 200, bool PRIVILEGED_OBSERVATION = false>
    struct ENVIRONMENT_FACTORY{
        using T = typename TYPE_POLICY::DEFAULT;
        using ENVIRONMENT_SPEC = rlt::rl::environments::flag::Specification<T, TI, rlt::rl::environments::flag::DefaultParameters<T, TI, MAX_EPISODE_LENGTH, PRIVILEGED_OBSERVATION>>;
        using ENVIRONMENT = rlt::rl::environments::Flag<ENVIRONMENT_SPEC>;
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif