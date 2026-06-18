#include <rl_tools/version.h>
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ZOO_ANT_V4_ENVIRONMENT_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ZOO_ANT_V4_ENVIRONMENT_H

#include <rl_tools/rl/environments/mujoco/ant/operations_cpu.h>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::zoo::ant_v4{
    namespace rlt = rl_tools;
    template <typename DEVICE, typename TYPE_POLICY, typename TI>
    struct ENVIRONMENT_FACTORY{
        using T_ENVIRONMENT = double;
        using ENVIRONMENT_PARAMETERS = rlt::rl::environments::mujoco::ant::DefaultParameters<T_ENVIRONMENT, TI>;
        using ENVIRONMENT_SPEC = rlt::rl::environments::mujoco::ant::Specification<T_ENVIRONMENT, TI, ENVIRONMENT_PARAMETERS>;
        using ENVIRONMENT = rlt::rl::environments::mujoco::Ant<ENVIRONMENT_SPEC>;
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif