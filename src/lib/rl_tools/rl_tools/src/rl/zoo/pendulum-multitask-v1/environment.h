#include <rl_tools/version.h>
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ZOO_PENDULUM_MULTITASK_V1_ENVIRONMENT_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ZOO_PENDULUM_MULTITASK_V1_ENVIRONMENT_H

#include <rl_tools/rl/environments/pendulum/operations_cpu.h>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::zoo::pendulum_multitask_v1{
    namespace rlt = rl_tools;
    template <typename DEVICE, typename TYPE_POLICY, typename TI>
    struct ENVIRONMENT_FACTORY{
        using T = typename TYPE_POLICY::DEFAULT;
        using ENVIRONMENT_SPEC = rlt::rl::environments::pendulum::Specification<T, TI, rlt::rl::environments::pendulum::DefaultParameters<T>>;
        // using ENVIRONMENT = rlt::rl::environments::PendulumMultiTask<ENVIRONMENT_SPEC>;
        using ENVIRONMENT = rlt::rl::environments::PendulumMeta<ENVIRONMENT_SPEC>;
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif