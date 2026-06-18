#include <rl_tools/version.h>
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ZOO_PENDULUM_VELOCITY_V1_ENVIRONMENT_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ZOO_PENDULUM_POSITION_V1_ENVIRONMENT_H

#include <rl_tools/rl/environments/pendulum/operations_cpu.h>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::zoo::pendulum_velocity_v1{
    template <typename DEVICE, typename TYPE_POLICY, typename TI>
    struct ENVIRONMENT_FACTORY{
        using T = typename TYPE_POLICY::DEFAULT;
        struct ENVIRONMENT_PARAMETERS: environments::pendulum::DefaultParameters<T>{
            constexpr static T OBSERVATION_NOISE_VELOCITY = 0.0;
        };
        using ENVIRONMENT_SPEC = environments::pendulum::Specification<T, TI, ENVIRONMENT_PARAMETERS>;
        using ENVIRONMENT = environments::PendulumVelocity<ENVIRONMENT_SPEC>;
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif