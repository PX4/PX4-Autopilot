#include <rl_tools/version.h>
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ZOO_BOTTLENECK_V0_ENVIRONMENT_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ZOO_BOTTLENECK_V0_ENVIRONMENT_H

#include <rl_tools/rl/environments/multi_agent/bottleneck/operations_cpu.h>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::zoo::bottleneck_v0{
    namespace rlt = rl_tools;
    template <typename DEVICE, typename TYPE_POLICY, typename TI>
    struct ENVIRONMENT_FACTORY{
        using T = typename TYPE_POLICY::DEFAULT;
        struct ENVIRONMENT_PARAMETERS: rlt::rl::environments::multi_agent::bottleneck::DefaultParameters<T, TI>{
            static constexpr TI N_AGENTS = 5;
            static constexpr TI LIDAR_RESOLUTION = 5;
            static constexpr T LIDAR_FOV = math::PI<T> * 90/180; // in radians (0 to PI)
            static constexpr T BOTTLENECK_WIDTH = 5;
            static constexpr TI EPISODE_STEP_LIMIT = 200;
            static constexpr bool SPAWN_BOTH_SIDES = false;
            static constexpr T AGENT_MAX_SPEED = 4;
            static constexpr T AGENT_MAX_ANGULAR_VELOCITY = 4;
            static constexpr T AGENT_MAX_ACCELERATION = 20;
            static constexpr T AGENT_MAX_ANGULAR_ACCELERATION = 20;
            static constexpr T DT = 0.05;
        };
        using ENVIRONMENT_SPEC = rlt::rl::environments::multi_agent::bottleneck::Specification<TYPE_POLICY, TI, ENVIRONMENT_PARAMETERS>;
        using ENVIRONMENT = rlt::rl::environments::multi_agent::Bottleneck<ENVIRONMENT_SPEC>;
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif