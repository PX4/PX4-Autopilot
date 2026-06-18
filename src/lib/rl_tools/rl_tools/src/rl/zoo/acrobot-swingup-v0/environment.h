#include <rl_tools/version.h>
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ZOO_ACROBOT_SWINGUP_V0_ENVIRONMENT_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ZOO_ACROBOT_SWINGUP_V0_ENVIRONMENT_H

#include <rl_tools/rl/environments/acrobot/operations_cpu.h>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::zoo::acrobot_swingup_v0{
    namespace rlt = rl_tools;
    template <typename DEVICE, typename T, typename TI>
    struct ENVIRONMENT_FACTORY{
        struct ENVIRONMENT_PARAMETERS: rlt::rl::environments::acrobot::EasyParameters<T>{
            static constexpr T DT = 0.02;
            static constexpr T MIN_TORQUE = -5;
            static constexpr T MAX_TORQUE = +5;
        };
        using ENVIRONMENT_SPEC = rlt::rl::environments::acrobot::Specification<T, TI, ENVIRONMENT_PARAMETERS>;
        using ENVIRONMENT = rlt::rl::environments::AcrobotSwingup<ENVIRONMENT_SPEC>;
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif


