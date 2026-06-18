#include "../../../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ENVIRONMENTS_L2F_PARAMETERS_INIT_DEFAULT_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENTS_L2F_PARAMETERS_INIT_DEFAULT_H

#include "../../multirotor.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::environments::l2f::parameters::init{
    template<typename SPEC>
    constexpr typename ParametersBase<SPEC>::MDP::Initialization init_20_deg = {
            0.1, // guidance
            0.5, // position
            1.5707963267948966/90.0 * 20.0,   // orientation
            1,   // linear velocity
            1,  // angular velocity
            true,// relative rpm
            -1,  // min rpm
            +1,  // max rpm
    };
    template<typename SPEC>
    constexpr typename ParametersBase<SPEC>::MDP::Initialization init_90_deg = {
            0.1, // guidance
            0.5, // position
            1.5707963267948966,   // orientation
            1,   // linear velocity
            1,  // angular velocity
            true,// relative rpm
            -1,  // min rpm
            0,  // max rpm
    };
    template<typename SPEC>
    constexpr typename ParametersBase<SPEC>::MDP::Initialization init_180_deg = {
            0.1, // guidance
            0.5, // position
            2*1.5707963267948966,   // orientation
            1,   // linear velocity
            1,  // angular velocity
            true,// relative rpm
            -1,  // min rpm
            +1,  // max rpm
    };
    template<typename SPEC>
    constexpr typename ParametersBase<SPEC>::MDP::Initialization init_0_deg = {
            0.1, // guidance
            0.5, // position
            0,   // orientation
            1,   // linear velocity
            1,  // angular velocity
            true,// relative rpm
            -1,  // min rpm
            +1,  // max rpm
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
