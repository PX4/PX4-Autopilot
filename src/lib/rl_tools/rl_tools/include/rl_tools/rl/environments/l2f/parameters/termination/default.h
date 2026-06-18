#include "../../../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ENVIRONMENTS_L2F_PARAMETERS_TERMINATION_DEFAULT_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENTS_L2F_PARAMETERS_TERMINATION_DEFAULT_H
#include "../../multirotor.h"

#include <rl_tools/math/operations_generic.h>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::environments::l2f::parameters::termination{
    template<typename SPEC>
    constexpr typename rl_tools::rl::environments::l2f::ParametersBase<SPEC>::MDP::Termination narrow = {
        true,           // enable
        1,            // position
        10,         // linear velocity
        35, // angular velocity
        10000, // position integral
        50000, // orientation integral
    };
    template<typename SPEC>
    constexpr typename rl_tools::rl::environments::l2f::ParametersBase<SPEC>::MDP::Termination fast_learning = {
        true,           // enable
        5,            // position
        10,         // linear velocity
        35, // angular velocity
        10000, // position integral
        50000, // orientation integral
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
