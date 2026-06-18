#include "../../../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ENVIRONMENTS_L2F_PARAMETERS_REWARD_FUNCTIONS_DEFAULT_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENTS_L2F_PARAMETERS_REWARD_FUNCTIONS_DEFAULT_H

#include "../../multirotor.h"
#include "./squared/squared.h"
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::environments::l2f::parameters::reward_functions{
    template<typename T>
    constexpr Squared<T> constant = {
            false, // non-negative
            0.1, // scale
            1, // constant
            0, // termination penalty
            0, // position
            0, // orientation
            0, // linear_velocity
            0, // angular_velocity
            0, // linear_acceleration
            0, // angular_acceleration
            0, // action
            0.0, // d_action
            0.0 // Position error integral
    };
    template<typename T>
    constexpr Squared<T> squared = {
            false, // non-negative
            0.1, // scale
            1.0, // constant
            0, // termination penalty
            10, // position
            0, // position_clip
            2.5, // orientation
            0.05, // linear_velocity
            0, // angular_velocity
            0, // linear_acceleration
            0, // angular_acceleration
            0.1, // action
            0.0, // d_action
            0.0 // Position error integral
    };
    template<typename T>
    constexpr Squared<T> squared_no_orientation = {
            false, // non-negative
            0.1, // scale
            1, // constant
            0, // termination penalty
            10, // position
            0, // orientation
            1, // linear_velocity
            0, // angular_velocity
            0, // linear_acceleration
            0, // angular_acceleration
            0.0, // action
            0.0, // d_action
            0.0 // Position error integral
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif