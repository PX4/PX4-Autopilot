#include "../../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ENVIRONMENTS_L2F_OPERATIONS_GENERIC_GET_DESIRED_STATE_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENTS_L2F_OPERATIONS_GENERIC_GET_DESIRED_STATE_H

#include "../multirotor.h"

#include <rl_tools/utils/generic/vector_operations.h>
#include "../quaternion_helper.h"

#include <rl_tools/utils/generic/typing.h>

#include <rl_tools/rl/environments/operations_generic.h>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static void get_desired_state(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, PARAMETERS& parameters, const rl::environments::l2f::StateBase<STATE_SPEC>& state, rl::environments::l2f::StateBase<STATE_SPEC>& desired_state, RNG& rng){
        using TI = typename DEVICE::index_t;
        using T = typename SPEC::T;
        desired_state.position[0] = 0;
        desired_state.position[1] = 0;
        desired_state.position[2] = 0;
        desired_state.linear_velocity[0] = 0;
        desired_state.linear_velocity[1] = 0;
        desired_state.linear_velocity[2] = 0;
    }
    template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static void get_desired_state(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, PARAMETERS& parameters, const rl::environments::l2f::StateTrajectory<STATE_SPEC>& state, rl::environments::l2f::StateTrajectory<STATE_SPEC>& desired_state, RNG& rng){
        using TI = typename DEVICE::index_t;
        using T = typename SPEC::T;
        switch (state.trajectory.type){
            case rl::environments::l2f::LANGEVIN:
                desired_state.position[0] = state.trajectory.langevin.position[0];
                desired_state.position[1] = state.trajectory.langevin.position[1];
                desired_state.position[2] = state.trajectory.langevin.position[2];
                desired_state.linear_velocity[0] = state.trajectory.langevin.velocity[0];
                desired_state.linear_velocity[1] = state.trajectory.langevin.velocity[1];
                desired_state.linear_velocity[2] = state.trajectory.langevin.velocity[2];
                break;
            default:
                desired_state.position[0] = 0;
                desired_state.position[1] = 0;
                desired_state.position[2] = 0;
                desired_state.linear_velocity[0] = 0;
                desired_state.linear_velocity[1] = 0;
                desired_state.linear_velocity[2] = 0;
                break;
        }
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif


