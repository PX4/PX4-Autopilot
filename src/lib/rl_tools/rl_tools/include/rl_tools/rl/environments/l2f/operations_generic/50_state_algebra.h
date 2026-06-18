#include "../../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ENVIRONMENTS_L2F_OPERATIONS_GENERIC_STATE_ALGEBRA_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENTS_L2F_OPERATIONS_GENERIC_STATE_ALGEBRA_H

#include "../multirotor.h"

#include <rl_tools/utils/generic/vector_operations.h>
#include "../quaternion_helper.h"

#include <rl_tools/utils/generic/typing.h>

#include <rl_tools/rl/environments/operations_generic.h>

// This file contains algebraic operations for states that REQUIRE_INTEGRATION.

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::environments::l2f{
    // State arithmetic for RK4 integration

    template<typename DEVICE, typename STATE_SPEC, typename T>
    RL_TOOLS_FUNCTION_PLACEMENT static void scalar_multiply_accumulate(DEVICE& device, const StateBase<STATE_SPEC>& state, T scalar, StateBase<STATE_SPEC>& out){
        for(int i = 0; i < 3; ++i){
            out.position[i]         += scalar * state.position[i]        ;
            out.orientation[i]      += scalar * state.orientation[i]     ;
            out.linear_velocity[i]  += scalar * state.linear_velocity[i] ;
            out.angular_velocity[i] += scalar * state.angular_velocity[i];
        }
        out.orientation[3] += scalar * state.orientation[3];
    }
    template<typename DEVICE, typename STATE_SPEC, typename T>
    RL_TOOLS_FUNCTION_PLACEMENT static void scalar_multiply_accumulate(DEVICE& device, const StatePoseErrorIntegral<STATE_SPEC>& state, T scalar, StatePoseErrorIntegral<STATE_SPEC>& out){
        using TI = typename DEVICE::index_t;
        scalar_multiply_accumulate(device, static_cast<const typename STATE_SPEC::NEXT_COMPONENT&>(state), scalar, static_cast<typename STATE_SPEC::NEXT_COMPONENT&>(out));
        for (TI dim_i=0; dim_i<3; dim_i++){
            out.position_integral[dim_i] += scalar * state.position_integral[dim_i];
        }
        out.orientation_integral += scalar * state.orientation_integral;
    }
    template<typename DEVICE, typename STATE_SPEC, typename T>
    RL_TOOLS_FUNCTION_PLACEMENT static void scalar_multiply_accumulate(DEVICE& device, const StateRotors<STATE_SPEC>& state, T scalar, StateRotors<STATE_SPEC>& out){
        scalar_multiply_accumulate(device, static_cast<const typename STATE_SPEC::NEXT_COMPONENT&>(state), scalar, static_cast<typename STATE_SPEC::NEXT_COMPONENT&>(out));
        if constexpr(!STATE_SPEC::CLOSED_FORM) {
            for(int i = 0; i < 4; ++i){
                out.rpm[i] += scalar * state.rpm[i];
            }
        }
    }
    template<typename DEVICE, typename STATE, typename T>
    RL_TOOLS_FUNCTION_PLACEMENT static void scalar_multiply_accumulate(DEVICE& device, const STATE& state, T scalar, STATE& out, rl_tools::utils::typing::enable_if_t<!STATE::REQUIRES_INTEGRATION, bool> disable = false){
        static_assert(!STATE::REQUIRES_INTEGRATION);
        scalar_multiply_accumulate(device, static_cast<const typename STATE::NEXT_COMPONENT&>(state), scalar, static_cast<typename STATE::NEXT_COMPONENT&>(out));
    }

}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif