#include "../../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ENVIRONMENTS_L2F_OPERATIONS_GENERIC_INITIAL_STATE_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENTS_L2F_OPERATIONS_GENERIC_INITIAL_STATE_H

#include "../multirotor.h"

#include <rl_tools/utils/generic/vector_operations.h>
#include "../quaternion_helper.h"

#include <rl_tools/utils/generic/typing.h>

#include <rl_tools/rl/environments/operations_generic.h>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE>
    static void initial_state(DEVICE& device, rl::environments::Multirotor<SPEC>& env, PARAMETERS& parameters, STATE& state);
    namespace rl::environments::l2f{
        template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC>
        static void _initial_state(DEVICE& device, rl::environments::Multirotor<SPEC>& env, PARAMETERS& parameters, rl::environments::l2f::StateBase<STATE_SPEC>& state){
            using TI = typename DEVICE::index_t;
            for(TI i = 0; i < 3; i++){
                state.position[i] = 0;
            }
            state.orientation[0] = 1;
            for(TI i = 1; i < 4; i++){
                state.orientation[i] = 0;
            }
            for(TI i = 0; i < 3; i++){
                state.linear_velocity[i] = 0;
            }
            for(TI i = 0; i < 3; i++){
                state.angular_velocity[i] = 0;
            }
        }
        template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC>
        static void _initial_state(DEVICE& device, rl::environments::Multirotor<SPEC>& env, PARAMETERS& parameters, rl::environments::l2f::StateLastAction<STATE_SPEC>& state){
            using TI = typename DEVICE::index_t;
            using STATE = rl::environments::l2f::StateLastAction<STATE_SPEC>;
            initial_state(device, env, parameters, static_cast<typename STATE_SPEC::NEXT_COMPONENT&>(state));
            for (TI action_i=0; action_i < STATE::ACTION_DIM; action_i++){
                state.last_action[action_i] = 0;
            }
        }
        template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC, typename NEXT_COMPONENT>
        static void _initial_state(DEVICE& device, rl::environments::Multirotor<SPEC>& env, PARAMETERS& parameters, rl::environments::l2f::StateLinearAcceleration<STATE_SPEC>& state){
            using TI = typename DEVICE::index_t;
            initial_state(device, env, parameters, static_cast<NEXT_COMPONENT&>(state));
            for(TI i = 0; i < 3; i++){
                state.linear_acceleration[i] = 0;
            }
        }
        template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC>
        static void _initial_state(DEVICE& device, rl::environments::Multirotor<SPEC>& env, PARAMETERS& parameters, rl::environments::l2f::StateAngularVelocityDelay<STATE_SPEC>& state){
            using TI = typename DEVICE::index_t;
            using STATE = rl::environments::l2f::StateAngularVelocityDelay<STATE_SPEC>;
            initial_state(device, env, parameters, static_cast<typename STATE_SPEC::NEXT_COMPONENT&>(state));
            for (TI step_i=0; step_i < STATE::HISTORY_MEM_LENGTH; step_i++){
                for (TI dim_i=0; dim_i < 3; dim_i++){
                    state.angular_velocity_history[step_i][dim_i] = state.angular_velocity[dim_i];
                }
            }
        }
        template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC>
        static void _initial_state(DEVICE& device, rl::environments::Multirotor<SPEC>& env, PARAMETERS& parameters, rl::environments::l2f::StateLinearVelocityDelay<STATE_SPEC>& state){
            using TI = typename DEVICE::index_t;
            using STATE = rl::environments::l2f::StateLinearVelocityDelay<STATE_SPEC>;
            initial_state(device, env, parameters, static_cast<typename STATE_SPEC::NEXT_COMPONENT&>(state));
            for (TI step_i=0; step_i < STATE::HISTORY_MEM_LENGTH; step_i++){
                for (TI dim_i=0; dim_i < 3; dim_i++){
                    state.linear_velocity_history[step_i][dim_i] = state.linear_velocity[dim_i];
                }
            }
        }
        template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC>
        static void _initial_state(DEVICE& device, rl::environments::Multirotor<SPEC>& env, PARAMETERS& parameters, rl::environments::l2f::StatePoseErrorIntegral<STATE_SPEC>& state){
            using TI = typename DEVICE::index_t;
            initial_state(device, env, parameters, static_cast<typename STATE_SPEC::NEXT_COMPONENT&>(state));
            for (TI dim_i=0; dim_i<3; dim_i++){
                state.position_integral[dim_i] = 0;
            }
            state.orientation_integral = 0;
        }
        template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC>
        static void _initial_state(DEVICE& device, rl::environments::Multirotor<SPEC>& env, PARAMETERS& parameters, rl::environments::l2f::StateRandomForce<STATE_SPEC>& state){
            initial_state(device, env, parameters, static_cast<typename STATE_SPEC::NEXT_COMPONENT&>(state));
            state.force[0] = 0;
            state.force[1] = 0;
            state.force[2] = 0;
            state.torque[0] = 0;
            state.torque[1] = 0;
            state.torque[2] = 0;
        }
        template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC>
        static void _initial_state(DEVICE& device, rl::environments::Multirotor<SPEC>& env, PARAMETERS& parameters, rl::environments::l2f::StateRandomOrientationOffset<STATE_SPEC>& state){
            initial_state(device, env, parameters, static_cast<typename STATE_SPEC::NEXT_COMPONENT&>(state));
            state.orientation_offset[0] = 1;
            state.orientation_offset[1] = 0;
            state.orientation_offset[2] = 0;
            state.orientation_offset[3] = 0;
        }
        template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC>
        static void _initial_state(DEVICE& device, rl::environments::Multirotor<SPEC>& env, PARAMETERS& parameters, rl::environments::l2f::StateRotors<STATE_SPEC>& state){
            initial_state(device, env, parameters, static_cast<typename STATE_SPEC::NEXT_COMPONENT&>(state));
            for(typename DEVICE::index_t i = 0; i < 4; i++){
    //            state.rpm[i] = (parameters.dynamics.action_limit.max - parameters.dynamics.action_limit.min) / 2 + parameters.dynamics.action_limit.min;
                state.rpm[i] = parameters.dynamics.hovering_throttle_relative * (parameters.dynamics.action_limit.max - parameters.dynamics.action_limit.min) + parameters.dynamics.action_limit.min;
            }
        }
        template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC>
        static void _initial_state(DEVICE& device, rl::environments::Multirotor<SPEC>& env, PARAMETERS& parameters, rl::environments::l2f::StateRotorsHistory<STATE_SPEC>& state){
            using TI = typename DEVICE::index_t;
            using STATE = rl::environments::l2f::StateRotorsHistory<STATE_SPEC>;
            using MULTIROTOR = rl::environments::Multirotor<SPEC>;
            initial_state(device, env, parameters, static_cast<typename STATE::NEXT_COMPONENT&>(state));
            state.current_step = 0;
            for(TI step_i = 0; step_i < STATE_SPEC::HISTORY_LENGTH; step_i++){
                for(TI action_i = 0; action_i < MULTIROTOR::ACTION_DIM; action_i++){
                    state.action_history[step_i][action_i] = (state.rpm[action_i] - parameters.dynamics.action_limit.min) / (parameters.dynamics.action_limit.max - parameters.dynamics.action_limit.min) * 2 - 1;
                }
            }
        }
        template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC>
        static void _initial_state(DEVICE& device, rl::environments::Multirotor<SPEC>& env, PARAMETERS& parameters, rl::environments::l2f::StateTrajectory<STATE_SPEC>& state){
            using TI = typename DEVICE::index_t;
            using STATE = rl::environments::l2f::StateTrajectory<STATE_SPEC>;
            using MULTIROTOR = rl::environments::Multirotor<SPEC>;
            initial_state(device, env, parameters, static_cast<typename STATE::NEXT_COMPONENT&>(state));
            state.current_step = 0;
            state.trajectory.type = POSITION;
        }
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif
