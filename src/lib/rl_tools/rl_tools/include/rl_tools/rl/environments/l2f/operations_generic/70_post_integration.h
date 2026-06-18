#include "../../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ENVIRONMENTS_L2F_OPERATIONS_GENERIC_POST_INTEGRATION_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENTS_L2F_OPERATIONS_GENERIC_POST_INTEGRATION_H

#include "../multirotor.h"

#include <rl_tools/utils/generic/vector_operations.h>
#include "../quaternion_helper.h"

#include <rl_tools/utils/generic/typing.h>

#include <rl_tools/rl/environments/operations_generic.h>

// This file contains functions for parts of the state that do not evolve through integration but through discrete steps or for refinement operations after integration (e.g. normalizing the quaternion)

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::environments::l2f{
    template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC, typename ACTION_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT void post_integration(DEVICE& device, const Multirotor<SPEC>& env, PARAMETERS& parameters, const StateBase<STATE_SPEC>& state, const Matrix<ACTION_SPEC>& action, StateBase<STATE_SPEC>& next_state, RNG& rng) {
        using T = typename STATE_SPEC::T;
        using TI = typename DEVICE::index_t;
        T quaternion_norm = 0;
        for(TI state_i = 0; state_i < 4; state_i++){
            quaternion_norm += next_state.orientation[state_i] * next_state.orientation[state_i];
        }
        quaternion_norm = math::sqrt(device.math, quaternion_norm);
        for(TI state_i = 0; state_i < 4; state_i++){
            next_state.orientation[state_i] /= quaternion_norm;
        }
        for(TI dim_i=0; dim_i < 3; dim_i++){
            using STATIC_PARAMETERS = typename SPEC::STATIC_PARAMETERS;
            next_state.position[dim_i]         = math::clamp(device.math, next_state.position[dim_i]       , -STATIC_PARAMETERS::STATE_LIMIT_POSITION, STATIC_PARAMETERS::STATE_LIMIT_POSITION);
            next_state.linear_velocity[dim_i]  = math::clamp(device.math, next_state.linear_velocity[dim_i], -STATIC_PARAMETERS::STATE_LIMIT_VELOCITY, STATIC_PARAMETERS::STATE_LIMIT_VELOCITY);
            next_state.angular_velocity[dim_i] = math::clamp(device.math, next_state.angular_velocity[dim_i], -STATIC_PARAMETERS::STATE_LIMIT_ANGULAR_VELOCITY, STATIC_PARAMETERS::STATE_LIMIT_ANGULAR_VELOCITY);
        }

    }
    template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC, typename ACTION_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT void post_integration(DEVICE& device, const Multirotor<SPEC>& env, PARAMETERS& parameters, const StateLastAction<STATE_SPEC>& state, const Matrix<ACTION_SPEC>& action, StateLastAction<STATE_SPEC>& next_state, RNG& rng) {
        using MULTIROTOR = Multirotor<SPEC>;
        using TI = typename DEVICE::index_t;
        static_assert(ACTION_SPEC::COLS == MULTIROTOR::ACTION_DIM);
        post_integration(device, env, parameters, static_cast<const typename STATE_SPEC::NEXT_COMPONENT&>(state), action, static_cast<typename STATE_SPEC::NEXT_COMPONENT&>(next_state), rng);
        for(TI action_i = 0; action_i < MULTIROTOR::ACTION_DIM; action_i++){
            next_state.last_action[action_i] = get(action, 0, action_i);
        }
    }
    template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC, typename ACTION_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT void post_integration(DEVICE& device, const Multirotor<SPEC>& env, PARAMETERS& parameters, const StateLinearAcceleration<STATE_SPEC>& state, const Matrix<ACTION_SPEC>& action, StateLinearAcceleration<STATE_SPEC>& next_state, RNG& rng) {
        using T = typename STATE_SPEC::T;
        using TI = typename DEVICE::index_t;
        post_integration(device, env, parameters, static_cast<const typename STATE_SPEC::NEXT_COMPONENT&>(state), action, static_cast<typename STATE_SPEC::NEXT_COMPONENT&>(next_state), rng);
        for(TI state_i = 0; state_i < 3; state_i++){
            next_state.linear_acceleration[state_i] = (next_state.linear_velocity[state_i] - state.linear_velocity[state_i])/parameters.integration.dt;
        }
    }
    template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC, typename ACTION_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT void post_integration(DEVICE& device, const Multirotor<SPEC>& env, PARAMETERS& parameters, const StateAngularVelocityDelay<STATE_SPEC>& state, const Matrix<ACTION_SPEC>& action, StateAngularVelocityDelay<STATE_SPEC>& next_state, RNG& rng) {
        using TI = typename DEVICE::index_t;
        post_integration(device, env, parameters, static_cast<const typename STATE_SPEC::NEXT_COMPONENT&>(state), action, static_cast<typename STATE_SPEC::NEXT_COMPONENT&>(next_state), rng);

        if constexpr (STATE_SPEC::HISTORY_LENGTH == 0){
            for(TI dim_i = 0; dim_i < 3; dim_i++){
                next_state.angular_velocity_history[0][dim_i] = next_state.angular_velocity[dim_i];
            }
        }
        else
        {
            for(TI step_i = 0; step_i < STATE_SPEC::HISTORY_LENGTH; step_i++){
                for(TI dim_i = 0; dim_i < 3; dim_i++){
                    if (step_i == (STATE_SPEC::HISTORY_LENGTH - 1)){
                        next_state.angular_velocity_history[STATE_SPEC::HISTORY_LENGTH-1][dim_i] = state.angular_velocity[dim_i];
                    }
                    else{
                        next_state.angular_velocity_history[step_i][dim_i] = state.angular_velocity_history[step_i+1][dim_i];
                    }
                }
            }
        }
    }
    template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC, typename ACTION_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT void post_integration(DEVICE& device, const Multirotor<SPEC>& env, PARAMETERS& parameters, const StateLinearVelocityDelay<STATE_SPEC>& state, const Matrix<ACTION_SPEC>& action, StateLinearVelocityDelay<STATE_SPEC>& next_state, RNG& rng) {
        using TI = typename DEVICE::index_t;
        post_integration(device, env, parameters, static_cast<const typename STATE_SPEC::NEXT_COMPONENT&>(state), action, static_cast<typename STATE_SPEC::NEXT_COMPONENT&>(next_state), rng);

        if constexpr (STATE_SPEC::HISTORY_LENGTH == 0){
            for(TI dim_i = 0; dim_i < 3; dim_i++){
                next_state.linear_velocity_history[0][dim_i] = next_state.linear_velocity[dim_i];
            }
        }
        else
        {
            for(TI step_i = 0; step_i < STATE_SPEC::HISTORY_LENGTH; step_i++){
                for(TI dim_i = 0; dim_i < 3; dim_i++){
                    if (step_i == (STATE_SPEC::HISTORY_LENGTH - 1)){
                        next_state.linear_velocity_history[STATE_SPEC::HISTORY_LENGTH-1][dim_i] = state.linear_velocity[dim_i];
                    }
                    else{
                        next_state.linear_velocity_history[step_i][dim_i] = state.linear_velocity_history[step_i+1][dim_i];
                    }
                }
            }
        }
    }
    //    template<typename DEVICE, typename SPEC, typename T, typename TI, typename NEXT_COMPONENT>
//    RL_TOOLS_FUNCTION_PLACEMENT void post_integration(DEVICE& device, const Multirotor<SPEC>& env, StateRotors<STATE_SPEC>& state) {
    template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC, typename ACTION_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT void post_integration(DEVICE& device, const Multirotor<SPEC>& env, PARAMETERS& parameters, const StateRotors<STATE_SPEC>& state, const Matrix<ACTION_SPEC>& action, StateRotors<STATE_SPEC>& next_state, RNG& rng) {
        post_integration(device, env, parameters, static_cast<const typename STATE_SPEC::NEXT_COMPONENT&>(state), action, static_cast<typename STATE_SPEC::NEXT_COMPONENT&>(next_state), rng);
        using MULTIROTOR = Multirotor<SPEC>;
        using T = typename STATE_SPEC::T;
        for(typename DEVICE::index_t rpm_i = 0; rpm_i < MULTIROTOR::ACTION_DIM; rpm_i++){
            if constexpr(STATE_SPEC::CLOSED_FORM) {
                T setpoint_clamped = math::clamp(typename DEVICE::SPEC::MATH{}, get(action, 0, rpm_i), parameters.dynamics.action_limit.min, parameters.dynamics.action_limit.max);
                T tau = setpoint_clamped >= state.rpm[rpm_i] ? parameters.dynamics.rotor_time_constants_rising[rpm_i] : parameters.dynamics.rotor_time_constants_falling[rpm_i] ;
                T alpha = math::exp(device.math, - parameters.integration.dt / tau);
                next_state.rpm[rpm_i] = alpha * state.rpm[rpm_i] + (1 - alpha) * setpoint_clamped;
            }
            else {
                next_state.rpm[rpm_i] = math::clamp(typename DEVICE::SPEC::MATH{}, next_state.rpm[rpm_i], parameters.dynamics.action_limit.min, parameters.dynamics.action_limit.max);
            }
        }
    }
    template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC, typename ACTION_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT void post_integration(DEVICE& device, const Multirotor<SPEC>& env, PARAMETERS& parameters, const StateRandomForce<STATE_SPEC>& state, const Matrix<ACTION_SPEC>& action, StateRandomForce<STATE_SPEC>& next_state, RNG& rng) {
        post_integration(device, env, parameters, static_cast<const typename STATE_SPEC::NEXT_COMPONENT&>(state), action, static_cast<typename STATE_SPEC::NEXT_COMPONENT&>(next_state), rng);
        next_state.force[0] = state.force[0];
        next_state.force[1] = state.force[1];
        next_state.force[2] = state.force[2];
        next_state.torque[0] = state.torque[0];
        next_state.torque[1] = state.torque[1];
        next_state.torque[2] = state.torque[2];
    }
    template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC, typename ACTION_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT void post_integration(DEVICE& device, const Multirotor<SPEC>& env, PARAMETERS& parameters, const StateRotorsHistory<STATE_SPEC>& state, const Matrix<ACTION_SPEC>& action, StateRotorsHistory<STATE_SPEC>& next_state, RNG& rng) {
        using MULTIROTOR = Multirotor<SPEC>;
        using TI = typename DEVICE::index_t;
        using STATE = StateRotorsHistory<STATE_SPEC>;
        static_assert(ACTION_SPEC::COLS == MULTIROTOR::ACTION_DIM);
        post_integration(device, env, parameters, static_cast<const typename STATE::NEXT_COMPONENT&>(state), action, static_cast<typename STATE::NEXT_COMPONENT&>(next_state), rng);
        if constexpr(STATE_SPEC::HISTORY_LENGTH > 0){
            TI current_step = state.current_step;
            for(TI action_i = 0; action_i < MULTIROTOR::ACTION_DIM; action_i++){
                next_state.action_history[current_step][action_i] = get(action, 0, action_i);
            }
            next_state.current_step = (state.current_step + 1) % STATE_SPEC::HISTORY_LENGTH;
        }
    }
    template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC, typename ACTION_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT void post_integration(DEVICE& device, const Multirotor<SPEC>& env, PARAMETERS& parameters, const StateTrajectory<STATE_SPEC>& state, const Matrix<ACTION_SPEC>& action, StateTrajectory<STATE_SPEC>& next_state, RNG& rng) {
        using MULTIROTOR = Multirotor<SPEC>;
        using TI = typename DEVICE::index_t;
        using T = typename STATE_SPEC::T;
        using STATE = StateTrajectory<STATE_SPEC>;
        using OPTS = typename PARAMETERS::TRAJECTORY_OPTIONS;
        static_assert(ACTION_SPEC::COLS == MULTIROTOR::ACTION_DIM);
        post_integration(device, env, parameters, static_cast<const typename STATE::NEXT_COMPONENT&>(state), action, static_cast<typename STATE::NEXT_COMPONENT&>(next_state), rng);
        if constexpr(OPTS::LANGEVIN){
            switch(state.trajectory.type){
                case POSITION:
                    break;
                case LANGEVIN: {
                        // todo put this into RK4?
                        const T gamma = parameters.trajectory.langevin.gamma;
                        const T omega = parameters.trajectory.langevin.omega;
                        const T sigma = parameters.trajectory.langevin.sigma;
                        const T dt    = parameters.integration.dt;
                        const T alpha = parameters.trajectory.langevin.alpha;

                        const T sqrt_dt = math::sqrt(device.math, dt);
                        for (TI dim_i = 0; dim_i < 3; ++dim_i){
                            const T x_prev = state.trajectory.langevin.position_raw[dim_i];
                            const T v_prev = state.trajectory.langevin.velocity_raw[dim_i];
                            const T dW = sqrt_dt * random::normal_distribution::sample(device.random, T(0), T(1), rng);
                            const T v_next = v_prev + (-gamma * v_prev - omega * omega * x_prev) * dt + sigma * dW;
                            const T x_next = x_prev + v_next * dt;
                            next_state.trajectory.langevin.position_raw[dim_i] = x_next;
                            next_state.trajectory.langevin.velocity_raw[dim_i] = v_next;
                            const T v_smooth_prev = state.trajectory.langevin.velocity[dim_i];
                            const T v_smooth = alpha * v_next + (T(1) - alpha) * v_smooth_prev;
                            const T x_smooth_prev = state.trajectory.langevin.position[dim_i];
                            const T x_smooth = x_smooth_prev + v_smooth * dt;
                            next_state.trajectory.langevin.position[dim_i] = x_smooth;
                            next_state.trajectory.langevin.velocity[dim_i] = v_smooth;
                        }
                    }
                    break;
                default:
                    break;
            }
        }
    }

}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
