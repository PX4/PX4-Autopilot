#include "../../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ENVIRONMENTS_L2F_OPERATIONS_GENERIC_SAMPLE_INITIAL_STATE_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENTS_L2F_OPERATIONS_GENERIC_SAMPLE_INITIAL_STATE_H

#include "../multirotor.h"

#include <rl_tools/utils/generic/vector_operations.h>
#include "../quaternion_helper.h"

#include <rl_tools/utils/generic/typing.h>

#include <rl_tools/rl/environments/operations_generic.h>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static void sample_initial_state(DEVICE& device, rl::environments::Multirotor<SPEC>& env, PARAMETERS& parameters, STATE& state, RNG& rng);
    namespace rl::environments::l2f{
        template <typename DEVICE, typename T, typename RNG>
        void sample_orientation(DEVICE& device, T limit, T output[4], RNG& rng){
            // Uniform sampling on the desired angle range
            T u = random::uniform_real_distribution(device.random, (T)0, (T)1, rng);
            T v = random::uniform_real_distribution(device.random, (T)0, (T)1, rng);
            T phi = 2.0 * math::PI<T> * u;
            T cos_theta = 1.0 - 2.0 * v;
            T sin_theta = math::sqrt(device.math, 1.0 - cos_theta*cos_theta);
            T x = sin_theta * math::cos(device.math, phi);
            T y = sin_theta * math::sin(device.math, phi);
            T z = cos_theta;
            T angle = random::uniform_real_distribution(device.random, (T)0, (T)1, rng);

            // Quaternion = [cos(angle/2), sin(angle/2)*axis]
            T half = 0.5 * angle;
            T s = math::sin(device.math, half);
            output[0] = math::cos(device.math, half);
            output[1] = x * s;
            output[2] = y * s;
            output[3] = z * s;
        }
        template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC, typename RNG>
        RL_TOOLS_FUNCTION_PLACEMENT static void _sample_initial_state(DEVICE& device, Multirotor<SPEC>& env, PARAMETERS& parameters, StateBase<STATE_SPEC>& state, RNG& rng, bool inherited_guidance = false){
            typename DEVICE::SPEC::MATH math_dev;
            typename DEVICE::SPEC::RANDOM random_dev;
            using STATE = StateBase<STATE_SPEC>;
            using T = typename STATE_SPEC::T;
            using TI = typename DEVICE::index_t;
            bool guidance;
            guidance = random::uniform_real_distribution(random_dev, (T)0, (T)1, rng) < parameters.mdp.init.guidance;
            if(!guidance){
                for(TI i = 0; i < 3; i++){
                    state.position[i] = random::uniform_real_distribution(random_dev, -parameters.mdp.init.max_position, parameters.mdp.init.max_position, rng);
                }
            }
            else{
                for(TI i = 0; i < 3; i++){
                    state.position[i] = 0;
                }
            }
            if(parameters.mdp.init.max_angle > 0 && !guidance){
                sample_orientation(device, parameters.mdp.init.max_angle, state.orientation, rng);
            }
            else{
                state.orientation[0] = 1;
                state.orientation[1] = 0;
                state.orientation[2] = 0;
                state.orientation[3] = 0;
            }
            if(!guidance) {
                for(TI i = 0; i < 3; i++){
                    state.linear_velocity[i] = random::uniform_real_distribution(random_dev, -parameters.mdp.init.max_linear_velocity, parameters.mdp.init.max_linear_velocity, rng);
                }
                for(TI i = 0; i < 3; i++){
                    state.angular_velocity[i] = random::uniform_real_distribution(random_dev, -parameters.mdp.init.max_angular_velocity, parameters.mdp.init.max_angular_velocity, rng);
                }
            }
            else{
                for(TI i = 0; i < 3; i++){
                    state.linear_velocity[i] = 0;
                }
                for(TI i = 0; i < 3; i++){
                    state.angular_velocity[i] = 0;
                }
            }
        }
        template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC, typename RNG>
        RL_TOOLS_FUNCTION_PLACEMENT static void _sample_initial_state(DEVICE& device, Multirotor<SPEC>& env, PARAMETERS& parameters, StateLastAction<STATE_SPEC>& state, RNG& rng){
            using TI = typename DEVICE::index_t;
            using STATE = StateLastAction<STATE_SPEC>;
            sample_initial_state(device, env, parameters, static_cast<typename STATE_SPEC::NEXT_COMPONENT&>(state), rng);
            for (TI action_i=0; action_i < STATE::ACTION_DIM; action_i++){
                state.last_action[action_i] = 0;
            }
        }
        template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC, typename RNG>
        RL_TOOLS_FUNCTION_PLACEMENT static void _sample_initial_state(DEVICE& device, Multirotor<SPEC>& env, PARAMETERS& parameters, StateLinearAcceleration<STATE_SPEC>& state, RNG& rng){
            using TI = typename DEVICE::index_t;
            sample_initial_state(device, env, parameters, static_cast<typename STATE_SPEC::NEXT_COMPONENT&>(state), rng);
            for(TI i = 0; i < 3; i++){
                state.linear_acceleration[i] = 0;
            }
        }
        template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC, typename RNG>
        RL_TOOLS_FUNCTION_PLACEMENT static void _sample_initial_state(DEVICE& device, Multirotor<SPEC>& env, PARAMETERS& parameters, StateAngularVelocityDelay<STATE_SPEC>& state, RNG& rng){
            using TI = typename DEVICE::index_t;
            using STATE = StateAngularVelocityDelay<STATE_SPEC>;
            sample_initial_state(device, env, parameters, static_cast<typename STATE_SPEC::NEXT_COMPONENT&>(state), rng);
            for (TI step_i=0; step_i < STATE::HISTORY_MEM_LENGTH; step_i++){
                for (TI dim_i=0; dim_i < 3; dim_i++){
                    state.angular_velocity_history[step_i][dim_i] = state.angular_velocity[dim_i];
                }
            }
        }
        template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC, typename RNG>
        RL_TOOLS_FUNCTION_PLACEMENT static void _sample_initial_state(DEVICE& device, Multirotor<SPEC>& env, PARAMETERS& parameters, StateLinearVelocityDelay<STATE_SPEC>& state, RNG& rng){
            using TI = typename DEVICE::index_t;
            using STATE = StateLinearVelocityDelay<STATE_SPEC>;
            sample_initial_state(device, env, parameters, static_cast<typename STATE_SPEC::NEXT_COMPONENT&>(state), rng);
            for (TI step_i=0; step_i < STATE::HISTORY_MEM_LENGTH; step_i++){
                for (TI dim_i=0; dim_i < 3; dim_i++){
                    state.linear_velocity_history[step_i][dim_i] = state.linear_velocity[dim_i];
                }
            }
        }
        template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC, typename RNG>
        RL_TOOLS_FUNCTION_PLACEMENT static void _sample_initial_state(DEVICE& device, Multirotor<SPEC>& env, PARAMETERS& parameters, StatePoseErrorIntegral<STATE_SPEC>& state, RNG& rng){
            using TI = typename DEVICE::index_t;
            sample_initial_state(device, env, parameters, static_cast<typename STATE_SPEC::NEXT_COMPONENT&>(state), rng);
            for (TI dim_i=0; dim_i<3; dim_i++){
                state.position_integral[dim_i] = 0;
            }
            state.orientation_integral = 0;
        }
        template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC, typename RNG>
        RL_TOOLS_FUNCTION_PLACEMENT static void _sample_initial_state(DEVICE& device, Multirotor<SPEC>& env, PARAMETERS& parameters, StateRandomForce<STATE_SPEC>& state, RNG& rng){
            typename DEVICE::SPEC::RANDOM random_dev;
            using T = typename SPEC::T;
    //        bool guidance = random::uniform_real_distribution(random_dev, (T)0, (T)1, rng) < parameters.mdp.init.guidance;
            sample_initial_state(device, env, parameters, static_cast<typename STATE_SPEC::NEXT_COMPONENT&>(state), rng);
    //        if(!guidance){
            {
                auto distribution = parameters.disturbances.random_force;
                state.force[0] = random::normal_distribution::sample(random_dev, (T)distribution.mean, (T)distribution.std, rng);
                state.force[1] = random::normal_distribution::sample(random_dev, (T)distribution.mean, (T)distribution.std, rng);
                state.force[2] = random::normal_distribution::sample(random_dev, (T)distribution.mean, (T)distribution.std, rng);
            }
            {
                auto distribution = parameters.disturbances.random_torque;
                state.torque[0] = random::normal_distribution::sample(random_dev, (T)distribution.mean, (T)distribution.std, rng);
                state.torque[1] = random::normal_distribution::sample(random_dev, (T)distribution.mean, (T)distribution.std, rng);
                state.torque[2] = random::normal_distribution::sample(random_dev, (T)distribution.mean, (T)distribution.std/100, rng);
            }
    //        }
    //        else{
    //            state.force[0] = 0;
    //            state.force[1] = 0;
    //            state.force[2] = 0;
    //            state.torque[0] = 0;
    //            state.torque[1] = 0;
    //            state.torque[2] = 0;
    //        }

        }
        template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC, typename RNG>
        RL_TOOLS_FUNCTION_PLACEMENT static void _sample_initial_state(DEVICE& device, Multirotor<SPEC>& env, PARAMETERS& parameters, StateRandomOrientationOffset<STATE_SPEC>& state, RNG& rng){
            typename DEVICE::SPEC::RANDOM random_dev;
            using T = typename SPEC::T;
            sample_initial_state(device, env, parameters, static_cast<typename STATE_SPEC::NEXT_COMPONENT&>(state), rng);
            sample_orientation(device, parameters.domain_randomization, state.orientation, rng);
        }
        template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC, typename RNG>
        RL_TOOLS_FUNCTION_PLACEMENT static void _sample_initial_state(DEVICE& device, Multirotor<SPEC>& env, PARAMETERS& parameters, StateRotors<STATE_SPEC>& state, RNG& rng){
            sample_initial_state(device, env, parameters, static_cast<typename STATE_SPEC::NEXT_COMPONENT&>(state), rng);
            using TI = typename DEVICE::index_t;
            using T = typename SPEC::T;
            T min_rpm, max_rpm;
            if(parameters.mdp.init.relative_rpm){
                min_rpm = (parameters.mdp.init.min_rpm + 1)/2 * (parameters.dynamics.action_limit.max - parameters.dynamics.action_limit.min) + parameters.dynamics.action_limit.min;
                max_rpm = (parameters.mdp.init.max_rpm + 1)/2 * (parameters.dynamics.action_limit.max - parameters.dynamics.action_limit.min) + parameters.dynamics.action_limit.min;
            }
            else{
                min_rpm = parameters.mdp.init.min_rpm < 0 ? parameters.dynamics.action_limit.min : parameters.mdp.init.min_rpm;
                max_rpm = parameters.mdp.init.max_rpm < 0 ? parameters.dynamics.action_limit.max : parameters.mdp.init.max_rpm;
                if(max_rpm > parameters.dynamics.action_limit.max){
                    max_rpm = parameters.dynamics.action_limit.max;
                }
                if(min_rpm > max_rpm){
                    min_rpm = max_rpm;
                }
            }
            for(TI i = 0; i < 4; i++){
                state.rpm[i] = random::uniform_real_distribution(typename DEVICE::SPEC::RANDOM(), min_rpm, max_rpm, rng);
            }
        }
        template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC, typename RNG>
        RL_TOOLS_FUNCTION_PLACEMENT static void _sample_initial_state(DEVICE& device, Multirotor<SPEC>& env, PARAMETERS& parameters, StateRotorsHistory<STATE_SPEC>& state, RNG& rng){
            using MULTIROTOR = Multirotor<SPEC>;
            using TI = typename DEVICE::index_t;
            using STATE = StateRotorsHistory<STATE_SPEC>;
            sample_initial_state(device, env, parameters, static_cast<typename STATE::NEXT_COMPONENT&>(state), rng);
            state.current_step = 0;
            for(TI step_i = 0; step_i < STATE_SPEC::HISTORY_LENGTH; step_i++){
                for(TI action_i = 0; action_i < MULTIROTOR::ACTION_DIM; action_i++){
                    state.action_history[step_i][action_i] = (state.rpm[action_i] - parameters.dynamics.action_limit.min) / (parameters.dynamics.action_limit.max - parameters.dynamics.action_limit.min) * 2 - 1;
                }
            }
        }
        template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC, typename RNG>
        RL_TOOLS_FUNCTION_PLACEMENT static void _sample_initial_state(DEVICE& device, Multirotor<SPEC>& env, PARAMETERS& parameters, StateTrajectory<STATE_SPEC>& state, RNG& rng){
            using MULTIROTOR = Multirotor<SPEC>;
            using TI = typename DEVICE::index_t;
            using T = typename SPEC::T;
            using STATE = StateTrajectory<STATE_SPEC>;
            using OPTS = typename PARAMETERS::TRAJECTORY_OPTIONS;
            sample_initial_state(device, env, parameters, static_cast<typename STATE::NEXT_COMPONENT&>(state), rng);
            if constexpr(OPTS::LANGEVIN){
                T threshold = random::uniform_real_distribution(device.random, (T)0, (T)1, rng);
                T acc = 0;
                state.trajectory.type = POSITION;
                for(TI type_i = 0; type_i < decltype(parameters.trajectory)::MIXTURE_N; type_i++){
                    acc += parameters.trajectory.mixture[type_i];
                    if(threshold < acc){
                        state.trajectory.type = static_cast<TrajectoryType>(type_i);
                        break;
                    }
                }
                switch(state.trajectory.type){
                    case POSITION:
                        break;
                    case LANGEVIN:
                        for(TI dim_i = 0; dim_i < 3; dim_i++){
                            state.trajectory.langevin.position[dim_i] = 0;
                            state.trajectory.langevin.velocity[dim_i] = 0;
                            state.trajectory.langevin.position_raw[dim_i] = 0;
                            state.trajectory.langevin.velocity_raw[dim_i] = 0;
                        }
                }
            }
        }
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif

