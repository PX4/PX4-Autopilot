
#include "../../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ENVIRONMENTS_L2F_OPERATIONS_GENERIC_OBSERVE_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENTS_L2F_OPERATIONS_GENERIC_OBSERVE_H

#include "../multirotor.h"

#include <rl_tools/utils/generic/vector_operations.h>
#include "../quaternion_helper.h"

#include <rl_tools/utils/generic/typing.h>

#include <rl_tools/rl/environments/operations_generic.h>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE, typename OBSERVATION, typename OBS_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static void observe(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, PARAMETERS& parameters, const STATE& state, const OBSERVATION& observation_type, Matrix<OBS_SPEC>& observation, RNG& rng);
    namespace rl::environments::l2f{
        template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE, typename OBSERVATION_TI, typename OBS_SPEC, typename RNG>
        RL_TOOLS_FUNCTION_PLACEMENT static void _observe(DEVICE& device, const Multirotor<SPEC>& env, PARAMETERS& parameters, const STATE& state, observation::LastComponent<OBSERVATION_TI>, Matrix<OBS_SPEC>& observation, RNG& rng){
            static_assert(OBS_SPEC::COLS == 0);
            static_assert(OBS_SPEC::ROWS == 1);
        }
        template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE, typename OBSERVATION_SPEC, typename OBS_SPEC, typename RNG>
        RL_TOOLS_FUNCTION_PLACEMENT static void _observe(DEVICE& device, const Multirotor<SPEC>& env, PARAMETERS& parameters, const STATE& state, observation::PoseIntegral<OBSERVATION_SPEC>, Matrix<OBS_SPEC>& observation, RNG& rng){
            using T = typename SPEC::T;
            using TI = typename DEVICE::index_t;
            using OBSERVATION = observation::PoseIntegral<OBSERVATION_SPEC>;
            static_assert(OBS_SPEC::COLS >= OBSERVATION::CURRENT_DIM);
            static_assert(OBS_SPEC::ROWS == 1);
            for (TI dim_i=0; dim_i<3; dim_i++){
                set(observation, 0, dim_i, state.position_integral[dim_i]);
            }
            set(observation, 0, 3, state.orientation_integral);
            auto current_observation = view(device, observation, matrix::ViewSpec<1, OBSERVATION::CURRENT_DIM>{}, 0, 0);
            auto next_observation = view(device, observation, matrix::ViewSpec<1, OBS_SPEC::COLS - OBSERVATION::CURRENT_DIM>{}, 0, OBSERVATION::CURRENT_DIM);
            observe(device, env, parameters, state, typename OBSERVATION::NEXT_COMPONENT{}, next_observation, rng);
        }
        template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE, typename OBSERVATION_SPEC, typename OBS_SPEC, typename RNG>
        RL_TOOLS_FUNCTION_PLACEMENT static void _observe(DEVICE& device, const Multirotor<SPEC>& env, PARAMETERS& parameters, const STATE& state, observation::Position<OBSERVATION_SPEC>, Matrix<OBS_SPEC>& observation, RNG& rng){
            using OBSERVATION = observation::Position<OBSERVATION_SPEC>;
            static_assert(OBS_SPEC::COLS >= OBSERVATION::CURRENT_DIM);
            static_assert(OBS_SPEC::ROWS == 1);
            using T = typename SPEC::T;
            using TI = typename DEVICE::index_t;

            for(TI i = 0; i < 3; i++){
                if constexpr(OBSERVATION_SPEC::PRIVILEGED && !SPEC::STATIC_PARAMETERS::PRIVILEGED_OBSERVATION_NOISE){
                    set(observation, 0, i, state.position[i]);
                }
                else{
                    T noise = random::normal_distribution::sample(typename DEVICE::SPEC::RANDOM{}, (T)0, parameters.mdp.observation_noise.position, rng);
                    set(observation, 0, i, state.position[i] + noise);
                }
            }
            auto next_observation = view(device, observation, matrix::ViewSpec<1, OBS_SPEC::COLS - OBSERVATION::CURRENT_DIM>{}, 0, OBSERVATION::CURRENT_DIM);
            observe(device, env, parameters, state, typename OBSERVATION::NEXT_COMPONENT{}, next_observation, rng);
        }
        template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE, typename OBSERVATION_SPEC, typename OBS_SPEC, typename RNG>
        RL_TOOLS_FUNCTION_PLACEMENT static void _observe(DEVICE& device, const Multirotor<SPEC>& env, PARAMETERS& parameters, const STATE& state, observation::OrientationQuaternion<OBSERVATION_SPEC>, Matrix<OBS_SPEC>& observation, RNG& rng){
            using T = typename SPEC::T;
            using TI = typename DEVICE::index_t;
            using OBSERVATION = observation::OrientationQuaternion<OBSERVATION_SPEC>;
            static_assert(OBS_SPEC::COLS >= OBSERVATION::CURRENT_DIM);
            static_assert(OBS_SPEC::ROWS == 1);
            for(TI i = 0; i < OBSERVATION::CURRENT_DIM; i++){
                if constexpr(OBSERVATION_SPEC::PRIVILEGED && !SPEC::STATIC_PARAMETERS::PRIVILEGED_OBSERVATION_NOISE){
                    set(observation, 0, i, state.orientation[i]);
                }
                else{
                    T noise = random::normal_distribution::sample(typename DEVICE::SPEC::RANDOM{}, (T)0, parameters.mdp.observation_noise.orientation, rng);
                    set(observation, 0, i, state.orientation[i] + noise);
                }
            }
            auto next_observation = view(device, observation, matrix::ViewSpec<1, OBS_SPEC::COLS - OBSERVATION::CURRENT_DIM>{}, 0, OBSERVATION::CURRENT_DIM);
            observe(device, env, parameters, state, typename OBSERVATION::NEXT_COMPONENT{}, next_observation, rng);
        }
        template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE, typename OBSERVATION_SPEC, typename OBS_SPEC, typename RNG>
        RL_TOOLS_FUNCTION_PLACEMENT static void _observe(DEVICE& device, const Multirotor<SPEC>& env, PARAMETERS& parameters, const STATE& state, observation::OrientationRotationMatrix<OBSERVATION_SPEC>, Matrix<OBS_SPEC>& observation, RNG& rng){
            using T = typename SPEC::T;
            using TI = typename DEVICE::index_t;
            using OBSERVATION = observation::OrientationRotationMatrix<OBSERVATION_SPEC>;
            static_assert(OBS_SPEC::COLS >= OBSERVATION::CURRENT_DIM);
            static_assert(OBS_SPEC::ROWS == 1);
            const typename SPEC::T* q = state.orientation;
            set(observation, 0, 0, (1 - 2*q[2]*q[2] - 2*q[3]*q[3]));
            set(observation, 0, 1, (    2*q[1]*q[2] - 2*q[0]*q[3]));
            set(observation, 0, 2, (    2*q[1]*q[3] + 2*q[0]*q[2]));
            set(observation, 0, 3, (    2*q[1]*q[2] + 2*q[0]*q[3]));
            set(observation, 0, 4, (1 - 2*q[1]*q[1] - 2*q[3]*q[3]));
            set(observation, 0, 5, (    2*q[2]*q[3] - 2*q[0]*q[1]));
            set(observation, 0, 6, (    2*q[1]*q[3] - 2*q[0]*q[2]));
            set(observation, 0, 7, (    2*q[2]*q[3] + 2*q[0]*q[1]));
            set(observation, 0, 8, (1 - 2*q[1]*q[1] - 2*q[2]*q[2]));
            if constexpr(!OBSERVATION_SPEC::PRIVILEGED || SPEC::STATIC_PARAMETERS::PRIVILEGED_OBSERVATION_NOISE){
                for(TI i = 0; i < OBSERVATION::CURRENT_DIM; i++){
                    T noise;
                    noise = random::normal_distribution::sample(typename DEVICE::SPEC::RANDOM(), (T)0, parameters.mdp.observation_noise.orientation, rng);
                    increment(observation, 0, i, noise);
                }
            }
            auto next_observation = view(device, observation, matrix::ViewSpec<1, OBS_SPEC::COLS - OBSERVATION::CURRENT_DIM>{}, 0, OBSERVATION::CURRENT_DIM);
            observe(device, env, parameters, state, typename OBSERVATION::NEXT_COMPONENT{}, next_observation, rng);
        }
        template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE, typename OBSERVATION_SPEC, typename OBS_SPEC, typename RNG>
        RL_TOOLS_FUNCTION_PLACEMENT static void _observe(DEVICE& device, const Multirotor<SPEC>& env, PARAMETERS& parameters, const STATE& state, observation::LinearVelocity<OBSERVATION_SPEC>, Matrix<OBS_SPEC>& observation, RNG& rng){
            using T = typename SPEC::T;
            using TI = typename DEVICE::index_t;
            using OBSERVATION = observation::LinearVelocity<OBSERVATION_SPEC>;
            static_assert(OBS_SPEC::COLS >= OBSERVATION::CURRENT_DIM);
            static_assert(OBS_SPEC::ROWS == 1);
            for(TI i = 0; i < OBSERVATION::CURRENT_DIM; i++){
                if constexpr(OBSERVATION_SPEC::PRIVILEGED && !SPEC::STATIC_PARAMETERS::PRIVILEGED_OBSERVATION_NOISE){
                    set(observation, 0, i, state.linear_velocity[i]);
                }
                else{
                    T noise = random::normal_distribution::sample(typename DEVICE::SPEC::RANDOM{}, (T)0, parameters.mdp.observation_noise.linear_velocity, rng);
                    set(observation, 0, i, state.linear_velocity[i] + noise);
                }
            }
            auto next_observation = view(device, observation, matrix::ViewSpec<1, OBS_SPEC::COLS - OBSERVATION::CURRENT_DIM>{}, 0, OBSERVATION::CURRENT_DIM);
            observe(device, env, parameters, state, typename OBSERVATION::NEXT_COMPONENT{}, next_observation, rng);
        }
        template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE, typename OBSERVATION_SPEC, typename OBS_SPEC, typename RNG>
        RL_TOOLS_FUNCTION_PLACEMENT static void _observe(DEVICE& device, const Multirotor<SPEC>& env, PARAMETERS& parameters, const STATE& state, observation::AngularVelocity<OBSERVATION_SPEC>, Matrix<OBS_SPEC>& observation, RNG& rng){
            using T = typename SPEC::T;
            using TI = typename DEVICE::index_t;
            using OBSERVATION = observation::AngularVelocity<OBSERVATION_SPEC>;
            static_assert(OBS_SPEC::COLS >= OBSERVATION::CURRENT_DIM);
            static_assert(OBS_SPEC::ROWS == 1);
            for(TI i = 0; i < OBSERVATION::CURRENT_DIM; i++){
                if constexpr(OBSERVATION_SPEC::PRIVILEGED && !SPEC::STATIC_PARAMETERS::PRIVILEGED_OBSERVATION_NOISE){
                    set(observation, 0, i, state.angular_velocity[i]);
                }
                else{
                    T noise = random::normal_distribution::sample(typename DEVICE::SPEC::RANDOM{}, (T)0, parameters.mdp.observation_noise.angular_velocity, rng);
                    set(observation, 0, i, state.angular_velocity[i] + noise);
                }
            }
            auto next_observation = view(device, observation, matrix::ViewSpec<1, OBS_SPEC::COLS - OBSERVATION::CURRENT_DIM>{}, 0, OBSERVATION::CURRENT_DIM);
            observe(device, env, parameters, state, typename OBSERVATION::NEXT_COMPONENT{}, next_observation, rng);
        }
        template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE, typename OBSERVATION_SPEC, typename OBS_SPEC, typename RNG>
        RL_TOOLS_FUNCTION_PLACEMENT static void _observe(DEVICE& device, const Multirotor<SPEC>& env, PARAMETERS& parameters, const STATE& state, observation::IMUAccelerometer<OBSERVATION_SPEC>, Matrix<OBS_SPEC>& observation, RNG& rng){
            using T = typename SPEC::T;
            using TI = typename DEVICE::index_t;
            using OBSERVATION = observation::IMUAccelerometer<OBSERVATION_SPEC>;
            static_assert(OBS_SPEC::COLS >= OBSERVATION::CURRENT_DIM);
            static_assert(OBS_SPEC::ROWS == 1);

//            observation = R_global_to_local * (acceleration - gravity)

            T conjugate_orientation[4];
            conjugate_orientation[0] = state.orientation[0];
            conjugate_orientation[1] = -state.orientation[1];
            conjugate_orientation[2] = -state.orientation[2];
            conjugate_orientation[3] = -state.orientation[3];

            T acceleration_observation_global[3];

            acceleration_observation_global[0] = state.linear_acceleration[0] - parameters.dynamics.gravity[0];
            acceleration_observation_global[1] = state.linear_acceleration[1] - parameters.dynamics.gravity[1];
            acceleration_observation_global[2] = state.linear_acceleration[2] - parameters.dynamics.gravity[2];

            T acceleration_observation[3];
            rotate_vector_by_quaternion<DEVICE, T>(conjugate_orientation, acceleration_observation_global, acceleration_observation);

            for(TI i = 0; i < OBSERVATION::CURRENT_DIM; i++){
                if constexpr(OBSERVATION_SPEC::PRIVILEGED && !SPEC::STATIC_PARAMETERS::PRIVILEGED_OBSERVATION_NOISE){
                    set(observation, 0, i, acceleration_observation[i]);
                }
                else{
                    T noise = random::normal_distribution::sample(typename DEVICE::SPEC::RANDOM{}, (T)0, parameters.mdp.observation_noise.imu_acceleration, rng);
                    set(observation, 0, i, acceleration_observation[i] + noise);
                }
            }
            auto next_observation = view(device, observation, matrix::ViewSpec<1, OBS_SPEC::COLS - OBSERVATION::CURRENT_DIM>{}, 0, OBSERVATION::CURRENT_DIM);
            observe(device, env, parameters, state, typename OBSERVATION::NEXT_COMPONENT{}, next_observation, rng);
        }
        template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE, typename OBSERVATION_SPEC, typename OBS_SPEC, typename RNG>
        RL_TOOLS_FUNCTION_PLACEMENT static void _observe(DEVICE& device, const Multirotor<SPEC>& env, PARAMETERS& parameters, const STATE& state, observation::Magnetometer<OBSERVATION_SPEC>, Matrix<OBS_SPEC>& observation, RNG& rng){
            using T = typename SPEC::T;
            using TI = typename DEVICE::index_t;
            using OBSERVATION = observation::Magnetometer<OBSERVATION_SPEC>;
            static_assert(OBS_SPEC::COLS >= OBSERVATION::CURRENT_DIM);
            static_assert(OBS_SPEC::ROWS == 1);

//            projecting the body x axis to the global xy plane


            T body_x_axis_local[3] = {1, 0, 0};
            T body_x_axis_world[3];
            rotate_vector_by_quaternion<DEVICE, T>(state.orientation, body_x_axis_local, body_x_axis_world);

            T pre_sqrt = body_x_axis_world[0]*body_x_axis_world[0] + body_x_axis_world[1]*body_x_axis_world[1];
            if(pre_sqrt > 0.01 * 0.01){
                T norm = math::sqrt(device.math, pre_sqrt);
                body_x_axis_world[0] /= norm;
                body_x_axis_world[1] /= norm;
            }
            else{
                body_x_axis_world[0] = 0;
                body_x_axis_world[1] = 0;
            }

            for(TI i = 0; i < OBSERVATION::CURRENT_DIM; i++){
                if constexpr(OBSERVATION_SPEC::PRIVILEGED && !SPEC::STATIC_PARAMETERS::PRIVILEGED_OBSERVATION_NOISE){
                    set(observation, 0, i, body_x_axis_world[i]);
                }
                else{
                    T noise = random::normal_distribution::sample(typename DEVICE::SPEC::RANDOM{}, (T)0, parameters.mdp.observation_noise.imu_acceleration, rng);
                    set(observation, 0, i, body_x_axis_world[i] + noise);
                }
            }
            auto next_observation = view(device, observation, matrix::ViewSpec<1, OBS_SPEC::COLS - OBSERVATION::CURRENT_DIM>{}, 0, OBSERVATION::CURRENT_DIM);
            observe(device, env, parameters, state, typename OBSERVATION::NEXT_COMPONENT{}, next_observation, rng);
        }
        template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC, typename OBSERVATION_SPEC, typename OBS_SPEC, typename RNG>
        RL_TOOLS_FUNCTION_PLACEMENT static void _observe_angular_velocity_delayed(DEVICE& device, const Multirotor<SPEC>& env, PARAMETERS& parameters, const StateAngularVelocityDelay<STATE_SPEC>& state, observation::AngularVelocityDelayed<OBSERVATION_SPEC>, Matrix<OBS_SPEC>& observation, RNG& rng){
            // this function is separate such that we can extract the angular velocity state from the generic state
            // we can not specialize in the generic observe function because otherwise the upcast might prevent calling the correct "observe" for the downstream observations
            using T = typename SPEC::T;
            using TI = typename DEVICE::index_t;
            using STATE = StateAngularVelocityDelay<STATE_SPEC>;
            using OBSERVATION = observation::AngularVelocityDelayed<OBSERVATION_SPEC>;
            static_assert(OBSERVATION_SPEC::DELAY <= STATE_SPEC::HISTORY_LENGTH, "The requested angular velocity delay in the observation needs to be larger than the history memory length of the state");
            for(TI i = 0; i < OBSERVATION::CURRENT_DIM; i++){
                T noise = 0;
                if constexpr(!OBSERVATION_SPEC::PRIVILEGED || SPEC::STATIC_PARAMETERS::PRIVILEGED_OBSERVATION_NOISE){
                    noise = random::normal_distribution::sample(typename DEVICE::SPEC::RANDOM{}, (T)0, parameters.mdp.observation_noise.angular_velocity, rng);
                }
                T base;
                if constexpr (OBSERVATION_SPEC::DELAY == 0){
                     base = state.angular_velocity[i];
                }
                else{
                     base = state.angular_velocity_history[STATE::HISTORY_MEM_LENGTH - OBSERVATION_SPEC::DELAY][i];
                }
                set(observation, 0, i, base + noise);
            }
        }
        template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE, typename OBSERVATION_SPEC, typename OBS_SPEC, typename RNG>
        RL_TOOLS_FUNCTION_PLACEMENT static void _observe(DEVICE& device, const Multirotor<SPEC>& env, PARAMETERS& parameters, const STATE& state, observation::AngularVelocityDelayed<OBSERVATION_SPEC>, Matrix<OBS_SPEC>& observation, RNG& rng){
            using T = typename SPEC::T;
            using TI = typename DEVICE::index_t;
            using OBSERVATION = observation::AngularVelocityDelayed<OBSERVATION_SPEC>;
            static_assert(OBS_SPEC::COLS >= OBSERVATION::CURRENT_DIM);
            static_assert(OBS_SPEC::ROWS == 1);
            _observe_angular_velocity_delayed(device, env, parameters, state, OBSERVATION{}, observation, rng);
            auto next_observation = view(device, observation, matrix::ViewSpec<1, OBS_SPEC::COLS - OBSERVATION::CURRENT_DIM>{}, 0, OBSERVATION::CURRENT_DIM);
            observe(device, env, parameters, state, typename OBSERVATION::NEXT_COMPONENT{}, next_observation, rng);
        }
        template <typename PARAMETERS, typename OBS_SPEC>
        constexpr auto _observe_linear_velocity_delay(const PARAMETERS&, const observation::LinearVelocityDelayed<OBS_SPEC>&){
            using OBSERVATION = observation::LinearVelocityDelayed<OBS_SPEC>;
            return OBSERVATION::LINEAR_VELOCITY_DELAY;
        }
        template <typename SPEC, typename OBS_SPEC>
        constexpr auto _observe_linear_velocity_delay(const ParametersObservationDelay<SPEC>& parameters, const observation::LinearVelocityDelayed<OBS_SPEC>&){
            return parameters.observation_delay.linear_velocity;
        }
        template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE_SPEC, typename OBSERVATION_SPEC, typename OBS_SPEC, typename RNG>
        RL_TOOLS_FUNCTION_PLACEMENT static void _observe_linear_velocity_delayed(DEVICE& device, const Multirotor<SPEC>& env, PARAMETERS& parameters, const StateLinearVelocityDelay<STATE_SPEC>& state, observation::LinearVelocityDelayed<OBSERVATION_SPEC>, Matrix<OBS_SPEC>& observation, RNG& rng){
            // this function is separate such that we can extract the angular velocity state from the generic state
            // we can not specialize in the generic observe function because otherwise the upcast might prevent calling the correct "observe" for the downstream observations
            using T = typename SPEC::T;
            using TI = typename DEVICE::index_t;
            using STATE = StateLinearVelocityDelay<STATE_SPEC>;
            using OBSERVATION = observation::LinearVelocityDelayed<OBSERVATION_SPEC>;
            static_assert(OBSERVATION_SPEC::DELAY <= STATE_SPEC::HISTORY_LENGTH, "The requested angular velocity delay in the observation needs to be larger than the history memory length of the state");
            for(TI i = 0; i < OBSERVATION::CURRENT_DIM; i++){
                T noise = 0;
                if constexpr(!OBSERVATION_SPEC::PRIVILEGED || SPEC::STATIC_PARAMETERS::PRIVILEGED_OBSERVATION_NOISE){
                    noise = random::normal_distribution::sample(typename DEVICE::SPEC::RANDOM{}, (T)0, parameters.mdp.observation_noise.linear_velocity, rng);
                }
                T base;
                TI delay = _observe_linear_velocity_delay(parameters, OBSERVATION{});
                if (delay == 0){
                     base = state.linear_velocity[i];
                }
                else{
                     base = state.linear_velocity_history[STATE::HISTORY_MEM_LENGTH - delay][i];
                }
                set(observation, 0, i, base + noise);
            }
        }
        template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE, typename OBSERVATION_SPEC, typename OBS_SPEC, typename RNG>
        RL_TOOLS_FUNCTION_PLACEMENT static void _observe(DEVICE& device, const Multirotor<SPEC>& env, PARAMETERS& parameters, const STATE& state, observation::LinearVelocityDelayed<OBSERVATION_SPEC>, Matrix<OBS_SPEC>& observation, RNG& rng){
            using T = typename SPEC::T;
            using TI = typename DEVICE::index_t;
            using OBSERVATION = observation::LinearVelocityDelayed<OBSERVATION_SPEC>;
            static_assert(OBS_SPEC::COLS >= OBSERVATION::CURRENT_DIM);
            static_assert(OBS_SPEC::ROWS == 1);
            _observe_linear_velocity_delayed(device, env, parameters, state, OBSERVATION{}, observation, rng);
            auto next_observation = view(device, observation, matrix::ViewSpec<1, OBS_SPEC::COLS - OBSERVATION::CURRENT_DIM>{}, 0, OBSERVATION::CURRENT_DIM);
            observe(device, env, parameters, state, typename OBSERVATION::NEXT_COMPONENT{}, next_observation, rng);
        }
        template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE, typename OBSERVATION_SPEC, typename OBS_SPEC, typename RNG>
        RL_TOOLS_FUNCTION_PLACEMENT static void _observe(DEVICE& device, const Multirotor<SPEC>& env, PARAMETERS& parameters, const STATE& state, observation::RotorSpeeds<OBSERVATION_SPEC>, Matrix<OBS_SPEC>& observation, RNG& rng){
            using T = typename SPEC::T;
            using TI = typename DEVICE::index_t;
            using OBSERVATION = observation::RotorSpeeds<OBSERVATION_SPEC>;
            static_assert(OBS_SPEC::COLS >= OBSERVATION::CURRENT_DIM);
            static_assert(OBS_SPEC::ROWS == 1);
            for(TI action_i = 0; action_i < OBSERVATION::CURRENT_DIM; action_i++){
                T action_value = (state.rpm[action_i] - parameters.dynamics.action_limit.min)/(parameters.dynamics.action_limit.max - parameters.dynamics.action_limit.min) * 2 - 1;
                set(observation, 0, action_i, action_value);
            }
            auto next_observation = view(device, observation, matrix::ViewSpec<1, OBS_SPEC::COLS - OBSERVATION::CURRENT_DIM>{}, 0, OBSERVATION::CURRENT_DIM);
            observe(device, env, parameters, state, typename OBSERVATION::NEXT_COMPONENT{}, next_observation, rng);
        }
        template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE, typename OBSERVATION_SPEC, typename OBS_SPEC, typename RNG>
        RL_TOOLS_FUNCTION_PLACEMENT static void _observe(DEVICE& device, const Multirotor<SPEC>& env, PARAMETERS& parameters, const STATE& state, observation::ActionHistory<OBSERVATION_SPEC>, Matrix<OBS_SPEC>& observation, RNG& rng){
            using T = typename SPEC::T;
            using TI = typename DEVICE::index_t;
            using OBSERVATION = observation::ActionHistory<OBSERVATION_SPEC>;
            static_assert(OBS_SPEC::COLS >= OBSERVATION::CURRENT_DIM);
            static_assert(OBS_SPEC::ROWS == 1);
            static constexpr TI STATE_HISTORY_LENGTH = Multirotor<SPEC>::State::HISTORY_LENGTH;
            static_assert(STATE_HISTORY_LENGTH >= OBSERVATION::HISTORY_LENGTH);
            static_assert(Multirotor<SPEC>::State::ACTION_DIM == OBSERVATION::ACTION_DIM);
            static_assert(Multirotor<SPEC>::ACTION_DIM == OBSERVATION::ACTION_DIM);
            // the ring buffer in the state moves forwards in time, we want to observe the most recent action first, hence we need to move backwards in time
            TI current_step = state.current_step == 0 ? STATE_HISTORY_LENGTH - 1 : state.current_step - 1;
            for(TI step_i = 0; step_i < OBSERVATION::HISTORY_LENGTH; step_i++){
                TI base = step_i*OBSERVATION::ACTION_DIM;
                for(TI action_i = 0; action_i < OBSERVATION::ACTION_DIM; action_i++){
                    set(observation, 0, base + action_i, state.action_history[current_step][action_i]);
                }
                current_step = current_step == 0 ? STATE_HISTORY_LENGTH - 1 : current_step - 1;
            }
            auto next_observation = view(device, observation, matrix::ViewSpec<1, OBS_SPEC::COLS - OBSERVATION::CURRENT_DIM>{}, 0, OBSERVATION::CURRENT_DIM);
            observe(device, env, parameters, state, typename OBSERVATION::NEXT_COMPONENT{}, next_observation, rng);
        }
        template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE, typename OBSERVATION_SPEC, typename OBS_SPEC, typename RNG>
        RL_TOOLS_FUNCTION_PLACEMENT static void _observe(DEVICE& device, const Multirotor<SPEC>& env, PARAMETERS& parameters, const STATE& state, observation::RandomForce<OBSERVATION_SPEC>, Matrix<OBS_SPEC>& observation, RNG& rng){
            using T = typename SPEC::T;
            using TI = typename DEVICE::index_t;
            using OBSERVATION = observation::RandomForce<OBSERVATION_SPEC>;
            static_assert(OBS_SPEC::COLS >= OBSERVATION::CURRENT_DIM);
            static_assert(OBS_SPEC::ROWS == 1);
            for(TI i = 0; i < 3; i++){
                set(observation, 0, i, state.force[i]);
                set(observation, 0, 3 + i, state.torque[i]);
            }
            auto next_observation = view(device, observation, matrix::ViewSpec<1, OBS_SPEC::COLS - OBSERVATION::CURRENT_DIM>{}, 0, OBSERVATION::CURRENT_DIM);
            observe(device, env, parameters, state, typename OBSERVATION::NEXT_COMPONENT{}, next_observation, rng);
        }
        template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE, typename OBSERVATION_SPEC, typename OBS_SPEC, typename RNG>
        RL_TOOLS_FUNCTION_PLACEMENT static void _observe(DEVICE& device, const Multirotor<SPEC>& env, PARAMETERS& parameters, const STATE& state, observation::ParametersMotorPosition<OBSERVATION_SPEC>, Matrix<OBS_SPEC>& observation, RNG& rng){
            using T = typename SPEC::T;
            using TI = typename DEVICE::index_t;
            static_assert(PARAMETERS::N == OBSERVATION_SPEC::N);
            using OBSERVATION = observation::ParametersThrustCurves<OBSERVATION_SPEC>;
            static_assert(OBS_SPEC::COLS >= OBSERVATION::CURRENT_DIM);
            static_assert(OBS_SPEC::ROWS == 1);
            for (TI rotor_i = 0; rotor_i < PARAMETERS::N; rotor_i++){
                T factor = 1.0 / 0.04;
                set(observation, 0, rotor_i * 3 + 0, parameters.dynamics.rotor_positions[rotor_i][0] * factor);
                set(observation, 0, rotor_i * 3 + 1, parameters.dynamics.rotor_positions[rotor_i][1] * factor);
                set(observation, 0, rotor_i * 3 + 2, parameters.dynamics.rotor_positions[rotor_i][2] * factor);
            }
            auto next_observation = view(device, observation, matrix::ViewSpec<1, OBS_SPEC::COLS - OBSERVATION::CURRENT_DIM>{}, 0, OBSERVATION::CURRENT_DIM);
            observe(device, env, parameters, state, typename OBSERVATION::NEXT_COMPONENT{}, next_observation, rng);
        }
        template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE, typename OBSERVATION_SPEC, typename OBS_SPEC, typename RNG>
        RL_TOOLS_FUNCTION_PLACEMENT static void _observe(DEVICE& device, const Multirotor<SPEC>& env, PARAMETERS& parameters, const STATE& state, observation::ParametersThrustCurves<OBSERVATION_SPEC>, Matrix<OBS_SPEC>& observation, RNG& rng){
            using T = typename SPEC::T;
            using TI = typename DEVICE::index_t;
            static_assert(PARAMETERS::N == OBSERVATION_SPEC::N);
            using OBSERVATION = observation::ParametersThrustCurves<OBSERVATION_SPEC>;
            static_assert(OBS_SPEC::COLS >= OBSERVATION::CURRENT_DIM);
            static_assert(OBS_SPEC::ROWS == 1);
            static constexpr T EPSILON = 1e-7;
            for (TI rotor_i = 0; rotor_i < PARAMETERS::N; rotor_i++){
                for (TI order_i = 0; order_i < 3; order_i++){
                    T normalized_value = parameters.dynamics.rotor_thrust_coefficients[rotor_i][order_i] / (env.parameters.dynamics.rotor_thrust_coefficients[rotor_i][order_i] + EPSILON);
                    set(observation, 0, rotor_i * 3 + order_i, normalized_value);
                }
            }
            auto next_observation = view(device, observation, matrix::ViewSpec<1, OBS_SPEC::COLS - OBSERVATION::CURRENT_DIM>{}, 0, OBSERVATION::CURRENT_DIM);
            observe(device, env, parameters, state, typename OBSERVATION::NEXT_COMPONENT{}, next_observation, rng);
        }
        template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE, typename OBSERVATION_SPEC, typename OBS_SPEC, typename RNG>
        RL_TOOLS_FUNCTION_PLACEMENT static void _observe(DEVICE& device, const Multirotor<SPEC>& env, PARAMETERS& parameters, const STATE& state, observation::ParametersMass<OBSERVATION_SPEC>, Matrix<OBS_SPEC>& observation, RNG& rng){
            using T = typename SPEC::T;
            using TI = typename DEVICE::index_t;
            using OBSERVATION = observation::ParametersMass<OBSERVATION_SPEC>;
            static_assert(OBS_SPEC::COLS >= OBSERVATION::CURRENT_DIM);
            static_assert(OBS_SPEC::ROWS == 1);
            static constexpr T EPSILON = 1e-7;
            T normalized_value = parameters.dynamics.mass / (env.parameters.dynamics.mass + EPSILON);
            set(observation, 0, 0, normalized_value);
            auto next_observation = view(device, observation, matrix::ViewSpec<1, OBS_SPEC::COLS - OBSERVATION::CURRENT_DIM>{}, 0, OBSERVATION::CURRENT_DIM);
            observe(device, env, parameters, state, typename OBSERVATION::NEXT_COMPONENT{}, next_observation, rng);
        }
        template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE, typename OBSERVATION_SPEC, typename OBS_SPEC, typename RNG>
        RL_TOOLS_FUNCTION_PLACEMENT static void _observe(DEVICE& device, const Multirotor<SPEC>& env, PARAMETERS& parameters, const STATE& state, observation::ParametersInertia<OBSERVATION_SPEC>, Matrix<OBS_SPEC>& observation, RNG& rng){
            using T = typename SPEC::T;
            using TI = typename DEVICE::index_t;
            using OBSERVATION = observation::ParametersInertia<OBSERVATION_SPEC>;
            static_assert(OBS_SPEC::COLS >= OBSERVATION::CURRENT_DIM);
            static_assert(OBS_SPEC::ROWS == 1);
            static constexpr T EPSILON = 1e-7;
#ifdef RL_TOOLS_DEBUG
            if(env.parameters.dynamics.J[0][0] < 10 * EPSILON){
                std::cerr << "Inertia close to EPSILON" << std::endl;
            }
#endif
            for(TI row_i=0; row_i < 3; row_i++){
                for(TI col_i=0; col_i < 3; col_i++){
                    T normalized_value = parameters.dynamics.J[row_i][col_i] / (env.parameters.dynamics.J[row_i][col_i] + EPSILON);
                    set(observation, 0, row_i * 3 + col_i, (normalized_value - 1));
                }
            }
            auto next_observation = view(device, observation, matrix::ViewSpec<1, OBS_SPEC::COLS - OBSERVATION::CURRENT_DIM>{}, 0, OBSERVATION::CURRENT_DIM);
            observe(device, env, parameters, state, typename OBSERVATION::NEXT_COMPONENT{}, next_observation, rng);
        }
        template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE, typename OBSERVATION_SPEC, typename OBS_SPEC, typename RNG>
        RL_TOOLS_FUNCTION_PLACEMENT static void _observe(DEVICE& device, const Multirotor<SPEC>& env, PARAMETERS& parameters, const STATE& state, observation::Multiplex<OBSERVATION_SPEC>, Matrix<OBS_SPEC>& observation, RNG& rng){
            using OBSERVATION = observation::Multiplex<OBSERVATION_SPEC>;
            auto current_observation = view(device, observation, matrix::ViewSpec<1, OBSERVATION::CURRENT_DIM>{}, 0, 0);
            auto next_observation = view(device, observation, matrix::ViewSpec<1, OBS_SPEC::COLS - OBSERVATION::CURRENT_DIM>{}, 0, OBSERVATION::CURRENT_DIM);
            if constexpr(OBSERVATION_SPEC::ENABLE){
                observe(device, env, parameters, state, typename OBSERVATION::CURRENT_COMPONENT{}, current_observation, rng);
            }
            observe(device, env, parameters, state, typename OBSERVATION::NEXT_COMPONENT{}, next_observation, rng);
        }
        template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE, typename OBSERVATION_SPEC, typename OBS_SPEC, typename RNG>
        RL_TOOLS_FUNCTION_PLACEMENT static void _observe(DEVICE& device, const Multirotor<SPEC>& env, PARAMETERS& parameters, const STATE& state, observation::TrajectoryTrackingPosition<OBSERVATION_SPEC>, Matrix<OBS_SPEC>& observation, RNG& rng){
            using OBSERVATION = observation::TrajectoryTrackingPosition<OBSERVATION_SPEC>;
            auto current_observation = view(device, observation, matrix::ViewSpec<1, OBSERVATION::CURRENT_DIM>{}, 0, 0);
            static_assert(OBS_SPEC::COLS >= OBSERVATION::CURRENT_DIM);
            static_assert(OBS_SPEC::ROWS == 1);
            using T = typename SPEC::T;
            using TI = typename DEVICE::index_t;
            STATE desired_state;
            get_desired_state(device, env, parameters, state, desired_state, rng);
            for(TI i = 0; i < 3; i++){
                if constexpr(OBSERVATION_SPEC::PRIVILEGED && !SPEC::STATIC_PARAMETERS::PRIVILEGED_OBSERVATION_NOISE){
                    set(observation, 0, i, state.position[i] - desired_state.position[i]);
                }
                else{
                    T noise = random::normal_distribution::sample(typename DEVICE::SPEC::RANDOM{}, (T)0, parameters.mdp.observation_noise.position, rng);
                    set(observation, 0, i, state.position[i] - desired_state.position[i] + noise);
                }
            }
            auto next_observation = view(device, observation, matrix::ViewSpec<1, OBS_SPEC::COLS - OBSERVATION::CURRENT_DIM>{}, 0, OBSERVATION::CURRENT_DIM);
            observe(device, env, parameters, state, typename OBSERVATION::NEXT_COMPONENT{}, next_observation, rng);
        }
        template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE, typename OBSERVATION_SPEC, typename OBS_SPEC, typename RNG>
        RL_TOOLS_FUNCTION_PLACEMENT static void _observe(DEVICE& device, const Multirotor<SPEC>& env, PARAMETERS& parameters, const STATE& state, observation::TrajectoryTrackingLinearVelocity<OBSERVATION_SPEC>, Matrix<OBS_SPEC>& observation, RNG& rng){
            using OBSERVATION = observation::TrajectoryTrackingLinearVelocity<OBSERVATION_SPEC>;
            auto current_observation = view(device, observation, matrix::ViewSpec<1, OBSERVATION::CURRENT_DIM>{}, 0, 0);
            static_assert(OBS_SPEC::COLS >= OBSERVATION::CURRENT_DIM);
            static_assert(OBS_SPEC::ROWS == 1);
            using T = typename SPEC::T;
            using TI = typename DEVICE::index_t;
            STATE desired_state;
            get_desired_state(device, env, parameters, state, desired_state, rng);
            for(TI i = 0; i < 3; i++){
                if constexpr(OBSERVATION_SPEC::PRIVILEGED && !SPEC::STATIC_PARAMETERS::PRIVILEGED_OBSERVATION_NOISE){
                    set(observation, 0, i, state.linear_velocity[i] - desired_state.linear_velocity[i]);
                }
                else{
                    T noise = random::normal_distribution::sample(typename DEVICE::SPEC::RANDOM{}, (T)0, parameters.mdp.observation_noise.linear_velocity, rng);
                    set(observation, 0, i, state.linear_velocity[i] - desired_state.linear_velocity[i] + noise);
                }
            }
            auto next_observation = view(device, observation, matrix::ViewSpec<1, OBS_SPEC::COLS - OBSERVATION::CURRENT_DIM>{}, 0, OBSERVATION::CURRENT_DIM);
            observe(device, env, parameters, state, typename OBSERVATION::NEXT_COMPONENT{}, next_observation, rng);
        }
    }

}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif


