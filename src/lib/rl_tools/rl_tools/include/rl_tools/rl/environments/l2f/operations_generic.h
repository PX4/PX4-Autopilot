#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ENVIRONMENTS_L2F_OPERATIONS_GENERIC_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENTS_L2F_OPERATIONS_GENERIC_H

#include "multirotor.h"

#include <rl_tools/utils/generic/vector_operations.h>
#include "quaternion_helper.h"

#include <rl_tools/utils/generic/typing.h>

#include <rl_tools/rl/environments/operations_generic.h>

#ifndef RL_TOOLS_FUNCTION_PLACEMENT
#define RL_TOOLS_FUNCTION_PLACEMENT
#endif

#include <rl_tools/utils/generic/integrators.h>


// Since L2F is quite flexible in the way states and observations are composed, the operations might need to call each other in arbitrary order (depending on the definition). Hence we implement a dispatch scheme, were a dispatch function is forward declared such that it can be called from all specialized functions (_xxx). This dispatch function serves as the public interface xxx as well.
#include "operations_generic/05_state_is_nan.h"
#include "operations_generic/10_sample_initial_parameters.h"
#include "operations_generic/20_initial_state.h"
#include "operations_generic/30_sample_initial_state.h"
#include "operations_generic/35_get_desired_state.h"
#include "operations_generic/40_observe.h"
#include "operations_generic/50_state_algebra.h"
#include "operations_generic/60_dynamics.h"
#include "operations_generic/70_post_integration.h"
#include "operations_generic/80_abs_diff.h"


RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools
{
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE&, rl::environments::Multirotor<SPEC>& env){
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE&, rl::environments::Multirotor<SPEC>&){ }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void init(DEVICE&, rl::environments::Multirotor<SPEC>& env){
        env.parameters = SPEC::STATIC_PARAMETERS::PARAMETER_VALUES;
    }
    template<typename DEVICE, typename SPEC, typename T, typename TI, TI N, typename... Args>
    static void permute_rotors(DEVICE& device, const rl::environments::Multirotor<SPEC>&, rl::environments::l2f::parameters::Dynamics<T, TI, N>& dynamics, Args... args){
        TI indices[N] = {static_cast<TI>(args)...};
        auto copy = dynamics;
        for (TI rotor_i=0; rotor_i < N; rotor_i++){
            for (TI j=0; j < 3; j++){
                dynamics.rotor_positions[rotor_i][j] = copy.rotor_positions[indices[rotor_i]][j];
                dynamics.rotor_thrust_directions[rotor_i][j] = copy.rotor_thrust_directions[indices[rotor_i]][j];
                dynamics.rotor_torque_directions[rotor_i][j] = copy.rotor_torque_directions[indices[rotor_i]][j];
            }
            for (TI j=0; j < 3; j++){
                dynamics.rotor_thrust_coefficients[rotor_i][j] = copy.rotor_thrust_coefficients[indices[rotor_i]][j];
            }
            dynamics.rotor_torque_constants[rotor_i] = copy.rotor_torque_constants[indices[rotor_i]];
            dynamics.rotor_time_constants_rising[rotor_i] = copy.rotor_time_constants_rising[indices[rotor_i]];
            dynamics.rotor_time_constants_falling[rotor_i] = copy.rotor_time_constants_falling[indices[rotor_i]];
        }
    }
    template<typename DEVICE, typename STATE>
    static bool is_nan(DEVICE& device, STATE& state){
        return rl::environments::l2f::_is_nan(device, state);
    }
    template<typename DEVICE, typename SPEC, typename PARAMETERS>
    static void initial_parameters(DEVICE& device, rl::environments::Multirotor<SPEC>& env, PARAMETERS& parameters){
        parameters = env.parameters;
        //        parameters = SPEC::STATIC_PARAMETERS::PARAMETER_VALUES;
    }
    template<typename DEVICE, typename SPEC, typename PARAMETERS, typename RNG>
    static void sample_initial_parameters(DEVICE& device, rl::environments::Multirotor<SPEC>& env, PARAMETERS& parameters, RNG& rng){
        // to allow out of declaration order dispatch
        rl::environments::l2f::_sample_initial_parameters(device, env, parameters, rng);
    }
    template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE>
    static void initial_state(DEVICE& device, rl::environments::Multirotor<SPEC>& env, PARAMETERS& parameters, STATE& state){
        rl::environments::l2f::_initial_state(device, env, parameters, state);
    }
    template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static void sample_initial_state(DEVICE& device, rl::environments::Multirotor<SPEC>& env, PARAMETERS& parameters, STATE& state, RNG& rng){
        rl::environments::l2f::_sample_initial_state(device, env, parameters, state, rng);
    }
    template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE, typename OBSERVATION, typename OBS_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static void observe(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, PARAMETERS& parameters, const STATE& state, const OBSERVATION& observation_type, Matrix<OBS_SPEC>& observation, RNG& rng){
        static_assert(OBS_SPEC::COLS == OBSERVATION::DIM);
        static_assert(OBS_SPEC::ROWS == 1);
        rl::environments::l2f::_observe(device, env, parameters, state, observation_type, observation, rng);
    }
    // todo: make state const again
    template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE, typename ACTION_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static typename SPEC::T step(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, PARAMETERS& parameters, const STATE& state, const Matrix<ACTION_SPEC>& action, STATE& next_state, RNG& rng) {
        using T = typename SPEC::T;
        using TI = typename DEVICE::index_t;
        constexpr auto STATE_DIM = STATE::DIM;
        constexpr auto ACTION_DIM = rl::environments::Multirotor<SPEC>::ACTION_DIM;
        static_assert(ACTION_SPEC::ROWS == 1);
        static_assert(ACTION_SPEC::COLS == ACTION_DIM);
        T action_scaled[ACTION_DIM];

        for(TI action_i = 0; action_i < ACTION_DIM; action_i++){
            T half_range = (parameters.dynamics.action_limit.max - parameters.dynamics.action_limit.min) / 2;
            T action_noisy = get(action, 0, action_i);
            action_noisy += random::normal_distribution::sample(typename DEVICE::SPEC::RANDOM(), (T)0, parameters.mdp.action_noise.normalized_rpm, rng);
            action_noisy = math::clamp(device.math, action_noisy, -(T)1, (T)1);
            action_scaled[action_i] = action_noisy * half_range + parameters.dynamics.action_limit.min + half_range;
        }
        if constexpr(SPEC::STATIC_PARAMETERS::N_SUBSTEPS == 1){
            utils::integrators::rk4  <DEVICE, typename SPEC::T, typename SPEC::PARAMETERS, STATE, ACTION_DIM, rl::environments::l2f::multirotor_dynamics_dispatch<DEVICE, typename SPEC::T, typename SPEC::PARAMETERS, STATE>>(device, parameters, state, action_scaled, parameters.integration.dt, next_state);
    //        utils::integrators::euler<DEVICE, typename SPEC::T, typename SPEC::PARAMETERS, STATE, ACTION_DIM, rl::environments::l2f::multirotor_dynamics_dispatch<DEVICE, typename SPEC::T, typename SPEC::PARAMETERS, STATE>>(device, parameters, state, action_scaled, parameters.integration.dt, next_state);
        }
        else{
            auto substep_state = state;
            auto substep_next_state = state;
            T substep_dt = parameters.integration.dt / SPEC::STATIC_PARAMETERS::N_SUBSTEPS;
            for (TI substep_i=0; substep_i < SPEC::STATIC_PARAMETERS::N_SUBSTEPS; substep_i++){
                utils::integrators::rk4  <DEVICE, typename SPEC::T, typename SPEC::PARAMETERS, STATE, ACTION_DIM, rl::environments::l2f::multirotor_dynamics_dispatch<DEVICE, typename SPEC::T, typename SPEC::PARAMETERS, STATE>>(device, parameters, substep_state, action_scaled, substep_dt, substep_next_state);
        //        utils::integrators::euler<DEVICE, typename SPEC::T, typename SPEC::PARAMETERS, STATE, ACTION_DIM, rl::environments::l2f::multirotor_dynamics_dispatch<DEVICE, typename SPEC::T, typename SPEC::PARAMETERS, STATE>>(device, parameters, substep_state, action_scaled, substep_dt, substep_next_state);
                substep_state = substep_next_state;
            }
            next_state = substep_next_state;
        }

        post_integration(device, env, parameters, state, action, next_state, rng);

        return parameters.integration.dt;
    }
    namespace rl::environments::l2f{
        template<typename STATE>
        RL_TOOLS_FUNCTION_PLACEMENT constexpr bool is_pose_error_integral(const STATE&){
            return false;
        }
        template<typename SPEC>
        RL_TOOLS_FUNCTION_PLACEMENT constexpr bool is_pose_error_integral(const StatePoseErrorIntegral<SPEC>&){
            return true;
        }
    }

    template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static bool terminated(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, const PARAMETERS& parameters, const STATE& state, RNG& rng){
        using T = typename SPEC::T;
        using TI = typename DEVICE::index_t;
        if(parameters.mdp.termination.enabled){
            for(TI i = 0; i < 3; i++){
                if(
                    math::abs(device.math, state.position[i]) > parameters.mdp.termination.position_threshold ||
                    math::abs(device.math, state.linear_velocity[i]) > parameters.mdp.termination.linear_velocity_threshold ||
                    math::abs(device.math, state.angular_velocity[i]) > parameters.mdp.termination.angular_velocity_threshold
                ){
                    return true;
                }
            }
        }
        if constexpr(rl::environments::l2f::is_pose_error_integral(STATE{})){
            // if(state.position_integral > parameters.mdp.termination.position_integral_threshold){
            //     return true;
            // }
            // if(state.orientation_integral > parameters.mdp.termination.orientation_integral_threshold){
            //     return true;
            // }
        }
        return false;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#include "parameters/reward_functions/reward_functions.h"
#include "parameters/reward_functions/squared/operations_generic.h" // such that terminated can be called from rwd functions
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE, typename ACTION_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static typename SPEC::T reward(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, PARAMETERS& parameters, const STATE& state, const Matrix<ACTION_SPEC>& action, const STATE& next_state, RNG& rng) {
        return rl::environments::l2f::parameters::reward_functions::reward(device, env, parameters, parameters.mdp.reward, state, action, next_state, rng);
    }
    template<typename DEVICE, typename SPEC, typename PARAMETERS, typename STATE, typename ACTION_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT void log_reward(DEVICE& device, const rl::environments::Multirotor<SPEC>& env, PARAMETERS& parameters, const STATE& state, const Matrix<ACTION_SPEC>& action, const STATE& next_state, RNG& rng, typename DEVICE::index_t cadence = 1) {
        rl::environments::l2f::parameters::reward_functions::log_reward(device, env, parameters, parameters.mdp.reward, state, action, next_state, rng, cadence);
    }

}
RL_TOOLS_NAMESPACE_WRAPPER_END

#include "parameters/default.h"

#endif