#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ENVIRONMENTS_L2F_OPERATIONS_MULTITASK_GENERIC_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENTS_L2F_OPERATIONS_MULTITASK_GENERIC_H

#include "multirotor.h"

#include <rl_tools/utils/generic/vector_operations.h>
#include "quaternion_helper.h"

#include <rl_tools/utils/generic/typing.h>

#include <rl_tools/rl/environments/operations_generic.h>
#include "operations_cpu.h"

#ifndef RL_TOOLS_FUNCTION_PLACEMENT
#define RL_TOOLS_FUNCTION_PLACEMENT
#endif

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template<typename DEVICE, typename SPEC, typename RNG, bool T_OVERWRITE>
    static void sample_initial_parameters(DEVICE& device, rl::environments::MultirotorMultiTask<SPEC>& env, typename rl::environments::MultirotorMultiTask<SPEC>::Parameters& parameters, RNG& rng){
        using TI = typename DEVICE::index_t;
        using T = typename SPEC::T;
        using PARAMETERS = typename rl::environments::MultirotorMultiTask<SPEC>::Parameters;
        parameters = env.parameters;
        if constexpr(T_OVERWRITE || SPEC::SAMPLE_INITIAL_PARAMETERS){
            static_assert(SPEC::STATIC_PARAMETERS::N_DYNAMICS_VALUES >= 1);
            TI index = random::uniform_int_distribution(device.random, (TI)0, (TI)(SPEC::STATIC_PARAMETERS::N_DYNAMICS_VALUES - 1), rng);
            parameters.dynamics = SPEC::STATIC_PARAMETERS::DYNAMICS_VALUES[index];
            if constexpr(SPEC::STATIC_PARAMETERS::RANDOMIZE_THRUST_CURVES){
                for(TI rotor_i = 0; rotor_i < PARAMETERS::N; rotor_i++){
                    T factor = random::uniform_real_distribution(device.random, (T)1.0, (T)4.0, rng);
                    for(TI order_i = 0; order_i < 3; order_i++){
                        parameters.dynamics.rotor_thrust_coefficients[rotor_i][order_i] *= factor;
                    }
                }
            }
            if constexpr(SPEC::STATIC_PARAMETERS::RANDOMIZE_MOTOR_MAPPING){
                static_assert(PARAMETERS::N == 4);
                TI mapping[PARAMETERS::N] = {0, 1, 2, 3};
                for(TI motor_i = 0; motor_i < PARAMETERS::N; motor_i++){
                    TI random_index = random::uniform_int_distribution(device.random, (TI)motor_i, (TI)(PARAMETERS::N - 1), rng);
                    TI previous = mapping[motor_i];
                    mapping[motor_i] = mapping[random_index];
                    mapping[random_index] = previous;
                }
                auto new_dynamics = parameters.dynamics;
                for(TI motor_i = 0; motor_i < PARAMETERS::N; motor_i++){
                    for(TI axis_i=0; axis_i < 3; axis_i++){
                        new_dynamics.rotor_positions[mapping[motor_i]][axis_i] = parameters.dynamics.rotor_positions[motor_i][axis_i];
                        new_dynamics.rotor_thrust_directions[mapping[motor_i]][axis_i] = parameters.dynamics.rotor_thrust_directions[motor_i][axis_i];
                        new_dynamics.rotor_torque_directions[mapping[motor_i]][axis_i] = parameters.dynamics.rotor_torque_directions[motor_i][axis_i];
                        new_dynamics.rotor_thrust_coefficients[mapping[motor_i]][axis_i] = parameters.dynamics.rotor_thrust_coefficients[motor_i][axis_i];
                    }
                }
                parameters.dynamics = new_dynamics;
            }
        }
    }

    namespace rl::environments::l2f::observations{
    }
    template <typename DEVICE, typename SPEC>
    auto get_description(DEVICE& device, rl::environments::MultirotorMultiTask<SPEC>& env){
        return "Multirotor Multitask: Samples random dynamics parameters from a distribution and provides the information as part of the observation to the agent.";
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif
