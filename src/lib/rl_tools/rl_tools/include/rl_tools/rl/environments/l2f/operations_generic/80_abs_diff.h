#include "../../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ENVIRONMENTS_L2F_OPERATIONS_GENERIC_ABS_DIFF_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENTS_L2F_OPERATIONS_GENERIC_ABS_DIFF_H

#include "../multirotor.h"

#include <rl_tools/utils/generic/vector_operations.h>
#include "../quaternion_helper.h"

#include <rl_tools/utils/generic/typing.h>

#include <rl_tools/rl/environments/operations_generic.h>

// This file contains functions for parts of the state that do not evolve through integration but through discrete steps or for refinement operations after integration (e.g. normalizing the quaternion)

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template<typename DEVICE, typename T_T_A, typename T_TI_A, typename T_T_B, typename T_TI_B, T_TI_A N>
    RL_TOOLS_FUNCTION_PLACEMENT T_T_A abs_diff(DEVICE& device, const rl::environments::l2f::parameters::Dynamics<T_T_A, T_TI_A, N>& a, const rl::environments::l2f::parameters::Dynamics<T_T_B, T_TI_B, N>& b) {
        using T = T_T_A;
        using TI = typename DEVICE::index_t;
        T acc = 0;


        for (TI rotor_i = 0; rotor_i < N; rotor_i++){
            for (TI dim_i = 0; dim_i < 3; dim_i++){
                acc += math::abs(device.math, a.rotor_positions[rotor_i][dim_i] - b.rotor_positions[rotor_i][dim_i]);
                acc += math::abs(device.math, a.rotor_thrust_directions[rotor_i][dim_i] - b.rotor_thrust_directions[rotor_i][dim_i]);
                acc += math::abs(device.math, a.rotor_torque_directions[rotor_i][dim_i] - b.rotor_torque_directions[rotor_i][dim_i]);
                acc += math::abs(device.math, a.rotor_thrust_coefficients[rotor_i][dim_i] - b.rotor_thrust_coefficients[rotor_i][dim_i]);
            }
            acc += math::abs(device.math, a.rotor_torque_constants[rotor_i] - b.rotor_torque_constants[rotor_i]);
            acc += math::abs(device.math, a.rotor_time_constants_rising[rotor_i] - b.rotor_time_constants_rising[rotor_i]);
            acc += math::abs(device.math, a.rotor_time_constants_falling[rotor_i] - b.rotor_time_constants_falling[rotor_i]);
        }
        acc += math::abs(device.math, a.mass - b.mass);
        for (TI dim1 = 0; dim1 < 3; dim1++){
            acc += math::abs(device.math, a.gravity[dim1] - b.gravity[dim1]);
            for (TI dim2 = 0; dim2 < 3; dim2++){
                acc += math::abs(device.math, a.J[dim1][dim2] - b.J[dim1][dim2]);
                acc += math::abs(device.math, a.J_inv[dim1][dim2] - b.J_inv[dim1][dim2]);
            }
        }
        acc += math::abs(device.math, a.hovering_throttle_relative - b.hovering_throttle_relative); // relative to the action limits [0, 1]
        acc += math::abs(device.math, a.action_limit.min - b.action_limit.min);
        acc += math::abs(device.math, a.action_limit.max - b.action_limit.max);
        return acc;
    }
    template<typename DEVICE, typename T_T_A, typename T_T_B>
    RL_TOOLS_FUNCTION_PLACEMENT T_T_A abs_diff(DEVICE& device, const rl::environments::l2f::parameters::Integration<T_T_A>& a, const rl::environments::l2f::parameters::Integration<T_T_B>& b) {
        return math::abs(device.math, a.dt - b.dt);
    }
    template<typename DEVICE, typename T_A, typename T_B>
    RL_TOOLS_FUNCTION_PLACEMENT T_A abs_diff(DEVICE& device, const rl::environments::l2f::parameters::Initialization<T_A>& a, const rl::environments::l2f::parameters::Initialization<T_B>& b) {
        T_A acc = 0;
        acc += math::abs(device.math, a.guidance - b.guidance);
        acc += math::abs(device.math, a.max_position - b.max_position);
        acc += math::abs(device.math, a.max_angle - b.max_angle);
        acc += math::abs(device.math, a.max_linear_velocity - b.max_linear_velocity);
        acc += math::abs(device.math, a.max_angular_velocity - b.max_angular_velocity);
        acc += (a.relative_rpm == b.relative_rpm ? 0 : 1);
        acc += math::abs(device.math, a.min_rpm - b.min_rpm);
        acc += math::abs(device.math, a.max_rpm - b.max_rpm);
        return acc;
    }
    template<typename DEVICE, typename T_A, typename T_B>
    RL_TOOLS_FUNCTION_PLACEMENT T_A abs_diff(DEVICE& device, const rl::environments::l2f::parameters::reward_functions::Squared<T_A>& a, const rl::environments::l2f::parameters::reward_functions::Squared<T_B>& b) {
        T_A acc = 0;
        acc += a.non_negative == b.non_negative ? 0 : 1;
        acc += math::abs(device.math, a.scale - b.scale);
        acc += math::abs(device.math, a.constant - b.constant);
        acc += math::abs(device.math, a.termination_penalty - b.termination_penalty);
        acc += math::abs(device.math, a.position - b.position);
        acc += math::abs(device.math, a.position_clip - b.position_clip);
        acc += math::abs(device.math, a.orientation - b.orientation);
        acc += math::abs(device.math, a.linear_velocity - b.linear_velocity);
        acc += math::abs(device.math, a.angular_velocity - b.angular_velocity);
        acc += math::abs(device.math, a.linear_acceleration - b.linear_acceleration);
        acc += math::abs(device.math, a.angular_acceleration - b.angular_acceleration);
        acc += math::abs(device.math, a.action - b.action);
        acc += math::abs(device.math, a.d_action - b.d_action);
        acc += math::abs(device.math, a.position_error_integral - b.position_error_integral);
        return acc;
    }
    template<typename DEVICE, typename T_A, typename T_B>
    RL_TOOLS_FUNCTION_PLACEMENT T_A abs_diff(DEVICE& device, const rl::environments::l2f::parameters::ObservationNoise<T_A>& a, const rl::environments::l2f::parameters::ObservationNoise<T_B>& b) {
        T_A acc = 0;
        acc += math::abs(device.math, a.position - b.position);
        acc += math::abs(device.math, a.orientation - b.orientation);
        acc += math::abs(device.math, a.linear_velocity - b.linear_velocity);
        acc += math::abs(device.math, a.angular_velocity - b.angular_velocity);
        acc += math::abs(device.math, a.imu_acceleration - b.imu_acceleration);
        return acc;
    }
    template<typename DEVICE, typename T_A, typename T_B>
    RL_TOOLS_FUNCTION_PLACEMENT T_A abs_diff(DEVICE& device, const rl::environments::l2f::parameters::ActionNoise<T_A>& a, const rl::environments::l2f::parameters::ActionNoise<T_B>& b) {
        return math::abs(device.math, a.normalized_rpm - b.normalized_rpm);
    }
    template<typename DEVICE, typename T_A, typename T_B>
    RL_TOOLS_FUNCTION_PLACEMENT T_A abs_diff(DEVICE& device, const rl::environments::l2f::parameters::Termination<T_A>& a, const rl::environments::l2f::parameters::Termination<T_B>& b) {
        T_A acc = 0;
        acc += a.enabled == b.enabled ? 0 : 1;
        acc += math::abs(device.math, a.position_threshold - b.position_threshold);
        acc += math::abs(device.math, a.linear_velocity_threshold - b.linear_velocity_threshold);
        acc += math::abs(device.math, a.angular_velocity_threshold - b.angular_velocity_threshold);
        acc += math::abs(device.math, a.position_integral_threshold - b.position_integral_threshold);
        acc += math::abs(device.math, a.orientation_integral_threshold - b.orientation_integral_threshold);
        return acc;
    }
    template<typename DEVICE, typename SPEC_A, typename SPEC_B>
    RL_TOOLS_FUNCTION_PLACEMENT typename SPEC_A::T abs_diff(DEVICE& device, const rl::environments::l2f::parameters::MDP<SPEC_A>& a, const rl::environments::l2f::parameters::MDP<SPEC_B>& b) {
        using T = typename SPEC_A::T;
        T acc = 0;
        acc += abs_diff(device, a.init, b.init);
        acc += abs_diff(device, a.reward, b.reward);
        acc += abs_diff(device, a.observation_noise, b.observation_noise);
        acc += abs_diff(device, a.action_noise, b.action_noise);
        acc += abs_diff(device, a.termination, b.termination);
        return acc;
    }
    template<typename DEVICE, typename SPEC_A, typename SPEC_B>
    RL_TOOLS_FUNCTION_PLACEMENT typename SPEC_A::T abs_diff(DEVICE& device, const rl::environments::l2f::ParametersBase<SPEC_A>& a, const rl::environments::l2f::ParametersBase<SPEC_B>& b){
        using T = typename SPEC_A::T;
        T acc = 0;
        acc += abs_diff(device, a.dynamics, b.dynamics);
        acc += abs_diff(device, a.integration, b.integration);
        acc += abs_diff(device, a.mdp, b.mdp);
        return acc;
    }
    template<typename DEVICE, typename T_A, typename T_B>
    RL_TOOLS_FUNCTION_PLACEMENT T_A abs_diff(DEVICE& device, const rl::environments::l2f::parameters::Disturbances<T_A>& a, const rl::environments::l2f::parameters::Disturbances<T_B>& b) {
        T_A acc = 0;
        acc += math::abs(device.math, a.random_force.mean - b.random_force.mean);
        acc += math::abs(device.math, a.random_force.std - b.random_force.std);
        acc += math::abs(device.math, a.random_torque.mean - b.random_torque.mean);
        acc += math::abs(device.math, a.random_torque.std - b.random_torque.std);
        return acc;
    }
    template<typename DEVICE, typename SPEC_A, typename SPEC_B>
    RL_TOOLS_FUNCTION_PLACEMENT typename SPEC_A::T abs_diff(DEVICE& device, const rl::environments::l2f::ParametersDisturbances<SPEC_A>& a, const rl::environments::l2f::ParametersDisturbances<SPEC_B>& b){
        using T = typename SPEC_A::T;
        T acc = 0;
        acc += abs_diff(device, static_cast<const typename SPEC_A::NEXT_COMPONENT&>(a), static_cast<const typename SPEC_B::NEXT_COMPONENT&>(b));
        acc += abs_diff(device, a.disturbances, b.disturbances);
        return acc;
    }
    template<typename DEVICE, typename T_A, typename T_B>
    RL_TOOLS_FUNCTION_PLACEMENT T_A abs_diff(DEVICE& device, const rl::environments::l2f::parameters::DomainRandomization<T_A>& a, const rl::environments::l2f::parameters::DomainRandomization<T_B>& b) {
        T_A acc = 0;
        acc += math::abs(device.math, a.thrust_to_weight_min - b.thrust_to_weight_min);
        acc += math::abs(device.math, a.thrust_to_weight_max - b.thrust_to_weight_max);
        acc += math::abs(device.math, a.torque_to_inertia_min - b.torque_to_inertia_min);
        acc += math::abs(device.math, a.torque_to_inertia_max - b.torque_to_inertia_max);
        acc += math::abs(device.math, a.mass_min - b.mass_min);
        acc += math::abs(device.math, a.mass_max - b.mass_max);
        acc += math::abs(device.math, a.mass_size_deviation - b.mass_size_deviation);
        acc += math::abs(device.math, a.rotor_time_constant_rising_min - b.rotor_time_constant_rising_min);
        acc += math::abs(device.math, a.rotor_time_constant_rising_max - b.rotor_time_constant_rising_max);
        acc += math::abs(device.math, a.rotor_time_constant_falling_min - b.rotor_time_constant_falling_min);
        acc += math::abs(device.math, a.rotor_time_constant_falling_max - b.rotor_time_constant_falling_max);
        acc += math::abs(device.math, a.rotor_torque_constant_min - b.rotor_torque_constant_min);
        acc += math::abs(device.math, a.rotor_torque_constant_max - b.rotor_torque_constant_max);
        acc += math::abs(device.math, a.orientation_offset_angle_max - b.orientation_offset_angle_max);
        acc += math::abs(device.math, a.disturbance_force_max - b.disturbance_force_max);
        return acc;
    }
    template<typename DEVICE, typename SPEC_A, typename SPEC_B>
    RL_TOOLS_FUNCTION_PLACEMENT typename SPEC_A::T abs_diff(DEVICE& device, const rl::environments::l2f::ParametersDomainRandomization<SPEC_A>& a, const rl::environments::l2f::ParametersDomainRandomization<SPEC_B>& b){
        using T = typename SPEC_A::T;
        T acc = 0;
        acc += abs_diff(device, static_cast<const typename SPEC_A::NEXT_COMPONENT&>(a), static_cast<const typename SPEC_B::NEXT_COMPONENT&>(b));
        acc += abs_diff(device, a.domain_randomization, b.domain_randomization);
        return acc;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif

