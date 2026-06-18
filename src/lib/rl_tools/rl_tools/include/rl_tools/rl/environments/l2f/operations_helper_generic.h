#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ENVIRONMENTS_L2F_OPERATIONS_HELPER_GENERIC_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENTS_L2F_OPERATIONS_HELPER_GENERIC_H

#ifndef RL_TOOLS_FUNCTION_PLACEMENT
#define RL_TOOLS_FUNCTION_PLACEMENT
#endif

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template <typename DEVICE, typename PARAMETERS>
    void compare_parameters(DEVICE& device, const PARAMETERS& nominal_main, const PARAMETERS& perturbed_main){
        using T = typename PARAMETERS::T;
        using TI = typename DEVICE::index_t;
        auto& nominal = nominal_main.dynamics;
        auto& perturbed = perturbed_main.dynamics;
        auto percentage_change = [](auto nominal_value, auto perturbed_value){
            return ((perturbed_value - nominal_value) / nominal_value) * 100.0;
        };
        log(device, device.logger, "Mass: ", nominal.mass, " -> ", perturbed.mass, " (", percentage_change(nominal.mass, perturbed.mass), "%)");
        for (TI i = 0; i < 3; ++i) {
            log(device, device.logger, "J[", i, "][", i, "]: ", nominal.J[i][i], " -> ", perturbed.J[i][i], " (", percentage_change(nominal.J[i][i], perturbed.J[i][i]), "%)");
        }
        for (TI rotor_i = 0; rotor_i < 1; ++rotor_i) {
            T rotor_distance_nominal = math::sqrt(device.math, nominal.rotor_positions[rotor_i][0] * nominal.rotor_positions[rotor_i][0] + nominal.rotor_positions[rotor_i][1] * nominal.rotor_positions[rotor_i][1] + nominal.rotor_positions[rotor_i][2] * nominal.rotor_positions[rotor_i][2]);
            T rotor_distance_perturbed = math::sqrt(device.math, perturbed.rotor_positions[rotor_i][0] * perturbed.rotor_positions[rotor_i][0] + perturbed.rotor_positions[rotor_i][1] * perturbed.rotor_positions[rotor_i][1] + perturbed.rotor_positions[rotor_i][2] * perturbed.rotor_positions[rotor_i][2]);
            log(device, device.logger, "Rotor distance[", rotor_i, "]: ", rotor_distance_nominal, " -> ", rotor_distance_perturbed, " (", percentage_change(rotor_distance_nominal, rotor_distance_perturbed), "%)");
        }
        for (TI rotor_i = 0; rotor_i < PARAMETERS::N; ++rotor_i){
            for (TI i = 0; i < 3; ++i) {
                log(device, device.logger, "Rotor ", rotor_i, " thrust coefficient[", i, "]: ", nominal.rotor_thrust_coefficients[rotor_i][i], " -> ", perturbed.rotor_thrust_coefficients[rotor_i][i], " (", percentage_change(nominal.rotor_thrust_coefficients[rotor_i][i], perturbed.rotor_thrust_coefficients[rotor_i][i]), "%)");
            }
        }
        for (TI rotor_i = 0; rotor_i < PARAMETERS::N; ++rotor_i){
            log(device, device.logger, "Rotor torque constant: ", rotor_i, " ", nominal.rotor_torque_constants[rotor_i], " -> ", perturbed.rotor_torque_constants[rotor_i], " (", percentage_change(nominal.rotor_torque_constants[rotor_i], perturbed.rotor_torque_constants[rotor_i]), "%)");
            log(device, device.logger, "Motor time constant (up): ", rotor_i, " ", nominal.rotor_time_constants_rising[rotor_i], " -> ", perturbed.rotor_time_constants_rising[rotor_i], " (", percentage_change(nominal.rotor_time_constants_rising[rotor_i], perturbed.rotor_time_constants_rising[rotor_i]), "%)");
        }
        log(device, device.logger, "Hovering throttle relative: ", nominal.hovering_throttle_relative, " -> ", perturbed.hovering_throttle_relative, " (", percentage_change(nominal.hovering_throttle_relative, perturbed.hovering_throttle_relative), "%)");
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END


#endif