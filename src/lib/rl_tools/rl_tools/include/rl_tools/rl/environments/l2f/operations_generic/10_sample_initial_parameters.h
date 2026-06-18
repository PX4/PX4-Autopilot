#include "../../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ENVIRONMENTS_L2F_OPERATIONS_GENERIC_SAMPLE_INITIAL_PARAMETERS_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENTS_L2F_OPERATIONS_GENERIC_SAMPLE_INITIAL_PARAMETERS_H

#include "../multirotor.h"

#include <rl_tools/utils/generic/vector_operations.h>
#include "../quaternion_helper.h"

#include <rl_tools/utils/generic/typing.h>

#include <rl_tools/rl/environments/operations_generic.h>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template<typename DEVICE, typename SPEC, typename PARAMETERS, typename RNG>
    static void sample_initial_parameters(DEVICE& device, rl::environments::Multirotor<SPEC>& env, PARAMETERS& parameters, RNG& rng); // forward declaration for out-of-declaration order dispatch
    namespace rl::environments::l2f{
        template<typename DEVICE, typename SPEC, typename PARAMETER_SPEC, typename RNG>
        static void _sample_initial_parameters(DEVICE& device, Multirotor<SPEC>& env, ParametersBase<PARAMETER_SPEC>& parameters, RNG& rng){
            parameters = env.parameters;
        }
        template<typename DEVICE, typename SPEC, typename PARAMETER_SPEC, typename RNG>
        static void _sample_initial_parameters(DEVICE& device, Multirotor<SPEC>& env, ParametersDisturbances<PARAMETER_SPEC>& parameters, RNG& rng){
            sample_initial_parameters(device, env, static_cast<typename PARAMETER_SPEC::NEXT_COMPONENT&>(parameters), rng);
            parameters.disturbances = env.parameters.disturbances;
        }
        template <typename T, typename DEVICE, typename RNG>
        T _sample_domain_randomization_factor(DEVICE& device, T range, RNG& rng) {
            T factor = random::normal_distribution::sample(device.random, -range, range, rng);
            factor = factor < 0 ? 1/(1-factor) : 1+factor; // reciprocal scale, note 1-factor because factor is negative in that case anyways
            return factor;
        }
        template<typename DEVICE, typename SPEC, typename PARAMETER_SPEC, typename RNG>
        static void _sample_initial_parameters(DEVICE& device, rl_tools::rl::environments::Multirotor<SPEC>& env, ParametersDomainRandomization<PARAMETER_SPEC>& parameters, RNG& rng){
            using T = typename SPEC::T;
            using TI = typename DEVICE::index_t;
            using PARAMETERS = ParametersDomainRandomization<PARAMETER_SPEC>;
            using OPTS = typename PARAMETER_SPEC::DOMAIN_RANDOMIZATION_OPTIONS;
            sample_initial_parameters(device, env, static_cast<typename PARAMETER_SPEC::NEXT_COMPONENT&>(parameters), rng);
            parameters.domain_randomization = env.parameters.domain_randomization;
            /*
             *  Strategy:
             *  1. Sample Thrust to Weight
             *  2. Sample Mass
             *  3. Calculate resulting thrust curve (based on max input)
             *  4. Get torque to inertia based on the scale (based on the sampled mass)
             *  5. Sample new torque_to_inertia around calculated one
             *  6. Sample a new size based on the scale (based on the sampled mass)
             *      a. Adjust the rotor positions
             *  7. Adjust inertia to fit the sampled torque to inertia ratio
             */
            T thrust_to_weight_nominal;
            {
                T max_action = parameters.dynamics.action_limit.max;
                T max_thrust_nominal = 0;
                for(TI rotor_i = 0; rotor_i < PARAMETERS::N; rotor_i++){
                    max_thrust_nominal += parameters.dynamics.rotor_thrust_coefficients[rotor_i][0] + parameters.dynamics.rotor_thrust_coefficients[rotor_i][1] * max_action + parameters.dynamics.rotor_thrust_coefficients[rotor_i][2] * max_action * max_action;
                }
                T gravity_norm = math::sqrt(device.math, parameters.dynamics.gravity[0] * parameters.dynamics.gravity[0] + parameters.dynamics.gravity[1] * parameters.dynamics.gravity[1] + parameters.dynamics.gravity[2] * parameters.dynamics.gravity[2]);
                thrust_to_weight_nominal = max_thrust_nominal / (parameters.dynamics.mass * gravity_norm); // this assumes all the rotors are pointing into the same direction
            }

            T thrust_to_weight = thrust_to_weight_nominal;
            T factor_thrust_to_weight = 1;
            if constexpr(OPTS::THRUST_TO_WEIGHT){
                rl_tools::utils::assert_exit(device, parameters.domain_randomization.thrust_to_weight_min < parameters.domain_randomization.thrust_to_weight_max, "L2f: Domain randomization thrust_to_weight max should be larger than min. If you intended to turn off this randomization please deactivate it in the static parameter options (cf. DefaultParametersDomainRandomizationOptions)");
                rl_tools::utils::assert_exit(device, parameters.domain_randomization.thrust_to_weight_min >= 1.5, "L2f: Domain randomization thrust_to_weight min should be larger than or equal to 1.5 to maintain maneuverability.");
                thrust_to_weight = random::uniform_real_distribution(device.random, parameters.domain_randomization.thrust_to_weight_min, parameters.domain_randomization.thrust_to_weight_max, rng);
                factor_thrust_to_weight = thrust_to_weight / thrust_to_weight_nominal;
            }
            else{
                rl_tools::utils::assert_exit(device, parameters.domain_randomization.thrust_to_weight_min == 0 && parameters.domain_randomization.mass_max == 0 , "L2f: Domain randomization mass max/min should be 0 if THRUST_TO_WEIGHT is false. If you intended to turn off this randomization please deactivate it in the static parameter options (cf. DefaultParametersDomainRandomizationOptions)");
            }

            T factor_mass = 1;
            T scale_relative = 1;
            if constexpr(OPTS::MASS){
                rl_tools::utils::assert_exit(device, parameters.domain_randomization.mass_min < parameters.domain_randomization.mass_max, "L2f: Domain randomization mass max should be larger than min. If you intended to turn off this randomization please deactivate it in the static parameter options (cf. DefaultParametersDomainRandomizationOptions)");
                // we want to sample uniformly in size (m) and mass goes with the cube of the size. If we were to sample uniformly in mass we would be biased towards larger quadrotors
                T relative_size_min = math::cbrt(device.math, parameters.domain_randomization.mass_min);
                T relative_size_max = math::cbrt(device.math, parameters.domain_randomization.mass_max);
                T size_new = random::uniform_real_distribution(device.random, relative_size_min, relative_size_max, rng);
                T mass_new = size_new * size_new * size_new;
                mass_new = math::clamp(device.math, mass_new, parameters.domain_randomization.mass_min, parameters.domain_randomization.mass_max);
                scale_relative = math::cbrt(device.math, mass_new/parameters.dynamics.mass);
                factor_mass = mass_new / parameters.dynamics.mass;
                parameters.dynamics.mass = mass_new;
            }
            else{
                rl_tools::utils::assert_exit(device, parameters.domain_randomization.mass_min == 0 && parameters.domain_randomization.mass_max == 0 , "L2f: Domain randomization mass max/min should be 0 if MASS is false. If you intended to turn off this randomization please deactivate it in the static parameter options (cf. DefaultParametersDomainRandomizationOptions)");
            }

            if constexpr(OPTS::THRUST_TO_WEIGHT || OPTS::MASS){
                T factor_thrust_coefficients = factor_thrust_to_weight * factor_mass;
                for(TI rotor_i = 0; rotor_i < PARAMETERS::N; rotor_i++){
                    for(TI order_i = 0; order_i < 3; order_i++){
                        parameters.dynamics.rotor_thrust_coefficients[rotor_i][order_i] *= factor_thrust_coefficients;
                    }
                }
            }

            T torque_to_inertia_factor = 1;
            if constexpr(OPTS::TORQUE_TO_INERTIA){
                // todo: think about min / max torque (which depend on the rotor positions)
                T max_action = parameters.dynamics.action_limit.max;
                // T max_thrust = parameters.dynamics.rotor_thrust_coefficients[0][0] + parameters.dynamics.rotor_thrust_coefficients[0][1] * max_action + parameters.dynamics.rotor_thrust_coefficients[0][2] * max_action * max_action;
                T gravity_norm = math::sqrt(device.math, parameters.dynamics.gravity[0] * parameters.dynamics.gravity[0] + parameters.dynamics.gravity[1] * parameters.dynamics.gravity[1] + parameters.dynamics.gravity[2] * parameters.dynamics.gravity[2]);
                T max_thrust = thrust_to_weight * parameters.dynamics.mass * gravity_norm / PARAMETERS::N;
                // T first_rotor_distance_nominal = math::sqrt(device.math, parameters.dynamics.rotor_positions[0][0] * parameters.dynamics.rotor_positions[0][0] + parameters.dynamics.rotor_positions[0][1] * parameters.dynamics.rotor_positions[0][1] + parameters.dynamics.rotor_positions[0][2] * parameters.dynamics.rotor_positions[0][2]);
                T first_rotor_distance_nominal = math::abs(device.math, parameters.dynamics.rotor_positions[0][0]);
                T max_torque = first_rotor_distance_nominal * 1.414213562373095 * max_thrust; // 2/sqrt(2) = sqrt(2): max thrust assuming all rotors have equal angles and the same distance to the center two rotors active
                T x_inertia = parameters.dynamics.J[0][0];
                T torque_to_inertia_nominal = max_torque / x_inertia;

                rl_tools::utils::assert_exit(device, parameters.domain_randomization.torque_to_inertia_min < parameters.domain_randomization.torque_to_inertia_max, "L2f: Domain randomization thrust_to_weight_by_torque_to_inertia max should be larger than min. If you intended to turn off this randomization please deactivate it in the static parameter options (cf. DefaultParametersDomainRandomizationOptions)");
                T torque_to_inertia = random::uniform_real_distribution(device.random, parameters.domain_randomization.torque_to_inertia_min, parameters.domain_randomization.torque_to_inertia_max, rng);
                torque_to_inertia_factor = torque_to_inertia / torque_to_inertia_nominal;
            }
            else{
                rl_tools::utils::assert_exit(device, parameters.domain_randomization.torque_to_inertia_min == 0 && parameters.domain_randomization.torque_to_inertia_max == 0 , "L2f: Domain randomization thrust_to_weight_by_torque_to_inertia max/min should be 0 if TORQUE_TO_INERTIA is false. If you intended to turn off this randomization please deactivate it in the static parameter options (cf. DefaultParametersDomainRandomizationOptions)");
            }

            T rotor_distance_factor = 1;
            if constexpr(OPTS::MASS_SIZE_DEVIATION){
                rl_tools::utils::assert_exit(device, parameters.domain_randomization.mass_size_deviation != 0, "L2f: Domain randomization mass_size_deviation should be != 0. If you intended to turn off this randomization please deactivate it in the static parameter options (cf. DefaultParametersDomainRandomizationOptions)");
                T size_factor = _sample_domain_randomization_factor(device, parameters.domain_randomization.mass_size_deviation, rng);
                rotor_distance_factor = scale_relative * size_factor;
            }
            else{
                rl_tools::utils::assert_exit(device, parameters.domain_randomization.mass_size_deviation == 0 , "L2f: Domain randomization mass_size_deviation should be 0 if MASS_SIZE_DEVIATION is falls. If you intended to turn off this randomization please deactivate it in the static parameter options (cf. DefaultParametersDomainRandomizationOptions)");
            }
            if constexpr(OPTS::TORQUE_TO_INERTIA || OPTS::MASS_SIZE_DEVIATION){
                T inertia_factor = torque_to_inertia_factor/rotor_distance_factor;

                for(TI axis_i = 0; axis_i < 3; axis_i++){
                    parameters.dynamics.J[axis_i][axis_i] /= inertia_factor;
                    parameters.dynamics.J_inv[axis_i][axis_i] *= inertia_factor;
                    // todo sample I_yy and I_zz, I_xx is random already through the torque_to_inertia mechanism
                }
                for(TI rotor_i = 0; rotor_i < 4; rotor_i++){
                    for(TI axis_i = 0; axis_i < 3; axis_i++){
                        parameters.dynamics.rotor_positions[rotor_i][axis_i] *= rotor_distance_factor;
                    }
                }
                T max_rotor_distance = 0;
                for (TI rotor_i=0; rotor_i < PARAMETERS::N; rotor_i++){
                    T rotor_distance = math::sqrt(device.math, parameters.dynamics.rotor_positions[rotor_i][0] * parameters.dynamics.rotor_positions[rotor_i][0] + parameters.dynamics.rotor_positions[rotor_i][1] * parameters.dynamics.rotor_positions[rotor_i][1] + parameters.dynamics.rotor_positions[rotor_i][2] * parameters.dynamics.rotor_positions[rotor_i][2]);
                    if (rotor_distance > max_rotor_distance){
                        max_rotor_distance = rotor_distance;
                    }
                }
                parameters.mdp.termination.position_threshold = max_rotor_distance * 20;
                parameters.mdp.init.max_position = max_rotor_distance * 10;
            }
            if constexpr(OPTS::ROTOR_TORQUE_CONSTANT){
                rl_tools::utils::assert_exit(device, parameters.domain_randomization.rotor_torque_constant_min != 0 && parameters.domain_randomization.rotor_torque_constant_max != 0, "L2f: Domain randomization rotor_torque_constant should be != 0 if ROTOR_TORQUE_CONSTANT is true. If you intended to turn off this randomization please deactivate it in the static parameter options (cf. DefaultParametersDomainRandomizationOptions)");
                T torque_constant = random::uniform_real_distribution(device.random, parameters.domain_randomization.rotor_torque_constant_min, parameters.domain_randomization.rotor_torque_constant_max, rng);
                for (TI rotor_i=0; rotor_i < PARAMETERS::N; rotor_i++){
                    parameters.dynamics.rotor_torque_constants[rotor_i] = torque_constant;
                }
            }
            else
            {
                rl_tools::utils::assert_exit(device, parameters.domain_randomization.rotor_torque_constant_min == 0 && parameters.domain_randomization.rotor_torque_constant_max == 0, "L2f: Domain randomization rotor_torque_constant should be 0 if ROTOR_TORQUE_CONSTANT is false. If you intended to turn off this randomization please deactivate it in the static parameter options (cf. DefaultParametersDomainRandomizationOptions)");
            }
            T disturbance_force_std = 0;
            if constexpr(OPTS::DISTURBANCE_FORCE){
                rl_tools::utils::assert_exit(device, parameters.domain_randomization.disturbance_force_max != 0, "L2f: Domain randomization disturbance_force_max should be != 0 if DISTURBANCE_FORCE is true. If you intended to turn off this randomization please deactivate it in the static parameter options (cf. DefaultParametersDomainRandomizationOptions)");
                T surplus_thrust_to_weight = thrust_to_weight - 1.0;
                if (surplus_thrust_to_weight < 0){
                    surplus_thrust_to_weight = 0;
                }
                T disturbance_force_thrust_to_weight_multiple = random::uniform_real_distribution(device.random, (T)0, surplus_thrust_to_weight * parameters.domain_randomization.disturbance_force_max, rng);
                disturbance_force_std = disturbance_force_thrust_to_weight_multiple * thrust_to_weight * parameters.dynamics.mass / 3; // divide by three to have 3 sigma containment
                parameters.disturbances.random_force.mean = 0;
                parameters.disturbances.random_force.std = disturbance_force_std;
            }
            else{
                rl_tools::utils::assert_exit(device, parameters.domain_randomization.disturbance_force_max == 0, "L2f: Domain randomization disturbance_force_max should be 0 if DISTURBANCE_FORCE is false. If you intended to turn off this randomization please deactivate it in the static parameter options (cf. DefaultParametersDomainRandomizationOptions)");
            }
            if constexpr(OPTS::ROTOR_TIME_CONSTANT){
                rl_tools::utils::assert_exit(device, parameters.domain_randomization.rotor_time_constant_rising_min != 0, "L2f: Domain randomization rotor_time_constant_rising_min should be != 0 if ROTOR_TIME_CONSTANT is true. If you intended to turn off this randomization please deactivate it in the static parameter options (cf. DefaultParametersDomainRandomizationOptions)");
                rl_tools::utils::assert_exit(device, parameters.domain_randomization.rotor_time_constant_rising_max != 0, "L2f: Domain randomization rotor_time_constant_rising_max should be != 0 if ROTOR_TIME_CONSTANT is true. If you intended to turn off this randomization please deactivate it in the static parameter options (cf. DefaultParametersDomainRandomizationOptions)");
                rl_tools::utils::assert_exit(device, parameters.domain_randomization.rotor_time_constant_falling_min != 0, "L2f: Domain randomization rotor_time_constant_falling_min should be != 0 if ROTOR_TIME_CONSTANT is true. If you intended to turn off this randomization please deactivate it in the static parameter options (cf. DefaultParametersDomainRandomizationOptions)");
                rl_tools::utils::assert_exit(device, parameters.domain_randomization.rotor_time_constant_falling_max != 0, "L2f: Domain randomization rotor_time_constant_falling_max should be != 0 if ROTOR_TIME_CONSTANT is true. If you intended to turn off this randomization please deactivate it in the static parameter options (cf. DefaultParametersDomainRandomizationOptions)");
                T rising = random::uniform_real_distribution(device.random, parameters.domain_randomization.rotor_time_constant_rising_min, parameters.domain_randomization.rotor_time_constant_rising_max, rng);
                T falling = random::uniform_real_distribution(device.random, parameters.domain_randomization.rotor_time_constant_falling_min, parameters.domain_randomization.rotor_time_constant_falling_max, rng);
                for (TI rotor_i=0; rotor_i < PARAMETERS::N; rotor_i++){
                    parameters.dynamics.rotor_time_constants_rising[rotor_i] = rising;
                    parameters.dynamics.rotor_time_constants_falling[rotor_i] = falling;
                }
            }
            else {
                rl_tools::utils::assert_exit(device, parameters.domain_randomization.rotor_time_constant_rising_min == 0, "L2f: Domain randomization rotor_time_constant_rising_min should be == 0 if ROTOR_TIME_CONSTANT is false. If you intended to turn off this randomization please deactivate it in the static parameter options (cf. DefaultParametersDomainRandomizationOptions)");
                rl_tools::utils::assert_exit(device, parameters.domain_randomization.rotor_time_constant_rising_max == 0, "L2f: Domain randomization rotor_time_constant_rising_max should be == 0 if ROTOR_TIME_CONSTANT is false. If you intended to turn off this randomization please deactivate it in the static parameter options (cf. DefaultParametersDomainRandomizationOptions)");
                rl_tools::utils::assert_exit(device, parameters.domain_randomization.rotor_time_constant_falling_min == 0, "L2f: Domain randomization rotor_time_constant_falling_min should be == 0 if ROTOR_TIME_CONSTANT is false. If you intended to turn off this randomization please deactivate it in the static parameter options (cf. DefaultParametersDomainRandomizationOptions)");
                rl_tools::utils::assert_exit(device, parameters.domain_randomization.rotor_time_constant_falling_max == 0, "L2f: Domain randomization rotor_time_constant_falling_max should be == 0 if ROTOR_TIME_CONSTANT is false. If you intended to turn off this randomization please deactivate it in the static parameter options (cf. DefaultParametersDomainRandomizationOptions)");
            }
        }
        template<typename DEVICE, typename SPEC, typename PARAMETER_SPEC, typename RNG>
        static void _sample_initial_parameters(DEVICE& device, Multirotor<SPEC>& env, ParametersTrajectory<PARAMETER_SPEC>& parameters, RNG& rng){
            sample_initial_parameters(device, env, static_cast<typename PARAMETER_SPEC::NEXT_COMPONENT&>(parameters), rng);
            parameters.trajectory = env.parameters.trajectory;
        }
        template<typename DEVICE, typename SPEC, typename PARAMETER_SPEC, typename RNG>
        static void _sample_initial_parameters(DEVICE& device, Multirotor<SPEC>& env, ParametersObservationDelay<PARAMETER_SPEC>& parameters, RNG& rng){
            sample_initial_parameters(device, env, static_cast<typename PARAMETER_SPEC::NEXT_COMPONENT&>(parameters), rng);
            parameters.observation_delay.linear_velocity = env.parameters.observation_delay.linear_velocity;
            parameters.observation_delay.angular_velocity = env.parameters.observation_delay.angular_velocity;
        }
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif