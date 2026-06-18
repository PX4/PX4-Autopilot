#include <rl_tools/version.h>
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ZOO_L2F_ENVIRONMENT_BIG_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ZOO_L2F_ENVIRONMENT_BIG_H

#include <rl_tools/rl/environments/l2f/operations_multitask_generic_forward.h>
#include <rl_tools/rl/environments/l2f/operations_cpu.h>
#include <rl_tools/rl/environments/l2f/persist_code.h>


#include "environment.h"
#include <rl_tools/utils/generic/typing.h>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::zoo::l2f{
    namespace rlt = rl_tools;
    using namespace rl_tools::rl::environments::l2f;
    template <typename DEVICE, typename TYPE_POLICY, typename TI, typename OPTIONS>
    struct ENVIRONMENT_BIG_FACTORY{
        using T = typename TYPE_POLICY::DEFAULT;
        using BASE_ENV = rl_tools::rl::environments::Multirotor<Specification<T, TI>>;

        static constexpr auto MODEL = parameters::dynamics::REGISTRY::soft_rigid;
        constexpr static auto MODEL_NAME = rl_tools::rl::environments::l2f::parameters::dynamics::registry_name<MODEL>;

        using REWARD_FUNCTION = parameters::reward_functions::Squared<T>;
        struct DOMAIN_RANDOMIZATION_OPTIONS{
            static constexpr bool ENABLED = false;
            static constexpr bool THRUST_TO_WEIGHT = ENABLED;
            static constexpr bool MASS = ENABLED;
            static constexpr bool TORQUE_TO_INERTIA = ENABLED;
            static constexpr bool MASS_SIZE_DEVIATION = ENABLED;
            static constexpr bool ROTOR_TORQUE_CONSTANT = ENABLED;
            static constexpr bool DISTURBANCE_FORCE = ENABLED;
            static constexpr bool ROTOR_TIME_CONSTANT = ENABLED;
        };

        struct TRAJECTORY_OPTIONS{
            static constexpr bool LANGEVIN = true;
        };
        using PARAMETERS_SPEC = ParametersBaseSpecification<T, TI, 4, REWARD_FUNCTION>;
        using PARAMETERS_TYPE = ParametersTrajectory<ParametersTrajectorySpecification<T, TI, TRAJECTORY_OPTIONS, ParametersDomainRandomization<ParametersDomainRandomizationSpecification<T, TI, DOMAIN_RANDOMIZATION_OPTIONS, ParametersDisturbances<ParametersSpecification<T, TI, ParametersBase<PARAMETERS_SPEC>>>>>>>;

        static constexpr TI SIMULATION_FREQUENCY = 100;

        static constexpr auto BASE_PARAMS = BASE_ENV::SPEC::PARAMETER_VALUES;

        static constexpr auto dynamics = parameters::dynamics::registry<MODEL, typename BASE_ENV::SPEC>;
        static constexpr auto mdp = [](){
            auto mdp = BASE_PARAMS.mdp;
            T x = dynamics.rotor_positions[0][0];
            T y = dynamics.rotor_positions[0][1];
            T rotor_distance = ((x > 0 ? x : -x) + (y > 0 ? y : -y))/2 * 1.4142135623730951; // sqrt is not available in constexpr
            mdp.termination.position_threshold = rotor_distance * 20;
            mdp.init.max_position = rotor_distance * 10;
            mdp.init.max_angle = 1.5707963267948966 * 90.0/90.0;   // orientation
            auto& reward = mdp.reward;
            reward = {
                false, // non-negative
                01.00, // scale
                01.50, // constant
                -100.00, // termination penalty
                01.00, // position
                00.00, // position_clip
                00.10, // orientation
                00.00, // linear_velocity
                00.00, // angular_velocity
                00.00, // linear_acceleration
                00.00, // angular_acceleration
                00.00, // action
                01.00, // d_action
                00.00, // position_error_integral
            };
            return mdp;
        }();

        static constexpr auto trajectory = [](){
                auto traj = BASE_PARAMS.trajectory;
                traj.mixture[0] = 1.0; // ensure that the probability of using position control is 1
                return traj;
        }();

        static constexpr decltype(BASE_PARAMS.domain_randomization) domain_randomization = {
            0, // thrust_to_weight_min;
            0, // thrust_to_weight_max;
            0, // torque_to_inertia_min;
            0, // torque_to_inertia_max;
            0, // mass_min;
            0, // mass_max;
            0, // mass_size_deviation;
            0, // motor_time_constant_rising_min;
            0, // motor_time_constant_rising_max;
            0, // motor_time_constant_falling_min;
            0, // motor_time_constant_falling_max;
            0, // rotor_torque_constant_min;
            0, // rotor_torque_constant_max;
            0, // orientation_offset_angle_max;
            0  // disturbance_force_max;
        };

        static constexpr PARAMETERS_TYPE nominal_parameters = {
            {
                {
                    {
                        dynamics,
                        BASE_PARAMS.integration,
                        mdp
                    }, // Base
                    BASE_PARAMS.disturbances
                }, // Disturbances
                domain_randomization
            }, // DomainRandomization
            trajectory
        }; // Trajectory

        struct ENVIRONMENT_STATIC_PARAMETERS{
            static constexpr TI N_SUBSTEPS = 1;
            static constexpr TI ACTION_HISTORY_LENGTH = 8;
            static constexpr TI EPISODE_STEP_LIMIT = 5 * SIMULATION_FREQUENCY;
            static constexpr TI CLOSED_FORM = false;
            static constexpr TI ANGULAR_VELOCITY_DELAY = 0; // one step at 100hz = 10ms ~ delay from IMU to input to the policy: 1.3ms time constant of the IIR in the IMU (bw ~110Hz) + synchronization delay (2ms) + (negligible SPI transfer latency due to it being interrupt-based) + 1ms sensor.c RTOS loop @ 1khz + 2ms for the RLtools loop
            using STATE_BASE = StateAngularVelocityDelay<StateAngularVelocityDelaySpecification<T, TI, ANGULAR_VELOCITY_DELAY, StateLastAction<StateSpecification<T, TI, StateBase<StateSpecification<T, TI>>>>>>;
            using STATE_TYPE_MOTOR_DELAY = StateTrajectory<StateSpecification<T, TI, StateRotorsHistory<StateRotorsHistorySpecification<T, TI, ACTION_HISTORY_LENGTH, CLOSED_FORM, StateRandomForce<StateSpecification<T, TI, STATE_BASE>>>>>>;
            using STATE_TYPE_NO_MOTOR_DELAY = StateRandomForce<StateSpecification<T, TI, STATE_BASE>>;
            using STATE_TYPE = rl_tools::utils::typing::conditional_t<OPTIONS::MOTOR_DELAY, STATE_TYPE_MOTOR_DELAY, STATE_TYPE_NO_MOTOR_DELAY>;
            using OBSERVATION_TYPE = observation::TrajectoryTrackingPosition<observation::PositionSpecification<T, TI,
                    observation::OrientationRotationMatrix<observation::OrientationRotationMatrixSpecification<T, TI,
                    observation::TrajectoryTrackingLinearVelocity<observation::LinearVelocitySpecification<T, TI,
                    observation::AngularVelocityDelayed<observation::AngularVelocityDelayedSpecification<T, TI, ANGULAR_VELOCITY_DELAY,
                    // observation::RandomForce<observation::RandomForceSpecification<T, TI,
                    observation::ActionHistory<observation::ActionHistorySpecification<T, TI, ACTION_HISTORY_LENGTH // one-step action history to Markovify the d_action regularization
            >>>>>>>>>>;
            using OBSERVATION_TYPE_PRIVILEGED = OBSERVATION_TYPE;
            static constexpr bool PRIVILEGED_OBSERVATION_NOISE = false;
            using PARAMETERS = PARAMETERS_TYPE;
            static constexpr auto PARAMETER_VALUES = nominal_parameters;
            static constexpr TI N_DYNAMICS_VALUES = 1;
            static constexpr typename PARAMETERS_TYPE::Dynamics DYNAMICS_VALUES[N_DYNAMICS_VALUES] = {
                rl_tools::rl::environments::l2f::parameters::dynamics::registry<parameters::dynamics::REGISTRY::crazyflie, PARAMETERS_SPEC>
            };
            static constexpr T STATE_LIMIT_POSITION = 100000;
            static constexpr T STATE_LIMIT_VELOCITY = 100000;
            static constexpr T STATE_LIMIT_ANGULAR_VELOCITY = 100000;
        };

        using ENVIRONMENT_SPEC = Specification<T, TI, ENVIRONMENT_STATIC_PARAMETERS>;
        using ENVIRONMENT = rl::environments::Multirotor<ENVIRONMENT_SPEC>;
        // static_assert(rl::environments::PREVENT_DEFAULT_GET_UI<ENVIRONMENT>::value);
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif