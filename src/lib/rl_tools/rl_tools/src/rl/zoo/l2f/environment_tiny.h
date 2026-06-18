#include <rl_tools/version.h>
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ZOO_L2F_ENVIRONMENT_TINY_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ZOO_L2F_ENVIRONMENT_TINY_H

#include <rl_tools/rl/environments/l2f/operations_multitask_generic_forward.h>
#include <rl_tools/rl/environments/l2f/operations_cpu.h>
#include <rl_tools/rl/environments/l2f/operations_multitask_generic.h>
#include <rl_tools/rl/environments/l2f/parameters/reward_functions/squared/operations_generic.h>
#include <rl_tools/rl/environments/l2f/parameters/reward_functions/default.h>
#include <rl_tools/rl/environments/l2f/parameters/default.h>
#include <rl_tools/rl/environments/l2f/parameters/dynamics/crazyflie.h>
#include <rl_tools/rl/environments/l2f/parameters/dynamics/arpl.h>
#include <rl_tools/rl/environments/l2f/parameters/dynamics/x500_sim.h>
#include <rl_tools/rl/environments/l2f/parameters/dynamics/x500_real.h>
#include <rl_tools/rl/environments/l2f/parameters/init/default.h>
#include <rl_tools/rl/environments/l2f/parameters/termination/default.h>

#include <rl_tools/rl/environments/l2f/persist_code.h>


#include "environment.h"
#include <rl_tools/utils/generic/typing.h>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::zoo::l2f{
    namespace rlt = rl_tools;
    using namespace rl_tools::rl::environments::l2f;
    template <typename DEVICE, typename TYPE_POLICY, typename TI>
    struct ENVIRONMENT_TINY_FACTORY{
        using T = typename TYPE_POLICY::DEFAULT;
        using ENVIRONMENT_FACTORY_BASE = ENVIRONMENT_FACTORY<DEVICE, TYPE_POLICY, TI>;
        using PARAMETERS_SPEC = typename ENVIRONMENT_FACTORY_BASE::PARAMETERS_SPEC;
        using PARAMETERS_TYPE = typename ENVIRONMENT_FACTORY_BASE::PARAMETERS_TYPE;

        static constexpr auto MODEL = rl_tools::rl::environments::l2f::parameters::dynamics::REGISTRY::crazyflie;
        constexpr static auto MODEL_NAME = rl_tools::rl::environments::l2f::parameters::dynamics::registry_name<MODEL>;
        static constexpr typename PARAMETERS_TYPE::Dynamics dynamics = [](){
            auto p = rl_tools::rl::environments::l2f::parameters::dynamics::registry<MODEL, PARAMETERS_SPEC>;
            p.rotor_time_constants_rising[0] = 0.072;
            p.rotor_time_constants_rising[1] = 0.072;
            p.rotor_time_constants_rising[2] = 0.072;
            p.rotor_time_constants_rising[3] = 0.072;
            p.rotor_time_constants_falling[0] = 0.072;
            p.rotor_time_constants_falling[1] = 0.072;
            p.rotor_time_constants_falling[2] = 0.072;
            p.rotor_time_constants_falling[3] = 0.072;
            p.mass = 0.025;
            p.rotor_thrust_coefficients[0][0] = 0;
            p.rotor_thrust_coefficients[1][0] = 0;
            p.rotor_thrust_coefficients[2][0] = 0;
            p.rotor_thrust_coefficients[3][0] = 0;
            p.rotor_thrust_coefficients[0][1] = 0;
            p.rotor_thrust_coefficients[1][1] = 0;
            p.rotor_thrust_coefficients[2][1] = 0;
            p.rotor_thrust_coefficients[3][1] = 0;
            p.rotor_thrust_coefficients[0][1] = 0;
            p.rotor_thrust_coefficients[1][1] = 0;
            p.rotor_thrust_coefficients[2][1] = 0;
            p.rotor_thrust_coefficients[3][1] = 0;
            p.rotor_thrust_coefficients[0][2] = 0.1302;
            p.rotor_thrust_coefficients[1][2] = 0.1302;
            p.rotor_thrust_coefficients[2][2] = 0.1302;
            p.rotor_thrust_coefficients[3][2] = 0.1302;
            return p;
        }();

        static constexpr typename ParametersBase<PARAMETERS_SPEC>::MDP::Initialization init = {
                0.0, // guidance
                0.5, // position
                1.5707963267948966 * 20.0/90.0,   // orientation
                1,   // linear velocity
                1,  // angular velocity
                true,// relative rpm
                -1,  // min rpm
                +1,  // max rpm
        };
        using REWARD_FUNCTION = rl_tools::rl::environments::l2f::parameters::reward_functions::Squared<T>;
        static constexpr REWARD_FUNCTION reward_function = {
                false, // non-negative
                00.10, // scale
                01.10, // constant
                00.00, // termination penalty
                10.00, // position
                00.00, // position_clip;
                02.50, // orientation
                01.00, // linear_velocity
                00.00, // angular_velocity
                00.00, // linear_acceleration
                00.00, // angular_acceleration
                02.00, // action
                00.00, // d_action
                00.00, // position_error_integral
        };
        static constexpr typename PARAMETERS_TYPE::MDP mdp = {
            init,
            reward_function,
            ENVIRONMENT_FACTORY_BASE::observation_noise,
            ENVIRONMENT_FACTORY_BASE::action_noise,
            ENVIRONMENT_FACTORY_BASE::termination
        };
        static constexpr TI SIMULATION_FREQUENCY = 50;
        static constexpr typename PARAMETERS_TYPE::Integration integration = {
            1.0/((T)SIMULATION_FREQUENCY) // integration dt
        };

        static constexpr decltype(ENVIRONMENT_FACTORY_BASE::trajectory) trajectory = {
            {1.0, 0.0}, // mixture weights
            typename PARAMETERS_TYPE::Trajectory::Langevin{
                1.00, // gamma
                2.00, // omega
                0.50, // sigma
                0.01 // alpha
            }
        };

        static constexpr PARAMETERS_TYPE nominal_parameters = {
            {
                {
                    {
                        dynamics,
                        integration,
                        mdp
                    },
                    ENVIRONMENT_FACTORY_BASE::disturbances
                },
                ENVIRONMENT_FACTORY_BASE::domain_randomization
            },
            ENVIRONMENT_FACTORY_BASE::trajectory
        };

        struct ENVIRONMENT_STATIC_PARAMETERS{
            static constexpr TI N_SUBSTEPS = 1;
            static constexpr TI ACTION_HISTORY_LENGTH = 2;
            static constexpr TI EPISODE_STEP_LIMIT = 5 * SIMULATION_FREQUENCY;
            static constexpr TI CLOSED_FORM = false;
            using STATE_BASE = StateBase<StateSpecification<T, TI>>;
            using STATE_TYPE = StateRotorsHistory<StateRotorsHistorySpecification<T, TI, ACTION_HISTORY_LENGTH, CLOSED_FORM, StateRandomForce<StateSpecification<T, TI, STATE_BASE>>>>;
            using OBSERVATION_TYPE = observation::Position<observation::PositionSpecification<T, TI,
                    observation::OrientationRotationMatrix<observation::OrientationRotationMatrixSpecification<T, TI,
                            observation::LinearVelocity<observation::LinearVelocitySpecification<T, TI,
                                    observation::AngularVelocity<observation::AngularVelocitySpecification<T, TI,
                                            observation::ActionHistory<observation::ActionHistorySpecification<T, TI, ACTION_HISTORY_LENGTH>>>>>>>>>>;
            using OBSERVATION_TYPE_PRIVILEGED = typename ENVIRONMENT_FACTORY_BASE::ENVIRONMENT_STATIC_PARAMETERS::OBSERVATION_TYPE_PRIVILEGED;
            static constexpr bool PRIVILEGED_OBSERVATION_NOISE = false;
            using PARAMETERS = PARAMETERS_TYPE;
            static constexpr auto PARAMETER_VALUES = nominal_parameters;
            static constexpr T STATE_LIMIT_POSITION = 100000;
            static constexpr T STATE_LIMIT_VELOCITY = 100000;
            static constexpr T STATE_LIMIT_ANGULAR_VELOCITY = 100000;
        };

        using ENVIRONMENT_SPEC = rl_tools::rl::environments::l2f::Specification<T, TI, ENVIRONMENT_STATIC_PARAMETERS>;
        using ENVIRONMENT = rl_tools::rl::environments::Multirotor<ENVIRONMENT_SPEC>;
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif