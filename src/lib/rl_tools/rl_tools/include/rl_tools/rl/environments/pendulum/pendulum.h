#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ENVIRONMENTS_PENDULUM_PENDULUM_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENTS_PENDULUM_PENDULUM_H

#include "../../../math/operations_generic.h"
#include "../environments.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::environments::pendulum {
    template <typename T>
    struct DefaultParameters {
        constexpr static T g = 10;
        constexpr static T max_speed = 8;
        constexpr static T max_torque = 2;
        constexpr static T dt = 0.05;
        constexpr static T m = 1;
        constexpr static T l = 1;
        constexpr static T initial_state_min_angle = -math::PI<T>;
        constexpr static T initial_state_max_angle = math::PI<T>;
        constexpr static T initial_state_min_speed = -1;
        constexpr static T initial_state_max_speed = 1;
        constexpr static T OBSERVATION_NOISE_POSITION = 0.0;
        constexpr static T OBSERVATION_NOISE_VELOCITY = 0.0;
    };
    template <typename T_T, typename T_TI, typename T_PARAMETERS = DefaultParameters<T_T>>
    struct Specification{
        using T = T_T;
        using TI = T_TI;
        using PARAMETERS = T_PARAMETERS;
    };

    template <typename TI>
    struct ObservationFourier{
        static constexpr TI DIM = 3;
    };
    template <typename TI>
    struct ObservationRaw{
        static constexpr TI DIM = 2;
    };
    template <typename TI>
    struct ObservationPosition{
        static constexpr TI DIM = 2;
    };
    template <typename TI>
    struct ObservationVelocity{
        static constexpr TI DIM = 1;
    };
    template <typename TI>
    struct ObservationVelocityLastAction{
        static constexpr TI DIM = 2;
    };
    template <typename TI>
    struct ObservationMultiTask{
        static constexpr TI DIM = 3 + 1;
    };
    template <typename TI>
    struct ObservationMeta{
        static constexpr TI DIM = 3 + 1;
    };


    template <typename T_T, typename T_TI>
    struct StateSpecification{
        using T = T_T;
        using TI = T_TI;
    };
    template <typename T_SPEC>
    struct State{
        using SPEC = T_SPEC;
        using T = typename SPEC::T;
        using TI = typename SPEC::TI;
        static constexpr TI DIM = 2;
        T theta;
        T theta_dot;
    };

    template <typename T_SPEC>
    struct StateLastAction: State<T_SPEC>{
        using SPEC = T_SPEC;
        using T = typename SPEC::T;
        T last_action;
    };
    template <typename T_SPEC>
    struct StateMultiTask: State<T_SPEC>{
        bool invert_action;
    };
    template <typename T_SPEC>
    struct StateMeta: StateLastAction<T_SPEC>{
        bool invert_action;
    };

}
RL_TOOLS_NAMESPACE_WRAPPER_END

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::environments{
    template <typename T_SPEC>
    struct Pendulum: Environment<typename T_SPEC::T, typename T_SPEC::TI>{
        using SPEC = T_SPEC;
        using T = typename SPEC::T;
        using TI = typename SPEC::TI;
        using State = pendulum::State<pendulum::StateSpecification<T, TI>>;
        using Parameters = typename SPEC::PARAMETERS;
        using Observation = pendulum::ObservationFourier<TI>;
        using ObservationPrivileged = Observation;
        static constexpr TI N_AGENTS = 1; // single agent
        static constexpr TI ACTION_DIM = 1;
        static constexpr TI EPISODE_STEP_LIMIT = 200;
    };
    template <typename T_SPEC>
    struct PendulumAsymmetric: Pendulum<T_SPEC>{
        using PENDULUM = Pendulum<T_SPEC>;
        using Observation = pendulum::ObservationRaw<typename PENDULUM::TI>;
        using ObservationPrivileged = pendulum::ObservationFourier<typename PENDULUM::TI>;
    };
    template <typename T_SPEC>
    struct PendulumPosition: Pendulum<T_SPEC>{
        using T = typename T_SPEC::T;
        using TI = typename T_SPEC::TI;
        using State = pendulum::StateLastAction<pendulum::StateSpecification<T, TI>>;
        using Observation = pendulum::ObservationPosition<typename T_SPEC::TI>;
        using ObservationPrivileged = Observation;
    };
    template <typename SPEC>
    struct PREVENT_DEFAULT_GET_UI<PendulumPosition<SPEC>>: rl_tools::utils::typing::true_type{};
    template <typename SPEC>
    struct PREVENT_DEFAULT_GET_DESCRIPTION<PendulumPosition<SPEC>>: rl_tools::utils::typing::true_type{};
    template <typename T_SPEC>
    struct PendulumVelocity: Pendulum<T_SPEC>{
        using T = typename T_SPEC::T;
        using TI = typename T_SPEC::TI;
        using State = pendulum::State<pendulum::StateSpecification<T, TI>>;
        using Observation = pendulum::ObservationVelocity<typename T_SPEC::TI>;
        using ObservationPrivileged = Observation;
    };
    template <typename SPEC>
    struct PREVENT_DEFAULT_GET_UI<PendulumVelocity<SPEC>>: rl_tools::utils::typing::true_type{};
    template <typename SPEC>
    struct PREVENT_DEFAULT_GET_DESCRIPTION<PendulumVelocity<SPEC>>: rl_tools::utils::typing::true_type{};
    template <typename T_SPEC>
    struct PendulumVelocityLastAction: Pendulum<T_SPEC>{
        using T = typename T_SPEC::T;
        using TI = typename T_SPEC::TI;
        using State = pendulum::StateLastAction<pendulum::StateSpecification<T, TI>>;
        using Observation = pendulum::ObservationVelocityLastAction<typename T_SPEC::TI>;
        using ObservationPrivileged = Observation;
    };
    template <typename SPEC>
    struct PREVENT_DEFAULT_GET_UI<PendulumVelocityLastAction<SPEC>>: rl_tools::utils::typing::true_type{};
    template <typename SPEC>
    struct PREVENT_DEFAULT_GET_DESCRIPTION<PendulumVelocityLastAction<SPEC>>: rl_tools::utils::typing::true_type{};

    template <typename T_SPEC>
    struct PendulumMultiTask: Pendulum<T_SPEC>{
        using T = typename T_SPEC::T;
        using TI = typename T_SPEC::TI;
        using State = pendulum::StateMultiTask<pendulum::StateSpecification<T, TI>>;
        using Observation = pendulum::ObservationMultiTask<typename T_SPEC::TI>;
        using ObservationPrivileged = Observation;
    };
    template <typename SPEC>
    struct PREVENT_DEFAULT_GET_UI<PendulumMultiTask<SPEC>>: rl_tools::utils::typing::true_type{};
    template <typename SPEC>
    struct PREVENT_DEFAULT_GET_DESCRIPTION<PendulumMultiTask<SPEC>>: rl_tools::utils::typing::true_type{};

    template <typename T_SPEC>
    struct PendulumMeta: Pendulum<T_SPEC>{
        using T = typename T_SPEC::T;
        using TI = typename T_SPEC::TI;
        using State = pendulum::StateMeta<pendulum::StateSpecification<T, TI>>;
        using Observation = pendulum::ObservationMeta<typename T_SPEC::TI>;
        using ObservationPrivileged = Observation;
    };
    template <typename SPEC>
    struct PREVENT_DEFAULT_GET_UI<PendulumMeta<SPEC>>: rl_tools::utils::typing::true_type{};
    template <typename SPEC>
    struct PREVENT_DEFAULT_GET_DESCRIPTION<PendulumMeta<SPEC>>: rl_tools::utils::typing::true_type{};

}
RL_TOOLS_NAMESPACE_WRAPPER_END







#endif
