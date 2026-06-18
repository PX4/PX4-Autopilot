#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ENVIRONMENTS_FLAG_ENVIRONMENT_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENTS_FLAG_ENVIRONMENT_H

#include "../../../math/operations_generic.h"
#include "../environments.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::environments::flag{
    template <typename T, typename TI, TI T_MAX_EPISODE_LENGTH = 200, bool T_PRIVILEGED_OBSERVATION = false>
    struct DefaultParameters {
        constexpr static T MAX_ACCELERATION = 50;
        constexpr static T MAX_VELOCITY = 5;
        constexpr static T FLAG_DISTANCE_THRESHOLD = 1;
        constexpr static T BOARD_SIZE = 10;
        constexpr static TI MAX_EPISODE_LENGTH = T_MAX_EPISODE_LENGTH;
        constexpr static T EPISODE_TIME = 4;
        constexpr static T DT = EPISODE_TIME / MAX_EPISODE_LENGTH;
        constexpr static T REWARD_SCALE = 1000;
        constexpr static bool SAMPLE_INITIAL_STATE_WITH_FLAG_1_VISITED = true;
        constexpr static bool PRIVILEGED_OBSERVATION = T_PRIVILEGED_OBSERVATION;
        T flag_positions[2][2];
    };
    template <typename T_T, typename T_TI, typename T_PARAMETERS = DefaultParameters<T_T, T_TI>>
    struct Specification{
        using T = T_T;
        using TI = T_TI;
        using PARAMETERS = T_PARAMETERS;
    };

    template <typename TI>
    struct Observation{
        static constexpr TI DIM = 4 + 3 + 4;
    };

    template <typename TI>
    struct ObservationPrivileged{
        static constexpr TI DIM = 4 + 3 + 4;
    };

    template <typename T, typename TI>
    struct State{
        enum class StateMachine{
            INITIAL = 0,
            FLAG_1_VISITED = 1,
            FLAG_2_VISITED = 2
        };
        static constexpr TI DIM = 5;
        T position[2];
        T velocity[2];
        StateMachine state_machine;
        TI step;
    };

}
RL_TOOLS_NAMESPACE_WRAPPER_END

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::environments{
    template <typename T_SPEC>
    struct Flag: Environment<typename T_SPEC::T, typename T_SPEC::TI>{
        using SPEC = T_SPEC;
        using T = typename SPEC::T;
        using TI = typename SPEC::TI;
        using State = flag::State<T, TI>;
        using Parameters = typename SPEC::PARAMETERS;
        using Observation = rl_tools::utils::typing::conditional_t<Parameters::PRIVILEGED_OBSERVATION, flag::ObservationPrivileged<TI>, flag::Observation<TI>>;
        using ObservationPrivileged = Observation; //flag::ObservationPrivileged<TI>;
        static constexpr TI N_AGENTS = 1; // single agent
        static constexpr TI ACTION_DIM = 2;
        static constexpr TI EPISODE_STEP_LIMIT = Parameters::MAX_EPISODE_LENGTH;
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END







#endif
