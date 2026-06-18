#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ENVIRONMENTS_MEMORY_PENDULUM_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENTS_MEMORY_PENDULUM_H

#include "../../../math/operations_generic.h"
#include "../environments.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::environments::memory {
    enum class Mode{
        COUNT_INPUT = 0,
        COUNT_STEPS_SINCE_LAST_INPUT = 1
    };
    template <typename T, typename TI>
    struct DefaultParameters {
        constexpr static TI HORIZON = 10;
        constexpr static T INPUT_PROBABILITY = (T)5/(T)HORIZON;
        constexpr static Mode MODE = Mode::COUNT_INPUT;
        static constexpr TI EPISODE_STEP_LIMIT = 20;
    };
    template <typename T_T, typename T_TI, typename T_PARAMETERS = DefaultParameters<T_T, T_TI>>
    struct Specification{
        using T = T_T;
        using TI = T_TI;
        using PARAMETERS = T_PARAMETERS;
    };

    template <typename TI>
    struct Observation{
        static constexpr TI DIM = 1;
    };

    template <typename T, typename TI, TI HORIZON>
    struct State{
        static constexpr TI DIM = HORIZON;
        T history[DIM];
    };

}
RL_TOOLS_NAMESPACE_WRAPPER_END

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::environments{
    template <typename T_SPEC>
    struct Memory: Environment<typename T_SPEC::T, typename T_SPEC::TI>{
        using SPEC = T_SPEC;
        using T = typename SPEC::T;
        using TI = typename SPEC::TI;
        using State = memory::State<T, TI, SPEC::PARAMETERS::HORIZON>;
        using Parameters = typename SPEC::PARAMETERS;
        using Observation = memory::Observation<TI>;
        using ObservationPrivileged = Observation;
        static constexpr TI N_AGENTS = 1; // single agent
        static constexpr TI ACTION_DIM = 1;
        static constexpr TI EPISODE_STEP_LIMIT = SPEC::PARAMETERS::EPISODE_STEP_LIMIT;
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END







#endif
