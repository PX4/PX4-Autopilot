#include "../../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_LOOP_STEPS_EVALUATION_CONFIG_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_LOOP_STEPS_EVALUATION_CONFIG_H

#include "../../../../rl/environments/environments.h"

#include "state.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::loop::steps::evaluation{
    struct ParametersTag{};
    template <typename TYPE_POLICY, typename TI, typename NEXT, TI T_NUM_EVALUATION_EPISODES = 10, TI T_EVALUATION_INTERVAL = 1000>
    struct Parameters{
        using TAG = ParametersTag;
        static constexpr bool DETERMINISTIC_EVALUATION = true;
        static constexpr TI EVALUATION_INTERVAL = T_EVALUATION_INTERVAL;
        static constexpr TI NUM_EVALUATION_EPISODES = T_NUM_EVALUATION_EPISODES;
        static constexpr TI N_EVALUATIONS = NEXT::CORE_PARAMETERS::STEP_LIMIT / EVALUATION_INTERVAL + 1;
        static constexpr TI EPISODE_STEP_LIMIT = NEXT::CORE_PARAMETERS::EPISODE_STEP_LIMIT;
        using EVALUATION_MODE = mode::Evaluation<>;
        static constexpr bool DETERMINISTIC_INITIAL_STATE = false;
        static constexpr bool SAMPLE_ENVIRONMENT_PARAMETERS = true;
    };
    struct ConfigTag{};
    template<typename T_NEXT, typename T_PARAMETERS = Parameters<typename T_NEXT::TYPE_POLICY, typename T_NEXT::TI, T_NEXT>, typename T_UI = environments::DummyUI>
    struct Config: T_NEXT {
        using TAG = ConfigTag;
        using TYPE_POLICY = typename T_NEXT::TYPE_POLICY;
        using NEXT = T_NEXT;
        using EVALUATION_PARAMETERS = T_PARAMETERS;
        using UI = T_UI;
        using TI = typename NEXT::TI;
        using EVALUATION_SPEC = rl::utils::evaluation::Specification<TYPE_POLICY, TI, typename NEXT::ENVIRONMENT_EVALUATION, EVALUATION_PARAMETERS::NUM_EVALUATION_EPISODES, EVALUATION_PARAMETERS::EPISODE_STEP_LIMIT>;
        using EVALUATION_RESULT_SPEC = rl::utils::evaluation::Specification<TYPE_POLICY, TI, typename NEXT::ENVIRONMENT_EVALUATION, EVALUATION_PARAMETERS::NUM_EVALUATION_EPISODES, EVALUATION_PARAMETERS::EPISODE_STEP_LIMIT, EVALUATION_PARAMETERS::DETERMINISTIC_INITIAL_STATE>;
        static_assert(EVALUATION_PARAMETERS::N_EVALUATIONS > 0);
        static_assert(EVALUATION_PARAMETERS::N_EVALUATIONS < 1000000);
        template <typename CONFIG>
        using State = State<CONFIG>;
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif




