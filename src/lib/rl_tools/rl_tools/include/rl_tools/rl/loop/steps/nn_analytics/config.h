#include "../../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_LOOP_STEPS_NN_ANALYTICS_CONFIG_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_LOOP_STEPS_NN_ANALYTICS_CONFIG_H


#include "../../../../rl/environments/environments.h"

#include "state.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::loop::steps::nn_analytics{
    struct ParametersTag{};
    template <typename TYPE_POLICY, typename TI, typename NEXT>
    struct Parameters{
        using TAG = ParametersTag;
        static constexpr TI INTERVAL = 1000;
        static constexpr TI WARMUP_STEPS = 0;
    };
    struct ConfigTag{};
    template<typename T_NEXT, typename T_PARAMETERS = Parameters<typename T_NEXT::TYPE_POLICY, typename T_NEXT::TI, T_NEXT>>
    struct Config: T_NEXT {
        using TAG = ConfigTag;
        using NEXT = T_NEXT;
        using NN_ANALYTICS_PARAMETERS = T_PARAMETERS;
        using TYPE_POLICY = typename NEXT::TYPE_POLICY;
        using TI = typename NEXT::TI;
        template <typename CONFIG>
        using State = State<CONFIG>;
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif




