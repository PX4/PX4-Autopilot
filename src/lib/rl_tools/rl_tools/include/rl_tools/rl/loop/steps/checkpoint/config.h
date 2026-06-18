#include "../../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_LOOP_STEPS_CHECKPOINT_CONFIG_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_LOOP_STEPS_CHECKPOINT_CONFIG_H

#include "state.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::loop::steps::checkpoint{
    template <typename TYPE_POLICY, typename TI>
    struct Parameters{
        static constexpr TI CHECKPOINT_INTERVAL = 1000;
        static constexpr TI TEST_INPUT_BATCH_SIZE = 13;
    };
    template<typename T_NEXT, typename T_PARAMETERS = Parameters<typename T_NEXT::TYPE_POLICY, typename T_NEXT::TI>>
    struct Config: T_NEXT {
        using NEXT = T_NEXT;
        using CHECKPOINT_PARAMETERS = T_PARAMETERS;
        template <typename CONFIG>
        using State = State<CONFIG>;
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif




