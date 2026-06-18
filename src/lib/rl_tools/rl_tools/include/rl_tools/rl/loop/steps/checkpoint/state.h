#include "../../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_LOOP_STEPS_CHECKPOINT_STATE_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_LOOP_STEPS_CHECKPOINT_STATE_H

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::loop::steps::checkpoint{
    template<typename T_CONFIG, typename T_NEXT = typename T_CONFIG::NEXT::template State<typename T_CONFIG::NEXT>>
    struct State: T_NEXT {
        using CONFIG = T_CONFIG;
        using NEXT = T_NEXT;
        typename CONFIG::RNG rng_checkpoint;
        bool checkpoint_this_step = false;
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif




