#include "../../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_LOOP_STEPS_TIMING_STATE_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_LOOP_STEPS_TIMING_STATE_H


#include <chrono>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::loop::steps::timing{
    template<typename T_CONFIG, typename T_NEXT = typename T_CONFIG::NEXT::template State<typename T_CONFIG::NEXT>>
    struct State: T_NEXT {
        using CONFIG = T_CONFIG;
        using NEXT = T_NEXT;
        using TI = typename CONFIG::TI;
        std::chrono::steady_clock::time_point start_time, last_steps_per_second_time;
        TI last_steps_per_second_step;
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif




