#include "../../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_LOOP_STEPS_EXTRACK_STATE_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_LOOP_STEPS_EXTRACK_STATE_H


#include <chrono>
#include <filesystem>
#include "../../../../utils/extrack/operations_cpu.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::loop::steps::extrack{
    template<typename T_CONFIG, typename T_NEXT = typename T_CONFIG::NEXT::template State<typename T_CONFIG::NEXT>>
    struct State: T_NEXT {
        using CONFIG = T_CONFIG;
        using NEXT = T_NEXT;
        rl_tools::utils::extrack::Config<typename T_CONFIG::TI> extrack_config;
        rl_tools::utils::extrack::Paths extrack_paths;
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif




