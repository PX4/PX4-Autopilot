#include "../../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_LOOP_STEPS_EXTRACK_OPERATIONS_CPU_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_LOOP_STEPS_EXTRACK_OPERATIONS_CPU_H

#include "config.h"
#include "../../../../utils/extrack/operations_cpu.h"

#include <iostream>
#include <chrono>
#include <iomanip>
#include <ctime>
#include <sstream>

#include <cstdlib>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template <typename DEVICE, typename T_CONFIG>
    void init(DEVICE& device, rl::loop::steps::extrack::State<T_CONFIG>& ts, typename T_CONFIG::TI seed = 0){
        using STATE = rl::loop::steps::extrack::State<T_CONFIG>;
        init(device, static_cast<typename STATE::NEXT&>(ts), seed);
        init(device, ts.extrack_config, ts.extrack_paths, seed);
    }


    template <typename DEVICE, typename T_CONFIG>
    void free(DEVICE& device, rl::loop::steps::extrack::State<T_CONFIG>& ts){
        using STATE = rl::loop::steps::extrack::State<T_CONFIG>;
        free(device, static_cast<typename STATE::NEXT&>(ts));
    }

    template <typename DEVICE, typename CONFIG>
    bool step(DEVICE& device, rl::loop::steps::extrack::State<CONFIG>& ts){
        using TI = typename CONFIG::TI;
        using STATE = rl::loop::steps::extrack::State<CONFIG>;
        bool finished = step(device, static_cast<typename STATE::NEXT&>(ts));
        return finished;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END


#endif