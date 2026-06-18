#include "../../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_LOOP_STEPS_TIMING_OPERATIONS_GENERIC_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_LOOP_STEPS_TIMING_OPERATIONS_GENERIC_H

#include "config.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template <typename DEVICE, typename T_CONFIG>
    void init(DEVICE& device, rl::loop::steps::timing::State<T_CONFIG>& ts, typename T_CONFIG::TI seed = 0){
        using STATE = rl::loop::steps::timing::State<T_CONFIG>;
        init(device, static_cast<typename STATE::NEXT&>(ts), seed);
        ts.start_time = std::chrono::steady_clock::now();
        ts.last_steps_per_second_time = ts.start_time;
        ts.last_steps_per_second_step = ts.step;
    }

    template <typename DEVICE, typename T_CONFIG>
    void free(DEVICE& device, rl::loop::steps::timing::State<T_CONFIG>& ts){
        using STATE = rl::loop::steps::timing::State<T_CONFIG>;
        free(device, static_cast<typename STATE::NEXT&>(ts));
    }

    template <typename DEVICE, typename CONFIG>
    bool step(DEVICE& device, rl::loop::steps::timing::State<CONFIG>& ts){
        using T = typename CONFIG::TYPE_POLICY::DEFAULT;
        using TI = typename CONFIG::TI;
        using STATE = rl::loop::steps::timing::State<CONFIG>;
        bool finished = step(device, static_cast<typename STATE::NEXT&>(ts));
        if(ts.step % CONFIG::PARAMETERS::INTERVAL == 0){
            auto now = std::chrono::steady_clock::now();
            if(now - ts.last_steps_per_second_time > std::chrono::seconds(10)){
                TI steps = ts.step - ts.last_steps_per_second_step;
                T steps_per_second = (T)steps / std::chrono::duration_cast<std::chrono::microseconds>(now - ts.last_steps_per_second_time).count() * 1000000 * CONFIG::ENVIRONMENT_STEPS_PER_LOOP_STEP;
                log(device, device.logger, "Loop step: ", ts.step, ", env step: ", ts.step * CONFIG::ENVIRONMENT_STEPS_PER_LOOP_STEP, ", SPS: ", steps_per_second, " (elapsed: ", std::chrono::duration_cast<std::chrono::milliseconds>(now - ts.start_time).count()/1000.0, " s)");
                add_scalar(device, device.logger, "steps_per_second", steps_per_second);
                ts.last_steps_per_second_time = now;
                ts.last_steps_per_second_step = ts.step;
            }
        }
        if(finished){
            auto now = std::chrono::steady_clock::now();
            log(device, device.logger, "Time: ", std::chrono::duration_cast<std::chrono::milliseconds>(now - ts.start_time).count()/1000.0, "s");
        }
        return finished;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END


#endif
