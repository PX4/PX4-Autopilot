#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ENVIRONMENTS_MEMORY_OPERATIONS_CPU_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENTS_MEMORY_OPERATIONS_CPU_H
#include "operations_generic.h"
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template<typename DEVICE, typename SPEC>
    std::string json(DEVICE& device, const rl::environments::Memory<SPEC>& env){
        std::string json = "{";
        json += "}";
        return json;
    }
    template<typename DEVICE, typename SPEC>
    std::string json(DEVICE& device, const rl::environments::Memory<SPEC>& env, typename rl::environments::Memory<SPEC>::Parameters& parameters){
        std::string json = "{";
        using TI = typename DEVICE::index_t;
        json += "\"HORIZON\": " + std::to_string(SPEC::PARAMETERS::HORIZON) + ", ";
        json += "\"INPUT_PROBABILITY\": " + std::to_string(SPEC::PARAMETERS::INPUT_PROBABILITY) + ", ";
        json += "\"MODE\": " + std::to_string(static_cast<TI>(SPEC::PARAMETERS::MODE));
        json += "}";
        return json;
    }
    template<typename DEVICE, typename SPEC>
    std::string json(DEVICE& device, const rl::environments::Memory<SPEC>& env, typename rl::environments::Memory<SPEC>::Parameters& parameters, typename rl::environments::Memory<SPEC>::State& state){
        std::string json = "{";
        using TI = typename DEVICE::index_t;
        json += std::string("\"history\": ") + std::string("[");
        for(TI step_i = 0; step_i < SPEC::PARAMETERS::HORIZON; step_i++){
            json += std::to_string(state.history[step_i]);
            if(step_i < SPEC::PARAMETERS::HORIZON - 1){
                json += ", ";
            }
        }
        json += "]}";
        return json;
    }
    template <typename DEVICE, typename SPEC>
    std::string get_description(DEVICE& device, rl::environments::Memory<SPEC>& env){
        std::string description;
        description += "Memory Environment.";
        return description;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
