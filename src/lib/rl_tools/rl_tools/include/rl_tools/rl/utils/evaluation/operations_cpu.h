#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_UTILS_EVALUATION_OPERATIONS_CPU_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_UTILS_EVALUATION_OPERATIONS_CPU_H
/*
 * This file relies on the environments methods hence it should be included after the operations of the environments that it will be used with
 */

#include "operations_generic.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template <typename DEVICE, typename SPEC>
    std::string json(DEVICE& device, rl::utils::evaluation::Result<SPEC>& result, typename DEVICE::index_t step = 0) {
        using TI = typename DEVICE::index_t;
        std::string data;
        data += "{";
        data += "\"step\": " + std::to_string(step) + ", ";
        data += "\"returns_mean\": " + std::to_string(result.returns_mean) + ", ";
        data += "\"returns_std\": " + std::to_string(result.returns_std) + ", ";
        data += "\"episode_length_mean\": " + std::to_string(result.episode_length_mean) + ", ";
        data += "\"episode_length_std\": " + std::to_string(result.episode_length_std) + ", ";
        data += "\"returns\": [";
        for(TI episode_i = 0; episode_i < SPEC::N_EPISODES; episode_i++){
            data += std::to_string(result.returns[episode_i]);
            if(episode_i < SPEC::N_EPISODES - 1){
                data += ", ";
            }
        }
        data += "], ";
        data += "\"episode_length\": [";
        for(TI episode_i = 0; episode_i < SPEC::N_EPISODES; episode_i++){
            data += std::to_string(result.episode_length[episode_i]);
            if(episode_i < SPEC::N_EPISODES - 1){
                data += ", ";
            }
        }
        data += "], ";
        data += "\"num_terminated\": " + std::to_string(result.num_terminated) + ", ";
        data += "\"share_terminated\": " + std::to_string(result.share_terminated);
        data += "}";
        return data;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END


#endif
