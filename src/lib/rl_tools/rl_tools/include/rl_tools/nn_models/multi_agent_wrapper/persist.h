#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_MODELS_MULTI_AGENT_WRAPPER_PERSIST_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_MODELS_MULTI_AGENT_WRAPPER_PERSIST_H
#include "../../nn/parameters/persist.h"
#include "../../nn/persist.h"
#include "model.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template<typename DEVICE, typename SPEC, typename GROUP>
    void save(DEVICE& device, nn_models::multi_agent_wrapper::ModuleForward<SPEC>& model, GROUP& group) {
        auto wrapper_group = create_group(device, group, "multi_agent_wrapper");
        auto content_group = create_group(device, wrapper_group, "content");
        save(device, model.content, content_group);
    }
    template<typename DEVICE, typename SPEC, typename GROUP>
    bool load(DEVICE& device, nn_models::multi_agent_wrapper::ModuleForward<SPEC>& model, GROUP& group, typename DEVICE::index_t layer_i = 0) {
        auto wrapper_group = get_group(device, group, "multi_agent_wrapper");
        auto content_group = get_group(device, wrapper_group, "content");
        return load(device, model.content, content_group);
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
