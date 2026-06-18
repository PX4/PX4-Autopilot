#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_LAYERS_TD3_SAMPLING_PERSIST_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_LAYERS_TD3_SAMPLING_PERSIST_H

#include "layer.h"
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools {
    template<typename DEVICE, typename SPEC, typename GROUP>
    void save(DEVICE& device, nn::layers::td3_sampling::LayerForward<SPEC>& layer, GROUP& group) { }
    template<typename DEVICE, typename SPEC, typename GROUP>
    bool load(DEVICE& device, nn::layers::td3_sampling::LayerForward<SPEC>& layer, GROUP& group) {
        return true;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
