#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_LAYERS_EMBEDDING_PERSIST_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_LAYERS_EMBEDDING_PERSIST_H

#include "layer.h"
#include <iostream>
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools {
    template<typename DEVICE, typename SPEC, typename GROUP>
    void save(DEVICE& device, nn::layers::embedding::LayerForward<SPEC>& layer, GROUP& group) {
        // todo: forward implementation to Parameter struct
        auto weights_group = create_group(device, group, "weights");
        save(device, layer.weights, weights_group);
        set_attribute(device, group, "type", "embedding");
        write_attributes(device, group);
    }
    template<typename DEVICE, typename SPEC, typename GROUP>
    void save(DEVICE& device, nn::layers::embedding::LayerBackward<SPEC>& layer, GROUP& group) {
        save(device, (nn::layers::embedding::LayerForward<SPEC>&)layer, group);
    }
    template<typename DEVICE, typename SPEC, typename GROUP>
    void save(DEVICE& device, nn::layers::embedding::LayerGradient<SPEC>& layer, GROUP& group) {
        save(device, (nn::layers::embedding::LayerBackward<SPEC>&)layer, group);
        save(device, layer.output, group, "output");
    }
    template<typename DEVICE, typename SPEC, typename GROUP>
    bool load(DEVICE& device, nn::layers::embedding::LayerForward<SPEC>& layer, GROUP& group) {
        auto weights_group = get_group(device, group, "weights");
        return load(device, layer.weights, weights_group);
    }
    template<typename DEVICE, typename SPEC, typename GROUP>
    bool load(DEVICE& device, nn::layers::embedding::LayerBackward<SPEC>& layer, GROUP& group) {
        return load(device, (nn::layers::embedding::LayerForward<SPEC>&)layer, group);
    }
    template<typename DEVICE, typename SPEC, typename GROUP>
    bool load(DEVICE& device, nn::layers::embedding::LayerGradient<SPEC>& layer, GROUP& group) {
        bool success = load(device, (nn::layers::embedding::LayerBackward<SPEC>&)layer, group);
        if(group_exists(device, group, "output")){
            success &= load(device, layer.output, group, "output");
        }
        return success;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
