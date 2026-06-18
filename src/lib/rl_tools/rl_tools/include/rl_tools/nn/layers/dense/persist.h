#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_LAYERS_DENSE_PERSIST_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_LAYERS_DENSE_PERSIST_H
#include "../../../version.h"
#include "layer.h"
#include "persist_common.h"
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools {
    template<typename DEVICE, typename SPEC, typename GROUP>
    void save(DEVICE& device, nn::layers::dense::LayerForward<SPEC>& layer, GROUP& group) {
        // todo: forward implementation to Parameter struct
        auto weights_group = create_group(device, group, "weights");
        auto biases_group = create_group(device, group, "biases");
        save(device, layer.weights, weights_group);
        save(device, layer.biases, biases_group);
        set_attribute(device, group, "activation_function", nn::layers::dense::persist::get_activation_function_string_short<SPEC::CONFIG::ACTIVATION_FUNCTION>());
        set_attribute(device, group, "type", "dense");
        write_attributes(device, group);
    }
    template<typename DEVICE, typename SPEC, typename GROUP>
    void save(DEVICE& device, nn::layers::dense::LayerBackward<SPEC>& layer, GROUP& group) {
        save(device, (nn::layers::dense::LayerForward<SPEC>&)layer, group);
        save(device, layer.pre_activations, group, "pre_activations");
    }
    template<typename DEVICE, typename SPEC, typename GROUP>
    void save(DEVICE& device, nn::layers::dense::LayerGradient<SPEC>& layer, GROUP& group) {
        save(device, (nn::layers::dense::LayerBackward<SPEC>&)layer, group);
        save(device, layer.output, group, "output");
    }
    template<typename DEVICE, typename SPEC, typename GROUP>
    bool load(DEVICE& device, nn::layers::dense::LayerForward<SPEC>& layer, GROUP& group) {
        auto weights_group = get_group(device, group, "weights");
        auto biases_group = get_group(device, group, "biases");
        bool success = load(device, layer.weights, weights_group);
        success &= load(device, layer.biases, biases_group);
        return success;
    }
    template<typename DEVICE, typename SPEC, typename GROUP>
    bool load(DEVICE& device, nn::layers::dense::LayerBackward<SPEC>& layer, GROUP& group) {
        bool success = load(device, (nn::layers::dense::LayerForward<SPEC>&)layer, group);
        if(group_exists(device, group, "pre_activations")){
            success &= load(device, layer.pre_activations, group, "pre_activations");
        }
        return success;
    }
    template<typename DEVICE, typename SPEC, typename GROUP>
    bool load(DEVICE& device, nn::layers::dense::LayerGradient<SPEC>& layer, GROUP& group) {
        bool success = load(device, (nn::layers::dense::LayerBackward<SPEC>&)layer, group);
        if(group_exists(device, group, "output")){
            success &= load(device, layer.output, group, "output");
        }
        return success;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
