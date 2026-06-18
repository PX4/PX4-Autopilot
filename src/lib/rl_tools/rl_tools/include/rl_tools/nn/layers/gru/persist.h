#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_LAYERS_GRU_PERSIST_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_LAYERS_GRU_PERSIST_H

#include "layer.h"
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools {
    template<typename DEVICE, typename SPEC, typename GROUP>
    void save(DEVICE& device, nn::layers::gru::LayerForward<SPEC>& layer, GROUP& group) {
        // todo: forward implementation to Parameter struct
        auto weights_input_group = create_group(device, group, "weights_input");
        save(device, layer.weights_input, weights_input_group);
        auto biases_input_group = create_group(device, group, "biases_input");
        save(device, layer.biases_input, biases_input_group);
        auto weights_hidden_group = create_group(device, group, "weights_hidden");
        save(device, layer.weights_hidden, weights_hidden_group);
        auto biases_hidden_group = create_group(device, group, "biases_hidden");
        save(device, layer.biases_hidden, biases_hidden_group);
        auto initial_hidden_state_group = create_group(device, group, "initial_hidden_state");
        save(device, layer.initial_hidden_state, initial_hidden_state_group);
        set_attribute(device, group, "type", "gru");
        write_attributes(device, group);
    }
    template<typename DEVICE, typename SPEC, typename GROUP>
    void save(DEVICE& device, nn::layers::gru::LayerBackward<SPEC>& layer, GROUP& group) {
        save(device, (nn::layers::gru::LayerForward<SPEC>&)layer, group);
        save(device, layer.post_activation, group, "post_activation");
        save(device, layer.n_pre_pre_activation, group, "n_pre_pre_activation");
        save(device, layer.output, group, "output");
    }
    template<typename DEVICE, typename SPEC, typename GROUP>
    void save(DEVICE& device, nn::layers::gru::LayerGradient<SPEC>& layer, GROUP& group) {
        save(device, (nn::layers::gru::LayerBackward<SPEC>&)layer, group);
    }
    template<typename DEVICE, typename SPEC, typename GROUP>
    bool load(DEVICE& device, nn::layers::gru::LayerForward<SPEC>& layer, GROUP& group) {
        auto weights_input_group = get_group(device, group, "weights_input");
        bool success = load(device, layer.weights_input, weights_input_group);
        auto biases_input_group = get_group(device, group, "biases_input");
        success &= load(device, layer.biases_input, biases_input_group);
        auto weights_hidden_group = get_group(device, group, "weights_hidden");
        success &= load(device, layer.weights_hidden, weights_hidden_group);
        auto biases_hidden_group = get_group(device, group, "biases_hidden");
        success &= load(device, layer.biases_hidden, biases_hidden_group);
        auto initial_hidden_state_group = get_group(device, group, "initial_hidden_state");
        success &= load(device, layer.initial_hidden_state, initial_hidden_state_group);
        return success;
    }
    template<typename DEVICE, typename SPEC, typename GROUP>
    bool load(DEVICE& device, nn::layers::gru::LayerBackward<SPEC>& layer, GROUP& group) {
        bool success = load(device, (nn::layers::gru::LayerForward<SPEC>&)layer, group);
        success &= load(device, layer.post_activation, group, "post_activation");
        success &= load(device, layer.n_pre_pre_activation, group, "n_pre_pre_activation");
        success &= load(device, layer.output, group, "output");
        return success;
    }
    template<typename DEVICE, typename SPEC, typename GROUP>
    bool load(DEVICE& device, nn::layers::gru::LayerGradient<SPEC>& layer, GROUP& group) {
        return load(device, (nn::layers::gru::LayerBackward<SPEC>&)layer, group);
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
