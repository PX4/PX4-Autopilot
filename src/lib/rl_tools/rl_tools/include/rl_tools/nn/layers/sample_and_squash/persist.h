#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_LAYERS_SAMPLE_AND_SQUASH_PERSIST_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_LAYERS_SAMPLE_AND_SQUASH_PERSIST_H

#include "layer.h"
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools {
    template<typename DEVICE, typename SPEC, typename GROUP>
    void save(DEVICE& device, nn::layers::sample_and_squash::LayerForward<SPEC>& layer, GROUP& group){
        set_attribute(device, group, "type", "sample_and_squash");
        write_attributes(device, group);
    }
    template<typename DEVICE, typename SPEC, typename GROUP>
    void save(DEVICE& device, nn::layers::sample_and_squash::LayerBackward<SPEC>& layer, GROUP& group) {
        save(device, (nn::layers::sample_and_squash::LayerForward<SPEC>&)layer, group);
        save(device, layer.pre_squashing, group, "pre_squashing");
        save(device, layer.noise, group, "noise");
    }
    template<typename DEVICE, typename SPEC, typename GROUP>
    void save(DEVICE& device, nn::layers::sample_and_squash::LayerGradient<SPEC>& layer, GROUP& group) {
        save(device, (nn::layers::sample_and_squash::LayerBackward<SPEC>&)layer, group);
        save(device, layer.log_probabilities, group, "log_probabilities");
        auto log_alpha_group = create_group(device, group, "log_alpha");
        save(device, layer.log_alpha, log_alpha_group);
        save(device, layer.output, group, "output");
    }
    template<typename DEVICE, typename SPEC, typename GROUP>
    bool load(DEVICE& device, nn::layers::sample_and_squash::LayerForward<SPEC>& layer, GROUP& group) {
        return true;
    }
    template<typename DEVICE, typename SPEC, typename GROUP>
    bool load(DEVICE& device, nn::layers::sample_and_squash::LayerBackward<SPEC>& layer, GROUP& group) {
        bool success = load(device, (nn::layers::sample_and_squash::LayerForward<SPEC>&)layer, group);
        success &= load(device, layer.pre_squashing, group, "pre_squashing");
        success &= load(device, layer.noise, group, "noise");
        return success;
    }
    template<typename DEVICE, typename SPEC, typename GROUP>
    bool load(DEVICE& device, nn::layers::sample_and_squash::LayerGradient<SPEC>& layer, GROUP& group) {
        bool success = load(device, (nn::layers::sample_and_squash::LayerBackward<SPEC>&)layer, group);
        success &= load(device, layer.log_probabilities, group, "log_probabilities");
        auto log_alpha_group = get_group(device, group, "log_alpha");
        success &= load(device, layer.log_alpha, log_alpha_group);
        success &= load(device, layer.output, group, "output");
        return success;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
