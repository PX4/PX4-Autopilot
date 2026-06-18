#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_LAYERS_STANDARDIZE_PERSIST_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_LAYERS_STANDARDIZE_PERSIST_H

#include "layer.h"
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools {
    template<typename DEVICE, typename SPEC, typename GROUP>
    void save(DEVICE& device, nn::layers::standardize::LayerForward<SPEC>& layer, GROUP& group) {
        // todo: forward implementation to Parameter struct
        auto mean_group = create_group(device, group, "mean");
        save(device, layer.mean, mean_group);
        auto precision_group = create_group(device, group, "precision");
        save(device, layer.precision, precision_group);
        set_attribute(device, group, "type", "standardize");
        write_attributes(device, group);
    }
    template<typename DEVICE, typename SPEC, typename GROUP>
    void save(DEVICE& device, nn::layers::standardize::LayerBackward<SPEC>& layer, GROUP& group) {
        save(device, (nn::layers::standardize::LayerForward<SPEC>&)layer, group);
    }
    template<typename DEVICE, typename SPEC, typename GROUP>
    void save(DEVICE& device, nn::layers::standardize::LayerGradient<SPEC>& layer, GROUP& group) {
        save(device, (nn::layers::standardize::LayerBackward<SPEC>&)layer, group);
        save(device, layer.output, group, "output");
    }
    template<typename DEVICE, typename SPEC, typename GROUP>
    bool load(DEVICE& device, nn::layers::standardize::LayerForward<SPEC>& layer, GROUP& group) {
        auto mean_group = get_group(device, group, "mean");
        bool success = load(device, layer.mean, mean_group);
        auto precision_group = get_group(device, group, "precision");
        success &= load(device, layer.precision, precision_group);
        return success;
    }
    template<typename DEVICE, typename SPEC, typename GROUP>
    bool load(DEVICE& device, nn::layers::standardize::LayerBackward<SPEC>& layer, GROUP& group) {
        return load(device, (nn::layers::standardize::LayerForward<SPEC>&)layer, group);
    }
    template<typename DEVICE, typename SPEC, typename GROUP>
    bool load(DEVICE& device, nn::layers::standardize::LayerGradient<SPEC>& layer, GROUP& group) {
        bool success = load(device, (nn::layers::standardize::LayerBackward<SPEC>&)layer, group);
        if(group_exists(device, group, "output")){
            success &=  load(device, layer.output, group, "output");
        }
        return success;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
