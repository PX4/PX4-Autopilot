#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_MODELS_SEQUENTIAL_PERSIST_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_MODELS_SEQUENTIAL_PERSIST_H
#include "../../nn/parameters/persist.h"
#include "../../nn/persist.h"
#include "../../utils/string/operations_generic.h"
#include "model.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template<typename DEVICE, typename SPEC, typename GROUP, typename DEVICE::index_t LAYER_I = 0>
    void save(DEVICE& device, nn_models::sequential::ModuleForward<SPEC>& model, GROUP& group) {
        using TI = typename DEVICE::index_t;
        if constexpr(LAYER_I == 0){
            set_attribute(device, group, "type", "sequential");
            write_attributes(device, group);
            group = create_group(device, group, "layers");
        }
        static constexpr TI BUFFER_SIZE = 10;
        char layer_index_str[BUFFER_SIZE];
        utils::string::int_to_string<long int, TI>(layer_index_str, BUFFER_SIZE, LAYER_I);
        auto layer_group = create_group(device, group, layer_index_str);
        save(device, model.content, layer_group);
        if constexpr (!utils::typing::is_same_v<typename SPEC::NEXT_MODULE, nn_models::sequential::OutputModule>){
            save<DEVICE, typename decltype(model.next_module)::SPEC, GROUP, LAYER_I+1>(device, model.next_module, group);
        }
    }
    template<typename DEVICE, typename SPEC, typename GROUP>
    bool load(DEVICE& device, nn_models::sequential::ModuleForward<SPEC>& model, GROUP& group, typename DEVICE::index_t layer_i = 0) {
        using TI = typename DEVICE::index_t;
        if(layer_i == 0){
            group = get_group(device, group, "layers");
        }
        static constexpr TI BUFFER_SIZE = 10;
        char layer_index_str[BUFFER_SIZE];
        utils::string::int_to_string<long int, TI>(layer_index_str, BUFFER_SIZE, layer_i);
        auto layer_group = get_group(device, group, layer_index_str);
        bool success = load(device, model.content, layer_group);
        if constexpr (!utils::typing::is_same_v<typename SPEC::NEXT_MODULE, nn_models::sequential::OutputModule>){
            success &= load(device, model.next_module, group, layer_i + 1);
        }
        return success;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
