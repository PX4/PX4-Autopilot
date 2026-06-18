#ifndef RL_TOOLS_TESTS_SRC_UTILS_NN_COMPARISON_MLP_H
#define RL_TOOLS_TESTS_SRC_UTILS_NN_COMPARISON_MLP_H

#include "nn_comparison.h"

template <typename DEVICE, typename SPEC>
typename SPEC::TYPE_POLICY::DEFAULT abs_diff(DEVICE& device, const rlt::nn_models::mlp::NeuralNetworkForward<SPEC>& n1, const rlt::nn_models::mlp::NeuralNetworkForward<SPEC>& n2) {
    using NetworkType = typename std::remove_reference<decltype(n1)>::type;
    using T = typename SPEC::TYPE_POLICY::DEFAULT;
    T acc = 0;
    acc += rlt::abs_diff(device, n1.input_layer, n2.input_layer);
    for(typename DEVICE::index_t layer_i = 0; layer_i < NetworkType::NUM_HIDDEN_LAYERS; layer_i++) {
        acc += rlt::abs_diff(device, n1.hidden_layers[layer_i], n2.hidden_layers[layer_i]);
    }
    acc += rlt::abs_diff(device, n1.output_layer, n2.output_layer);
    return acc;
}
template <typename DEVICE, typename SPEC>
typename SPEC::TYPE_POLICY::DEFAULT abs_diff_grad(DEVICE& device, const rlt::nn_models::mlp::NeuralNetworkGradient<SPEC>& n1, const rlt::nn_models::mlp::NeuralNetworkGradient<SPEC>& n2) {
    using NetworkType = typename std::remove_reference<decltype(n1)>::type;
    using T = typename SPEC::TYPE_POLICY::DEFAULT;
//    constexpr typename DEVICE::index_t BATCH_SIZE = 1;
    using GradNetworkType = rlt::nn_models::mlp::NeuralNetwork<SPEC, rlt::nn::capability::Gradient<rlt::nn::parameters::Gradient>, typename SPEC::INPUT_SHAPE>;
    GradNetworkType n1g;
    rlt::malloc(device, n1g);
    rlt::copy(device, device, n1, n1g);
    rlt::reset_forward_state(device, n1g);
    GradNetworkType n2g;
    rlt::malloc(device, n2g);
    rlt::copy(device, device, n2, n2g);
    rlt::reset_forward_state(device, n2g);
    T acc = 0;
    acc += rlt::abs_diff(device, n1g.input_layer, n2g.input_layer);
    for(typename DEVICE::index_t layer_i = 0; layer_i < NetworkType::NUM_HIDDEN_LAYERS; layer_i++) {
        acc += rlt::abs_diff(device, n1g.hidden_layers[layer_i], n2g.hidden_layers[layer_i]);
    }
    acc += rlt::abs_diff(device, n1g.output_layer, n2g.output_layer);
    rlt::free(device, n1g);
    rlt::free(device, n2g);
    return acc;
}

template <typename DEVICE, typename SPEC>
typename std::enable_if<std::is_same<typename SPEC::PARAMETER_TYPE, rlt::nn::parameters::Adam>::value, typename SPEC::TYPE_POLICY::DEFAULT>::type
abs_diff_adam(DEVICE& device, const rlt::nn_models::mlp::NeuralNetworkGradient<SPEC>& n1, const rlt::nn_models::mlp::NeuralNetworkGradient<SPEC>& n2) {
    using NetworkType = typename std::remove_reference<decltype(n1)>::type;
    using T = typename SPEC::TYPE_POLICY::DEFAULT;
    T acc = 0;
    acc += rlt::abs_diff(device, n1.input_layer, n2.input_layer);
    for(typename DEVICE::index_t layer_i = 0; layer_i < NetworkType::NUM_HIDDEN_LAYERS; layer_i++) {
        acc += rlt::abs_diff(device, n1.hidden_layers[layer_i], n2.hidden_layers[layer_i]);
    }
    acc += rlt::abs_diff(device, n1.output_layer, n2.output_layer);
    return acc;
}

#endif
