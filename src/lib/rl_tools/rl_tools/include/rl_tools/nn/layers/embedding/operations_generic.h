#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_LAYERS_EMBEDDING_OPERATIONS_GENERIC_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_LAYERS_EMBEDDING_OPERATIONS_GENERIC_H

#include "../../../containers/matrix/matrix.h"
#include "../../../nn/parameters/operations_generic.h"

#include "layer.h"
#ifndef RL_TOOLS_FUNCTION_PLACEMENT
#define RL_TOOLS_FUNCTION_PLACEMENT
#endif

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, nn::layers::embedding::LayerForward<SPEC>& layer) {
        malloc(device, layer.weights);
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, nn::layers::embedding::LayerForward<SPEC>& layer) {
        free(device, layer.weights);
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, nn::layers::embedding::LayerBackward<SPEC>& layer) {
        malloc(device, (nn::layers::embedding::LayerForward<SPEC>&) layer);
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, nn::layers::embedding::LayerBackward<SPEC>& layer) {
        free(device, (nn::layers::embedding::LayerForward<SPEC>&) layer);
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, nn::layers::embedding::LayerGradient<SPEC>& layer) {
        malloc(device, (nn::layers::embedding::LayerBackward<SPEC>&) layer);
        malloc(device, layer.output);
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, nn::layers::embedding::LayerGradient<SPEC>& layer) {
        free(device, (nn::layers::embedding::LayerBackward<SPEC>&) layer);
        free(device, layer.output);
    }
    template<typename DEVICE>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, nn::layers::embedding::State& state) { } // no-op
    template <typename SOURCE_DEVICE, typename TARGET_DEVICE>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, nn::layers::embedding::State& source, nn::layers::embedding::State& target){}
    template<typename DEVICE, typename SPEC, typename RNG, typename MODE>
    RL_TOOLS_FUNCTION_PLACEMENT void reset(DEVICE& device, const nn::layers::embedding::LayerForward<SPEC>& layer, nn::layers::embedding::State& state, RNG&, Mode<MODE> mode = Mode<mode::Default<>>{}) { } // no-op
    template<typename DEVICE>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, nn::layers::embedding::State& state) { } // no-op
    template<typename DEVICE>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, nn::layers::embedding::Buffer& buffer) { } // no-op
    template<typename DEVICE>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, nn::layers::embedding::Buffer& buffer) { } // no-op

    template<typename DEVICE, typename SPEC, typename INITIALIZER_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT void init_weights(DEVICE& device, nn::layers::embedding::LayerForward<SPEC>& layer, const nn::layers::embedding::StandardNormal<INITIALIZER_SPEC>& initializer, RNG& rng) {
        using T = typename decltype(layer.weights.parameters)::SPEC::T;
        using TI = typename SPEC::TI;
        for(TI class_i = 0; class_i < SPEC::NUM_CLASSES; class_i++){
            for(TI dim_i = 0; dim_i < SPEC::EMBEDDING_DIM; dim_i++){
                T value = random::normal_distribution::sample(device.random, (T)0, (T)1, rng);
                set(device, layer.weights.parameters, value, class_i, dim_i);
            }
        }
    }
    template<typename DEVICE, typename SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT void init_weights(DEVICE& device, nn::layers::embedding::LayerForward<SPEC>& layer, RNG& rng) {
        init_weights(device, layer, typename SPEC::INITIALIZER{}, rng);
    }

//#ifndef RL_TOOLS_NN_DISABLE_GENERIC_FORWARD_BACKWARD
    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void evaluate(DEVICE& device, const nn::layers::embedding::LayerForward<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, Tensor<OUTPUT_SPEC>& output, nn::layers::embedding::Buffer&, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
//        static_assert(nn::layers::embedding::check_input_output<LAYER_SPEC, INPUT_SPEC, OUTPUT_SPEC>);
        // Warning do not use the same buffer for input and output!
        using TI = typename DEVICE::index_t;
//        auto input_flat = reshape
        constexpr TI INPUT_ELEMENTS = get<0>(tensor::CumulativeProduct<typename INPUT_SPEC::SHAPE>{});
        auto input_view = reshape_row_major(device, input, tensor::Shape<TI, INPUT_ELEMENTS>{});
        auto output_view = reshape_row_major(device, output, tensor::Shape<TI, INPUT_ELEMENTS, LAYER_SPEC::EMBEDDING_DIM>{});
        for(TI batch_i=0; batch_i < get<0>(typename decltype(input_view)::SPEC::SHAPE{}); batch_i++){
            auto index = get(device, input_view, batch_i);
            auto embedding = view(device, layer.weights.parameters, index, tensor::ViewSpec<0>{});
            auto output_row = view(device, output_view, batch_i, tensor::ViewSpec<0>{});
            copy(device, device, embedding, output_row);
        }
    }
    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void evaluate_step(DEVICE& device, const nn::layers::embedding::LayerForward<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, nn::layers::embedding::State& state, Tensor<OUTPUT_SPEC>& output, nn::layers::embedding::Buffer& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        evaluate(device, layer, input, output, buffer, rng, mode);
    }

    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void forward(DEVICE& device, nn::layers::embedding::LayerBackward<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, Tensor<OUTPUT_SPEC>& output, nn::layers::embedding::Buffer& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        evaluate(device, static_cast<nn::layers::embedding::LayerForward<LAYER_SPEC>&>(layer), input, output, buffer, rng, mode);
    }
//#endif

    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void forward(DEVICE& device, nn::layers::embedding::LayerGradient<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, nn::layers::embedding::Buffer& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        forward(device, static_cast<nn::layers::embedding::LayerBackward<LAYER_SPEC>&>(layer), input, layer.output, buffer, rng, mode);
    }
    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void forward(DEVICE& device, nn::layers::embedding::LayerGradient<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, Tensor<OUTPUT_SPEC>& output, nn::layers::embedding::Buffer& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        forward(device, layer, input, buffer, rng, mode);
        copy(device, device, layer.output, output);
    }

//#ifndef RL_TOOLS_NN_DISABLE_GENERIC_FORWARD_BACKWARD
    // backward_input / backward_full are not supported because the inputs are discrete classes
    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename D_OUTPUT_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void backward(DEVICE& device, nn::layers::embedding::LayerGradient<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, Tensor<D_OUTPUT_SPEC>& d_output, nn::layers::embedding::Buffer&, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        using T = typename decltype(layer.weights.gradient)::SPEC::T;
        using TI = typename DEVICE::index_t;
        constexpr TI EMBEDDING_DIM = LAYER_SPEC::EMBEDDING_DIM;

        constexpr TI INPUT_ELEMENTS = get<0>(tensor::CumulativeProduct<typename INPUT_SPEC::SHAPE>{});
        auto input_view = reshape_row_major(device, input, tensor::Shape<TI, INPUT_ELEMENTS>{});
        auto d_output_view = reshape_row_major(device, d_output, tensor::Shape<TI, INPUT_ELEMENTS, EMBEDDING_DIM>{});

        for(TI element_i=0; element_i < INPUT_ELEMENTS; element_i++){
            for(TI output_i = 0; output_i < EMBEDDING_DIM; output_i++){
                typename INPUT_SPEC::T class_id = get(device, input_view, element_i);
                T d_output_value = get(device, d_output_view, element_i, output_i);
                T gradient_value = get(device, layer.weights.gradient, (TI)class_id, output_i);
                gradient_value += d_output_value;
                set(device, layer.weights.gradient, gradient_value, (TI)class_id, output_i);
            }
        }
    }
//#endif
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void zero_gradient(DEVICE& device, nn::layers::embedding::LayerGradient<SPEC>& layer) {
        zero_gradient(device, layer.weights);
    }
    template<typename DEVICE, typename SPEC, typename OPTIMIZER>
    RL_TOOLS_FUNCTION_PLACEMENT void update(DEVICE& device, nn::layers::embedding::LayerGradient<SPEC>& layer, OPTIMIZER& optimizer){
        update(device, layer.weights, optimizer);
    }

    template<typename DEVICE, typename SPEC, typename OPTIMIZER>
    RL_TOOLS_FUNCTION_PLACEMENT void _reset_optimizer_state(DEVICE& device, nn::layers::embedding::LayerGradient<SPEC>& layer, OPTIMIZER& optimizer) {
        _reset_optimizer_state(device, layer.weights, optimizer);
    }

    template<typename SOURCE_DEVICE, typename TARGET_DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, const  nn::layers::embedding::LayerForward<SOURCE_SPEC>& source, nn::layers::embedding::LayerForward<TARGET_SPEC>& target){
        copy(source_device, target_device, source.weights, target.weights);
    }
    template<typename SOURCE_DEVICE, typename TARGET_DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, const nn::layers::embedding::LayerBackward<SOURCE_SPEC>& source, nn::layers::embedding::LayerBackward<TARGET_SPEC>& target){
        copy(source_device, target_device, static_cast<const nn::layers::embedding::LayerForward<SOURCE_SPEC>&>(source), static_cast<nn::layers::embedding::LayerForward<TARGET_SPEC>&>(target));
    }
    template<typename SOURCE_DEVICE, typename TARGET_DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, const nn::layers::embedding::LayerGradient<SOURCE_SPEC>& source, nn::layers::embedding::LayerGradient<TARGET_SPEC>& target){
        copy(source_device, target_device, static_cast<const nn::layers::embedding::LayerBackward<SOURCE_SPEC>&>(source), static_cast<nn::layers::embedding::LayerBackward<TARGET_SPEC>&>(target));
    }
    template <typename DEVICE, typename SPEC_1, typename SPEC_2>
    RL_TOOLS_FUNCTION_PLACEMENT typename SPEC_1::TYPE_POLICY::DEFAULT abs_diff(DEVICE& device, const rl_tools::nn::layers::embedding::LayerForward<SPEC_1>& l1, const rl_tools::nn::layers::embedding::LayerForward<SPEC_2>& l2) {
        return abs_diff(device, l1.weights, l2.weights);
    }
    template <typename DEVICE, typename SPEC_1, typename SPEC_2>
    RL_TOOLS_FUNCTION_PLACEMENT typename SPEC_1::T abs_diff(DEVICE& device, const rl_tools::nn::layers::embedding::LayerBackward<SPEC_1>& l1, const rl_tools::nn::layers::embedding::LayerBackward<SPEC_2>& l2) {
        using T = typename SPEC_1::T;
        return abs_diff(device, static_cast<const rl_tools::nn::layers::embedding::LayerForward<SPEC_1>&>(l1), static_cast<const rl_tools::nn::layers::embedding::LayerForward<SPEC_2>&>(l2));
    }
    template <typename DEVICE, typename SPEC_1, typename SPEC_2>
    RL_TOOLS_FUNCTION_PLACEMENT typename SPEC_1::T abs_diff(DEVICE& device, const rl_tools::nn::layers::embedding::LayerGradient<SPEC_1>& l1, const rl_tools::nn::layers::embedding::LayerGradient<SPEC_2>& l2) {
        typename SPEC_1::T diff = abs_diff(device, l1.output, l2.output);
        diff += abs_diff(device, static_cast<const rl_tools::nn::layers::embedding::LayerBackward<SPEC_1>&>(l1), static_cast<const rl_tools::nn::layers::embedding::LayerBackward<SPEC_2>&>(l2));
        return diff;
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void reset_forward_state(DEVICE& device, rl_tools::nn::layers::embedding::LayerForward<SPEC>& l) { }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void reset_forward_state(DEVICE& device, rl_tools::nn::layers::embedding::LayerBackward<SPEC>& l) {
        reset_forward_state(device, (rl_tools::nn::layers::embedding::LayerForward<SPEC>&) l);
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void reset_forward_state(DEVICE& device, rl_tools::nn::layers::embedding::LayerGradient<SPEC>& l) {
        reset_forward_state(device, static_cast<rl_tools::nn::layers::embedding::LayerBackward<SPEC>&>(l));
        set_all(device, l.output, (typename decltype(l.output)::SPEC::T)0);
    }
    template <typename DEVICE, typename SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT bool is_nan(DEVICE& device, const rl_tools::nn::layers::embedding::LayerForward<SPEC>& l, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        return is_nan(device, l.weights, mode);
    }
    template <typename DEVICE, typename SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT bool is_nan(DEVICE& device, const rl_tools::nn::layers::embedding::LayerBackward<SPEC>& l, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        return is_nan(device, static_cast<const rl_tools::nn::layers::embedding::LayerForward<SPEC>&>(l), mode);
    }
    template <typename DEVICE, typename SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT bool is_nan(DEVICE& device, const rl_tools::nn::layers::embedding::LayerGradient<SPEC>& l, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        bool upstream_nan = is_nan(device, static_cast<const rl_tools::nn::layers::embedding::LayerBackward<SPEC>&>(l), mode);
        if constexpr(mode::is<MODE, nn::parameters::mode::ParametersOnly>){
            return upstream_nan;
        }
        return upstream_nan || is_nan(device, l.output, mode);
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT auto output(DEVICE& device, nn::layers::embedding::LayerGradient<SPEC>& l){
        // return l.output;
        auto tensor_flat = to_tensor(device, l.output);
        auto tensor = view_memory<typename SPEC::OUTPUT_SHAPE>(device, tensor_flat);
        return tensor;
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT auto gradient_norm(DEVICE& device, const nn::layers::embedding::LayerGradient<SPEC>& layer) {
        return gradient_norm(device, layer.weights);
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
