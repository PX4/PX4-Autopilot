#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_LAYERS_DENSE_OPERATIONS_GENERIC_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_LAYERS_DENSE_OPERATIONS_GENERIC_H

#include "../../../containers/matrix/matrix.h"
#include "../../../nn/parameters/operations_generic.h"

#include "layer.h"
#ifndef RL_TOOLS_FUNCTION_PLACEMENT
#define RL_TOOLS_FUNCTION_PLACEMENT
#endif

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, nn::layers::dense::LayerForward<SPEC>& layer) {
        malloc(device, layer.weights);
        malloc(device, layer.biases);
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, nn::layers::dense::LayerForward<SPEC>& layer) {
        free(device, layer.weights);
        free(device, layer.biases);
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, nn::layers::dense::LayerBackward<SPEC>& layer) {
        malloc(device, (nn::layers::dense::LayerForward<SPEC>&) layer);
        malloc(device, layer.pre_activations);
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, nn::layers::dense::LayerBackward<SPEC>& layer) {
        free(device, (nn::layers::dense::LayerForward<SPEC>&) layer);
        free(device, layer.pre_activations);
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, nn::layers::dense::LayerGradient<SPEC>& layer) {
        malloc(device, (nn::layers::dense::LayerBackward<SPEC>&) layer);
        malloc(device, layer.output);
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, nn::layers::dense::LayerGradient<SPEC>& layer) {
        free(device, (nn::layers::dense::LayerBackward<SPEC>&) layer);
        free(device, layer.output);
    }
    template<typename DEVICE>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, nn::layers::dense::State& state) { } // no-op
    template <typename SOURCE_DEVICE, typename TARGET_DEVICE>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, nn::layers::dense::State& source, nn::layers::dense::State& target){}
    template<typename SPEC, typename DEVICE, typename RNG, typename MODE>
    RL_TOOLS_FUNCTION_PLACEMENT void reset(DEVICE& device, const nn::layers::dense::LayerForward<SPEC>& layer, nn::layers::dense::State& state, RNG&, Mode<MODE> mode = Mode<mode::Default<>>{}) { } // no-op
    template<typename DEVICE>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, nn::layers::dense::State& state) { } // no-op
    template<typename DEVICE>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, nn::layers::dense::Buffer& buffer) { } // no-op
    template<typename DEVICE>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, nn::layers::dense::Buffer& buffer) { } // no-op

    template<typename DEVICE, typename SPEC, typename INITIALIZER_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT void init_weights(DEVICE& device, nn::layers::dense::LayerForward<SPEC>& layer, const nn::layers::dense::KaimingUniform<INITIALIZER_SPEC>& initializer, RNG& rng){
        using T = typename SPEC::TYPE_POLICY::DEFAULT;
        using TI = typename SPEC::TI;
        T gain;
        if constexpr(INITIALIZER_SPEC::INIT_LEGACY){
            T negative_slope = math::sqrt(device.math, (T)5);
            gain = math::sqrt(device.math, (T)2.0 / (1 + negative_slope * negative_slope));
        }
        else{
            gain = math::sqrt(device.math, (T)2.0) * INITIALIZER_SPEC::SCALE;
        }
        T fan = SPEC::INPUT_DIM;
        T std = gain / math::sqrt(device.math, fan);
        T weight_bound = math::sqrt(device.math, (T)3.0) * std;
        T bias_bound = 1/math::sqrt(device.math, (T)SPEC::INPUT_DIM);
        using PARAMETER_TYPE = typename decltype(layer.weights.parameters)::SPEC::T;
        for(TI i = 0; i < SPEC::OUTPUT_DIM; i++) {
            if constexpr(INITIALIZER_SPEC::INIT_LEGACY) {
                set(device, layer.biases.parameters, (PARAMETER_TYPE)random::uniform_real_distribution(device.random, -bias_bound, bias_bound, rng), i);
            }
            else{
                set(device, layer.biases.parameters, 0, i);
            }
            for(TI j = 0; j < SPEC::INPUT_DIM; j++) {
                set(device, layer.weights.parameters, (PARAMETER_TYPE)random::uniform_real_distribution(device.random, -weight_bound, weight_bound, rng), i, j);
            }
        }
    }
    template<typename DEVICE, typename SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT void init_weights(DEVICE& device, nn::layers::dense::LayerForward<SPEC>& layer, RNG& rng) {
        init_weights(device, layer, typename SPEC::INITIALIZER{}, rng);
    }

#ifndef RL_TOOLS_NN_DISABLE_GENERIC_FORWARD_BACKWARD
    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void evaluate(DEVICE& device, const nn::layers::dense::LayerForward<LAYER_SPEC>& layer, const Matrix<INPUT_SPEC>& input, Matrix<OUTPUT_SPEC>& output, nn::layers::dense::Buffer&, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        static_assert(nn::layers::dense::check_input_output<LAYER_SPEC, INPUT_SPEC, OUTPUT_SPEC>);
        // Warning do not use the same buffer for input and output!
        using TI = typename DEVICE::index_t;
        using ACCUMULATOR_TYPE = typename OUTPUT_SPEC::T;
        constexpr TI BATCH_SIZE = INPUT_SPEC::ROWS;
        for(TI batch_i=0; batch_i < BATCH_SIZE; batch_i++){
            for(TI output_i = 0; output_i < LAYER_SPEC::OUTPUT_DIM; output_i++) {
                set(output, batch_i, output_i, get(device, layer.biases.parameters, output_i));
                for(TI input_i = 0; input_i < LAYER_SPEC::INPUT_DIM; input_i++) {
                    increment(output, batch_i, output_i, get(device, layer.weights.parameters, output_i, input_i) * get(input, batch_i, input_i));
                }
                set(output, batch_i, output_i, activation<typename DEVICE::SPEC::MATH, ACCUMULATOR_TYPE, LAYER_SPEC::ACTIVATION_FUNCTION>(get(output, batch_i, output_i)));
            }
        }
    }

    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void forward(DEVICE& device, nn::layers::dense::LayerBackward<LAYER_SPEC>& layer, const Matrix<INPUT_SPEC>& input, Matrix<OUTPUT_SPEC>& output, nn::layers::dense::Buffer& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        // Warning do not use the same buffer for input and output!
        static_assert(nn::layers::dense::check_input_output<LAYER_SPEC, INPUT_SPEC, OUTPUT_SPEC>);
        constexpr auto BATCH_SIZE = INPUT_SPEC::ROWS;
        using ACCUMULATOR_TYPE = typename OUTPUT_SPEC::T;
        using TI = typename DEVICE::index_t;

        for(TI batch_i=0; batch_i < BATCH_SIZE; batch_i++){
            for(TI i = 0; i < LAYER_SPEC::OUTPUT_DIM; i++) {
                set(layer.pre_activations, batch_i, i, get(device, layer.biases.parameters, i));
                for(TI j = 0; j < LAYER_SPEC::INPUT_DIM; j++) {
                    increment(layer.pre_activations, batch_i, i, get(device, layer.weights.parameters, i, j) * get(input, batch_i, j));
                }
                set(output, batch_i, i, activation<typename DEVICE::SPEC::MATH, ACCUMULATOR_TYPE, LAYER_SPEC::ACTIVATION_FUNCTION>(get(layer.pre_activations, batch_i, i)));
            }
        }
    }
#endif

    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void forward(DEVICE& device, nn::layers::dense::LayerGradient<LAYER_SPEC>& layer, const Matrix<INPUT_SPEC>& input, nn::layers::dense::Buffer& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        static_assert(nn::layers::dense::check_input_output<LAYER_SPEC, INPUT_SPEC, typename decltype(layer.output)::SPEC>);
        forward(device, static_cast<nn::layers::dense::LayerBackward<LAYER_SPEC>&>(layer), input, layer.output, buffer, rng, mode);
    }
    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void forward(DEVICE& device, nn::layers::dense::LayerGradient<LAYER_SPEC>& layer, const Matrix<INPUT_SPEC>& input, Matrix<OUTPUT_SPEC>& output, nn::layers::dense::Buffer& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        static_assert(nn::layers::dense::check_input_output<LAYER_SPEC, INPUT_SPEC, OUTPUT_SPEC>);
        // compile time warning if used
        forward(device, layer, input, buffer, rng, mode);
        copy(device, device, layer.output, output);
    }

#ifndef RL_TOOLS_NN_DISABLE_GENERIC_FORWARD_BACKWARD
    template<typename DEVICE, typename LAYER_SPEC, typename D_OUTPUT_SPEC, typename D_INPUT_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void backward_input(DEVICE& device, const nn::layers::dense::LayerBackward<LAYER_SPEC>& layer, const Matrix<D_OUTPUT_SPEC>& d_output, Matrix<D_INPUT_SPEC>& d_input, nn::layers::dense::Buffer&, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        static_assert(nn::layers::dense::check_input_output<LAYER_SPEC, D_INPUT_SPEC, D_OUTPUT_SPEC>);
        // todo: create sparate function that does not set d_input (to save cost on backward pass for the first layer)
        using SPEC = LAYER_SPEC;
        constexpr auto BATCH_SIZE = D_OUTPUT_SPEC::ROWS;
        using T = typename SPEC::TYPE_POLICY::template GET<numeric_types::categories::Gradient>;
        using TI = typename DEVICE::index_t;
        for(TI batch_i=0; batch_i < BATCH_SIZE; batch_i++){
            for(TI output_i = 0; output_i < SPEC::OUTPUT_DIM; output_i++) {
                T d_pre_activation = d_activation_d_x<typename DEVICE::SPEC::MATH, T, LAYER_SPEC::ACTIVATION_FUNCTION>(get(layer.pre_activations, batch_i, output_i)) * get(d_output, batch_i, output_i);
                for(TI input_j = 0; input_j < SPEC::INPUT_DIM; input_j++) {
                    if(output_i == 0){
                        set(d_input, batch_i, input_j, 0);
                    }
                    increment(d_input, batch_i, input_j, get(device, layer.weights.parameters, output_i, input_j) * d_pre_activation);
                }
            }
        }
    }

    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename D_OUTPUT_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void backward(DEVICE& device, nn::layers::dense::LayerGradient<LAYER_SPEC>& layer, const Matrix<INPUT_SPEC>& input, Matrix<D_OUTPUT_SPEC>& d_output, nn::layers::dense::Buffer&, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        // todo: create sparate function that does not set d_input (to save cost on backward pass for the first layer)
        // todo: think about storing gradient in column major order to avoid iterating over the minor dimension
        static_assert(nn::layers::dense::check_input_output<LAYER_SPEC, INPUT_SPEC, D_OUTPUT_SPEC>);
        constexpr auto INPUT_DIM = LAYER_SPEC::INPUT_DIM;
        constexpr auto OUTPUT_DIM = LAYER_SPEC::OUTPUT_DIM;
        constexpr auto BATCH_SIZE = D_OUTPUT_SPEC::ROWS;
        using GRADIENT_TYPE = typename LAYER_SPEC::TYPE_POLICY::template GET<numeric_types::categories::Gradient>;
        using TI = typename DEVICE::index_t;

        for(TI batch_i=0; batch_i < BATCH_SIZE; batch_i++){
            for(TI output_i = 0; output_i < OUTPUT_DIM; output_i++) {
                GRADIENT_TYPE d_pre_activation = d_activation_d_x<typename DEVICE::SPEC::MATH, GRADIENT_TYPE, LAYER_SPEC::ACTIVATION_FUNCTION>(get(layer.pre_activations, batch_i, output_i)) * get(d_output, batch_i, output_i);
                increment(device, layer.biases.gradient, d_pre_activation, output_i);
                for(TI input_i = 0; input_i < INPUT_DIM; input_i++){
                    increment(device, layer.weights.gradient, d_pre_activation * get(input, batch_i, input_i), output_i, input_i);
                }
            }
        }
    }

    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename D_OUTPUT_SPEC, typename D_INPUT_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void backward_full(DEVICE& device, nn::layers::dense::LayerGradient<LAYER_SPEC>& layer, const Matrix<INPUT_SPEC>& input, Matrix<D_OUTPUT_SPEC>& d_output, Matrix<D_INPUT_SPEC>& d_input, nn::layers::dense::Buffer&, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        // todo: create sparate function that does not set d_input (to save cost on backward pass for the first layer)
        // todo: think about storing gradient in column major order to avoid iterating over the minor dimension
        static_assert(nn::layers::dense::check_input_output<LAYER_SPEC, D_INPUT_SPEC, D_OUTPUT_SPEC>);
        static_assert(nn::layers::dense::check_input_output<LAYER_SPEC, INPUT_SPEC, D_OUTPUT_SPEC>);
        constexpr auto INPUT_DIM = LAYER_SPEC::INPUT_DIM;
        constexpr auto OUTPUT_DIM = LAYER_SPEC::OUTPUT_DIM;
        constexpr auto BATCH_SIZE = D_OUTPUT_SPEC::ROWS;
        using GRADIENT_TYPE = typename LAYER_SPEC::TYPE_POLICY::template GET<numeric_types::categories::Gradient>;
        using TI = typename DEVICE::index_t;

        for(TI batch_i=0; batch_i < BATCH_SIZE; batch_i++){
            for(TI output_i = 0; output_i < OUTPUT_DIM; output_i++) {
                GRADIENT_TYPE d_pre_activation = d_activation_d_x<typename DEVICE::SPEC::MATH, GRADIENT_TYPE, LAYER_SPEC::ACTIVATION_FUNCTION>(get(layer.pre_activations, batch_i, output_i)) * get(d_output, batch_i, output_i);
                increment(device, layer.biases.gradient, d_pre_activation, output_i);
                for(TI input_i = 0; input_i < INPUT_DIM; input_i++){
                    if(output_i == 0){
                        set(d_input, batch_i, input_i, 0);
                    }
                    increment(d_input, batch_i, input_i, get(device, layer.weights.parameters, output_i, input_i) * d_pre_activation);
                    increment(device, layer.weights.gradient, d_pre_activation * get(input, batch_i, input_i), output_i, input_i);
                }
            }
        }
    }

#endif
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void zero_gradient(DEVICE& device, nn::layers::dense::LayerGradient<SPEC>& layer) {
        zero_gradient(device, layer.weights);
        zero_gradient(device, layer.biases);
    }
    template<typename DEVICE, typename SPEC, typename OPTIMIZER>
    RL_TOOLS_FUNCTION_PLACEMENT void update(DEVICE& device, nn::layers::dense::LayerGradient<SPEC>& layer, OPTIMIZER& optimizer){
        update(device, layer.weights, optimizer);
        update(device, layer.biases, optimizer);
    }

    template<typename DEVICE, typename SPEC, typename OPTIMIZER>
    RL_TOOLS_FUNCTION_PLACEMENT void _reset_optimizer_state(DEVICE& device, nn::layers::dense::LayerGradient<SPEC>& layer, OPTIMIZER& optimizer) {
        _reset_optimizer_state(device, layer.weights, optimizer);
        _reset_optimizer_state(device, layer.biases, optimizer);
    }

    template<typename SOURCE_DEVICE, typename TARGET_DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, const  nn::layers::dense::LayerForward<SOURCE_SPEC>& source, nn::layers::dense::LayerForward<TARGET_SPEC>& target){
        static_assert(nn::layers::dense::check_spec_memory<SOURCE_SPEC, TARGET_SPEC>);
        copy(source_device, target_device, source.weights, target.weights);
        copy(source_device, target_device, source.biases, target.biases);
    }
    template<typename SOURCE_DEVICE, typename TARGET_DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, const nn::layers::dense::LayerBackward<SOURCE_SPEC>& source, nn::layers::dense::LayerBackward<TARGET_SPEC>& target){
        static_assert(nn::layers::dense::check_spec_memory<SOURCE_SPEC, TARGET_SPEC>);
        copy(source_device, target_device, static_cast<const nn::layers::dense::LayerForward<SOURCE_SPEC>&>(source), static_cast<nn::layers::dense::LayerForward<TARGET_SPEC>&>(target));
        copy(source_device, target_device, source.pre_activations, target.pre_activations);
    }
    template<typename SOURCE_DEVICE, typename TARGET_DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, const nn::layers::dense::LayerGradient<SOURCE_SPEC>& source, nn::layers::dense::LayerGradient<TARGET_SPEC>& target){
        static_assert(nn::layers::dense::check_spec_memory<SOURCE_SPEC, TARGET_SPEC>);
        copy(source_device, target_device, static_cast<const nn::layers::dense::LayerBackward<SOURCE_SPEC>&>(source), static_cast<nn::layers::dense::LayerBackward<TARGET_SPEC>&>(target));
        copy(source_device, target_device, source.output, target.output);

    }
    template<typename SOURCE_DEVICE, typename TARGET_DEVICE, typename SOURCE, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy_from_generic(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, const SOURCE& source, nn::layers::dense::LayerForward<TARGET_SPEC>& target){
        static_assert(nn::layers::dense::check_spec_memory<typename SOURCE::SPEC, TARGET_SPEC>);
        copy_from_generic(source_device, target_device, source.weights, target.weights);
        copy_from_generic(source_device, target_device, source.biases, target.biases);
    }
    template<typename SOURCE_DEVICE, typename TARGET_DEVICE, typename SOURCE, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy_from_generic(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, const SOURCE& source, nn::layers::dense::LayerBackward<TARGET_SPEC>& target){
        static_assert(nn::layers::dense::check_spec_memory<typename SOURCE::SPEC, TARGET_SPEC>);
        copy_from_generic(source_device, target_device, static_cast<const typename SOURCE::PARENT&>(source), static_cast<nn::layers::dense::LayerForward<TARGET_SPEC>&>(target));
        copy_from_generic(source_device, target_device, source.pre_activations, target.pre_activations);
    }
    template<typename SOURCE_DEVICE, typename TARGET_DEVICE, typename SOURCE, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy_from_generic(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, const SOURCE& source, nn::layers::dense::LayerGradient<TARGET_SPEC>& target){
        static_assert(nn::layers::dense::check_spec_memory<typename SOURCE::SPEC, TARGET_SPEC>);
        copy_from_generic(source_device, target_device, static_cast<const typename SOURCE::PARENT&>(source), static_cast<nn::layers::dense::LayerBackward<TARGET_SPEC>&>(target));
        copy_from_generic(source_device, target_device, source.output, target.output);

    }
    template <typename DEVICE, typename SPEC_1, typename SPEC_2>
    RL_TOOLS_FUNCTION_PLACEMENT typename SPEC_1::TYPE_POLICY::DEFAULT abs_diff(DEVICE& device, const rl_tools::nn::layers::dense::LayerForward<SPEC_1>* l1, const rl_tools::nn::layers::dense::LayerForward<SPEC_2>* l2) {
        static_assert(nn::layers::dense::check_spec_memory<SPEC_1, SPEC_2>);
        using T = typename SPEC_1::TYPE_POLICY::DEFAULT;
        T acc = 0;
        acc += abs_diff(device, l1->weights, l2->weights);
        acc += abs_diff(device, l1->biases, l2->biases);
        return acc;
    }
    template <typename DEVICE, typename SPEC_1, typename SPEC_2>
    RL_TOOLS_FUNCTION_PLACEMENT typename SPEC_1::TYPE_POLICY::DEFAULT abs_diff(DEVICE& device, const rl_tools::nn::layers::dense::LayerForward<SPEC_1>& l1, const rl_tools::nn::layers::dense::LayerForward<SPEC_2>& l2) {
        static_assert(nn::layers::dense::check_spec_memory<SPEC_1, SPEC_2>);
        return abs_diff(device, &l1, &l2);
    }
    template <typename DEVICE, typename SPEC_1, typename SPEC_2>
    RL_TOOLS_FUNCTION_PLACEMENT typename SPEC_1::TYPE_POLICY::DEFAULT abs_diff(DEVICE& device, const rl_tools::nn::layers::dense::LayerBackward<SPEC_1>* l1, const rl_tools::nn::layers::dense::LayerBackward<SPEC_2>* l2) {
        static_assert(nn::layers::dense::check_spec_memory<SPEC_1, SPEC_2>);
        using T = typename SPEC_1::TYPE_POLICY::DEFAULT ;
        T acc = abs_diff(device, (rl_tools::nn::layers::dense::LayerForward<SPEC_1>*) l1, (rl_tools::nn::layers::dense::LayerForward<SPEC_2>*) l2);
        acc += abs_diff(device, l1->pre_activations, l2->pre_activations);
        return acc;
    }
    template <typename DEVICE, typename SPEC_1, typename SPEC_2>
    RL_TOOLS_FUNCTION_PLACEMENT typename SPEC_1::TYPE_POLICY::DEFAULT abs_diff(DEVICE& device, const rl_tools::nn::layers::dense::LayerBackward<SPEC_1>& l1, const rl_tools::nn::layers::dense::LayerBackward<SPEC_2>& l2) {
        static_assert(nn::layers::dense::check_spec_memory<SPEC_1, SPEC_2>);
        return abs_diff(device, &l1, &l2);
    }
    template <typename DEVICE, typename SPEC_1, typename SPEC_2>
    RL_TOOLS_FUNCTION_PLACEMENT typename SPEC_1::TYPE_POLICY::DEFAULT abs_diff(DEVICE& device, const rl_tools::nn::layers::dense::LayerGradient<SPEC_1>* l1, const rl_tools::nn::layers::dense::LayerGradient<SPEC_2>* l2) {
        static_assert(nn::layers::dense::check_spec_memory<SPEC_1, SPEC_2>);
        using T = typename SPEC_1::TYPE_POLICY::DEFAULT;
        T acc = abs_diff(device, (rl_tools::nn::layers::dense::LayerBackward<SPEC_1>*) l1, (rl_tools::nn::layers::dense::LayerBackward<SPEC_2>*) l2);
        acc += abs_diff(device, l1->output, l2->output);
        return acc;
    }
    template <typename DEVICE, typename SPEC_1, typename SPEC_2>
    RL_TOOLS_FUNCTION_PLACEMENT typename SPEC_1::TYPE_POLICY::DEFAULT abs_diff(DEVICE& device, const rl_tools::nn::layers::dense::LayerGradient<SPEC_1>& l1, const rl_tools::nn::layers::dense::LayerGradient<SPEC_2>& l2) {
        static_assert(nn::layers::dense::check_spec_memory<SPEC_1, SPEC_2>);
        return abs_diff(device, &l1, &l2);
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void reset_forward_state(DEVICE& device, rl_tools::nn::layers::dense::LayerBackward<SPEC>* l) {
        set_all(device, l->pre_activations, 0);
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void reset_forward_state(DEVICE& device, rl_tools::nn::layers::dense::LayerBackward<SPEC>& l) {
        reset_forward_state(device, (rl_tools::nn::layers::dense::LayerForward<SPEC>*) l);
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void reset_forward_state(DEVICE& device, rl_tools::nn::layers::dense::LayerGradient<SPEC>* l) {
        reset_forward_state(device, (rl_tools::nn::layers::dense::LayerBackward<SPEC>*) l);
        set_all(device, l->output, 0);
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void reset_forward_state(DEVICE& device, rl_tools::nn::layers::dense::LayerGradient<SPEC>& l) {
        reset_forward_state(device, &l);
    }
    template <typename DEVICE, typename SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT bool is_nan(DEVICE& device, const rl_tools::nn::layers::dense::LayerForward<SPEC>& l, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        return is_nan(device, l.weights, mode) || is_nan(device, l.biases, mode);
    }
    template <typename DEVICE, typename SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT bool is_nan(DEVICE& device, const rl_tools::nn::layers::dense::LayerBackward<SPEC>& l, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        bool upstream_nan = is_nan(device, static_cast<const rl_tools::nn::layers::dense::LayerForward<SPEC>&>(l), mode);
        if(mode::is<MODE, nn::parameters::mode::ParametersOnly>){
            return upstream_nan;
        }
        return  upstream_nan || is_nan(device, l.pre_activations, mode);
    }
    template <typename DEVICE, typename SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT bool is_nan(DEVICE& device, const rl_tools::nn::layers::dense::LayerGradient<SPEC>& l, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        bool upstream_nan = is_nan(device, static_cast<const rl_tools::nn::layers::dense::LayerBackward<SPEC>&>(l), mode);
        if constexpr(mode::is<MODE, nn::parameters::mode::ParametersOnly>){
            return upstream_nan;
        }
        return  upstream_nan || is_nan(device, l.output, mode);
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT auto output(DEVICE& device, nn::layers::dense::LayerGradient<SPEC>& l){
        auto tensor_flat = to_tensor(device, l.output);
        auto tensor = view_memory<typename SPEC::OUTPUT_SHAPE>(device, tensor_flat);
        return tensor;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

// Tensor proxies
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void evaluate(DEVICE& device, const nn::layers::dense::LayerForward<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, Tensor<OUTPUT_SPEC>& output, nn::layers::dense::Buffer& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        auto matrix_view_input = matrix_view(device, input);
        auto matrix_view_output = matrix_view(device, output);
        evaluate(device, layer, matrix_view_input, matrix_view_output, buffer, rng, mode);
    }
    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void evaluate_step(DEVICE& device, const nn::layers::dense::LayerForward<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, nn::layers::dense::State& state, Tensor<OUTPUT_SPEC>& output, nn::layers::dense::Buffer& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        auto matrix_view_input = matrix_view(device, input);
        auto matrix_view_output = matrix_view(device, output);
        evaluate(device, layer, matrix_view_input, matrix_view_output, buffer, rng, mode);
    }
    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void forward(DEVICE& device, nn::layers::dense::LayerBackward<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, Tensor<OUTPUT_SPEC>& output, nn::layers::dense::Buffer& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        auto matrix_view_input = matrix_view(device, input);
        auto matrix_view_output = matrix_view(device, output);
        forward(device, layer, matrix_view_input, matrix_view_output, buffer, rng, mode);
    }
    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void forward(DEVICE& device, nn::layers::dense::LayerGradient<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, nn::layers::dense::Buffer& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        auto matrix_view_input = matrix_view(device, input);
        forward(device, layer, matrix_view_input, layer.output, buffer, rng);
    }
    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void forward(DEVICE& device, nn::layers::dense::LayerGradient<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, Tensor<OUTPUT_SPEC>& output, nn::layers::dense::Buffer& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        auto matrix_view_input = matrix_view(device, input);
        auto matrix_view_output = matrix_view(device, output);
        forward(device, layer, matrix_view_input, matrix_view_output, buffer, rng, mode);
    }
    template<typename DEVICE, typename LAYER_SPEC, typename D_OUTPUT_SPEC, typename D_INPUT_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void backward_input(DEVICE& device, const nn::layers::dense::LayerBackward<LAYER_SPEC>& layer, const Tensor<D_OUTPUT_SPEC>& d_output, Tensor<D_INPUT_SPEC>& d_input, nn::layers::dense::Buffer& buffer, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        auto matrix_view_d_output = matrix_view(device, d_output);
        auto matrix_view_d_input = matrix_view(device, d_input);
        backward_input(device, layer, matrix_view_d_output, matrix_view_d_input, buffer, mode);
    }

    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename D_OUTPUT_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void backward(DEVICE& device, nn::layers::dense::LayerGradient<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, Tensor<D_OUTPUT_SPEC>& d_output, nn::layers::dense::Buffer& buffer, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        auto matrix_view_input = matrix_view(device, input);
        auto matrix_view_d_output = matrix_view(device, d_output);
        backward(device, layer, matrix_view_input, matrix_view_d_output, buffer, mode);
    }

    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename D_OUTPUT_SPEC, typename D_INPUT_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void backward_full(DEVICE& device, nn::layers::dense::LayerGradient<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, Tensor<D_OUTPUT_SPEC>& d_output, Tensor<D_INPUT_SPEC>& d_input, nn::layers::dense::Buffer& buffer, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        auto matrix_view_input = matrix_view(device, input);
        auto matrix_view_d_output = matrix_view(device, d_output);
        auto matrix_view_d_input = matrix_view(device, d_input);
        backward_full(device, layer, matrix_view_input, matrix_view_d_output, matrix_view_d_input, buffer, mode);
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT auto gradient_norm(DEVICE& device, const nn::layers::dense::LayerGradient<SPEC>& layer) {
        return gradient_norm(device, layer.weights) + gradient_norm(device, layer.biases);
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif
