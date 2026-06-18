#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_LAYERS_STANDARDIZE_OPERATIONS_GENERIC_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_LAYERS_STANDARDIZE_OPERATIONS_GENERIC_H
#include "../../../utils/generic/typing.h"
#include "../../../containers/matrix/matrix.h"

#include "../../../nn/nn.h"
#include "../../../nn/parameters/parameters.h"
#include "layer.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, nn::layers::standardize::LayerForward<SPEC>& layer){
        malloc(device, layer.mean);
        malloc(device, layer.precision);
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, nn::layers::standardize::LayerForward<SPEC>& layer){
        free(device, layer.mean);
        free(device, layer.precision);
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, nn::layers::standardize::LayerGradient<SPEC>& layer){
        malloc(device, static_cast<nn::layers::standardize::LayerForward<SPEC>&>(layer));
        malloc(device, layer.output);
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, nn::layers::standardize::LayerGradient<SPEC>& layer){
        free(device, static_cast<nn::layers::standardize::LayerForward<SPEC>&>(layer));
        free(device, layer.output);
    }
    template<typename DEVICE>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, nn::layers::standardize::State& state) { } // no-op
    template <typename SOURCE_DEVICE, typename TARGET_DEVICE>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, nn::layers::standardize::State& source, nn::layers::standardize::State& target){}
    template<typename DEVICE, typename SPEC, typename RNG, typename MODE>
    RL_TOOLS_FUNCTION_PLACEMENT void reset(DEVICE& device, const nn::layers::standardize::LayerForward<SPEC>& layer, nn::layers::standardize::State& state, RNG&, Mode<MODE> mode = Mode<mode::Default<>>{}) { } // no-op
    template<typename DEVICE>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, nn::layers::standardize::State& state) { } // no-op
    template <typename DEVICE>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, nn::layers::standardize::Buffer& buffer){ }
    template <typename DEVICE>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, nn::layers::standardize::Buffer& buffer){ }
    template<typename DEVICE, typename SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT void init_weights(DEVICE& device, nn::layers::standardize::LayerForward<SPEC>& layer, RNG& rng) {
        set_all(device, layer.mean.parameters, 0);
        set_all(device, layer.precision.parameters, 1);
    }
    template<typename DEVICE, typename LAYER_SPEC, typename MEAN_SPEC, typename STD_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void set_statistics(DEVICE& device, nn::layers::standardize::LayerForward<LAYER_SPEC>& layer, const Matrix<MEAN_SPEC>& mean, const Matrix<STD_SPEC>& std) {
        using T = typename LAYER_SPEC::TYPE_POLICY::DEFAULT;
        using TI = typename DEVICE::index_t;
        using LAYER = nn::layers::standardize::LayerForward<LAYER_SPEC>;
        auto layer_mean = matrix_view(device, layer.mean.parameters);
        using MATRIX = decltype(layer_mean);
        static_assert(containers::check_structure<MEAN_SPEC, typename MATRIX::SPEC>);
        static_assert(containers::check_structure<STD_SPEC, typename MATRIX::SPEC>);
        static_assert(MEAN_SPEC::ROWS == 1);
        copy(device, device, mean, layer_mean);
        for(TI i=0; i < LAYER_SPEC::INPUT_DIM; i++){
            T current_std = get(std, 0, i);
            T precision = current_std == 0 ? 0 : 1/current_std;
            set(device, layer.precision.parameters, precision, i);
        }
    }
    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void evaluate(DEVICE& device, const nn::layers::standardize::LayerForward<LAYER_SPEC>& layer, const Matrix<INPUT_SPEC>& input, Matrix<OUTPUT_SPEC>& output, nn::layers::standardize::Buffer& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        static_assert(nn::layers::standardize::check_input_output<LAYER_SPEC, INPUT_SPEC, OUTPUT_SPEC>);
        using T = typename LAYER_SPEC::TYPE_POLICY::DEFAULT;
        using TI = typename DEVICE::index_t;
        constexpr TI BATCH_SIZE = INPUT_SPEC::ROWS;
        for(TI batch_i=0; batch_i < BATCH_SIZE; batch_i++){
            for(TI output_i = 0; output_i < LAYER_SPEC::OUTPUT_DIM; output_i++) {
                T mean = get(device, layer.mean.parameters, output_i);
                T precision = get(device, layer.precision.parameters, output_i);
                T input_value = get(input, batch_i, output_i);
                T output_value = (input_value - mean);
                if(precision != 0){
                    output_value *= precision;
                }
                set(output, batch_i, output_i, output_value);
            }
        }
    }
    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void forward(DEVICE& device, nn::layers::standardize::LayerBackward<LAYER_SPEC>& layer, const Matrix<INPUT_SPEC>& input, Matrix<OUTPUT_SPEC>& output, nn::layers::standardize::Buffer& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        static_assert(nn::layers::standardize::check_input_output<LAYER_SPEC, INPUT_SPEC, OUTPUT_SPEC>);
        evaluate(device, layer, input, output, buffer, rng, mode);
    }
    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void forward(DEVICE& device, nn::layers::standardize::LayerGradient<LAYER_SPEC>& layer, const Matrix<INPUT_SPEC>& input, nn::layers::standardize::Buffer& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        static_assert(nn::layers::standardize::check_input_output<LAYER_SPEC, INPUT_SPEC, typename decltype(layer.output)::SPEC>);
        forward(device, layer, input, layer.output, buffer, rng, mode);
    }
//    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename RNG>
//    void forward(DEVICE& device, nn::layers::standardize::LayerGradient<LAYER_SPEC>& layer, const Matrix<INPUT_SPEC>& input, Matrix<OUTPUT_SPEC>& output, RNG& rng) {
//        static_assert(nn::layers::standardize::check_input_output<LAYER_SPEC, INPUT_SPEC, OUTPUT_SPEC>);
//        forward(device, layer, input, layer.output, rng);
//        copy(device, device, layer.output, output);
//    }

    template<typename DEVICE, typename LAYER_SPEC, typename D_OUTPUT_SPEC, typename D_INPUT_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void backward_input(DEVICE& device, nn::layers::standardize::LayerBackward<LAYER_SPEC>& layer, const Matrix<D_OUTPUT_SPEC>& d_output, Matrix<D_INPUT_SPEC>& d_input, nn::layers::standardize::Buffer& buffer, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        static_assert(nn::layers::standardize::check_input_output<LAYER_SPEC, D_INPUT_SPEC, D_OUTPUT_SPEC>);
        static_assert(nn::layers::standardize::check_input_output<LAYER_SPEC, D_INPUT_SPEC, D_OUTPUT_SPEC>);
        static_assert(LAYER_SPEC::INPUT_DIM == LAYER_SPEC::OUTPUT_DIM);
        using TI = typename DEVICE::index_t;
        constexpr TI INPUT_DIM = LAYER_SPEC::INPUT_DIM;
        constexpr TI OUTPUT_DIM = LAYER_SPEC::OUTPUT_DIM;
        constexpr TI BATCH_SIZE = D_OUTPUT_SPEC::ROWS;
        using T = typename LAYER_SPEC::T;

        for(TI batch_i=0; batch_i < BATCH_SIZE; batch_i++){
            for(TI output_i = 0; output_i < OUTPUT_DIM; output_i++) {
//                out = (in - mu) * prec
                T d_output_value = get(d_output, batch_i, output_i);
                T precision = get(layer.precision.parameters, 0, output_i);
                T d_input_value = d_output_value * precision;
                set(d_input, batch_i, output_i, d_input_value);
            }
        }
    }

    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename D_OUTPUT_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void backward(DEVICE& device, nn::layers::standardize::LayerGradient<LAYER_SPEC>& layer, const Matrix<INPUT_SPEC>& input, Matrix<D_OUTPUT_SPEC>& d_output, nn::layers::standardize::Buffer& buffer, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        // this is a no-op as the standardize layer does not have trainable parameters
    }

    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename D_OUTPUT_SPEC, typename D_INPUT_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void backward_full(DEVICE& device, nn::layers::standardize::LayerGradient<LAYER_SPEC>& layer, const Matrix<INPUT_SPEC>& input, Matrix<D_OUTPUT_SPEC>& d_output, Matrix<D_INPUT_SPEC>& d_input, nn::layers::standardize::Buffer& buffer, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        // this is the same as the standardize layer does not have trainable parameters
        backward_input(device, layer, d_output, d_input, buffer, mode);
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void zero_gradient(DEVICE& device, nn::layers::standardize::LayerGradient<SPEC>& layer) {
        // this is a no-op as the standardize layer does not have trainable parameters
    }
    template<typename DEVICE, typename SPEC, typename OPTIMIZER>
    RL_TOOLS_FUNCTION_PLACEMENT void update(DEVICE& device, nn::layers::standardize::LayerGradient<SPEC>& layer, OPTIMIZER& optimizer){
        // this is a no-op as the standardize layer does not have trainable parameters
    }

    template<typename DEVICE, typename SPEC, typename OPTIMIZER>
    RL_TOOLS_FUNCTION_PLACEMENT void _reset_optimizer_state(DEVICE& device, nn::layers::standardize::LayerGradient<SPEC>& layer, OPTIMIZER& optimizer) {
        // this is a no-op as the standardize layer does not have trainable parameters
    }
    template<typename SOURCE_DEVICE, typename TARGET_DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, const  nn::layers::standardize::LayerForward<SOURCE_SPEC>& source, nn::layers::standardize::LayerForward<TARGET_SPEC>& target){
        static_assert(nn::layers::standardize::check_compatibility<SOURCE_SPEC, TARGET_SPEC>);
        copy(source_device, target_device, source.mean, target.mean);
        copy(source_device, target_device, source.precision, target.precision);
    }
    template<typename SOURCE_DEVICE, typename TARGET_DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, const nn::layers::standardize::LayerGradient<SOURCE_SPEC>& source, nn::layers::standardize::LayerGradient<TARGET_SPEC>& target){
        static_assert(nn::layers::standardize::check_compatibility<SOURCE_SPEC, TARGET_SPEC>);
        copy(source_device, target_device, static_cast<const nn::layers::standardize::LayerForward<SOURCE_SPEC>&>(source), static_cast<nn::layers::standardize::LayerForward<TARGET_SPEC>&>(target));
        copy(source_device, target_device, source.output, target.output);
    }
    template <typename DEVICE, typename SPEC_1, typename SPEC_2>
    RL_TOOLS_FUNCTION_PLACEMENT typename SPEC_1::T abs_diff(DEVICE& device, const rl_tools::nn::layers::standardize::LayerForward<SPEC_1>& l1, const rl_tools::nn::layers::standardize::LayerForward<SPEC_2>& l2) {
        static_assert(nn::layers::standardize::check_compatibility<SPEC_1, SPEC_2>);
        using T = typename SPEC_1::T;
        T acc = 0;
        acc += abs_diff(device, l1.mean, l2.mean);
        acc += abs_diff(device, l1.precision, l2.precision);
        return acc;
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT constexpr auto output(DEVICE& device, nn::layers::standardize::LayerGradient<SPEC>& l){
        auto tensor_flat = to_tensor(device, l.output);
        auto tensor = view_memory<typename SPEC::OUTPUT_SHAPE>(device, tensor_flat);
        return tensor;
    }
    template <typename DEVICE, typename SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT bool is_nan(DEVICE& device, const rl_tools::nn::layers::standardize::LayerForward<SPEC>& l, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        if constexpr(mode::is<MODE, nn::parameters::mode::ParametersOnly>){
            return false;
        }
        return is_nan(device, l.mean, mode) || is_nan(device, l.precision, mode);
    }
    template <typename DEVICE, typename SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT bool is_nan(DEVICE& device, const rl_tools::nn::layers::standardize::LayerBackward<SPEC>& l, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        return is_nan(device, static_cast<const rl_tools::nn::layers::standardize::LayerForward<SPEC>&>(l), mode);
    }
    template <typename DEVICE, typename SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT bool is_nan(DEVICE& device, const rl_tools::nn::layers::standardize::LayerGradient<SPEC>& l, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        bool upstream_nan = is_nan(device, static_cast<const rl_tools::nn::layers::standardize::LayerBackward<SPEC>&>(l), mode);
        if constexpr(mode::is<MODE, nn::parameters::mode::ParametersOnly>){
            return upstream_nan;
        }
        return upstream_nan || is_nan(device, l.output, mode);
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

// Tensor proxies
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void evaluate(DEVICE& device, const nn::layers::standardize::LayerForward<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, Tensor<OUTPUT_SPEC>& output, nn::layers::standardize::Buffer& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        auto matrix_view_input = matrix_view(device, input);
        auto matrix_view_output = matrix_view(device, output);
        evaluate(device, layer, matrix_view_input, matrix_view_output, buffer, rng, mode);
    }
    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void evaluate_step(DEVICE& device, const nn::layers::standardize::LayerForward<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, nn::layers::standardize::State& state, Tensor<OUTPUT_SPEC>& output, nn::layers::standardize::Buffer& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        auto matrix_view_input = matrix_view(device, input);
        auto matrix_view_output = matrix_view(device, output);
        evaluate(device, layer, matrix_view_input, matrix_view_output, buffer, rng, mode);
    }
    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void forward(DEVICE& device, nn::layers::standardize::LayerBackward<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, Tensor<OUTPUT_SPEC>& output, nn::layers::standardize::Buffer& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        auto matrix_view_input = matrix_view(device, input);
        auto matrix_view_output = matrix_view(device, output);
        forward(device, layer, matrix_view_input, matrix_view_output, buffer, rng, mode);
    }
    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void forward(DEVICE& device, nn::layers::standardize::LayerGradient<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, nn::layers::standardize::Buffer& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        auto matrix_view_input = matrix_view(device, input);
        forward(device, layer, matrix_view_input, layer.output, buffer, rng);
    }
    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void forward(DEVICE& device, nn::layers::standardize::LayerGradient<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, Tensor<OUTPUT_SPEC>& output, nn::layers::standardize::Buffer& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        auto matrix_view_input = matrix_view(device, input);
        auto matrix_view_output = matrix_view(device, output);
        forward(device, layer, matrix_view_input, matrix_view_output, buffer, rng, mode);
    }
    template<typename DEVICE, typename LAYER_SPEC, typename D_OUTPUT_SPEC, typename D_INPUT_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void backward_input(DEVICE& device, const nn::layers::standardize::LayerBackward<LAYER_SPEC>& layer, const Tensor<D_OUTPUT_SPEC>& d_output, Tensor<D_INPUT_SPEC>& d_input, nn::layers::standardize::Buffer& buffer, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        auto matrix_view_d_output = matrix_view(device, d_output);
        auto matrix_view_d_input = matrix_view(device, d_input);
        backward_input(device, layer, matrix_view_d_output, matrix_view_d_input, buffer, mode);
    }

    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename D_OUTPUT_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void backward(DEVICE& device, nn::layers::standardize::LayerGradient<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, Tensor<D_OUTPUT_SPEC>& d_output, nn::layers::standardize::Buffer& buffer, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        auto matrix_view_input = matrix_view(device, input);
        auto matrix_view_d_output = matrix_view(device, d_output);
        backward(device, layer, matrix_view_input, matrix_view_d_output, buffer, mode);
    }

    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename D_OUTPUT_SPEC, typename D_INPUT_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void backward_full(DEVICE& device, nn::layers::standardize::LayerGradient<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, Tensor<D_OUTPUT_SPEC>& d_output, Tensor<D_INPUT_SPEC>& d_input, nn::layers::standardize::Buffer& buffer, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        auto matrix_view_input = matrix_view(device, input);
        auto matrix_view_d_output = matrix_view(device, d_output);
        auto matrix_view_d_input = matrix_view(device, d_input);
        backward_full(device, layer, matrix_view_input, matrix_view_d_output, matrix_view_d_input, buffer, mode);
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT auto gradient_norm(DEVICE& device, const nn::layers::standardize::LayerGradient<SPEC>& layer) {
        return 0;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif