#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_LAYERS_TD3_SAMPLING_OPERATIONS_GENERIC_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_LAYERS_TD3_SAMPLING_OPERATIONS_GENERIC_H
#include "../../../nn/activation_functions.h"
#include "../../../utils/generic/typing.h"
#include "../../../containers/matrix/matrix.h"


#include "layer.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, nn::layers::td3_sampling::LayerForward<SPEC>& layer){ }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, nn::layers::td3_sampling::LayerForward<SPEC>& layer){ }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, nn::layers::td3_sampling::LayerBackward<SPEC>& layer){
        malloc(device, static_cast<nn::layers::td3_sampling::LayerForward<SPEC>&>(layer));
        malloc(device, layer.pre_clip);
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, nn::layers::td3_sampling::LayerBackward<SPEC>& layer){
        free(device, static_cast<nn::layers::td3_sampling::LayerForward<SPEC>&>(layer));
        free(device, layer.pre_clip);
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, nn::layers::td3_sampling::LayerGradient<SPEC>& layer){
        malloc(device, static_cast<nn::layers::td3_sampling::LayerBackward<SPEC>&>(layer));
        malloc(device, layer.output);
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, nn::layers::td3_sampling::LayerGradient<SPEC>& layer){
        free(device, static_cast<nn::layers::td3_sampling::LayerBackward<SPEC>&>(layer));
        free(device, layer.output);
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, nn::layers::td3_sampling::Buffer<SPEC>& buffer) {
        malloc(device, buffer.noise);
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, nn::layers::td3_sampling::Buffer<SPEC>& buffer) {
        free(device, buffer.noise);
    }
    template<typename DEVICE>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, nn::layers::td3_sampling::State& state) { } // no-op
    template <typename SOURCE_DEVICE, typename TARGET_DEVICE>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, nn::layers::td3_sampling::State& source, nn::layers::td3_sampling::State& target){}
    template<typename DEVICE, typename SPEC, typename RNG, typename MODE>
    RL_TOOLS_FUNCTION_PLACEMENT void reset(DEVICE& device, const nn::layers::td3_sampling::LayerForward<SPEC>& layer, nn::layers::td3_sampling::State& state, RNG&, Mode<MODE> mode = Mode<mode::Default<>>{}) { } // no-op
    template<typename DEVICE>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, nn::layers::td3_sampling::State& state) { } // no-op
    template <typename SOURCE_DEVICE, typename TARGET_DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, nn::layers::td3_sampling::Buffer<SOURCE_SPEC>& source, nn::layers::td3_sampling::Buffer<TARGET_SPEC>& target){
        copy(source_device, target_device, source.noise, target.noise);
    }
    template <typename DEVICE, typename SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT void init_weights(DEVICE& device, nn::layers::td3_sampling::LayerForward<SPEC>& layer, RNG& rng){ }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void reset_forward_state(DEVICE& device, rl_tools::nn::layers::td3_sampling::LayerBackward<SPEC>& l) { }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void zero_gradient(DEVICE& device, nn::layers::td3_sampling::LayerGradient<SPEC>& layer) { }
    template<typename DEVICE, typename SPEC, typename OPTIMIZER>
    RL_TOOLS_FUNCTION_PLACEMENT void update(DEVICE& device, nn::layers::td3_sampling::LayerGradient<SPEC>& layer, OPTIMIZER& optimizer){ }
    template<typename DEVICE, typename SPEC, typename OPTIMIZER>
    RL_TOOLS_FUNCTION_PLACEMENT void _reset_optimizer_state(DEVICE& device, nn::layers::td3_sampling::LayerGradient<SPEC>& layer, OPTIMIZER& optimizer) { }

    template<typename SOURCE_DEVICE, typename TARGET_DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, const  nn::layers::td3_sampling::LayerForward<SOURCE_SPEC>& source, nn::layers::td3_sampling::LayerForward<TARGET_SPEC>& target){ }
    template<typename DEVICE, typename SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT void sample(DEVICE& device, nn::layers::td3_sampling::Buffer<SPEC>& buffer, RNG& rng) {
        randn(device, buffer.noise, rng);
    }
    template <bool SET_PRE_CLIPPING, typename DEVICE, typename SPEC, typename INPUT_SPEC, typename PRE_CLIP_SPEC, typename OUTPUT_SPEC, typename BUFFER_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void evaluate_per_sample(DEVICE& device, const nn::layers::td3_sampling::LayerForward<SPEC>& layer, const Matrix<INPUT_SPEC>& input, Matrix<PRE_CLIP_SPEC>& pre_clip, Matrix<OUTPUT_SPEC>& output, nn::layers::td3_sampling::Buffer<BUFFER_SPEC>& buffer, RNG& rng, typename DEVICE::index_t row_i, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        using TI = typename DEVICE::index_t;
        using T = typename SPEC::TYPE_POLICY::DEFAULT;
        using PARAMETERS = typename SPEC::PARAMETERS;
        for(TI col_i = 0; col_i < SPEC::DIM; col_i++){
            T mean = get(input, row_i, col_i);
            add_scalar(device, device.logger, "actor_eval_std", layer.std, 100);
            T noise;
            if constexpr(mode::is<MODE, nn::layers::td3_sampling::mode::ExternalNoise>){
                noise = get(buffer.noise, row_i, col_i);
            }
            else{
                if constexpr(mode::is<MODE, mode::Default> || mode::is<MODE, mode::Rollout>){
                    noise = random::normal_distribution::sample(device.random, (T)0, (T)1, rng);
                }
                else{
                    if constexpr(mode::is<MODE, mode::Evaluation>){
                        noise = 0;
                    }
                    else{
                        noise = 0;
                        utils::assert_exit(device.logger, false, "Invalid mode");
                    }
                }
            }
            T sample;
            if constexpr(mode::is<MODE, mode::Evaluation>){
                sample = mean;
            }
            else{
                sample = mean + noise * layer.std;
            }

            if constexpr(SET_PRE_CLIPPING){
                set(pre_clip, row_i, col_i, sample);
            }

            T sample_clipped = math::clamp(device.math, sample, (T)-1, (T)1);
            set(output, row_i, col_i, sample_clipped);
        }
    }
    template <typename DEVICE, typename SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename BUFFER_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void evaluate(DEVICE& device, const nn::layers::td3_sampling::LayerForward<SPEC>& layer, const Matrix<INPUT_SPEC>& input, Matrix<OUTPUT_SPEC>& output, nn::layers::td3_sampling::Buffer<BUFFER_SPEC>& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        static_assert(INPUT_SPEC::COLS == SPEC::DIM);
        static_assert(OUTPUT_SPEC::COLS == SPEC::DIM);
        static_assert(INPUT_SPEC::ROWS == OUTPUT_SPEC::ROWS);
        using TI = typename DEVICE::index_t;
        for(TI row_i = 0; row_i < INPUT_SPEC::ROWS; row_i++){
            evaluate_per_sample<false>(device, layer, input, output, output, buffer, rng, row_i, mode); // we can pass output for pre_clipping because we don't set it
        }
    }
    template <typename DEVICE, typename SPEC, typename INPUT_SPEC, typename BUFFER_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void forward(DEVICE& device, nn::layers::td3_sampling::LayerGradient<SPEC>& layer, const Matrix<INPUT_SPEC>& input, nn::layers::td3_sampling::Buffer<BUFFER_SPEC>& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        static_assert(INPUT_SPEC::COLS == SPEC::DIM);
        static_assert(INPUT_SPEC::ROWS == decltype(layer.output)::SPEC::ROWS);
        using TI = typename DEVICE::index_t;
        for(TI row_i = 0; row_i < INPUT_SPEC::ROWS; row_i++){
            evaluate_per_sample<true>(device, layer, input, layer.pre_clip, layer.output, buffer, rng, row_i, mode);
        }
    }
    template <typename DEVICE, typename SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename BUFFER_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void forward(DEVICE& device, nn::layers::td3_sampling::LayerGradient<SPEC>& layer, const Matrix<INPUT_SPEC>& input, Matrix<OUTPUT_SPEC>& output, nn::layers::td3_sampling::Buffer<BUFFER_SPEC>& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        forward(device, layer, input, buffer, rng, mode);
        copy(device, device, layer.output, output);
    }
    template<typename DEVICE, typename SPEC, typename D_OUTPUT_SPEC, typename D_INPUT_SPEC, typename BUFFER_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void backward_input(DEVICE& device, const nn::layers::td3_sampling::LayerBackward<SPEC>& layer, const Matrix<D_OUTPUT_SPEC>& d_output, Matrix<D_INPUT_SPEC>& d_input, nn::layers::td3_sampling::Buffer<BUFFER_SPEC>&, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        utils::assert_exit(device, false, "Not implemented");
    }
    template<typename DEVICE, typename SPEC, typename INPUT_SPEC, typename D_OUTPUT_SPEC, typename BUFFER_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void backward(DEVICE& device, nn::layers::td3_sampling::LayerGradient<SPEC>& layer, const Matrix<INPUT_SPEC>& input, Matrix<D_OUTPUT_SPEC>& d_output, nn::layers::td3_sampling::Buffer<BUFFER_SPEC>&, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        utils::assert_exit(device, false, "Not implemented");
    }
    template<typename DEVICE, typename SPEC, typename INPUT_SPEC, typename D_OUTPUT_SPEC, typename D_INPUT_SPEC, typename BUFFER_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void backward_full_per_sample(DEVICE& device, nn::layers::td3_sampling::LayerGradient<SPEC>& layer, const Matrix<INPUT_SPEC>& input, Matrix<D_OUTPUT_SPEC>& d_output, Matrix<D_INPUT_SPEC>& d_input, nn::layers::td3_sampling::Buffer<BUFFER_SPEC>&, typename DEVICE::index_t batch_i, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        using T = typename SPEC::TYPE_POLICY::DEFAULT;
        using TI = typename DEVICE::index_t;
        constexpr TI ACTION_DIM = SPEC::DIM;
        for(TI action_i = 0; action_i < ACTION_DIM; action_i++){
            T action = get(layer.output, batch_i, action_i);
            T d_output_value = get(d_output, batch_i, action_i);
            T pre_clip = get(layer.pre_clip, batch_i, action_i);
            T d_input_value = d_output_value;
            if(pre_clip < -1 || pre_clip > 1){
                d_input_value = 0;
            }
            set(d_input, batch_i, action_i, d_input_value);
        }
    }
    template<typename DEVICE, typename SPEC, typename INPUT_SPEC, typename D_OUTPUT_SPEC, typename D_INPUT_SPEC, typename BUFFER_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void backward_full(DEVICE& device, nn::layers::td3_sampling::LayerGradient<SPEC>& layer, const Matrix<INPUT_SPEC>& input, Matrix<D_OUTPUT_SPEC>& d_output, Matrix<D_INPUT_SPEC>& d_input, nn::layers::td3_sampling::Buffer<BUFFER_SPEC>& buffer, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        using TI = typename DEVICE::index_t;
        using LAYER = nn::layers::td3_sampling::LayerGradient<SPEC>;
        constexpr TI INTERNAL_BATCH_SIZE = LAYER::INTERNAL_BATCH_SIZE;
        for(TI batch_i = 0; batch_i < INTERNAL_BATCH_SIZE; batch_i++){
            backward_full_per_sample(device, layer, input, d_output, d_input, buffer, batch_i, mode);
        }
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT auto output(DEVICE& device, nn::layers::td3_sampling::LayerGradient<SPEC>& l){
        auto tensor_flat = to_tensor(device, l.output);
        auto tensor = view_memory<typename SPEC::OUTPUT_SHAPE>(device, tensor_flat);
        return tensor;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

// Tensor proxies
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename BUFFER_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void evaluate(DEVICE& device, const nn::layers::td3_sampling::LayerForward<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, Tensor<OUTPUT_SPEC>& output, nn::layers::td3_sampling::Buffer<BUFFER_SPEC>& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        auto matrix_view_input = matrix_view(device, input);
        auto matrix_view_output = matrix_view(device, output);
        evaluate(device, layer, matrix_view_input, matrix_view_output, buffer, rng, mode);
    }
    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename BUFFER_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void evaluate_step(DEVICE& device, const nn::layers::td3_sampling::LayerForward<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, nn::layers::td3_sampling::State& state, Tensor<OUTPUT_SPEC>& output, nn::layers::td3_sampling::Buffer<BUFFER_SPEC>& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        auto matrix_view_input = matrix_view(device, input);
        auto matrix_view_output = matrix_view(device, output);
        evaluate(device, layer, matrix_view_input, matrix_view_output, buffer, rng, mode);
    }
    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename BUFFER_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void forward(DEVICE& device, nn::layers::td3_sampling::LayerBackward<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, Tensor<OUTPUT_SPEC>& output, nn::layers::td3_sampling::Buffer<BUFFER_SPEC>& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        auto matrix_view_input = matrix_view(device, input);
        auto matrix_view_output = matrix_view(device, output);
        forward(device, layer, matrix_view_input, buffer, rng, mode);
        copy(device, device, layer.output, matrix_view_output);
    }
    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename BUFFER_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void forward(DEVICE& device, nn::layers::td3_sampling::LayerGradient<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, nn::layers::td3_sampling::Buffer<BUFFER_SPEC>& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        auto matrix_view_input = matrix_view(device, input);
        forward(device, layer, matrix_view_input, buffer, rng, mode);
    }
    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename BUFFER_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void forward(DEVICE& device, nn::layers::td3_sampling::LayerGradient<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, Tensor<OUTPUT_SPEC>& output, nn::layers::td3_sampling::Buffer<BUFFER_SPEC>& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        auto matrix_view_input = matrix_view(device, input);
        auto matrix_view_output = matrix_view(device, output);
        forward(device, layer, matrix_view_input, matrix_view_output, buffer, rng, mode);
    }
    template<typename DEVICE, typename LAYER_SPEC, typename D_OUTPUT_SPEC, typename D_INPUT_SPEC, typename BUFFER_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void backward_input(DEVICE& device, const nn::layers::td3_sampling::LayerBackward<LAYER_SPEC>& layer, const Tensor<D_OUTPUT_SPEC>& d_output, Tensor<D_INPUT_SPEC>& d_input, nn::layers::td3_sampling::Buffer<BUFFER_SPEC>& buffer, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        auto matrix_view_d_output = matrix_view(device, d_output);
        auto matrix_view_d_input = matrix_view(device, d_input);
        backward_input(device, layer, matrix_view_d_output, matrix_view_d_input, buffer, mode);
    }

    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename D_OUTPUT_SPEC, typename BUFFER_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void backward(DEVICE& device, nn::layers::td3_sampling::LayerGradient<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, Tensor<D_OUTPUT_SPEC>& d_output, nn::layers::td3_sampling::Buffer<BUFFER_SPEC>& buffer, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        auto matrix_view_input = matrix_view(device, input);
        auto matrix_view_d_output = matrix_view(device, d_output);
        backward(device, layer, matrix_view_input, matrix_view_d_output, buffer, mode);
    }

    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename D_OUTPUT_SPEC, typename D_INPUT_SPEC, typename BUFFER_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void backward_full(DEVICE& device, nn::layers::td3_sampling::LayerGradient<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, Tensor<D_OUTPUT_SPEC>& d_output, Tensor<D_INPUT_SPEC>& d_input, nn::layers::td3_sampling::Buffer<BUFFER_SPEC>& buffer, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        auto matrix_view_input = matrix_view(device, input);
        auto matrix_view_d_output = matrix_view(device, d_output);
        auto matrix_view_d_input = matrix_view(device, d_input);
        backward_full(device, layer, matrix_view_input, matrix_view_d_output, matrix_view_d_input, buffer, mode);
    }
    template <typename DEVICE, typename SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT bool is_nan(DEVICE& device, const rl_tools::nn::layers::td3_sampling::LayerForward<SPEC>& l, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        return false;
    }
    template <typename DEVICE, typename SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT bool is_nan(DEVICE& device, const rl_tools::nn::layers::td3_sampling::LayerBackward<SPEC>& l, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        bool upstream_nan = is_nan(device, static_cast<const rl_tools::nn::layers::td3_sampling::LayerForward<SPEC>&>(l), mode);
        if constexpr(mode::is<MODE, nn::parameters::mode::ParametersOnly>){
            return upstream_nan;
        }
        return upstream_nan || is_nan(device, l.pre_squashing, mode) || is_nan(device, l.noise, mode);
    }
    template <typename DEVICE, typename SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT bool is_nan(DEVICE& device, const rl_tools::nn::layers::td3_sampling::LayerGradient<SPEC>& l, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        bool upstream_nan = is_nan(device, static_cast<const rl_tools::nn::layers::td3_sampling::LayerBackward<SPEC>&>(l), mode);
        upstream_nan =  upstream_nan || is_nan(device, l.log_alpha, mode);
        if constexpr(mode::is<MODE, nn::parameters::mode::ParametersOnly>){
            return upstream_nan;
        }
        return upstream_nan || is_nan(device, l.log_probabilities, mode) || is_nan(device, l.output, mode);
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT auto gradient_norm(DEVICE& device, const nn::layers::td3_sampling::LayerGradient<SPEC>& layer) {
        return 0;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
