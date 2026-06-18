#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_MODELS_MLP_OPERATIONS_GENERIC_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_MODELS_MLP_OPERATIONS_GENERIC_H

#include "network.h"
#include "../../nn/operations_generic.h"
#include "../../nn/parameters/operations_generic.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools {
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, nn_models::mlp::NeuralNetworkForward<SPEC>& network) {
        malloc(device, network.input_layer);
        for (typename DEVICE::index_t layer_i = 0; layer_i < SPEC::NUM_HIDDEN_LAYERS; layer_i++){
            malloc(device, network.hidden_layers[layer_i]);
        }
        malloc(device, network.output_layer);
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, nn_models::mlp::NeuralNetworkForward<SPEC>& network) {
        free(device, network.input_layer);
        for (typename DEVICE::index_t layer_i = 0; layer_i < SPEC::NUM_HIDDEN_LAYERS; layer_i++){
            free(device, network.hidden_layers[layer_i]);
        }
        free(device, network.output_layer);
    }
    template<typename DEVICE>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, nn_models::mlp::State& state) { } // no-op
    template <typename SOURCE_DEVICE, typename TARGET_DEVICE>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, nn_models::mlp::State& source, nn_models::mlp::State& target){}
    template<typename DEVICE, typename SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void reset(DEVICE& device, const nn_models::mlp::NeuralNetworkForward<SPEC>& layer, nn_models::mlp::State& state, RNG&, Mode<MODE> mode = Mode<mode::Default<>>{}) { } // no-op
    template<typename DEVICE>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, nn_models::mlp::State& state) { } // no-op
    template<typename DEVICE, typename BUFFER_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, nn_models::mlp::NeuralNetworkBuffers<BUFFER_SPEC>& buffers) {
        malloc(device, buffers.tick);
        malloc(device, buffers.tock);
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, nn_models::mlp::NeuralNetworkBuffers<SPEC>& buffers) {
        free(device, buffers.tick);
        free(device, buffers.tock);
    }
    template<typename DEVICE, typename SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT void init_weights(DEVICE& device, nn_models::mlp::NeuralNetworkForward<SPEC>& network, RNG& rng) {
        init_weights(device, network.input_layer, rng);
        for (typename DEVICE::index_t layer_i = 0; layer_i < SPEC::NUM_HIDDEN_LAYERS; layer_i++){
            init_weights(device, network.hidden_layers[layer_i], rng);
        }
        init_weights(device, network.output_layer, rng);
    }

    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT constexpr auto output(DEVICE& device, nn_models::mlp::NeuralNetworkForward<SPEC>& m){
        auto tensor_flat = to_tensor(device, m.output_layer.output);
        auto tensor = view_memory<typename SPEC::OUTPUT_SHAPE>(device, tensor_flat);
        return tensor;
    }

    // evaluate does not set intermediate outputs and hence can also be called from stateless layers, for register efficiency use forward when working with "Backward" compatible layers

    namespace nn_models::mlp{
        template <typename MODEL_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC>
        RL_TOOLS_FUNCTION_PLACEMENT constexpr bool check_input_output_f(){
            static_assert(INPUT_SPEC::COLS == MODEL_SPEC::INPUT_DIM);
            static_assert(INPUT_SPEC::ROWS == OUTPUT_SPEC::ROWS);
            static_assert(OUTPUT_SPEC::COLS == MODEL_SPEC::OUTPUT_DIM);
            // static_assert(utils::typing::is_same_v<typename MODEL_SPEC::T, typename INPUT_SPEC::T>);
            // static_assert(utils::typing::is_same_v<typename INPUT_SPEC::T, typename OUTPUT_SPEC::T>);
            return true;
        }
        template <typename MODEL_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC>
        constexpr bool check_input_output = check_input_output_f<MODEL_SPEC, INPUT_SPEC, OUTPUT_SPEC>();
    }
    template<typename DEVICE, typename MODEL_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename BUFFER_MODEL_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void evaluate(DEVICE& device, const nn_models::mlp::NeuralNetworkForward<MODEL_SPEC>& network, const Matrix<INPUT_SPEC>& input, Matrix<OUTPUT_SPEC>& output, nn_models::mlp::NeuralNetworkBuffers<BUFFER_MODEL_SPEC>& buffers, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        static_assert(nn_models::mlp::check_input_output<MODEL_SPEC, INPUT_SPEC, OUTPUT_SPEC>);
        constexpr auto BATCH_SIZE = INPUT_SPEC::ROWS;
        static_assert(OUTPUT_SPEC::ROWS >= BATCH_SIZE);
        static_assert(BUFFER_MODEL_SPEC::DIM >= MODEL_SPEC::HIDDEN_DIM);
        matrix::ViewSpec<BATCH_SIZE, MODEL_SPEC::HIDDEN_DIM> hidden_vs;
        {
            auto output_buffer_view = view(device, buffers.tick, matrix::ViewSpec<BATCH_SIZE, MODEL_SPEC::HIDDEN_DIM>{});
            evaluate(device, network.input_layer, input, output_buffer_view, buffers.layer_buffer, rng, mode);
        }
        for (typename DEVICE::index_t layer_i = 0; layer_i < MODEL_SPEC::NUM_HIDDEN_LAYERS; layer_i++){
            auto& input_buffer = (layer_i % 2 == 0) ? buffers.tick : buffers.tock;
            auto input_buffer_view = view(device, input_buffer, hidden_vs);
            auto& output_buffer = (layer_i % 2 == 0) ? buffers.tock : buffers.tick;
            auto output_buffer_view = view(device, output_buffer, hidden_vs);
            evaluate(device, network.hidden_layers[layer_i], input_buffer_view, output_buffer_view, buffers.layer_buffer, rng, mode);
        }
        if constexpr(MODEL_SPEC::NUM_HIDDEN_LAYERS % 2 == 0){
            auto input_buffer_view = view(device, buffers.tick, hidden_vs);
            evaluate(device, network.output_layer, input_buffer_view, output, buffers.layer_buffer, rng, mode);
        } else {
            auto input_buffer_view = view(device, buffers.tock, hidden_vs);
            evaluate(device, network.output_layer, input_buffer_view, output, buffers.layer_buffer, rng, mode);
        }
    }
    template<typename DEVICE, typename MODEL_SPEC, typename INPUT_SPEC, typename BUFFER_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void forward(DEVICE& device, nn_models::mlp::NeuralNetworkGradient<MODEL_SPEC>& network, const Matrix<INPUT_SPEC>& input, nn_models::mlp::NeuralNetworkBuffers<BUFFER_SPEC>& buffers, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        nn::layers::dense::Buffer layer_buffer;
        forward(device, network.input_layer, input, layer_buffer, rng, mode);

        auto current_output = network.input_layer.output;
        for (typename DEVICE::index_t layer_i = 0; layer_i < MODEL_SPEC::NUM_HIDDEN_LAYERS; layer_i++){
            forward(device, network.hidden_layers[layer_i], current_output, layer_buffer, rng, mode);
            current_output = network.hidden_layers[layer_i].output;
        }
        forward(device, network.output_layer, current_output, layer_buffer, rng, mode);
    }
    template<typename DEVICE, typename MODEL_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename BUFFER_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void forward(DEVICE& device, nn_models::mlp::NeuralNetworkGradient<MODEL_SPEC>& network, const Matrix<INPUT_SPEC>& input, Matrix<OUTPUT_SPEC>& output, nn_models::mlp::NeuralNetworkBuffers<BUFFER_SPEC>& buffers, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        static_assert(nn_models::mlp::check_input_output<MODEL_SPEC, INPUT_SPEC, OUTPUT_SPEC>);
        forward(device, network, input, buffers, rng, mode);
        copy(device, device, network.output_layer.output, output);
    }

    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void zero_gradient(DEVICE& device, nn_models::mlp::NeuralNetworkGradient<SPEC>& network) {
        zero_gradient(device, network.input_layer);
        for(typename DEVICE::index_t i = 0; i < SPEC::NUM_HIDDEN_LAYERS; i++){
            zero_gradient(device, network.hidden_layers[i]);
        }
        zero_gradient(device, network.output_layer);
    }
    template<typename DEVICE, typename MODEL_SPEC, typename D_OUTPUT_SPEC, typename D_INPUT_SPEC, typename BUFFER_MODEL_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void backward_input(DEVICE& device, nn_models::mlp::NeuralNetworkBackward<MODEL_SPEC>& network, Matrix<D_OUTPUT_SPEC>& d_output, Matrix<D_INPUT_SPEC>& d_input, nn_models::mlp::NeuralNetworkBuffers<BUFFER_MODEL_SPEC>& buffer, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        // ATTENTION: this modifies d_output (uses it as a buffer for the d_pre_activations
        static_assert(nn_models::mlp::check_input_output<MODEL_SPEC, D_INPUT_SPEC, D_OUTPUT_SPEC>);
        constexpr auto BATCH_SIZE = D_INPUT_SPEC::ROWS;
        static_assert(BUFFER_MODEL_SPEC::INTERNAL_BATCH_SIZE == BATCH_SIZE);
        static_assert(BUFFER_MODEL_SPEC::DIM >= MODEL_SPEC::HIDDEN_DIM);
        using TI = typename DEVICE::index_t;

        backward_input(device, network.output_layer, d_output, buffer.tick, buffer.layer_buffer, mode);
        for (TI layer_i_plus_one = MODEL_SPEC::NUM_HIDDEN_LAYERS; layer_i_plus_one > 0; layer_i_plus_one--){
            TI layer_i = layer_i_plus_one - 1;
            if(layer_i % 2 == (MODEL_SPEC::NUM_HIDDEN_LAYERS - 1) % 2){ // we are starting with the last hidden layer where the result should go to tock
                backward_input(device, network.hidden_layers[layer_i], buffer.tick, buffer.tock, buffer.layer_buffer, mode);
            } else {
                backward_input(device, network.hidden_layers[layer_i], buffer.tock, buffer.tick, buffer.layer_buffer, mode);
            }
        }
        auto& target_d_output_buffer = (MODEL_SPEC::NUM_HIDDEN_LAYERS % 2 == 0) ? buffer.tick : buffer.tock;
        backward_input(device, network.input_layer, target_d_output_buffer, d_input, buffer.layer_buffer, mode);
    }

    template<typename DEVICE, typename MODEL_SPEC, typename INPUT_SPEC, typename D_OUTPUT_SPEC, typename BUFFER_MODEL_SPEC, typename D_INPUT_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void backward_full(DEVICE& device, nn_models::mlp::NeuralNetworkGradient<MODEL_SPEC>& network, const Matrix<INPUT_SPEC>& input, Matrix<D_OUTPUT_SPEC>& d_output, Matrix<D_INPUT_SPEC>& d_input, nn_models::mlp::NeuralNetworkBuffers<BUFFER_MODEL_SPEC>& buffer, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        // ATTENTION: this modifies d_output (uses it as a buffer for the d_pre_activations
        static_assert(nn_models::mlp::check_input_output<MODEL_SPEC, D_INPUT_SPEC, D_OUTPUT_SPEC>);
        static_assert(nn_models::mlp::check_input_output<MODEL_SPEC, INPUT_SPEC, D_OUTPUT_SPEC>);
        constexpr auto BATCH_SIZE = D_INPUT_SPEC::ROWS;
        static_assert(BUFFER_MODEL_SPEC::INTERNAL_BATCH_SIZE == BATCH_SIZE);
        static_assert(BUFFER_MODEL_SPEC::DIM >= MODEL_SPEC::HIDDEN_DIM);

        {
            auto& previous_output = MODEL_SPEC::NUM_HIDDEN_LAYERS > 0 ? network.hidden_layers[MODEL_SPEC::NUM_HIDDEN_LAYERS - 1].output : network.input_layer.output;
            backward_full(device, network.output_layer, previous_output, d_output, buffer.tick, buffer.layer_buffer, mode);
        }
        for (typename DEVICE::index_t layer_i_plus_one = MODEL_SPEC::NUM_HIDDEN_LAYERS; layer_i_plus_one > 0; layer_i_plus_one--){
            typename DEVICE::index_t layer_i = layer_i_plus_one - 1;
            auto& previous_output_hidden = layer_i > 0 ? network.hidden_layers[layer_i - 1].output : network.input_layer.output;
            if(layer_i % 2 == (MODEL_SPEC::NUM_HIDDEN_LAYERS - 1) % 2){ // we are starting with the last hidden layer where the result should go to tock
                backward_full(device, network.hidden_layers[layer_i], previous_output_hidden, buffer.tick, buffer.tock, buffer.layer_buffer, mode);
            } else {
                backward_full(device, network.hidden_layers[layer_i], previous_output_hidden, buffer.tock, buffer.tick, buffer.layer_buffer, mode);
            }
        }
        if constexpr(MODEL_SPEC::NUM_HIDDEN_LAYERS % 2 == 0){
            backward_full(device, network.input_layer, input, buffer.tick, d_input, buffer.layer_buffer, mode);
        } else {
            backward_full(device, network.input_layer, input, buffer.tock, d_input, buffer.layer_buffer, mode);
        }
    }
    template<typename DEVICE, typename MODEL_SPEC, typename INPUT_SPEC, typename D_OUTPUT_SPEC, typename BUFFER_MODEL_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void backward(DEVICE& device, nn_models::mlp::NeuralNetworkGradient<MODEL_SPEC>& network, const Matrix<INPUT_SPEC>& input, Matrix<D_OUTPUT_SPEC>& d_output, nn_models::mlp::NeuralNetworkBuffers<BUFFER_MODEL_SPEC>& buffer, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        // ATTENTION: this modifies d_output (uses it as a buffer for the d_pre_activations
        static_assert(nn_models::mlp::check_input_output<MODEL_SPEC, INPUT_SPEC, D_OUTPUT_SPEC>);
        constexpr auto BATCH_SIZE = D_OUTPUT_SPEC::ROWS;
        static_assert(BUFFER_MODEL_SPEC::INTERNAL_BATCH_SIZE == BATCH_SIZE);
        static_assert(BUFFER_MODEL_SPEC::DIM >= MODEL_SPEC::HIDDEN_DIM);

        auto previous_output = MODEL_SPEC::NUM_HIDDEN_LAYERS > 0 ? network.hidden_layers[MODEL_SPEC::NUM_HIDDEN_LAYERS - 1].output : network.input_layer.output;
        backward_full(device, network.output_layer, previous_output, d_output, buffer.tick, buffer.layer_buffer, mode);
        for (typename DEVICE::index_t layer_i_plus_one = MODEL_SPEC::NUM_HIDDEN_LAYERS; layer_i_plus_one > 0; layer_i_plus_one--){
            typename DEVICE::index_t layer_i = layer_i_plus_one - 1;
            previous_output = layer_i > 0 ? network.hidden_layers[layer_i - 1].output : network.input_layer.output;
            if(layer_i % 2 == (MODEL_SPEC::NUM_HIDDEN_LAYERS - 1) % 2){ // we are starting with the last hidden layer where the result should go to tock
                backward_full(device, network.hidden_layers[layer_i], previous_output, buffer.tick, buffer.tock, buffer.layer_buffer, mode);
            } else {
                backward_full(device, network.hidden_layers[layer_i], previous_output, buffer.tock, buffer.tick, buffer.layer_buffer, mode);
            }
        }
        if constexpr(MODEL_SPEC::NUM_HIDDEN_LAYERS % 2 == 0){
            backward(device, network.input_layer, input, buffer.tick, buffer.layer_buffer, mode);
        } else {
            backward(device, network.input_layer, input, buffer.tock, buffer.layer_buffer, mode);
        }
    }
//    template<typename DEVICE, typename MODEL_SPEC, typename INPUT_SPEC, typename D_OUTPUT_SPEC, typename BUFFER_MODEL_SPEC>
//    void backward_param(DEVICE& device, nn_models::mlp::NeuralNetworkGradient<MODEL_SPEC>& network, const Matrix<INPUT_SPEC>& input, Matrix<D_OUTPUT_SPEC>& d_output, nn_models::mlp::NeuralNetworkBuffers<BUFFER_MODEL_SPEC> buffer) {
//        backward(device, network, input, d_output, buffer);
//    }

    template<typename DEVICE, typename SPEC, typename ADAM_PARAMETERS>
    RL_TOOLS_FUNCTION_PLACEMENT void update(DEVICE& device, nn_models::mlp::NeuralNetworkGradient<SPEC>& network, nn::optimizers::Adam<ADAM_PARAMETERS>& optimizer) {
        update(device, network.input_layer, optimizer);
        for(typename DEVICE::index_t layer_i = 0; layer_i < SPEC::NUM_HIDDEN_LAYERS; layer_i++){
            update(device, network.hidden_layers[layer_i], optimizer);
        }
        update(device, network.output_layer, optimizer);
    }

    template<typename DEVICE, typename SPEC, typename OPTIMIZER>
    RL_TOOLS_FUNCTION_PLACEMENT void _reset_optimizer_state(DEVICE& device, nn_models::mlp::NeuralNetworkGradient<SPEC>& network, OPTIMIZER& optimizer) {
        // this function is marked with a underscore because it should usually be called from the reset_optimizer_state function of the optimizer to have one coherent entrypoint for resetting the optimizer state in the optimizer and in the model
        _reset_optimizer_state(device, network.input_layer, optimizer);
        for(typename DEVICE::index_t layer_i = 0; layer_i < SPEC::NUM_HIDDEN_LAYERS; layer_i++){
            _reset_optimizer_state(device, network.hidden_layers[layer_i], optimizer);
        }
        _reset_optimizer_state(device, network.output_layer, optimizer);
    }

    // The following copy operators are more powerful than the default copy assignment operator in that they can e.g. copy between networks with different activation functions
    template<typename SOURCE_DEVICE, typename TARGET_DEVICE,  typename SOURCE_SPEC, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, const  nn_models::mlp::NeuralNetworkForward<SOURCE_SPEC>& source, nn_models::mlp::NeuralNetworkForward<TARGET_SPEC>& target){
        static_assert(rl_tools::nn_models::mlp::check_spec_memory<SOURCE_SPEC, TARGET_SPEC>, "The source and target network must have the same structure");
        copy(source_device, target_device, source.input_layer, target.input_layer);
        for(typename SOURCE_SPEC::TI layer_i = 0; layer_i <  SOURCE_SPEC::NUM_HIDDEN_LAYERS; layer_i++){
            copy(source_device, target_device, source.hidden_layers[layer_i], target.hidden_layers[layer_i]);
        }
        copy(source_device, target_device, source.output_layer, target.output_layer);
    }
    template<typename SOURCE_DEVICE, typename TARGET_DEVICE,  typename SOURCE, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy_from_generic(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, const SOURCE& source, nn_models::mlp::NeuralNetworkForward<TARGET_SPEC>& target){
        static_assert(rl_tools::nn_models::mlp::check_spec_memory<SOURCE, TARGET_SPEC>, "The source and target network must have the same structure");
        copy(source_device, target_device, source.input_layer, target.input_layer);
        for(typename SOURCE::TI layer_i = 0; layer_i <  SOURCE::NUM_HIDDEN_LAYERS; layer_i++){
            copy(source_device, target_device, source.hidden_layers[layer_i], target.hidden_layers[layer_i]);
        }
        copy(source_device, target_device, source.output_layer, target.output_layer);
    }

//    template<typename SOURCE_DEVICE, typename TARGET_DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC>
//    void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, const  nn_models::mlp::NeuralNetworkAdam<SOURCE_SPEC>& source, nn_models::mlp::NeuralNetworkAdam<TARGET_SPEC>& target){
//        static_assert(rl_tools::nn_models::mlp::check_spec_memory<SOURCE_SPEC, TARGET_SPEC>, "The source and target network must have the same structure");
//        copy(source_device, target_device, (nn_models::mlp::NeuralNetworkForward<SOURCE_SPEC>&)source, (nn_models::mlp::NeuralNetworkForward<TARGET_SPEC>&)target);
//    }

    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void reset_forward_state(DEVICE& device, nn_models::mlp::NeuralNetworkForward<SPEC>& n){
        reset_forward_state(device, n.input_layer);
        for(typename DEVICE::index_t layer_i = 0; layer_i <  SPEC::NUM_HIDDEN_LAYERS; layer_i++){
            reset_forward_state(device, n.hidden_layers[layer_i]);
        }
        reset_forward_state(device, n.output_layer);
    }

    template<typename DEVICE, typename SPEC_1, typename SPEC_2>
    RL_TOOLS_FUNCTION_PLACEMENT typename SPEC_1::TYPE_POLICY::DEFAULT abs_diff(DEVICE& device, nn_models::mlp::NeuralNetworkForward<SPEC_1>& n1, const nn_models::mlp::NeuralNetworkForward<SPEC_2>& n2){
        using T = typename SPEC_1::TYPE_POLICY::DEFAULT;
        static_assert(rl_tools::nn_models::mlp::check_spec_memory<SPEC_1, SPEC_2>, "The source and target network must have the same structure");
        T acc = 0;

        acc += abs_diff(device, n1.output_layer, n2.output_layer);
        for(typename DEVICE::index_t layer_i = 0; layer_i < SPEC_1::NUM_HIDDEN_LAYERS; layer_i++){
            acc += abs_diff(device, n1.hidden_layers[layer_i], n2.hidden_layers[layer_i]);
        }
        acc += abs_diff(device, n1.input_layer, n2.input_layer);
        return acc;
    }
    template <typename DEVICE, typename SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT bool is_nan(DEVICE& device, const rl_tools::nn_models::mlp::NeuralNetworkForward<SPEC>& n, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        bool found_nan = false;
        found_nan = found_nan || is_nan(device, n.input_layer, mode);
        for(typename DEVICE::index_t layer_i = 0; layer_i < SPEC::NUM_HIDDEN_LAYERS; layer_i++){
            found_nan = found_nan || is_nan(device, n.hidden_layers[layer_i], mode);
        }
        found_nan = found_nan || is_nan(device, n.output_layer, mode);
        return found_nan;
    }
    template<typename SOURCE_DEVICE, typename TARGET_DEVICE,  typename SOURCE_SPEC, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, const  nn_models::mlp::NeuralNetworkBuffers<SOURCE_SPEC>& source, nn_models::mlp::NeuralNetworkBuffers<TARGET_SPEC>& target){
        copy(source_device, target_device, source.tick, target.tick);
        copy(source_device, target_device, source.tock, target.tock);
    }
    template<typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT constexpr auto& output(nn_models::mlp::NeuralNetworkGradient<SPEC>& nn){
        return nn.output_layer.output;
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT constexpr auto output(DEVICE& device, nn_models::mlp::NeuralNetworkGradient<SPEC>& nn){
        return output(device, nn.output_layer);
    }
    template <typename DEVICE, typename BUFFER_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT void sample(DEVICE& device, nn_models::mlp::NeuralNetworkBuffers<BUFFER_SPEC>& buffers, RNG& rng){ }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

// Tensor proxies
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template<typename DEVICE, typename SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename BUFFER_MODEL_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void evaluate(DEVICE& device, const nn_models::mlp::NeuralNetworkForward<SPEC>& model, const Tensor<INPUT_SPEC>& input, Tensor<OUTPUT_SPEC>& output,nn_models::mlp::NeuralNetworkBuffers<BUFFER_MODEL_SPEC>& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        auto matrix_view_input = matrix_view(device, input);
        auto matrix_view_output = matrix_view(device, output);
        evaluate(device, model, matrix_view_input, matrix_view_output, buffer, rng, mode);
    }
    template<typename DEVICE, typename SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename BUFFER_MODEL_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void evaluate_step(DEVICE& device, const nn_models::mlp::NeuralNetworkForward<SPEC>& model, const Tensor<INPUT_SPEC>& input, nn_models::mlp::State, Tensor<OUTPUT_SPEC>& output,nn_models::mlp::NeuralNetworkBuffers<BUFFER_MODEL_SPEC>& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        auto matrix_view_input = matrix_view(device, input);
        auto matrix_view_output = matrix_view(device, output);
        evaluate(device, model, matrix_view_input, matrix_view_output, buffer, rng, mode);
    }
    template<typename DEVICE, typename SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename BUFFER_MODEL_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void forward(DEVICE& device, nn_models::mlp::NeuralNetworkBackward<SPEC>& model, const Tensor<INPUT_SPEC>& input, Tensor<OUTPUT_SPEC>& output,nn_models::mlp::NeuralNetworkBuffers<BUFFER_MODEL_SPEC>& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        auto matrix_view_input = matrix_view(device, input);
        auto matrix_view_output = matrix_view(device, output);
        forward(device, model, matrix_view_input, matrix_view_output, buffer, rng, mode);
    }
    template<typename DEVICE, typename SPEC, typename INPUT_SPEC, typename BUFFER_MODEL_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void forward(DEVICE& device, nn_models::mlp::NeuralNetworkGradient<SPEC>& model, const Tensor<INPUT_SPEC>& input,nn_models::mlp::NeuralNetworkBuffers<BUFFER_MODEL_SPEC>& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        auto matrix_view_input = matrix_view(device, input);
        forward(device, model, matrix_view_input, buffer, rng, mode);
    }
    template<typename DEVICE, typename SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename BUFFER_MODEL_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void forward(DEVICE& device, nn_models::mlp::NeuralNetworkGradient<SPEC>& model, const Tensor<INPUT_SPEC>& input, Tensor<OUTPUT_SPEC>& output,nn_models::mlp::NeuralNetworkBuffers<BUFFER_MODEL_SPEC>& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        auto matrix_view_input = matrix_view(device, input);
        auto matrix_view_output = matrix_view(device, output);
        forward(device, model, matrix_view_input, matrix_view_output, buffer, rng, mode);
    }
    template<typename DEVICE, typename SPEC, typename D_OUTPUT_SPEC, typename D_INPUT_SPEC, typename BUFFER_MODEL_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void backward_input(DEVICE& device, nn_models::mlp::NeuralNetworkBackward<SPEC>& model, Tensor<D_OUTPUT_SPEC>& d_output, Tensor<D_INPUT_SPEC>& d_input,nn_models::mlp::NeuralNetworkBuffers<BUFFER_MODEL_SPEC>& buffer, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        auto matrix_view_d_output = matrix_view(device, d_output);
        auto matrix_view_d_input = matrix_view(device, d_input);
        backward_input(device, model, matrix_view_d_output, matrix_view_d_input, buffer, mode);
    }

    template<typename DEVICE, typename SPEC, typename INPUT_SPEC, typename D_OUTPUT_SPEC, typename BUFFER_MODEL_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void backward(DEVICE& device, nn_models::mlp::NeuralNetworkGradient<SPEC>& model, const Tensor<INPUT_SPEC>& input, Tensor<D_OUTPUT_SPEC>& d_output,nn_models::mlp::NeuralNetworkBuffers<BUFFER_MODEL_SPEC>& buffer, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        auto matrix_view_input = matrix_view(device, input);
        auto matrix_view_d_output = matrix_view(device, d_output);
        backward(device, model, matrix_view_input, matrix_view_d_output, buffer, mode);
    }

    template<typename DEVICE, typename SPEC, typename INPUT_SPEC, typename D_OUTPUT_SPEC, typename D_INPUT_SPEC, typename BUFFER_MODEL_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void backward_full(DEVICE& device, nn_models::mlp::NeuralNetworkGradient<SPEC>& model, const Tensor<INPUT_SPEC>& input, Tensor<D_OUTPUT_SPEC>& d_output, Tensor<D_INPUT_SPEC>& d_input,nn_models::mlp::NeuralNetworkBuffers<BUFFER_MODEL_SPEC>& buffer, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        auto matrix_view_input = matrix_view(device, input);
        auto matrix_view_d_output = matrix_view(device, d_output);
        auto matrix_view_d_input = matrix_view(device, d_input);
        backward_full(device, model, matrix_view_input, matrix_view_d_output, matrix_view_d_input, buffer, mode);
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT auto gradient_norm(DEVICE& device, const nn_models::mlp::NeuralNetworkForward<SPEC>& model){
        using TI = typename DEVICE::index_t;
        using T = typename SPEC::TYPE_POLICY::DEFAULT;
        T return_value = 0;
        return_value += gradient_norm(device, model.input_layer);
        for(TI layer_i = 0; layer_i < SPEC::NUM_HIDDEN_LAYERS; layer_i++){
            return_value += gradient_norm(device, model.hidden_layers[layer_i]);
        }
        return_value += gradient_norm(device, model.output_layer);
        return return_value;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif
