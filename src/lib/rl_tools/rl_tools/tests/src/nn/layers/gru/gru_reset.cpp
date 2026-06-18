#ifdef MUX
#include <rl_tools/operations/cpu_mux.h>
#else
#include <rl_tools/operations/cpu.h>
#endif

#include <rl_tools/nn/optimizers/adam/instance/operations_generic.h>
#include <rl_tools/nn/layers/gru/operations_generic.h>
#include <rl_tools/nn/optimizers/adam/operations_generic.h>

namespace rlt = rl_tools;

#ifdef MUX
using DEVICE = rlt::devices::DEVICE_FACTORY<>;
#else
using DEVICE = rlt::devices::DefaultCPU;
#endif
using T = double;
using TYPE_POLICY = rlt::numeric_types::Policy<T>;
using TI = typename DEVICE::index_t;
constexpr TI SEQUENCE_LENGTH = 16;
constexpr TI BATCH_SIZE = 32;
constexpr TI INPUT_DIM = 8;
constexpr TI HIDDEN_DIM = 24;

#ifndef DISABLE_TEST
#include <gtest/gtest.h>
#endif

#ifndef DISABLE_TEST
TEST(RL_TOOLS_NN_LAYERS_GRU, RESET){
#else
    int main(){
#endif
    using CAPABILITY = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam>;

    using INPUT_SHAPE = rlt::tensor::Shape<TI, SEQUENCE_LENGTH, BATCH_SIZE, INPUT_DIM>;
    using INPUT_SHAPE_4X = rlt::tensor::Shape<TI, SEQUENCE_LENGTH*4, BATCH_SIZE/4, INPUT_DIM>;
    using GRU_CONFIG = rlt::nn::layers::gru::Configuration<TYPE_POLICY, TI, HIDDEN_DIM>;
    using GRU_CONFIG_4X = rlt::nn::layers::gru::Configuration<TYPE_POLICY, TI, HIDDEN_DIM>;
    using GRU = rlt::nn::layers::gru::Layer<GRU_CONFIG, CAPABILITY, INPUT_SHAPE>;
    using GRU_4X = rlt::nn::layers::gru::Layer<GRU_CONFIG_4X, CAPABILITY, INPUT_SHAPE_4X>;

    rlt::Tensor<rlt::tensor::Specification<T, TI, rlt::tensor::Shape<TI, SEQUENCE_LENGTH, BATCH_SIZE, INPUT_DIM>>> input, input_back_copy;
    rlt::Tensor<rlt::tensor::Specification<T, TI, rlt::tensor::Shape<TI, SEQUENCE_LENGTH*4, BATCH_SIZE/4, INPUT_DIM>>> input_4x;
    rlt::Tensor<rlt::tensor::Specification<T, TI, rlt::tensor::Shape<TI, SEQUENCE_LENGTH, BATCH_SIZE, 1>>> reset;
    rlt::Tensor<rlt::tensor::Specification<T, TI, rlt::tensor::Shape<TI, SEQUENCE_LENGTH*4, BATCH_SIZE/4, 1>>> reset_4x;
    rlt::Tensor<rlt::tensor::Specification<T, TI, rlt::tensor::Shape<TI, SEQUENCE_LENGTH, BATCH_SIZE, HIDDEN_DIM>>> output_reset, output_default, d_output, d_output2, d_output_copy;
    rlt::Tensor<rlt::tensor::Specification<T, TI, rlt::tensor::Shape<TI, SEQUENCE_LENGTH*4, BATCH_SIZE/4, HIDDEN_DIM>>> d_output_4x;

    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);
    GRU gru, gru_copy;
    GRU_4X gru_4x;
    GRU::Buffer<true> buffer, buffer_copy;
    GRU_4X::Buffer<true> buffer_4x;
    using RESET_MODE_SPEC = rlt::nn::layers::gru::ResetModeSpecification<TI, decltype(reset)>;
    using RESET_MODE = rlt::nn::layers::gru::ResetMode<rlt::mode::Default<>, RESET_MODE_SPEC>;
    rlt::Mode<RESET_MODE> mode;
    rlt::Mode<rlt::mode::Default<>> mode_default;

    rlt::malloc(device, input);
    rlt::malloc(device, input_4x);
    rlt::malloc(device, input_back_copy);
    rlt::malloc(device, reset);
    rlt::malloc(device, reset_4x);
    rlt::malloc(device, output_reset);
    rlt::malloc(device, output_default);
    rlt::malloc(device, d_output);
    rlt::malloc(device, d_output2);
    rlt::malloc(device, d_output_copy);
    rlt::malloc(device, d_output_4x);
    rlt::malloc(device, gru);
    rlt::malloc(device, gru_copy);
    rlt::malloc(device, gru_4x);
    rlt::malloc(device, buffer);
    rlt::malloc(device, buffer_copy);
    rlt::malloc(device, buffer_4x);
    rlt::init_weights(device, gru, rng);
    rlt::copy(device, device, gru, gru_copy);
    rlt::copy(device, device, gru, gru_4x);
    rlt::randn(device, input, rng);
    rlt::set_all(device, reset, false);
    auto reset_first_step = rlt::view(device, reset, 0);
    rlt::randn(device, d_output, rng);
    rlt::copy(device, device, d_output, d_output2);
    rlt::copy(device, device, d_output, d_output_copy);

    for(TI reset_first_step_i = 0; reset_first_step_i < 2; reset_first_step_i++){
        rlt::set_all(device, reset_first_step, reset_first_step_i == 0);
        mode.reset_container = reset;

        rlt::forward(device, gru, input, buffer, rng, mode_default);
        rlt::copy(device, device, rlt::output(device, gru), output_reset);
        rlt::forward(device, gru_copy, input, buffer_copy, rng, mode);
        rlt::copy(device, device, rlt::output(device, gru_copy), output_default);
        T diff_reset = rlt::abs_diff(device, output_reset, output_default);
        std::cout << "Diff reset: " << diff_reset << std::endl;
        ASSERT_EQ(0, diff_reset);

        rlt::zero_gradient(device, gru);
        rlt::zero_gradient(device, gru_copy);
        rlt::backward(device, gru, input, d_output_copy, buffer, mode_default);
        rlt::backward(device, gru_copy, input, d_output, buffer_copy, mode);
        T diff_acc = rlt::abs_diff(device, gru.weights_input.gradient, gru_copy.weights_input.gradient);
        diff_acc += rlt::abs_diff(device, gru.weights_hidden.gradient, gru_copy.weights_hidden.gradient);
        diff_acc += rlt::abs_diff(device, gru.biases_input.gradient, gru_copy.biases_input.gradient);
        diff_acc += rlt::abs_diff(device, gru.initial_hidden_state.gradient, gru_copy.initial_hidden_state.gradient);
        std::cout << "Diff acc: " << diff_acc << std::endl;
        ASSERT_EQ(0, diff_acc);
    }

    using RESET_MODE_4X_SPEC = rlt::nn::layers::gru::ResetModeSpecification<TI, decltype(reset_4x)>;
    using RESET_MODE_4X = rlt::nn::layers::gru::ResetMode<rlt::mode::Default<>, RESET_MODE_4X_SPEC>;
    rlt::Mode<RESET_MODE_4X> mode_4x;
    rlt::set_all(device, reset_4x, false);
    for(TI seq_i = 0; seq_i < SEQUENCE_LENGTH*4; seq_i += SEQUENCE_LENGTH){
        auto reset_view = rlt::view(device, reset_4x, seq_i);
        rlt::set_all(device, reset_view, true);
    }
    mode_4x.reset_container = reset_4x;

    for(TI seq_i=0; seq_i < SEQUENCE_LENGTH; seq_i++){
        auto input_step = rlt::view(device, input, seq_i, rlt::tensor::ViewSpec<0>{});
        auto d_output_step = rlt::view(device, d_output2, seq_i, rlt::tensor::ViewSpec<0>{});
        for(TI batch_i=0; batch_i < BATCH_SIZE; batch_i++){
            auto input_sample = rlt::view(device, input_step, batch_i, rlt::tensor::ViewSpec<0>{});
            auto d_output_sample = rlt::view(device, d_output_step, batch_i, rlt::tensor::ViewSpec<0>{});
            auto input_step_target = rlt::view(device, input_4x, (batch_i % 4) * SEQUENCE_LENGTH + seq_i, rlt::tensor::ViewSpec<0>{});
            auto d_output_step_target = rlt::view(device, d_output_4x, (batch_i % 4) * SEQUENCE_LENGTH + seq_i, rlt::tensor::ViewSpec<0>{});
            auto input_sample_target = rlt::view(device, input_step_target, batch_i / 4, rlt::tensor::ViewSpec<0>{});
            auto d_output_sample_target = rlt::view(device, d_output_step_target, batch_i / 4, rlt::tensor::ViewSpec<0>{});
            rlt::copy(device, device, input_sample, input_sample_target);
            rlt::copy(device, device, d_output_sample, d_output_sample_target);
        }
    }


    rlt::forward(device, gru, input, buffer, rng, mode_default);
    rlt::copy(device, device, rlt::output(device, gru), output_default);

    rlt::forward(device, gru_4x, input_4x, buffer_4x, rng, mode_4x);

    for(TI seq_i=0; seq_i < 4*SEQUENCE_LENGTH; seq_i++){
        auto output = rlt::output(device, gru_4x);
        auto output_step_source = rlt::view(device, output, seq_i, rlt::tensor::ViewSpec<0>{});
        auto input_step_source = rlt::view(device, input_4x, seq_i, rlt::tensor::ViewSpec<0>{});
        for(TI batch_i=0; batch_i < BATCH_SIZE/4; batch_i++){
            auto output_sample_source = rlt::view(device, output_step_source, batch_i, rlt::tensor::ViewSpec<0>{});
            auto input_sample_source = rlt::view(device, input_step_source, batch_i, rlt::tensor::ViewSpec<0>{});
            auto output_step_target = rlt::view(device, output_reset, seq_i % SEQUENCE_LENGTH, rlt::tensor::ViewSpec<0>{});
            auto input_step_target = rlt::view(device, input_back_copy, seq_i % SEQUENCE_LENGTH, rlt::tensor::ViewSpec<0>{});
            auto output_sample_target = rlt::view(device, output_step_target, batch_i * 4 + seq_i / SEQUENCE_LENGTH, rlt::tensor::ViewSpec<0>{});
            auto input_sample_target = rlt::view(device, input_step_target, batch_i * 4 + seq_i / SEQUENCE_LENGTH, rlt::tensor::ViewSpec<0>{});
            rlt::copy(device, device, output_sample_source, output_sample_target);
            rlt::copy(device, device, input_sample_source, input_sample_target);
        }
    }
    T diff_input_back_copy = rlt::abs_diff(device, input_back_copy, input);
    std::cout << "Diff input_back_copy: " << diff_input_back_copy << std::endl;
    ASSERT_EQ(0, diff_input_back_copy);

    for(TI batch_i=0; batch_i < BATCH_SIZE; batch_i++){
        auto first_sequence_default = rlt::view(device, output_default, batch_i, rlt::tensor::ViewSpec<1>{});
        auto first_sequence_reset = rlt::view(device, output_reset, batch_i, rlt::tensor::ViewSpec<1>{});
//        std::cout << "First sequence default:" << std::endl;
//        rlt::print(device, first_sequence_default);
//        std::cout << "First sequence reset:" << std::endl;
//        rlt::print(device, first_sequence_reset);
        T diff_first_sequence = rlt::abs_diff(device, first_sequence_default, first_sequence_reset);
        std::cout << "Diff " << batch_i << " sequence: " << diff_first_sequence << std::endl;
        ASSERT_EQ(0, diff_first_sequence);
    }

    T diff_reset = rlt::abs_diff(device, output_reset, output_default);
    std::cout << "Diff reset: " << diff_reset << std::endl;
    ASSERT_EQ(0, diff_reset);

    // backward

    {
        rlt::zero_gradient(device, gru);
        rlt::zero_gradient(device, gru_4x);
        rlt::backward(device, gru, input, d_output2, buffer, mode_default);
        rlt::backward(device, gru_4x, input_4x, d_output_4x, buffer_4x, mode_4x);

        T diff_acc = rlt::abs_diff(device, gru.weights_input.gradient, gru_4x.weights_input.gradient);
        diff_acc += rlt::abs_diff(device, gru.weights_hidden.gradient, gru_4x.weights_hidden.gradient);
        diff_acc += rlt::abs_diff(device, gru.biases_input.gradient, gru_4x.biases_input.gradient);
        diff_acc += rlt::abs_diff(device, gru.initial_hidden_state.gradient, gru_4x.initial_hidden_state.gradient);
        T diff_normalized = diff_acc/(HIDDEN_DIM * HIDDEN_DIM * 3 + HIDDEN_DIM * INPUT_DIM * 3);
        std::cout << "Backward Diff acc: " << diff_normalized << std::endl;
        ASSERT_LT(diff_normalized, 1e-13);
    }
    {
        rlt::zero_gradient(device, gru);
        rlt::zero_gradient(device, gru_4x);
        rlt::backward(device, gru, input, d_output2, buffer, mode_default);
        rlt::backward(device, gru_4x, input_4x, d_output_4x, buffer_4x, mode_default); // ignoring reset

        T diff_acc = rlt::abs_diff(device, gru.weights_input.gradient, gru_4x.weights_input.gradient);
        diff_acc += rlt::abs_diff(device, gru.weights_hidden.gradient, gru_4x.weights_hidden.gradient);
        diff_acc += rlt::abs_diff(device, gru.biases_input.gradient, gru_4x.biases_input.gradient);
        diff_acc += rlt::abs_diff(device, gru.initial_hidden_state.gradient, gru_4x.initial_hidden_state.gradient);
        T diff_normalized = diff_acc/(HIDDEN_DIM * HIDDEN_DIM * 3 + HIDDEN_DIM * INPUT_DIM * 3);
        std::cout << "Backward Diff acc: " << diff_normalized << std::endl;
        ASSERT_GT(diff_normalized, 1e-2);
    }
}
