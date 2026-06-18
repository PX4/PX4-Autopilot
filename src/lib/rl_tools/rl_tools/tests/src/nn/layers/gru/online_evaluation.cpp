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
TEST(RL_TOOLS_NN_LAYERS_GRU, ONLINE_EVALUATION){
#else
int main(){
#endif
//    using CAPABILITY = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam, BATCH_SIZE>;
    using CAPABILITY = rlt::nn::capability::Forward<>;

    using INPUT_SHAPE = rlt::tensor::Shape<TI, SEQUENCE_LENGTH, BATCH_SIZE, INPUT_DIM>;
    using GRU_CONFIG = rlt::nn::layers::gru::Configuration<TYPE_POLICY, TI, HIDDEN_DIM>;
    using GRU = rlt::nn::layers::gru::Layer<GRU_CONFIG, CAPABILITY, INPUT_SHAPE>;
    GRU::State<true> gru_state;

    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);
    GRU gru;
    GRU::Buffer<true> buffer;
    rlt::Mode<rlt::mode::Evaluation<>> mode;

    rlt::Tensor<rlt::tensor::Specification<T, TI, rlt::tensor::Shape<TI, 1, BATCH_SIZE, INPUT_DIM>>> input;
    rlt::Tensor<rlt::tensor::Specification<T, TI, rlt::tensor::Shape<TI, 1, BATCH_SIZE, HIDDEN_DIM>>> output_1, output_2;

    rlt::malloc(device, input);
    rlt::malloc(device, output_1);
    rlt::malloc(device, output_2);
    rlt::malloc(device, gru);
    rlt::malloc(device, gru_state);
    rlt::malloc(device, buffer);
    rlt::init_weights(device, gru, rng);
    rlt::randn(device, input, rng);

    std::cout << "Post activation: " << std::endl;
    rlt::print(device, decltype(buffer.post_activation)::SPEC::SHAPE{});
    std::cout << std::endl;

    {
        rlt::evaluate(device, gru, input, output_1, buffer, rng, mode);
        rlt::evaluate(device, gru, input, output_2, buffer, rng, mode);
        T diff_reset = rlt::abs_diff(device, output_1, output_2);
        std::cout << "Diff reset: " << diff_reset << std::endl;
#ifndef DISABLE_TEST
        ASSERT_EQ(diff_reset, 0);
#endif
    }

    {
        rlt::reset(device, gru, gru_state, rng);
        auto input_squeezed = rlt::squeeze(device, input);
        auto output_1_squeezed = rlt::squeeze(device, output_1);
        auto output_2_squeezed = rlt::squeeze(device, output_2);
        rlt::evaluate_step(device, gru, input_squeezed, gru_state, output_1_squeezed, buffer, rng, mode);
        rlt::evaluate_step(device, gru, input_squeezed, gru_state, output_2_squeezed, buffer, rng, mode);
        T diff_reset = rlt::abs_diff(device, output_1, output_2);
        std::cout << "Diff reset: " << diff_reset << std::endl;
#ifndef DISABLE_TEST
        ASSERT_GT(diff_reset, 0);
#endif
    }

}

