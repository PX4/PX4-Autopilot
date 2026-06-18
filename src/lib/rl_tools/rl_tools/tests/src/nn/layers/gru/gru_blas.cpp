#undef RL_TOOLS_ENABLE_TRACY
#include <rl_tools/operations/cpu_mux.h>
#include <rl_tools/nn/optimizers/adam/instance/operations_generic.h>
#include <rl_tools/nn/layers/gru/operations_generic.h>
#include <rl_tools/nn/operations_cpu_mux.h>
#include <rl_tools/nn/optimizers/adam/operations_generic.h>

namespace rlt = rl_tools;

#include <gtest/gtest.h>

using DEVICE_GENERIC = rlt::devices::DefaultCPU;
using TI = DEVICE_GENERIC::index_t;
using T = double;
using TYPE_POLICY = rlt::numeric_types::Policy<T>;
using DEVICE_BLAS = rlt::devices::DEVICE_FACTORY<>;
constexpr TI SEQUENCE_LENGTH = 128;
constexpr TI INPUT_DIM = 10;
constexpr TI HIDDEN_DIM = 15;
constexpr TI BATCH_SIZE = 15;

TEST(RL_TOOLS_NN_LAYERS_GRU, BLAS){

    using INPUT_SHAPE = rlt::tensor::Shape<TI, SEQUENCE_LENGTH, BATCH_SIZE, INPUT_DIM>;
    using CONFIG = rlt::nn::layers::gru::Configuration<TYPE_POLICY, TI, HIDDEN_DIM>;
    using CAPABILITY = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam>;
    using GRU = rlt::nn::layers::gru::Layer<CONFIG, CAPABILITY, INPUT_SHAPE>;
    GRU gru_generic, gru_blas;
    typename GRU::template Buffer<true> buffer;

    DEVICE_GENERIC device_generic;
    DEVICE_BLAS device_blas;
    DEVICE_GENERIC::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device_generic, rng);
    rlt::init(device_generic, rng, 0);

    using INPUT_SPEC = rlt::tensor::Specification<T, TI, GRU::INPUT_SHAPE>;
    using INPUT = rlt::Tensor<INPUT_SPEC>;
    INPUT input, d_input_generic, d_input_blas;
    using OUTPUT_SPEC = rlt::tensor::Specification<T, TI, GRU::OUTPUT_SHAPE>;
    using OUTPUT = rlt::Tensor<OUTPUT_SPEC>;
    OUTPUT output, d_output_generic, d_output_blas;

    rlt::malloc(device_generic, gru_generic);
    rlt::malloc(device_generic, gru_blas);
    rlt::malloc(device_generic, buffer);
    rlt::malloc(device_generic, input);
    rlt::malloc(device_generic, d_input_generic);
    rlt::malloc(device_generic, d_input_blas);
    rlt::malloc(device_generic, output);
    rlt::malloc(device_generic, d_output_generic);
    rlt::malloc(device_generic, d_output_blas);

    rlt::init_weights(device_generic, gru_generic, rng);
    rlt::copy(device_generic, device_blas, gru_generic, gru_blas);

    rlt::randn(device_generic, input, rng);
    rlt::forward(device_generic, gru_generic, input, buffer, rng);
    rlt::forward(device_blas, gru_blas, input, buffer, rng);

    T forward_diff = rlt::abs_diff(device_generic, rlt::output(device_generic, gru_generic), rlt::output(device_blas, gru_blas));
    std::cout << "Forward diff: " << forward_diff << std::endl;
    ASSERT_LT(forward_diff, 1e-10);

    rlt::zero_gradient(device_generic, gru_generic);
    rlt::zero_gradient(device_generic, gru_blas);
    rlt::randn(device_generic, d_output_generic, rng);
    rlt::copy(device_generic, device_blas, d_output_generic, d_output_blas);

    rlt::backward_full(device_generic, gru_generic, input, d_output_generic, d_input_generic, buffer);
    rlt::backward_full(device_blas, gru_blas, input, d_output_blas, d_input_blas, buffer);

    T backward_diff = rlt::abs_diff(device_generic, d_input_generic, d_input_blas) / decltype(d_input_generic)::SPEC::SIZE;
    std::cout << "Backward diff: " << backward_diff << std::endl;
    ASSERT_LT(backward_diff, 1e-10);
}

