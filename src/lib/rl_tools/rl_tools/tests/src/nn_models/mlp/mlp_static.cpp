#include <rl_tools/operations/cpu.h>
#include <rl_tools/nn/optimizers/adam/instance/operations_generic.h>
#include <rl_tools/nn_models/operations_generic.h>
#include <rl_tools/nn/optimizers/adam/operations_generic.h>

#include <gtest/gtest.h>

namespace rlt = rl_tools;

using DEVICE = rlt::devices::DefaultCPU;
using TI = typename DEVICE::index_t;
using T = double;
using TYPE_POLICY = rlt::numeric_types::Policy<T>;

constexpr TI BATCH_SIZE = 1;
using INPUT_SHAPE = rlt::tensor::Shape<TI, 1, BATCH_SIZE, 13>;
using CONFIG = rlt::nn_models::mlp::Configuration<TYPE_POLICY, TI, 12, 3, 5, rlt::nn::activation_functions::RELU, rlt::nn::activation_functions::IDENTITY>;

using OPTIMIZER_SPEC = rlt::nn::optimizers::adam::Specification<TYPE_POLICY, TI>;
using OPTIMIZER = rlt::nn::optimizers::Adam<OPTIMIZER_SPEC>;
using CAPABILITY_ADAM_DYNAMIC = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam, true>;
using CAPABILITY_ADAM_STATIC = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam, false>;
using MLP_DYNAMIC = rlt::nn_models::mlp::NeuralNetwork<CONFIG, CAPABILITY_ADAM_DYNAMIC, INPUT_SHAPE>;
using MLP_STATIC = rlt::nn_models::mlp::NeuralNetwork<CONFIG, CAPABILITY_ADAM_STATIC, INPUT_SHAPE>;


TEST(RL_TOOLS_NN_MODELS_MLP, STATIC){
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);

    MLP_DYNAMIC mlp_dynamic;
    MLP_DYNAMIC::Buffer<true> mlp_dynamic_buffer;
    MLP_STATIC mlp_static;
    MLP_STATIC::Buffer<true> mlp_static_buffer;
    OPTIMIZER optimizer_dynamic, optimizer_static;

    rlt::malloc(device, optimizer_dynamic);
    rlt::malloc(device, optimizer_static);
    rlt::malloc(device, mlp_dynamic);
    rlt::malloc(device, mlp_dynamic_buffer);
    rlt::malloc(device, mlp_static_buffer);

    rlt::init(device, optimizer_dynamic);
    rlt::init(device, optimizer_static);

    rlt::init_weights(device, mlp_dynamic, rng);

    rlt::copy(device, device, mlp_dynamic, mlp_static);

    rlt::Tensor<rlt::tensor::Specification<T, TI, INPUT_SHAPE, false>> input;
    rlt::Tensor<rlt::tensor::Specification<T, TI, MLP_DYNAMIC::OUTPUT_SHAPE, false>> output_dynamic, output_static, d_output;

    rlt::randn(device, input, rng);
    rlt::randn(device, d_output, rng);

    rlt::forward(device, mlp_dynamic, input, mlp_dynamic_buffer, rng);
    rlt::forward(device, mlp_static, input, mlp_static_buffer, rng);

    T forward_diff = rlt::abs_diff(device, rlt::output(device, mlp_dynamic), rlt::output(device, mlp_static));
    std::cout << "Forward diff: " << forward_diff << std::endl;
    ASSERT_LT(forward_diff, 1e-10);

    rlt::zero_gradient(device, mlp_dynamic);
    rlt::zero_gradient(device, mlp_static);
    rlt::reset_optimizer_state(device, optimizer_dynamic, mlp_dynamic);
    rlt::reset_optimizer_state(device, optimizer_static, mlp_static);
    rlt::backward(device, mlp_dynamic, input, d_output, mlp_dynamic_buffer);
    rlt::backward(device, mlp_static, input, d_output, mlp_static_buffer);

    T backward_diff = rlt::abs_diff(device, mlp_dynamic, mlp_static);
    std::cout << "Backward diff: " << backward_diff << std::endl;
    ASSERT_LT(backward_diff, 1e-10);

    rlt::print(device, mlp_dynamic.output_layer.biases.parameters);
    std::cout << std::endl;
    rlt::print(device, mlp_static.output_layer.biases.parameters);

    rlt::step(device, optimizer_dynamic, mlp_dynamic);
    rlt::step(device, optimizer_static, mlp_static);

    rlt::print(device, mlp_dynamic.output_layer.biases.parameters);
    std::cout << std::endl;
    rlt::print(device, mlp_static.output_layer.biases.parameters);

    T step_diff = rlt::abs_diff(device, mlp_dynamic, mlp_static);
    std::cout << "Step diff: " << step_diff << std::endl;
    ASSERT_LT(step_diff, 1e-10);
}
