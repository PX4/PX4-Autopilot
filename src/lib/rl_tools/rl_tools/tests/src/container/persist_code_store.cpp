#include <rl_tools/operations/cpu.h>
#include <rl_tools/numeric_types/persist_code.h>
#include <rl_tools/containers/matrix/persist_code.h>
#include <rl_tools/containers/tensor/persist_code.h>
#include <rl_tools/nn/optimizers/adam/instance/operations_generic.h>
#include <rl_tools/nn/optimizers/adam/instance/persist_code.h>
#include <rl_tools/nn/parameters/persist_code.h>
#include <rl_tools/nn/layers/dense/operations_cpu.h>
#include <rl_tools/nn/layers/dense/persist_code.h>
#include <rl_tools/nn/optimizers/adam/instance/persist_code.h>
#include <rl_tools/nn_models/mlp/operations_generic.h>
#include <rl_tools/nn_models/mlp/persist_code.h>
#include <rl_tools/nn/optimizers/adam/operations_generic.h>

namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools;


#include <gtest/gtest.h>
#include <iostream>
#include <fstream>
#include <filesystem>


constexpr bool const_declaration = true;


TEST(RL_TOOLS_CONTAINER_PERSIST_CODE_STORE, TEST){
    using DEVICE = rlt::devices::DefaultCPU;
    using DTYPE = float;
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);
    rlt::Matrix<rlt::matrix::Specification<DTYPE, typename DEVICE::index_t, 3, 3>> m;
    rlt::malloc(device, m);
    rlt::randn(device, m, rng);
    rlt::print(device, m);
    auto output = rlt::save_code(device, m, "matrix_1", const_declaration);
    std::cout << "output: " << output << std::endl;
    std::filesystem::create_directories("data");
    std::ofstream file;
    file.open ("data/test_rl_tools_container_persist_matrix.h");
    file << output;
    file.close();

    ASSERT_TRUE(true);
}

TEST(RL_TOOLS_CONTAINER_PERSIST_CODE_STORE, TEST_DENSE_LAYER){
    using DEVICE = rlt::devices::DefaultCPU;
    using TI = DEVICE::index_t;
    using DTYPE = float;
    using TYPE_POLICY = rlt::numeric_types::Policy<DTYPE>;
    using OPTIMIZER_SPEC = rlt::nn::optimizers::adam::Specification<TYPE_POLICY, TI>;
    using OPTIMIZER = rlt::nn::optimizers::Adam<OPTIMIZER_SPEC>;
    OPTIMIZER optimizer;
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);
    constexpr TI BATCH_SIZE = 1;
    using INPUT_SHAPE = rlt::tensor::Shape<TI, 1, BATCH_SIZE, 3>;
    using LAYER_SPEC = rlt::nn::layers::dense::Configuration<TYPE_POLICY, typename DEVICE::index_t, 3, rlt::nn::activation_functions::ActivationFunction::RELU>;
    using CAPABILITY_ADAM = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam>;
    rlt::nn::layers::dense::Layer<LAYER_SPEC, CAPABILITY_ADAM, INPUT_SHAPE> layer;
    rlt::malloc(device, optimizer);
    rlt::malloc(device, layer);
    rlt::init_weights(device, layer, rng);
    rlt::zero_gradient(device, layer);
    rlt::reset_forward_state(device, layer);
    rlt::init(device, optimizer);
    rlt::reset_optimizer_state(device, optimizer, layer);
    rlt::randn(device, layer.weights.gradient, rng);
    rlt::randn(device, layer.weights.gradient_first_order_moment, rng);
    rlt::randn(device, layer.weights.gradient_second_order_moment, rng);
    rlt::randn(device, layer.biases.gradient, rng);
    rlt::randn(device, layer.biases.gradient_first_order_moment, rng);
    rlt::randn(device, layer.biases.gradient_second_order_moment, rng);
    auto output = rlt::save_code(device, layer, "layer_1", const_declaration);
    std::cout << "output: " << output << std::endl;
    std::filesystem::create_directories("data");
    std::ofstream file;
    file.open("data/test_rl_tools_nn_layers_dense_persist_code.h");
    file << output;
    file.close();

    ASSERT_TRUE(true);
}

TEST(RL_TOOLS_CONTAINER_PERSIST_CODE_STORE, TEST_MLP){
    using DEVICE = rlt::devices::DefaultCPU;
    using TI = typename DEVICE::index_t;
    using DTYPE = float;
    using TYPE_POLICY = rlt::numeric_types::Policy<DTYPE>;
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);
    constexpr TI BATCH_SIZE = 1;
    using INPUT_SHAPE = rlt::tensor::Shape<TI, 1, BATCH_SIZE, 13>;
    using SPEC = rlt::nn_models::mlp::Configuration<TYPE_POLICY, typename DEVICE::index_t, 4, 3, 64, rlt::nn::activation_functions::ActivationFunction::RELU, rlt::nn::activation_functions::ActivationFunction::IDENTITY, rlt::nn::layers::dense::DefaultInitializer<TYPE_POLICY, TI>>;
    using CAPABILITY_ADAM = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam, BATCH_SIZE>;
    rlt::nn_models::mlp::NeuralNetwork<SPEC, CAPABILITY_ADAM, INPUT_SHAPE> mlp;
    rlt::malloc(device, mlp);
    rlt::init_weights(device, mlp, rng);
    auto output = rlt::save_code(device, mlp, "mlp_1", const_declaration);
    std::cout << "output: " << output << std::endl;
    std::filesystem::create_directories("data");
    std::ofstream file;
    file.open ("data/test_rl_tools_nn_models_mlp_persist_code.h");
    file << output;
    file.close();

    ASSERT_TRUE(true);
}
