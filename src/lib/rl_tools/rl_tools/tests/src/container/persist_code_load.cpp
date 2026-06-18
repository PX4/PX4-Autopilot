#include <rl_tools/operations/cpu.h>

#include <rl_tools/containers/matrix/persist_code.h>
#include <rl_tools/nn/layers/dense/operations_cpu.h>
#include <rl_tools/nn_models/mlp/operations_cpu.h>

namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools;


#include <gtest/gtest.h>
#include <iostream>
#include <fstream>
#include <filesystem>
#include "../../../data/test_rl_tools_container_persist_matrix.h"

constexpr bool const_declaration = false;

TEST(RL_TOOLS_CONTAINER_PERSIST_CODE_LOAD, TEST){
    using DEVICE = rlt::devices::DefaultCPU;
    using DTYPE = float;
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 1);
    rlt::Matrix<rlt::matrix::Specification<DTYPE, typename DEVICE::index_t, 3, 3>> orig;
    rlt::malloc(device, orig);
    rlt::randn(device, orig, rng);
    std::cout << "orig: " << std::endl;
    rlt::print(device, orig);
    std::cout << "loaded: " << std::endl;
    rlt::print(device, matrix_1::container);

    auto abs_diff = rlt::abs_diff(device, orig, matrix_1::container);
    ASSERT_FLOAT_EQ(0, abs_diff);
}

#include "../../../data/test_rl_tools_nn_layers_dense_persist_code.h"

TEST(RL_TOOLS_CONTAINER_PERSIST_CODE_LOAD, TEST_DENSE_LAYER){
    using DEVICE = rlt::devices::DefaultCPU;
    using DTYPE = float;
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 1);
    rlt::nn::layers::dense::LayerForward<rlt::nn::layers::dense::Specification<DTYPE, typename DEVICE::index_t, 3, 3, rlt::nn::activation_functions::ActivationFunction::RELU>> layer;
    rlt::malloc(device, layer);
    rlt::init_weights(device, layer, rng);
    rlt::increment(layer.weights.parameters, 2, 1, 10);
    auto abs_diff = rlt::abs_diff(device, layer, layer_1::layer);
    ASSERT_FLOAT_EQ(10, abs_diff);
}

TEST(RL_TOOLS_CONTAINER_PERSIST_CODE_LOAD, TEST_DENSE_LAYER_ADAM){
    using DEVICE = rlt::devices::DefaultCPU;
    using DTYPE = float;
    using OPTIMIZER_PARAMETERS = rlt::nn::optimizers::adam::Specification<DTYPE, DEVICE::index_t>;
    using OPTIMIZER = rlt::nn::optimizers::Adam<OPTIMIZER_PARAMETERS>;
    OPTIMIZER optimizer;
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 1);
    rlt::nn::layers::dense::LayerGradient<rlt::nn::layers::dense::Specification<DTYPE, typename DEVICE::index_t, 3, 3, rlt::nn::activation_functions::ActivationFunction::RELU, rlt::nn::parameters::Adam>> layer;
    rlt::malloc(device, layer);
    rlt::malloc(device, optimizer);
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
    rlt::increment(layer.weights.parameters, 2, 1, 10);
    rlt::increment(layer.weights.gradient, 2, 1, 5);
    rlt::increment(layer.weights.gradient_first_order_moment, 2, 1, 2);
    rlt::increment(layer.weights.gradient_second_order_moment, 2, 1, 1);
    auto abs_diff = rlt::abs_diff(device, layer, layer_1::layer);
    ASSERT_FLOAT_EQ(10 + 5 + 2 + 1, abs_diff);
}

#include "../../../data/test_rl_tools_nn_models_mlp_persist_code.h"

TEST(RL_TOOLS_CONTAINER_PERSIST_CODE_LOAD, TEST_MLP){
    using DEVICE = rlt::devices::DefaultCPU;
    using DTYPE = float;
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 1);
    using SPEC = rlt::nn_models::mlp::ForwardSpecification<rlt::nn_models::mlp::StructureSpecification<DTYPE, typename DEVICE::index_t, 13, 4, 3, 64, rlt::nn::activation_functions::ActivationFunction::RELU, rlt::nn::activation_functions::ActivationFunction::IDENTITY, 1, rlt::MatrixDynamicTag, true, rlt::matrix::layouts::RowMajorAlignment<typename DEVICE::index_t, 1>>>;
    rlt::nn_models::mlp::NeuralNetwork<SPEC> mlp;
    rlt::malloc(device, mlp);
    rlt::init_weights(device, mlp, rng);
    rlt::increment(mlp.hidden_layers[0].biases.parameters, 0, 2, 10);
    auto abs_diff = rlt::abs_diff(device, mlp, mlp_1::mlp);
    ASSERT_FLOAT_EQ(10, abs_diff);
}

TEST(RL_TOOLS_CONTAINER_PERSIST_CODE_LOAD, TEST_MLP_ADAM){
    using DEVICE = rlt::devices::DefaultCPU;
    using DTYPE = float;
    DEVICE device;
    using OPTIMIZER_PARAMETERS = rlt::nn::optimizers::adam::Specification<DTYPE, DEVICE::index_t>;
    using OPTIMIZER = rlt::nn::optimizers::Adam<OPTIMIZER_PARAMETERS>;
    OPTIMIZER optimizer;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 1);
    using SPEC = rlt::nn_models::mlp::AdamSpecification<rlt::nn_models::mlp::StructureSpecification<DTYPE, typename DEVICE::index_t, 13, 4, 3, 64, rlt::nn::activation_functions::ActivationFunction::RELU, rlt::nn::activation_functions::ActivationFunction::IDENTITY, 1, rlt::MatrixDynamicTag, true, rlt::matrix::layouts::RowMajorAlignment<typename DEVICE::index_t, 1>>>;
    rlt::nn_models::mlp::NeuralNetworkAdam<SPEC> mlp;
    rlt::malloc(device, mlp);
    rlt::malloc(device, optimizer);
    rlt::init_weights(device, mlp, rng);
    rlt::zero_gradient(device, mlp);
    rlt::reset_forward_state(device, mlp);
    rlt::init(device, optimizer);
    rlt::reset_optimizer_state(device, optimizer, mlp);
    rlt::increment(mlp.hidden_layers[0].biases.parameters, 0, 2, 10);
    rlt::copy(device, device, mlp.input_layer, mlp_1::input_layer::layer);
    auto abs_diff = rlt::abs_diff(device, mlp, mlp_1::mlp);
    ASSERT_FLOAT_EQ(10, abs_diff);
}

TEST(RL_TOOLS_CONTAINER_PERSIST_CODE_LOAD, TEST_MLP_EVALUATE){
    using DEVICE = rlt::devices::DefaultCPU;
    using DTYPE = float;
    constexpr typename DEVICE::index_t BATCH_SIZE = 10;
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 1);
    using STRUCTURE_SPEC = rlt::nn_models::mlp::StructureSpecification<DTYPE, typename DEVICE::index_t, 13, 4, 3, 64, rlt::nn::activation_functions::ActivationFunction::RELU, rlt::nn::activation_functions::ActivationFunction::IDENTITY, 1, rlt::MatrixDynamicTag, true, rlt::matrix::layouts::RowMajorAlignment<typename DEVICE::index_t, 1>>;
    using SPEC = rlt::nn_models::mlp::ForwardSpecification<STRUCTURE_SPEC>;
    rlt::nn_models::mlp::NeuralNetwork<SPEC> mlp;
    rlt::malloc(device, mlp);
    rlt::init_weights(device, mlp, rng);

    rlt::Matrix<rlt::matrix::Specification<DTYPE, typename DEVICE::index_t, BATCH_SIZE, SPEC::STRUCTURE_SPEC::INPUT_DIM>> input;
    rlt::Matrix<rlt::matrix::Specification<DTYPE, typename DEVICE::index_t, BATCH_SIZE, SPEC::STRUCTURE_SPEC::OUTPUT_DIM>> output_orig, output_loaded;
    rlt::malloc(device, input);
    rlt::malloc(device, output_orig);
    rlt::malloc(device, output_loaded);
    rlt::randn(device, input, rng);
    rlt::evaluate(device, mlp, input, output_orig);
    rlt::evaluate(device, mlp_1::mlp, input, output_loaded);
    rlt::print(device, output_orig);

    auto output = rlt::save(device, input, "input", const_declaration);
    output += rlt::save(device, output_orig, "expected_output", const_declaration);

    std::filesystem::create_directories("data");
    std::ofstream file;
    file.open ("data/test_rl_tools_nn_models_mlp_evaluation.h");
    file << output;
    file.close();

    auto abs_diff = rlt::abs_diff(device, output_orig, output_loaded);
    ASSERT_FLOAT_EQ(0, abs_diff);
}
