#include <rl_tools/operations/cpu.h>
#include <rl_tools/nn/optimizers/adam/instance/operations_generic.h>
#include <rl_tools/nn/layers/embedding/operations_generic.h>
#include <rl_tools/nn/layers/gru/operations_generic.h>
#include <rl_tools/nn/operations_cpu.h>
#include <rl_tools/nn/loss_functions/categorical_cross_entropy/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>
#include <rl_tools/nn/optimizers/adam/operations_generic.h>

#include <rl_tools/persist/backends/hdf5/operations_cpu.h>
#include <rl_tools/nn/optimizers/adam/instance/persist.h>
#include <rl_tools/nn/layers/embedding/persist.h>
#include <rl_tools/nn/layers/gru/persist.h>
#include <rl_tools/nn/layers/dense/persist.h>
#include <rl_tools/nn_models/sequential/persist.h>

#include <highfive/H5File.hpp>

namespace rlt = rl_tools;

#include "gru_model.h"

#include <gtest/gtest.h>
#include <filesystem>

//using DEVICE = rlt::devices::DEVICE_FACTORY<>;
using DEVICE = rlt::devices::DefaultCPU;
using TI = typename DEVICE::index_t;
using T = double;
using TYPE_POLICY = rlt::numeric_types::Policy<T>;

TEST(RL_TOOLS_NN_LAYERS_GRU, PERSIST){
    using CONFIG = Config<TYPE_POLICY, TI>;
    using GRU = typename CONFIG::GRU::Layer<typename CONFIG::CAPABILITY, typename CONFIG::INPUT_SHAPE>;
    GRU gru;
    typename GRU::Buffer<true> buffer;
    typename CONFIG::ADAM optimizer;

    rlt::Tensor<rlt::tensor::Specification<T, TI, GRU::INPUT_SHAPE>> input;
    rlt::Tensor<rlt::tensor::Specification<T, TI, GRU::OUTPUT_SHAPE>> d_output;

    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);

    rlt::malloc(device, optimizer);
    rlt::malloc(device, gru);
    rlt::malloc(device, buffer);
    rlt::malloc(device, input);
    rlt::malloc(device, d_output);
    rlt::init_weights(device, gru, rng);
    rlt::reset_optimizer_state(device, optimizer, gru);
    rlt::randn(device, input, rng);
    rlt::randn(device, d_output, rng);

    rlt::forward(device, gru, input, buffer, rng);
    rlt::zero_gradient(device, gru);
    rlt::backward(device, gru, input, d_output, buffer);
    rlt::step(device, optimizer, gru);


    std::filesystem::path FILE_PATH = "tests_nn_layers_gru_persist.h5";
    {
        std::cout << "Checkpointing" << std::endl;
        auto file = HighFive::File(FILE_PATH.string(), HighFive::File::Overwrite);
        rlt::zero_gradient(device, gru);
        rlt::reset_forward_state(device, gru);
        auto test_gru_group = rlt::create_group(device, file, "test_gru");
        rlt::save(device, gru, test_gru_group);
    }
    {
        auto file = HighFive::File(FILE_PATH.string(), HighFive::File::ReadOnly);
        GRU gru_copy;
        rlt::malloc(device, gru_copy);
        auto group = rlt::get_group(device, file, "test_gru");
        rlt::load(device, gru_copy, group);
        T abs_diff = rlt::abs_diff(device, gru, gru_copy);
        std::cout << "GRU abs_diff: " << abs_diff << std::endl;
        rlt::utils::assert_exit(device, abs_diff < 1e-15, "Checkpoint failed");
        rlt::free(device, gru_copy);
    }
}
