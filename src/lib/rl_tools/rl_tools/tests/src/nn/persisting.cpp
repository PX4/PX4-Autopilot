#include <gtest/gtest.h>
#include <highfive/H5File.hpp>

#include <rl_tools/operations/cpu.h>

#include <rl_tools/nn/optimizers/adam/instance/operations_generic.h>
#include <rl_tools/persist/backends/hdf5/operations_cpu.h>
#include <rl_tools/nn/parameters/persist.h>
#include <rl_tools/nn/optimizers/adam/instance/persist.h>
#include "rl_tools/nn_models/persist.h"
#include "rl_tools/nn_models/mlp/operations_generic.h"
#include <rl_tools/nn/optimizers/adam/operations_generic.h>

#include "../utils/utils.h"

#include "default_network_mlp.h"
#include "../utils/nn_comparison_mlp.h"

#include <random>
TEST(RL_TOOLS_NN_PERSIST, Saving) {

    NN_DEVICE device;
    OPTIMIZER optimizer;
    NetworkType network_1, network_2;
    rlt::malloc(device, optimizer);
    rlt::malloc(device, network_1);
    rlt::malloc(device, network_2);
    NN_DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 2);
    rlt::init_weights(device, network_1, rng);
    rlt::init_weights(device, network_2, rng);
    rlt::reset_forward_state(device, network_1);
    rlt::init(device, optimizer);
    rlt::reset_optimizer_state(device, optimizer, network_1);
    rlt::zero_gradient(device, network_1);
    rlt::reset_forward_state(device, network_2);
    rlt::init(device, optimizer);
    rlt::reset_optimizer_state(device, optimizer, network_2);
    rlt::zero_gradient(device, network_2);
    rlt::increment(device, network_1.input_layer.weights.gradient_first_order_moment, 10, 2, 3);
    {
        auto output_file = HighFive::File(std::string("test.hdf5"), HighFive::File::Overwrite);
        rlt::persist::backends::hdf5::Group<> group = {output_file.createGroup("three_layer_fc")};
        rlt::save(device, network_1, group);
    }

    DTYPE diff_pre_load = abs_diff(device, network_1, network_2);
    ASSERT_GT(diff_pre_load, 10);
    std::cout << "diff_pre_load: " << diff_pre_load << std::endl;
    {
        auto input_file = HighFive::File(std::string("test.hdf5"), HighFive::File::ReadOnly);
        auto three_layer_fc_group = rlt::get_group(device, input_file, "three_layer_fc");
        rlt::load(device, network_2, three_layer_fc_group);
    }
    rlt::reset_forward_state(device, network_1);
    rlt::reset_forward_state(device, network_2);
    DTYPE diff_post_load = abs_diff(device, network_1, network_2);
    ASSERT_EQ(diff_post_load, 0);
    std::cout << "diff_post_load: " << diff_post_load << std::endl;
}

