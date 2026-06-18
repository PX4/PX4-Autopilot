#include <gtest/gtest.h>
#include <iostream>
#include <rl_tools/operations/cpu.h>
#include <rl_tools/persist/backends/hdf5/operations_cpu.h>
#include <rl_tools/persist/backends/hdf5/operations_cpu.h>
#include <rl_tools/nn/layers/dense/operations_cpu.h>
#include <rl_tools/nn/parameters/persist.h>
#include <rl_tools/nn/layers/dense/persist.h>

namespace rlt = rl_tools;

using DEVICE = rl_tools::devices::DefaultCPU;
using RNG = DEVICE::SPEC::RANDOM::ENGINE<>;
using T = float;
using TYPE_POLICY = rlt::numeric_types::Policy<T>;
using TI = typename DEVICE::index_t;

#define RL_TOOLS_STRINGIZE(x) #x
#define RL_TOOLS_MACRO_TO_STR(macro) RL_TOOLS_STRINGIZE(macro)

TEST(TEST_PERSIST_BACKENDS_HDF5_HDF5, test) {
    DEVICE device;
    RNG rng;
    constexpr TI seed = 0;
    rlt::init(device);
    rlt::malloc(device, rng);
    rlt::init(device, rng, seed);
    rlt::Tensor<rlt::tensor::Specification<T, TI, rlt::tensor::Shape<TI, 4, 3>>> A;
    rlt::malloc(device, A);
    rlt::randn(device, A, rng);
    rlt::print(device, A);
    std::string data_file_name = "test_persist_backends_hdf5.h5";
    const char *data_path_stub = RL_TOOLS_MACRO_TO_STR(RL_TOOLS_TEST_DATA_PATH);
    std::string data_file_path = std::string(data_path_stub) + "/" + data_file_name;
    auto output_file = HighFive::File(data_file_path, HighFive::File::Overwrite);
    rlt::persist::backends::hdf5::Group<rlt::persist::backends::hdf5::GroupSpecification<>> group{output_file.createGroup("test")};
    rlt::save(device, A, group, "A");


    using CONFIG = rlt::nn::layers::dense::Configuration<TYPE_POLICY, TI, 10, rlt::nn::activation_functions::ActivationFunction::RELU>;
    using CAPABILITY = rlt::nn::capability::Forward<>;
    using SPEC = rlt::nn::layers::dense::Specification<CONFIG, CAPABILITY, rlt::tensor::Shape<TI, 1, 10, 15>>;
    rlt::nn::layers::dense::LayerForward<SPEC> layer;
    rlt::malloc(device, layer);
    rlt::init_weights(device, layer, rng);
    rlt::print(device, layer.weights.parameters);
    auto layer_group = rlt::create_group(device, group, "layer");
    rlt::save(device, layer, layer_group);
}