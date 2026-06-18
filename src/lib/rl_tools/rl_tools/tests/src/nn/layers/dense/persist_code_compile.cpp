#include <rl_tools/operations/cpu_mux.h>
#include <rl_tools/nn/layers/dense/operations_cpu.h>

#include <rl_tools/containers/matrix/persist_code.h>
#include <rl_tools/containers/tensor/persist_code.h>
#include <rl_tools/nn/parameters/persist_code.h>
#include <rl_tools/nn/optimizers/adam/instance/persist_code.h>
#include <rl_tools/nn/layers/dense/persist_code.h>
#include "../../../../../tests/data/test_nn_layers_dense_persist_code.h"

namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools;

using DEVICE = rlt::devices::DEVICE_FACTORY<>;
using RNG = typename DEVICE::SPEC::RANDOM::ENGINE<>;
using TI = typename DEVICE::index_t;


#include <gtest/gtest.h>



TEST(RL_TOOLS_NN_LAYERS_DENSE_PERSIST_CODE, COMPILE) {
    DEVICE device;
    RNG rng;
    layer::TYPE::Buffer<> buffer;
    using TYPE_POLICY = decltype(layer::module)::TYPE_POLICY;
    using T = typename TYPE_POLICY::DEFAULT;
    rlt::Tensor<rlt::tensor::Specification<T, TI, layer::TYPE::OUTPUT_SHAPE>> output;

    rlt::init(device);
    rlt::malloc(device, rng);
    rlt::malloc(device, buffer);
    rlt::malloc(device, output);

    constexpr TI seed = 0;
    rlt::init(device, rng, seed);

    rlt::evaluate(device, layer::module, input::container, output, buffer, rng);

    T abs_diff = rlt::abs_diff(device, output::container, output);
    std::cout << "abs_diff: " << abs_diff << std::endl;
    EXPECT_LT(abs_diff, 1e-6);
}
