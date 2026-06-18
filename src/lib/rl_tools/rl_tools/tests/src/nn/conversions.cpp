// Group 1
#include <rl_tools/devices/cpu.h>
#include <rl_tools/math/operations_cpu.h>
#include <rl_tools/random/operations_cpu.h>
#include <rl_tools/logging/operations_cpu.h>
#include <rl_tools/devices/dummy.h>
#include <rl_tools/math/operations_dummy.h>
#include <rl_tools/random/operations_dummy.h>
#include <rl_tools/logging/operations_dummy.h>
#include <rl_tools/numeric_types/policy.h>

// Group 2: depends on logging
#include <rl_tools/utils/assert/operations_cpu.h>
#include <rl_tools/utils/assert/operations_dummy.h>
// Group 3: dependent on assert
#include <rl_tools/containers/matrix/operations_cpu.h>
#include <rl_tools/containers/matrix/operations_dummy.h>
#include <rl_tools/containers/tensor/operations_generic.h>

#include <rl_tools/nn/operations_cpu.h>
#include <rl_tools/utils/generic/typing.h>

#include <gtest/gtest.h>

namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools;
using DTYPE = float;
using TYPE_POLICY = rlt::numeric_types::Policy<DTYPE>;
using index_t = unsigned;
constexpr index_t OUTER_INPUT_DIM = 10;
constexpr index_t OUTER_OUTPUT_DIM = 10;
constexpr unsigned OUTER_INPUT_DIM_2 = 10;
constexpr unsigned OUTER_OUTPUT_DIM_2 = 10;
constexpr index_t BATCH_SIZE = 1;
using INPUT_SHAPE = rlt::tensor::Shape<index_t, 1, BATCH_SIZE, OUTER_INPUT_DIM>;
using LayerConfig1 = rlt::nn::layers::dense::Configuration<TYPE_POLICY, index_t, OUTER_OUTPUT_DIM, rlt::nn::activation_functions::ActivationFunction::IDENTITY>;
using LayerConfig2 = rlt::nn::layers::dense::Configuration<TYPE_POLICY, index_t, OUTER_OUTPUT_DIM, rlt::nn::activation_functions::ActivationFunction::IDENTITY>;
using LayerConfig3 = rlt::nn::layers::dense::Configuration<TYPE_POLICY, index_t, OUTER_OUTPUT_DIM, rlt::nn::activation_functions::ActivationFunction::RELU>;
using LayerConfig4 = rlt::nn::layers::dense::Configuration<TYPE_POLICY, index_t, OUTER_OUTPUT_DIM_2, rlt::nn::activation_functions::ActivationFunction::RELU>;


static_assert(rlt::utils::typing::is_same_v<LayerConfig1, LayerConfig2>);
// these should fail
//static_assert(rlt::utils::typing::is_same_v<LayerSpec1, LayerSpec3>);
//static_assert(rlt::utils::typing::is_same_v<LayerSpec1, LayerSpec4>);
//static_assert(rlt::utils::typing::is_same_v<LayerSpec1, LayerSpec5>);



TEST(RL_TOOLS_NN_MLP_CONVERSIONS, CONVERSIONS) {
    using Device1 = rlt::devices::DefaultDummy;
    using Layer1 = rlt::nn::layers::dense::Layer<LayerConfig1, rlt::nn::capability::Forward<>, INPUT_SHAPE>;

    Device1 device1;
    Layer1 layer1;

    Layer1 layer11;

    using Device2 = rlt::devices::DefaultCPU;
    using Layer2 = rlt::nn::layers::dense::Layer<LayerConfig2, rlt::nn::capability::Forward<>, INPUT_SHAPE>;

    Device2 device2;
    Layer2 layer2;
    Layer2 layer22;
    Layer2 layer222;

    rlt::malloc(device1, layer1);
    rlt::malloc(device2, layer2);
    rlt::malloc(device2, layer22);
    rlt::malloc(device2, layer222);

    Device2::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device2, rng);
    rlt::init(device2, rng, 0);
    rlt::init_weights(device2, layer2, rng);
    rlt::init_weights(device2, layer22, rng);
    rlt::init_weights(device2, layer222, rng);

    ASSERT_GT(rlt::abs_diff(device2, layer2, layer22), 0);

    rlt::copy(device2, device2, layer222, layer22);

    ASSERT_GT(rlt::abs_diff(device2, layer2, layer22), 0);
    ASSERT_EQ(rlt::abs_diff(device2, layer22, layer222), 0);

    rlt::copy(device2, device2, layer22, layer2);

    ASSERT_EQ(rlt::abs_diff(device2, layer2, layer222), 0);

    rlt::copy(device1, device1, layer2, layer1);

    ASSERT_EQ(rlt::abs_diff(device1, layer1, layer222), 0);

}