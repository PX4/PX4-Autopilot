#include <rl_tools/operations/cpu.h>

#include "../../../../tests/data/nn_models_sequential_persist_code_model_mlp_forward.h"

#include <rl_tools/nn/operations_generic.h>
#include <rl_tools/nn/layers/standardize/operations_generic.h>
#include <rl_tools/nn_models/mlp_unconditional_stddev//operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>


namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools;
#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>

using T = double;
using DEVICE = rlt::devices::DefaultCPU;
using TI = typename DEVICE::index_t;

TEST(RL_TOOLS_NN_MODELS_SEQUENTIAL_PERSIST_CODE_COMPILE, MODEL_MLP_FORWARD){
    DEVICE device;
    rlt::Tensor<rlt::tensor::Specification<T, TI, rl_tools_export::model::TYPE::OUTPUT_SHAPE>> output;
    rl_tools_export::model::TYPE::Buffer<> buffer;

    rlt::malloc(device, output);
    rlt::malloc(device, buffer);

    bool rng = false;

    const rl_tools_export::model::TYPE module = rl_tools_export::model::factory_function(); // MSVC fix:
    rlt::evaluate(device, module, rl_tools_export::input::container, output, buffer, rng);

    auto abs_diff = rlt::abs_diff(device, output, rl_tools_export::output::container);

    std::cout << "Oiriginal output:" << std::endl;
    rlt::print(device, rl_tools_export::output::container);
    std::cout << "Loaded output:" << std::endl;
    rlt::print(device, output);

    std::cout << "abs_diff: " << abs_diff << std::endl;
    ASSERT_LT(abs_diff, 1e-5);
}

