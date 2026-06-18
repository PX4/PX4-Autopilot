#include <rl_tools/operations/cpu.h>
#include <rl_tools/nn/optimizers/adam/instance/operations_generic.h>
#include <rl_tools/nn/layers/embedding/operations_generic.h>
#include <rl_tools/nn/layers/gru/operations_generic.h>
#include <rl_tools/nn/operations_cpu.h>
#include <rl_tools/nn/loss_functions/categorical_cross_entropy/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>
#include <rl_tools/nn/optimizers/adam/operations_generic.h>

#include <rl_tools/numeric_types/persist_code.h>
#include <rl_tools/containers/tensor/persist_code.h>
#include <rl_tools/nn/parameters/persist_code.h>
#include <rl_tools/nn/optimizers/adam/instance/persist_code.h>
#include <rl_tools/containers/tensor/persist_code.h>
#include <rl_tools/nn/optimizers/adam/instance/persist_code.h>
#include <rl_tools/nn/layers/gru/persist_code.h>
#include <rl_tools/nn_models/sequential/persist_code.h>

// #include <highfive/H5File.hpp>

namespace rlt = rl_tools;

#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <optional>

std::optional<std::string> get_env_var(const std::string& var) {
    const char* value = std::getenv(var.c_str());
    if (value) {
        return std::string(value);
    } else {
        return std::nullopt;
    }
}

//using DEVICE = rlt::devices::DEVICE_FACTORY<>;
using DEVICE = rlt::devices::DefaultCPU;
using TI = typename DEVICE::index_t;
using T = double;
using TYPE_POLICY = rlt::numeric_types::Policy<T>;

template <typename T_CONTENT, typename T_NEXT_MODULE = rlt::nn_models::sequential::OutputModule>
using Module = typename rlt::nn_models::sequential::Module<T_CONTENT, T_NEXT_MODULE>;
TEST(RL_TOOLS_NN_LAYERS_GRU, PERSIST_CODE){
    static constexpr TI SEQUENCE_LENGTH = 2;
    static constexpr TI BATCH_SIZE = 3;
    static constexpr TI INPUT_DIM = 4;
    static constexpr TI OUTPUT_DIM = 5;
    using INPUT_SHAPE = rlt::tensor::Shape<TI, SEQUENCE_LENGTH, BATCH_SIZE, INPUT_DIM>;
    using GRU_CONFIG = rlt::nn::layers::gru::Configuration<TYPE_POLICY, TI, OUTPUT_DIM, rlt::nn::parameters::groups::Normal, true>;
    using GRU = rlt::nn::layers::gru::BindConfiguration<GRU_CONFIG>;
    using CAPABILITY = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam>;

    using MODULE_CHAIN = Module<GRU>;
    using GRU_MODEL = rlt::nn_models::sequential::Build<CAPABILITY, MODULE_CHAIN, INPUT_SHAPE>;

    GRU_MODEL gru;
    typename GRU_MODEL::Buffer<true> buffer;
    using ADAM_SPEC = rlt::nn::optimizers::adam::Specification<TYPE_POLICY, TI>;
    using ADAM = rlt::nn::optimizers::Adam<ADAM_SPEC>;
    ADAM optimizer;

    rlt::Tensor<rlt::tensor::Specification<T, TI, rlt::tensor::Replace<GRU_MODEL::INPUT_SHAPE, BATCH_SIZE, 1>>> input;
    rlt::Tensor<rlt::tensor::Specification<T, TI, rlt::tensor::Replace<GRU_MODEL::OUTPUT_SHAPE, BATCH_SIZE, 1>>> output, d_output;

    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);

    rlt::malloc(device, optimizer);
    rlt::malloc(device, gru);
    rlt::malloc(device, buffer);
    rlt::malloc(device, input);
    rlt::malloc(device, output);
    rlt::malloc(device, d_output);
    rlt::init_weights(device, gru, rng);
    rlt::init(device, optimizer);
    rlt::reset_optimizer_state(device, optimizer, gru);
    rlt::randn(device, input, rng);
    rlt::randn(device, d_output, rng);

    rlt::forward(device, gru, input, buffer, rng);

    std::string code_output;
    code_output += rlt::save_code(device, gru, "gru", true);
    code_output += rlt::save_code(device, input, "input", true);
    auto output_tensor = rlt::output(device, gru);
    code_output += rlt::save_code(device, output_tensor, "output", true);

    std::ofstream file;
    std::string output_path = "tests/data/test_nn_layers_gru_persist_code.h" + std::string((get_env_var("GITHUB_ACTIONS") ? ".disabled" : ""));
    file.open(output_path, std::ios::out | std::ios::trunc);
    std::cout << "Working directory: " << std::filesystem::current_path() << std::endl;
    std::cout << "Full file path: " << std::filesystem::absolute(output_path) << std::endl;
    file << code_output;
    file.close();
}
