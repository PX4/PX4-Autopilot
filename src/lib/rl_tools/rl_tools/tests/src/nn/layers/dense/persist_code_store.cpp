#include <rl_tools/operations/cpu_mux.h>
#include <rl_tools/nn/optimizers/adam/instance/operations_generic.h>
#include <rl_tools/nn/layers/dense/operations_cpu.h>
#include <rl_tools/nn/optimizers/adam/operations_generic.h>

#include <rl_tools/numeric_types/persist_code.h>
#include <rl_tools/containers/matrix/persist_code.h>
#include <rl_tools/containers/tensor/persist_code.h>
#include <rl_tools/nn/parameters/persist_code.h>
#include <rl_tools/nn/optimizers/adam/instance/persist_code.h>
#include <rl_tools/nn/layers/dense/persist_code.h>

namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools;

using DEVICE = rlt::devices::DEVICE_FACTORY<>;
using TI = typename DEVICE::index_t;


#include <gtest/gtest.h>


#include <fstream>
#include <filesystem>

std::optional<std::string> get_env_var(const std::string& var) {
    const char* value = std::getenv(var.c_str());
    if (value) {
        return std::string(value);
    } else {
        return std::nullopt;
    }
}


TEST(RL_TOOLS_NN_LAYERS_DENSE_PERSIST_CODE, STORE) {
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    using T = double;
    using USE_CASE = rlt::numeric_types::UseCase<rlt::numeric_types::categories::Accumulator, float>;
    using USE_CASE2 = rlt::numeric_types::UseCase<rlt::numeric_types::categories::Input, float>;
    using TYPE_POLICY = RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools::numeric_types::Policy<double, RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools::numeric_types::UseCase<RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools::numeric_types::categories::Accumulator, float>, RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools::numeric_types::UseCase<RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools::numeric_types::categories::Input, float>>;
    // using TYPE_POLICY = rlt::numeric_types::Policy<T, USE_CASE, USE_CASE2>;
    std::cout << rlt::to_string(TYPE_POLICY{}) << std::endl;
    constexpr TI INPUT_DIM = 10;
    constexpr TI OUTPUT_DIM = 20;
    constexpr TI BATCH_SIZE = 5;
    using LAYER_CONFIG = rlt::nn::layers::dense::Configuration<TYPE_POLICY, TI, OUTPUT_DIM, rlt::nn::activation_functions::ActivationFunction::RELU>;
    using CAPA = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam>;
    using INPUT_SHAPE = rlt::tensor::Shape<TI, 1, BATCH_SIZE, INPUT_DIM>;
    using LAYER = rlt::nn::layers::dense::Layer<LAYER_CONFIG, CAPA, INPUT_SHAPE>;

    LAYER layer;
    LAYER::Buffer<> buffer;
    rlt::Tensor<rlt::tensor::Specification<T, TI, INPUT_SHAPE>> input;
    rlt::Tensor<rlt::tensor::Specification<T, TI, typename LAYER::OUTPUT_SHAPE>> output;

    rlt::init(device);

    rlt::malloc(device, rng);
    rlt::malloc(device, layer);
    rlt::malloc(device, buffer);
    rlt::malloc(device, input);
    rlt::malloc(device, output);

    rlt::init(device, rng, 0);
    rlt::init_weights(device, layer, rng);
    rlt::randn(device, input, rng);

    rlt::evaluate(device, layer, input, output, buffer, rng);

    std::string output_code;
    output_code += rlt::save_code(device, layer, "layer");
    output_code += rlt::save_code(device, input, "input", true);
    output_code += rlt::save_code(device, output, "output", true);

    std::ofstream file;
    std::string output_path = "tests/data/test_nn_layers_dense_persist_code.h" + std::string((get_env_var("GITHUB_ACTIONS") ? ".disabled" : ""));
    file.open(output_path, std::ios::out | std::ios::trunc);
    std::cout << "Working directory: " << std::filesystem::current_path() << std::endl;
    std::cout << "Full file path: " << std::filesystem::absolute(output_path) << std::endl;
    file << output_code;
    file.close();
}
