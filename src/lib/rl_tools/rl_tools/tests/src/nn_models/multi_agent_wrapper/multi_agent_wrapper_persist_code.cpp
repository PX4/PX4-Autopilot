#include <rl_tools/operations/cpu.h>
#include <rl_tools/nn/optimizers/adam/instance/operations_generic.h>
#include <rl_tools/nn/layers/dense/operations_generic.h>
#include <rl_tools/nn/layers/standardize/operations_generic.h>
#include <rl_tools/nn/layers/sample_and_squash/operations_generic.h>
#include <rl_tools/nn_models/mlp/operations_generic.h>
#include <rl_tools/nn_models/mlp_unconditional_stddev//operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>
#include <rl_tools/nn_models/multi_agent_wrapper/operations_generic.h>

#include <rl_tools/rl/environments/multi_agent/bottleneck/operations_cpu.h>
#include <rl_tools/rl/algorithms/ppo/loop/core/config.h>

#include <rl_tools/numeric_types/persist_code.h>
#include <rl_tools/containers/tensor/persist_code.h>
#include <rl_tools/nn/parameters/persist_code.h>
#include <rl_tools/nn/optimizers/adam/instance/persist_code.h>
#include <rl_tools/nn/layers/dense/persist_code.h>
#include <rl_tools/nn/layers/standardize/persist_code.h>
#include <rl_tools/nn/layers/sample_and_squash/persist_code.h>
#include <rl_tools/nn_models/mlp/persist_code.h>
#include <rl_tools/nn_models/mlp_unconditional_stddev/persist_code.h>
#include <rl_tools/nn_models/sequential/persist_code.h>
#include <rl_tools/nn_models/multi_agent_wrapper/persist_code.h>


namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools;
#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <cstdlib>
#include <optional>
#include <string>

using DEVICE = rlt::devices::DefaultCPU;
using TI = typename DEVICE::index_t;
using T = double;
using TYPE_POLICY = rlt::numeric_types::Policy<T>;

std::optional<std::string> get_env_var(const std::string& var) {
    const char* value = std::getenv(var.c_str());
    if (value) {
        return std::string(value);
    } else {
        return std::nullopt;
    }
}

template<typename ENVIRONMENT>
struct LOOP_CORE_PARAMETERS: rlt::rl::algorithms::ppo::loop::core::DefaultParameters<TYPE_POLICY, TI, ENVIRONMENT>{
    static constexpr TI ACTOR_HIDDEN_DIM = 7;
    static constexpr TI ACTOR_NUM_LAYERS = 3;
    static constexpr auto ACTOR_ACTIVATION_FUNCTION = rlt::nn::activation_functions::ActivationFunction::RELU;
    static constexpr TI CRITIC_HIDDEN_DIM = 8;
    static constexpr TI CRITIC_NUM_LAYERS = 3;
    static constexpr TI BATCH_SIZE = 3;
    static constexpr auto CRITIC_ACTIVATION_FUNCTION = rlt::nn::activation_functions::ActivationFunction::RELU;
};
TEST(RL_TOOLS_NN_MODELS_MULTI_AGENT_WRAPPER_PERSIST_CODE, GRADIENT) {

    using ENVIRONMENT_SPEC = rlt::rl::environments::multi_agent::bottleneck::Specification<TYPE_POLICY, TI>;
    using ENVIRONMENT = rlt::rl::environments::multi_agent::Bottleneck<ENVIRONMENT_SPEC>;
    using APPROXIMATORS = rlt::rl::algorithms::ppo::loop::core::ConfigApproximatorsSequentialMultiAgent<TYPE_POLICY, TI, ENVIRONMENT, LOOP_CORE_PARAMETERS<ENVIRONMENT>>;
    using CAPABILITY = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam>;
    using MODEL = APPROXIMATORS::Actor<CAPABILITY>::MODEL;

    DEVICE device;
    MODEL model;
    MODEL::Buffer<1> buffer;

    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);

    rlt::Tensor<rlt::tensor::Specification<T, TI, MODEL::INPUT_SHAPE>> input;
    rlt::Tensor<rlt::tensor::Specification<T, TI, MODEL::OUTPUT_SHAPE>> output;

    rlt::malloc(device, input);
    rlt::malloc(device, output);
    rlt::malloc(device, model);
    rlt::malloc(device, buffer);

    rlt::init_weights(device, model, rng);
    rlt::randn(device, input, rng);

    rlt::evaluate(device, model, input, output, buffer, rng);

    rlt::print(device, output);



    {
        auto model_code = rlt::save_code_split(device, model, "model", true, 1);
        auto input_code = rlt::save_code_split(device, input, "input", true, 1);
        auto output_code = rlt::save_code_split(device, output, "output", true, 1);
        auto header = model_code.header + "\n" + input_code.header + "\n" + output_code.header;
        auto body = model_code.body + "\n" + input_code.body + "\n" + output_code.body;

        auto wrapped = rlt::embed_in_namespace(device, {header, body}, "rl_tools_export", 0);

        auto output = wrapped.header + "\n" + wrapped.body;
        //        std::cout << "output: " << output << std::endl;
        //        std::filesystem::create_directories("data");
        std::ofstream file;
        std::string output_path = "tests/data/nn_models_multi_agent_wrapper_persist_code.h" + std::string((get_env_var("GITHUB_ACTIONS") ? ".disabled" : ""));
        file.open(output_path, std::ios::out | std::ios::trunc);
        std::cout << "Working directory: " << std::filesystem::current_path() << std::endl;
        std::cout << "Full file path: " << std::filesystem::absolute(output_path) << std::endl;
        file << output;
        file.close();
    }
}

TEST(RL_TOOLS_NN_MODELS_MULTI_AGENT_WRAPPER_PERSIST_CODE, BACKWARD) {

    using ENVIRONMENT_SPEC = rlt::rl::environments::multi_agent::bottleneck::Specification<TYPE_POLICY, TI>;
    using ENVIRONMENT = rlt::rl::environments::multi_agent::Bottleneck<ENVIRONMENT_SPEC>;
    using APPROXIMATORS = rlt::rl::algorithms::ppo::loop::core::ConfigApproximatorsSequentialMultiAgent<TYPE_POLICY, TI, ENVIRONMENT, LOOP_CORE_PARAMETERS<ENVIRONMENT>>;
    using CAPABILITY = rlt::nn::capability::Backward<>;
    using MODEL = APPROXIMATORS::Actor<CAPABILITY>::MODEL;

    DEVICE device;
    MODEL model;
    MODEL::Buffer<1> buffer;

    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);

    rlt::Tensor<rlt::tensor::Specification<T, TI, MODEL::INPUT_SHAPE>> input;
    rlt::Tensor<rlt::tensor::Specification<T, TI, MODEL::OUTPUT_SHAPE>> output;

    rlt::malloc(device, input);
    rlt::malloc(device, output);
    rlt::malloc(device, model);
    rlt::malloc(device, buffer);

    rlt::init_weights(device, model, rng);
    rlt::randn(device, input, rng);

    rlt::evaluate(device, model, input, output, buffer, rng);

    rlt::print(device, output);



    {
        auto model_code = rlt::save_code_split(device, model, "model", true, 1);
        auto input_code = rlt::save_code_split(device, input, "input", true, 1);
        auto output_code = rlt::save_code_split(device, output, "output", true, 1);
        auto header = model_code.header + "\n" + input_code.header + "\n" + output_code.header;
        auto body = model_code.body + "\n" + input_code.body + "\n" + output_code.body;

        auto wrapped = rlt::embed_in_namespace(device, {header, body}, "rl_tools_export", 0);

        auto output = wrapped.header + "\n" + wrapped.body;
        //        std::cout << "output: " << output << std::endl;
        //        std::filesystem::create_directories("data");
        std::ofstream file;
        std::string output_path = "tests/data/nn_models_multi_agent_wrapper_persist_code_backward.h" + std::string((get_env_var("GITHUB_ACTIONS") ? ".disabled" : ""));
        file.open(output_path, std::ios::out | std::ios::trunc);
        std::cout << "Working directory: " << std::filesystem::current_path() << std::endl;
        std::cout << "Full file path: " << std::filesystem::absolute(output_path) << std::endl;
        file << output;
        file.close();
    }
}

TEST(RL_TOOLS_NN_MODELS_MULTI_AGENT_WRAPPER_PERSIST_CODE, FORWARD) {

    using ENVIRONMENT_SPEC = rlt::rl::environments::multi_agent::bottleneck::Specification<TYPE_POLICY, TI>;
    using ENVIRONMENT = rlt::rl::environments::multi_agent::Bottleneck<ENVIRONMENT_SPEC>;
    using APPROXIMATORS = rlt::rl::algorithms::ppo::loop::core::ConfigApproximatorsSequentialMultiAgent<TYPE_POLICY, TI, ENVIRONMENT, LOOP_CORE_PARAMETERS<ENVIRONMENT>>;
    using CAPABILITY = rlt::nn::capability::Forward<>;
    using MODEL = APPROXIMATORS::Actor<CAPABILITY>::MODEL;

    DEVICE device;
    MODEL model;
    MODEL::Buffer<1> buffer;

    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);

    rlt::Tensor<rlt::tensor::Specification<T, TI, MODEL::INPUT_SHAPE>> input;
    rlt::Tensor<rlt::tensor::Specification<T, TI, MODEL::OUTPUT_SHAPE>> output;

    rlt::malloc(device, input);
    rlt::malloc(device, output);
    rlt::malloc(device, model);
    rlt::malloc(device, buffer);

    rlt::init_weights(device, model, rng);
    rlt::randn(device, input, rng);

    rlt::evaluate(device, model, input, output, buffer, rng);

    rlt::print(device, output);



    {
        auto model_code = rlt::save_code_split(device, model, "model", true, 1);
        auto input_code = rlt::save_code_split(device, input, "input", true, 1);
        auto output_code = rlt::save_code_split(device, output, "output", true, 1);
        auto header = model_code.header + "\n" + input_code.header + "\n" + output_code.header;
        auto body = model_code.body + "\n" + input_code.body + "\n" + output_code.body;

        auto wrapped = rlt::embed_in_namespace(device, {header, body}, "rl_tools_export", 0);

        auto output = wrapped.header + "\n" + wrapped.body;
        //        std::cout << "output: " << output << std::endl;
        //        std::filesystem::create_directories("data");
        std::ofstream file;
        std::string output_path = "tests/data/nn_models_multi_agent_wrapper_persist_code_forward.h" + std::string((get_env_var("GITHUB_ACTIONS") ? ".disabled" : ""));
        file.open(output_path, std::ios::out | std::ios::trunc);
        std::cout << "Working directory: " << std::filesystem::current_path() << std::endl;
        std::cout << "Full file path: " << std::filesystem::absolute(output_path) << std::endl;
        file << output;
        file.close();
    }
}
