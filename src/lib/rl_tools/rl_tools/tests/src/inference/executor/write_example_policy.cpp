#include <rl_tools/operations/cpu.h>

#include <rl_tools/nn/optimizers/adam/instance/operations_generic.h>
#include <rl_tools/nn/layers/dense/operations_generic.h>
#include <rl_tools/nn/layers/gru/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>
#include <rl_tools/nn/optimizers/adam/operations_generic.h>

#include <rl_tools/numeric_types/persist_code.h>
#include <rl_tools/containers/matrix/persist_code.h>
#include <rl_tools/containers/tensor/persist_code.h>
#include <rl_tools/nn/parameters/persist_code.h>
#include <rl_tools/nn/optimizers/adam/instance/persist_code.h>
#include <rl_tools/nn/layers/dense/persist_code.h>
#include <rl_tools/nn/layers/gru/persist_code.h>
#include <rl_tools/nn_models/sequential/persist_code.h>

namespace rlt = rl_tools;

using DEVICE = rlt::devices::DefaultCPU;
using RNG = DEVICE::SPEC::RANDOM::ENGINE<>;

using TI = typename DEVICE::index_t;

constexpr TI SEQUENCE_LENGTH = 10;
constexpr TI BATCH_SIZE = 10;
constexpr TI INPUT_DIM = 22;
constexpr TI HIDDEN_DIM = 10;
constexpr TI OUTPUT_DIM = 4;


using TYPE_POLICY = rlt::numeric_types::Policy<float>;


using INPUT_SHAPE = rlt::tensor::Shape<TI, SEQUENCE_LENGTH, BATCH_SIZE, INPUT_DIM>;
using INPUT_LAYER_SPEC = rlt::nn::layers::dense::Configuration<TYPE_POLICY, TI, HIDDEN_DIM, rlt::nn::activation_functions::RELU>;
using INPUT_LAYER = rlt::nn::layers::dense::BindConfiguration<INPUT_LAYER_SPEC>;
using GRU_CONFIG = rlt::nn::layers::gru::Configuration<TYPE_POLICY, TI, HIDDEN_DIM, rlt::nn::parameters::groups::Normal, true>;
using GRU = rlt::nn::layers::gru::BindConfiguration<GRU_CONFIG>;
//    using GRU2_CONFIG = rlt::nn::layers::gru::Configuration<T, TI, PARAMS::HIDDEN_DIM, rlt::nn::parameters::groups::Normal, true>;
//    using GRU2 = rlt::nn::layers::gru::BindConfiguration<GRU2_CONFIG>;
using DOWN_PROJECTION_LAYER_CONFIG = rlt::nn::layers::dense::Configuration<TYPE_POLICY, TI, OUTPUT_DIM, rlt::nn::activation_functions::ActivationFunction::IDENTITY, rlt::nn::layers::dense::DefaultInitializer<TYPE_POLICY, TI>, rlt::nn::parameters::groups::Normal>;
using DOWN_PROJECTION_LAYER_TEMPLATE = rlt::nn::layers::dense::BindConfiguration<DOWN_PROJECTION_LAYER_CONFIG>;
using DENSE_LAYER_CONFIG = rlt::nn::layers::dense::Configuration<TYPE_POLICY, TI, OUTPUT_DIM, rlt::nn::activation_functions::ActivationFunction::IDENTITY, rlt::nn::layers::dense::DefaultInitializer<TYPE_POLICY, TI>, rlt::nn::parameters::groups::Normal>;
using DENSE_LAYER_TEMPLATE = rlt::nn::layers::dense::BindConfiguration<DENSE_LAYER_CONFIG>;

template <typename T_CONTENT, typename T_NEXT_MODULE = rlt::nn_models::sequential::OutputModule>
using Module = typename rlt::nn_models::sequential::Module<T_CONTENT, T_NEXT_MODULE>;

//    using MODULE_CHAIN = Module<EMBEDDING_LAYER, Module<GRU, Module<GRU2, Module<DOWN_PROJECTION_LAYER_TEMPLATE, Module<DENSE_LAYER_TEMPLATE>>>>>;
using MODULE_CHAIN = Module<INPUT_LAYER, Module<GRU, Module<DOWN_PROJECTION_LAYER_TEMPLATE, Module<DENSE_LAYER_TEMPLATE>>>>;
using CAPABILITY = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam>;
using MODEL = rlt::nn_models::sequential::Build<CAPABILITY, MODULE_CHAIN, INPUT_SHAPE>;

using ADAM_SPEC = rlt::nn::optimizers::adam::Specification<TYPE_POLICY, TI>;
using OPTIMIZER = rlt::nn::optimizers::Adam<ADAM_SPEC>;


#include <fstream>
#include <gtest/gtest.h>

TEST(RL_TOOLS_INFERENCE_EXECUTOR, WRITE_EXAMPLE_POLICY){
    DEVICE device;
    RNG rng;
    MODEL model;
    MODEL::Buffer<> buffer;
    OPTIMIZER optimizer;
    rlt::Tensor<rlt::tensor::Specification<typename TYPE_POLICY::DEFAULT, TI, MODEL::INPUT_SHAPE>> input_example;
    rlt::Tensor<rlt::tensor::Specification<typename TYPE_POLICY::DEFAULT, TI, MODEL::OUTPUT_SHAPE>> output_example;

    rlt::init(device);

    constexpr TI seed = 0;
    rlt::malloc(device, rng);
    rlt::init(device, rng, seed);
    rlt::malloc(device, optimizer);
    rlt::malloc(device, input_example);
    rlt::malloc(device, output_example);
    rlt::malloc(device, model);
    rlt::malloc(device, buffer);

    rlt::init_weights(device, model, rng);
    rlt::reset_optimizer_state(device, optimizer, model);
    rlt::randn(device, input_example, rng);

    rlt::evaluate(device, model, input_example, output_example, buffer, rng);

    std::string code = "namespace rl_tools::checkpoint{\n";
    code += rlt::save_code(device, model, "actor", true);
    code += "\n}";

    code += "namespace rl_tools::checkpoint::example{\n";
    code += rlt::save_code(device, input_example, "input", true);
    code += rlt::save_code(device, output_example, "output", true);
    code += "\n}";

    std::stringstream output_ss;
    output_ss << "\n" << "namespace rl_tools::checkpoint::meta{";
    output_ss << "\n" << "   " << "char name[] = \"test_policy\";";
    output_ss << "\n" << "   " << "char commit_hash[] = \"" << RL_TOOLS_STRINGIFY(RL_TOOLS_COMMIT_HASH) << "\";";
    output_ss << "\n" << "}";

    code += output_ss.str();

    std::ofstream output_file_stub("tests/data/test_inference_executor_policy.h");
    output_file_stub << code;
    output_file_stub.close();

}
