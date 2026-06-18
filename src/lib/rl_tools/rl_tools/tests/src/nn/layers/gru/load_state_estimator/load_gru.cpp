#include <rl_tools/operations/cpu.h>

#include <rl_tools/nn/layers/dense/operations_generic.h>
#include <rl_tools/nn/layers/gru/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>



#include <rl_tools/nn/parameters/persist.h>
#include <rl_tools/nn/layers/dense/persist.h>
#include <rl_tools/nn/layers/gru/persist.h>
#include <rl_tools/nn_models/sequential/persist.h>

namespace rlt = rl_tools;


using DEVICE = rl_tools::devices::DefaultCPU;
using T = float;
using TYPE_POLICY = rlt::numeric_types::Policy<T>;
using TI = typename DEVICE::index_t;
using RNG = typename DEVICE::SPEC::RANDOM::ENGINE<>;

constexpr TI SEQUENCE_LENGTH = 200;
constexpr TI BATCH_SIZE = 20;
constexpr TI HIDDEN_DIM = 32;
constexpr TI INPUT_DIM = 1;
constexpr TI OUTPUT_DIM = 2;
using GRU_CONFIG = rlt::nn::layers::gru::Configuration<TYPE_POLICY, TI, HIDDEN_DIM>;
using GRU = rlt::nn::layers::gru::BindConfiguration<GRU_CONFIG>;
using DENSE_CONFIG = rlt::nn::layers::dense::Configuration<TYPE_POLICY, TI, OUTPUT_DIM, rlt::nn::activation_functions::IDENTITY>;
using DENSE = rlt::nn::layers::dense::BindConfiguration<DENSE_CONFIG>;

template <typename T_CONTENT, typename T_NEXT_MODULE = rlt::nn_models::sequential::OutputModule>
using Module = typename rlt::nn_models::sequential::Module<T_CONTENT, T_NEXT_MODULE>;
using MODULE_CHAIN = Module<GRU, Module<DENSE>>;

using CAPABILITY = rlt::nn::capability::Forward<>;
using INPUT_SHAPE = rlt::tensor::Shape<TI, SEQUENCE_LENGTH, BATCH_SIZE, INPUT_DIM>;
using MODEL = rlt::nn_models::sequential::Build<CAPABILITY, MODULE_CHAIN, INPUT_SHAPE>;

#include "../../../../utils/utils.h"

#include <gtest/gtest.h>


constexpr T EPSILON = 1e-7;

TEST(RL_TOOLS_NN_LAYERS_GRU, LOAD_STATE_ESTIMATOR){
    DEVICE device;
    MODEL model;
    MODEL::Buffer<> buffer;
    RNG rng;
    rlt::Tensor<rlt::tensor::Specification<T, TI, INPUT_SHAPE>> input;
    rlt::Tensor<rlt::tensor::Specification<T, TI, MODEL::OUTPUT_SHAPE>> output, output_target;
    rlt::init(device);
    rlt::malloc(device, rng);
    rlt::malloc(device, model);
    rlt::malloc(device, input);
    rlt::malloc(device, output);
    rlt::malloc(device, output_target);
    rlt::malloc(device, buffer);

    rlt::init(device, rng);
    std::string DATA_FILE_NAME = "load_gru_pytorch_model.h5";
    const char *data_path_stub = RL_TOOLS_MACRO_TO_STR(RL_TOOLS_TEST_DATA_PATH);
    std::string DATA_FILE_PATH = std::string(data_path_stub) + "/" + DATA_FILE_NAME;
    std::cout << "DATA_FILE_PATH: " << DATA_FILE_PATH << std::endl;
    auto file = HighFive::File(std::string(DATA_FILE_PATH), HighFive::File::ReadOnly);
    auto model_group = rlt::get_group(device, file, "model");
    rlt::load(device, model, model_group);
    auto test_group = rlt::get_group(device, file, "test");
    rlt::load(device, input, test_group, "input");
    rlt::load(device, output_target, test_group, "output");
    rlt::evaluate(device, model, input, output, buffer, rng);


    T diff = rlt::abs_diff(device, output, output_target);
    T diff_per_var = diff / decltype(output)::SPEC::SIZE;
    rlt::log(device, device.logger, "Diff: ", diff_per_var);
    ASSERT_LT(diff_per_var, EPSILON);
}