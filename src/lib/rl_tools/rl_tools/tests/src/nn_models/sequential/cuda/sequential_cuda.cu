#include <rl_tools/operations/cpu_mux.h>

#include <rl_tools/nn/layers/dense/operations_generic.h>
#include <rl_tools/nn/layers/sample_and_squash/operations_generic.h>
#include <rl_tools/nn_models/mlp/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>

namespace rlt = rl_tools;

using DEVICE = rlt::devices::DEVICE_FACTORY<>;
using RNG = DEVICE::SPEC::RANDOM::ENGINE<>;
using T = float;
using TYPE_POLICY = rlt::numeric_types::Policy<T>;
using TI = typename DEVICE::index_t;


constexpr TI SEQUENCE_LENGTH = 1;
constexpr TI BATCH_SIZE = 10;
constexpr TI INPUT_DIM = 3;
constexpr TI OUTPUT_DIM = 4;

constexpr TI MLP_NUM_LAYERS = 3;
constexpr TI MLP_HIDDEN_DIM = 4;
constexpr auto MLP_HIDDEN_ACTIVATION = rlt::nn::activation_functions::RELU;
constexpr auto MLP_OUTPUT_ACTIVATION = rlt::nn::activation_functions::IDENTITY;


using INPUT_SHAPE = rlt::tensor::Shape<TI, SEQUENCE_LENGTH, BATCH_SIZE, INPUT_DIM>;
using MLP_CONFIG = rlt::nn_models::mlp::Configuration<TYPE_POLICY, TI, OUTPUT_DIM, MLP_NUM_LAYERS, MLP_HIDDEN_DIM, MLP_HIDDEN_ACTIVATION, MLP_OUTPUT_ACTIVATION>;
using MLP = rlt::nn_models::mlp::BindConfiguration<MLP_CONFIG>;
using SAMPLE_AND_SQUASH_CONFIG = rlt::nn::layers::sample_and_squash::Configuration<TYPE_POLICY, TI>;
using SAMPLE_AND_SQUASH = rlt::nn::layers::sample_and_squash::BindConfiguration<SAMPLE_AND_SQUASH_CONFIG>;
template <typename T_CONTENT, typename T_NEXT_MODULE = rlt::nn_models::sequential::OutputModule>
using Module = typename rlt::nn_models::sequential::Module<T_CONTENT, T_NEXT_MODULE>;
using MODULE_CHAIN = Module<MLP, Module<SAMPLE_AND_SQUASH>>;


using CAPABILITY = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam, true>;

using MODEL = rlt::nn_models::sequential::Build<CAPABILITY, MODULE_CHAIN, INPUT_SHAPE>;



int main() {
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);
    rlt::Tensor<rlt::tensor::Specification<T, TI, INPUT_SHAPE>> input;
    rlt::Tensor<rlt::tensor::Specification<T, TI, typename MODEL::OUTPUT_SHAPE>> output;
    MODEL model;
    MODEL::Buffer<> buffer;

    rlt::malloc(device, input);
    rlt::malloc(device, output);
    rlt::malloc(device, model);
    rlt::malloc(device, buffer);

    rlt::init_weights(device, model, rng);
    rlt::evaluate(device, model, input, output, buffer, rng);

    return 0;
}
