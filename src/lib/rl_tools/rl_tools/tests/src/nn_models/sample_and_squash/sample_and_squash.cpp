#include <rl_tools/operations/cpu.h>
#include <rl_tools/nn/activation_functions.h>
#include <rl_tools/nn/layers/dense/operations_generic.h>
#include <rl_tools/nn/layers/sample_and_squash/operations_generic.h>
#include <rl_tools/nn_models/mlp/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>

namespace rlt = rl_tools;

using DEVICE = rlt::devices::DefaultCPU;

using T = float;
using TYPE_POLICY = rlt::numeric_types::Policy<T>;
using TI = typename DEVICE::index_t;

constexpr TI INPUT_DIM = 10;
constexpr TI OUTPUT_DIM = 5;
constexpr TI NUM_LAYERS = 3;
constexpr TI HIDDEN_DIM = 16;
constexpr auto ACTIVATION_FUNCTION = rlt::nn::activation_functions::TANH;
constexpr TI BATCH_SIZE = 16;
using INPUT_SHAPE = rlt::tensor::Shape<TI, 1, BATCH_SIZE, INPUT_DIM>;
using MLP_SPEC = rlt::nn_models::mlp::Configuration<TYPE_POLICY, TI, 2*OUTPUT_DIM, NUM_LAYERS, HIDDEN_DIM, ACTIVATION_FUNCTION, rlt::nn::activation_functions::IDENTITY, rlt::nn::layers::dense::DefaultInitializer<TYPE_POLICY, TI>>;
using MLP_TYPE = rlt::nn_models::mlp::BindConfiguration<MLP_SPEC>;

using SAMPLE_AND_SQUASH_PARAMETERS = rlt::nn::layers::sample_and_squash::DefaultParameters<TYPE_POLICY>;
using SAMPLE_AND_SQUASH_SPEC = rlt::nn::layers::sample_and_squash::Configuration<TYPE_POLICY, TI, SAMPLE_AND_SQUASH_PARAMETERS>;
using SAMPLE_AND_SQUASH = rlt::nn::layers::sample_and_squash::BindConfiguration<SAMPLE_AND_SQUASH_SPEC>;

//using SAMPLE_AND_SQUASH_MODULE_SPEC = rlt::nn_models::sequential::Specification<SAMPLE_AND_SQUASH>;
using CAPABILITY_ADAM = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam>;

template <typename T_CONTENT, typename T_NEXT_MODULE = rlt::nn_models::sequential::OutputModule>
using Module = typename rlt::nn_models::sequential::Module<T_CONTENT, T_NEXT_MODULE>;
using MODULE_CHAIN = Module<MLP_TYPE, Module<SAMPLE_AND_SQUASH>>;

using ACTOR = rlt::nn_models::sequential::Build<CAPABILITY_ADAM, MODULE_CHAIN, INPUT_SHAPE>;

int main(){
    ACTOR actor;
    ACTOR::CONTENT::Buffer<> actor_buffer;
    ACTOR::Buffer<> actor_buffer_sequential;
    DEVICE device;

    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);

    rlt::Tensor<rlt::tensor::Specification<T, TI, ACTOR::INPUT_SHAPE, false>> input;
    rlt::Tensor<rlt::tensor::Specification<T, TI, ACTOR::CONTENT::OUTPUT_SHAPE, false>> intermediate_output;
    rlt::Tensor<rlt::tensor::Specification<T, TI, ACTOR::OUTPUT_SHAPE, false>> output, output_sequential;
    rlt::malloc(device, actor);
    rlt::malloc(device, actor_buffer);
    rlt::malloc(device, actor_buffer_sequential);


    rlt::randn(device, input, rng);
    rlt::init_weights(device, actor, rng);

    auto rng2 = rng;
    rlt::evaluate(device, actor, input, output_sequential, actor_buffer_sequential, rng);
    rlt::evaluate(device, actor.content, input, intermediate_output, actor_buffer, rng2);


    auto& sas_buffer = rlt::get_buffer<1>(actor_buffer_sequential);
    auto& sas_layer = rlt::get_layer<1>(actor);

    rlt::print(device, sas_layer.log_probabilities);


    T abs_diff = rlt::abs_diff(device, output, output_sequential);

    rlt::print(device, output);
    std::cout << "abs_diff: " << abs_diff << std::endl;

    return 0;
}