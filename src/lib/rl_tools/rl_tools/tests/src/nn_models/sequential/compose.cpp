#include <rl_tools/operations/cpu.h>

#include <rl_tools/nn/optimizers/adam/instance/operations_generic.h>
#include <rl_tools/nn/layers/standardize/operations_generic.h>
#include <rl_tools/nn_models/mlp_unconditional_stddev/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>

namespace rlt = rl_tools;

#include <gtest/gtest.h>


using DEVICE = rlt::devices::DefaultCPU;
using T = double;
using TYPE_POLICY = rlt::numeric_types::Policy<T>;
using TI = DEVICE::index_t;
constexpr TI INPUT_DIM = 10;
constexpr TI OUTPUT_DIM = 5;
constexpr TI BATCH_SIZE_DEFINITION_MLP = 28;
constexpr TI BATCH_SIZE_DEFINITION = 32;
constexpr TI BATCH_SIZE_OTHER = 30;


template <typename CAPABILITY>
struct Actor{
    using ACTOR_INPUT_SHAPE = rlt::tensor::Shape<TI, 1, BATCH_SIZE_DEFINITION, INPUT_DIM>;
    using ACTOR_SPEC = rlt::nn_models::mlp::Configuration<TYPE_POLICY, TI, OUTPUT_DIM, 3, 256, rlt::nn::activation_functions::ActivationFunction::RELU, rlt::nn::activation_functions::IDENTITY>;
    using ACTOR_TYPE = rlt::nn_models::mlp_unconditional_stddev::BindConfiguration<ACTOR_SPEC>;
    using STANDARDIZATION_LAYER_SPEC = rlt::nn::layers::standardize::Configuration<TYPE_POLICY, TI>;
    using STANDARDIZATION_LAYER = rlt::nn::layers::standardize::BindConfiguration<STANDARDIZATION_LAYER_SPEC>;

    template <typename T_CONTENT, typename T_NEXT_MODULE = rlt::nn_models::sequential::OutputModule>
    using Module = typename rlt::nn_models::sequential::Module<T_CONTENT, T_NEXT_MODULE>;
    using MODULE_CHAIN = Module<STANDARDIZATION_LAYER, Module<ACTOR_TYPE>>;

    using MODEL = rlt::nn_models::sequential::Build<CAPABILITY, MODULE_CHAIN, ACTOR_INPUT_SHAPE>;
};
using CAPABILITY = rlt::nn::capability::Forward<>;
using ACTOR = Actor<CAPABILITY>::MODEL;

TEST(RL_TOOLS_NN_MODELS_SEQUENTIAL_COMPOSE, MAIN){
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 1);

    ACTOR actor;
    using ACTOR_OTHER = ACTOR::CHANGE_BATCH_SIZE<TI, BATCH_SIZE_OTHER>;
    ACTOR_OTHER::template Buffer<> buffer;
    rlt::Tensor<rlt::tensor::Specification<T, TI, typename ACTOR_OTHER::INPUT_SHAPE, false>> input;
    rlt::Tensor<rlt::tensor::Specification<T, TI, typename ACTOR_OTHER::OUTPUT_SHAPE, false>> output;

    rlt::malloc(device, actor);
    rlt::malloc(device, buffer);

    rlt::init_weights(device, actor, rng);
    rlt::evaluate(device, actor, input, output, buffer, rng);

    rlt::free(device, actor);
}
