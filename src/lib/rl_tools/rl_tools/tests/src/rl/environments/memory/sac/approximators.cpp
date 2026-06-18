#include <rl_tools/operations/cpu_mux.h>
#include <rl_tools/nn/optimizers/adam/instance/operations_generic.h>
#include <rl_tools/nn/operations_cpu_mux.h>
#include <rl_tools/nn/layers/gru/operations_generic.h>
#include <rl_tools/nn/layers/sample_and_squash/operations_generic.h>
#include <rl_tools/rl/environments/memory/operations_generic.h>
#include <rl_tools/nn_models/mlp/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>
#include <rl_tools/nn/optimizers/adam/operations_generic.h>


#include <rl_tools/rl/algorithms/sac/loop/core/config.h>
#include <rl_tools/rl/loop/steps/evaluation/config.h>
#include <rl_tools/rl/loop/steps/timing/config.h>
#include <rl_tools/rl/algorithms/sac/loop/core/operations_generic.h>
#include <rl_tools/rl/loop/steps/evaluation/operations_generic.h>
#include <rl_tools/rl/loop/steps/timing/operations_cpu.h>

namespace rlt = rl_tools;

#include <rl_tools/rl/algorithms/sac/loop/core/approximators_mlp.h>

#include <gtest/gtest.h>


using DEVICE = rlt::devices::DEVICE_FACTORY<>;
//using DEVICE = rlt::devices::DefaultCPU;
using RNG = DEVICE::SPEC::RANDOM::ENGINE<>;
using T = double;
using TYPE_POLICY = rlt::numeric_types::Policy<T>;
using TI = typename DEVICE::index_t;

constexpr TI SEQUENCE_LENGTH = 16;
constexpr TI BATCH_SIZE = 32;

using ENVIRONMENT_SPEC = rlt::rl::environments::memory::Specification<T, TI, rlt::rl::environments::memory::DefaultParameters<T, TI>>;
using ENVIRONMENT = rlt::rl::environments::Memory<ENVIRONMENT_SPEC>;
struct LOOP_CORE_PARAMETERS: rlt::rl::algorithms::sac::loop::core::DefaultParameters<TYPE_POLICY, TI, ENVIRONMENT>{
    struct SAC_PARAMETERS: rlt::rl::algorithms::sac::DefaultParameters<TYPE_POLICY, TI, ENVIRONMENT::ACTION_DIM>{
        static constexpr TI ACTOR_BATCH_SIZE = BATCH_SIZE;
        static constexpr TI CRITIC_BATCH_SIZE = BATCH_SIZE;
    };
    static constexpr TI STEP_LIMIT = 10000;
    static constexpr TI ACTOR_NUM_LAYERS = 3;
    static constexpr TI ACTOR_HIDDEN_DIM = 64;
    static constexpr TI CRITIC_NUM_LAYERS = 3;
    static constexpr TI CRITIC_HIDDEN_DIM = 64;
    struct ACTOR_OPTIMIZER_PARAMETERS: rlt::nn::optimizers::adam::DEFAULT_PARAMETERS_TENSORFLOW<TYPE_POLICY>{
        static constexpr T ALPHA = 0.01;
    };
};
using RNG = DEVICE::SPEC::RANDOM::ENGINE<>;

TEST(RL_TOOLS_RL_ALGORITHMS_SAC_SEQUENTIAL, APPROXIMATORS){
    TI seed = 0;
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, seed);
    using APPROXIMATORS = rlt::rl::algorithms::sac::loop::core::ConfigApproximatorsGRU<TYPE_POLICY, TI, ENVIRONMENT, LOOP_CORE_PARAMETERS>;
    using CAPABILITY = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam>;
    using ACTOR = APPROXIMATORS::Actor<CAPABILITY>::MODEL;
    using CRITIC = APPROXIMATORS::Critic<CAPABILITY>::MODEL;
    ACTOR actor;
    ACTOR::Buffer<> actor_buffer;
    CRITIC critic;
    CRITIC::Buffer<> critic_buffer;
    CRITIC::CONTENT::Buffer<> critic_gru_buffer;
    APPROXIMATORS::ACTOR_OPTIMIZER actor_optimizer;
    APPROXIMATORS::CRITIC_OPTIMIZER critic_optimizer;

    std::cout << "Actor input shape: ";
    rlt::print(device, ACTOR::INPUT_SHAPE{});
    std::cout << std::endl;
    std::cout << "Actor output shape: ";
    rlt::print(device, ACTOR::OUTPUT_SHAPE{});
    std::cout << std::endl;

    rlt::malloc(device, actor_optimizer);
    rlt::malloc(device, critic_optimizer);
    rlt::malloc(device, actor);
    rlt::malloc(device, actor_buffer);
    rlt::malloc(device, critic);
    rlt::malloc(device, critic_buffer);
    rlt::malloc(device, critic_gru_buffer);
    rlt::init_weights(device, actor, rng);
    rlt::init_weights(device, critic, rng);
    rlt::init(device, actor_optimizer);
    rlt::init(device, critic_optimizer);
    rlt::reset_optimizer_state(device, actor_optimizer, actor);
    rlt::reset_optimizer_state(device, critic_optimizer, critic);

    rlt::Tensor<rlt::tensor::Specification<T, TI, ACTOR::INPUT_SHAPE>> actor_input;
    rlt::Tensor<rlt::tensor::Specification<T, TI, ACTOR::OUTPUT_SHAPE>> actor_output_evaluate, actor_output, d_actor_output, actor_output_after_update;
    rlt::Tensor<rlt::tensor::Specification<T, TI, CRITIC::INPUT_SHAPE>> critic_input;
    rlt::Tensor<rlt::tensor::Specification<T, TI, CRITIC::CONTENT::OUTPUT_SHAPE>> critic_gru_output;
    rlt::Tensor<rlt::tensor::Specification<T, TI, CRITIC::OUTPUT_SHAPE>> critic_output, d_critic_output;
    rlt::malloc(device, actor_input);
    rlt::malloc(device, actor_output_evaluate);
    rlt::malloc(device, actor_output);
    rlt::malloc(device, d_actor_output);
    rlt::malloc(device, actor_output_after_update);
    rlt::malloc(device, critic_input);
    rlt::malloc(device, critic_gru_output);
    rlt::malloc(device, critic_output);
    rlt::malloc(device, d_critic_output);

    rlt::randn(device, actor_input, rng);
    rlt::set_all(device, d_actor_output, 1);
    rlt::randn(device, critic_input, rng);
    rlt::randn(device, d_critic_output, rng);

//    std::cout << "Actual batch size: " << decltype(actor.next_module.content)::INTERNAL_BATCH_SIZE << std::endl;
//    std::cout << "Actual batch size layer: " << decltype(actor.next_module.content.output_layer)::SPEC::BATCH_SIZE << std::endl;
//    std::cout << "Actual batch size layer: " << decltype(actor.next_module.content.output_layer)::ACTUAL_BATCH_SIZE << std::endl;
//    using MLP_OUTPUT = rlt::utils::typing::remove_reference<decltype(rl_tools::output(device, actor))>::type;
//    std::cout << "Actual rows sample and squash output layer: " << MLP_OUTPUT::SPEC::ROWS << std::endl;
//    std::cout << "Actual rows mlp output layer: " << decltype(actor.next_module.content.output_layer.output)::ROWS << std::endl;

    auto output_tensor = to_tensor(device, rl_tools::output(device, actor));
    std::cout << "Output tensor shape: ";
    rlt::print(device, decltype(output_tensor)::SPEC::SHAPE{});
    std::cout << "Critic output shape";
    std::cout << std::endl;
    rlt::print(device, decltype(critic_output)::SPEC::SHAPE{});
    std::cout << std::endl;
//    auto output_tensor_reshaped = reshape_row_major(device, output_tensor, typename MODULE::OUTPUT_SHAPE{});
    rlt::Mode<rlt::mode::Evaluation<>> mode;
    rlt::evaluate(device, actor, actor_input, actor_output_evaluate, actor_buffer, rng, mode);
    rlt::forward(device, actor, actor_input, actor_output, actor_buffer, rng, mode);
    T eval_forward_diff = rlt::abs_diff(device, actor_output_evaluate, actor_output);
    std::cout << "Evaluate Forward diff: " << eval_forward_diff << std::endl;
    ASSERT_LT(eval_forward_diff, 1e-10);
    rlt::utils::assert_exit(device, !rlt::is_nan(device, rlt::matrix_view(device, actor_output)), "Actor output contains NaNs");
    rlt::forward(device, critic, critic_input, critic_output, critic_buffer, rng);
    rlt::zero_gradient(device, actor);
    rlt::zero_gradient(device, critic);
    rlt::backward(device, actor, actor_input, d_actor_output, actor_buffer);
    rlt::utils::assert_exit(device, !rlt::is_nan(device, rlt::matrix_view(device, actor.content.weights.gradient)), "Actor gradient contains NaNs");
    rlt::step(device, actor_optimizer, actor);
    rlt::forward(device, actor, actor_input, actor_output_after_update, actor_buffer, rng);
    T sum = rlt::sum(device, actor_output);
    T sum_after_update = rlt::sum(device, actor_output_after_update);
    std::cout << "Sum before update: " << sum << std::endl;
    std::cout << "Sum after update: " << sum_after_update << std::endl;
    ASSERT_LT(sum_after_update, sum);
}
