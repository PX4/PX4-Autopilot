// this is a test to check if everything compiles without any dependencies (by replacing the dependency based math functions with dummy implementations)
#ifdef RL_TOOLS_OPERATIONS_CPU
#include <rl_tools/operations/cpu.h>
#else
#include <rl_tools/operations/dummy.h>
#endif

#include <rl_tools/rl/environments/pendulum/operations_generic.h>
#include <rl_tools/nn/optimizers/adam/instance/operations_generic.h>
#include <rl_tools/nn_models/models.h>
#include <rl_tools/nn/layers/standardize/operations_generic.h>
#include <rl_tools/nn/layers/sample_and_squash/operations_generic.h>
#include <rl_tools/nn/layers/td3_sampling/operations_generic.h>
#include <rl_tools/nn_models/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>
#include <rl_tools/nn/optimizers/adam/operations_generic.h>
#include <rl_tools/rl/rl.h>
#include <rl_tools/rl/components/off_policy_runner/operations_generic.h>
#include <rl_tools/rl/algorithms/td3/operations_generic.h>

#include <rl_tools/rl/utils/evaluation/operations_generic.h>

namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools;

#ifdef RL_TOOLS_OPERATIONS_CPU
using DEVICE = rlt::devices::DefaultCPU;
using NN_DEVICE = rlt::devices::DefaultCPU;
using AC_DEVICE = rlt::devices::DefaultCPU;
#else
using DEVICE = rlt::devices::DefaultDummy;
using NN_DEVICE = rlt::devices::DefaultDummy;
using AC_DEVICE = rlt::devices::DefaultDummy;
#endif
using T = float;
using TYPE_POLICY = rlt::numeric_types::Policy<float>;
using TI = typename DEVICE::index_t;
typedef rlt::rl::environments::pendulum::Specification<TYPE_POLICY::DEFAULT, DEVICE::index_t, rlt::rl::environments::pendulum::DefaultParameters<TYPE_POLICY::DEFAULT>> PENDULUM_SPEC;
typedef rlt::rl::environments::Pendulum<PENDULUM_SPEC> ENVIRONMENT;
ENVIRONMENT envs[1];
ENVIRONMENT::Parameters env_parameters[1];
ENVIRONMENT& env = envs[0];

template <typename T>
struct TD3PendulumParameters: rlt::rl::algorithms::td3::DefaultParameters<T, AC_DEVICE::index_t>{
    constexpr static typename DEVICE::index_t CRITIC_BATCH_SIZE = 100;
    constexpr static typename DEVICE::index_t ACTOR_BATCH_SIZE = 100;
};

using TD3_PARAMETERS = TD3PendulumParameters<TYPE_POLICY>;

using ACTOR_INPUT_SHAPE = rlt::tensor::Shape<DEVICE::index_t, 1, TD3_PARAMETERS::ACTOR_BATCH_SIZE, ENVIRONMENT::Observation::DIM>;
using ACTOR_NETWORK_SPEC = rlt::nn_models::mlp::Configuration<TYPE_POLICY, DEVICE::index_t, ENVIRONMENT::ACTION_DIM, 3, 64, rlt::nn::activation_functions::RELU, rlt::nn::activation_functions::TANH>;
using ACTOR = rlt::nn_models::mlp::BindConfiguration<ACTOR_NETWORK_SPEC>;
using CRITIC_INPUT_SHAPE = rlt::tensor::Shape<DEVICE::index_t, 1, TD3_PARAMETERS::CRITIC_BATCH_SIZE, ENVIRONMENT::Observation::DIM + ENVIRONMENT::ACTION_DIM>;
using CRITIC_NETWORK_SPEC = rlt::nn_models::mlp::Configuration<TYPE_POLICY, DEVICE::index_t, 1, 3, 64, rlt::nn::activation_functions::RELU, rlt::nn::activation_functions::IDENTITY>;


using ACTOR_CAPABILITY = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam>;
using OPTIMIZER_SPEC = typename rlt::nn::optimizers::adam::Specification<TYPE_POLICY, TI>;
using OPTIMIZER = rlt::nn::optimizers::Adam<OPTIMIZER_SPEC>;

template <typename T_CONTENT, typename T_NEXT_MODULE = rlt::nn_models::sequential::OutputModule>
using Module = typename rlt::nn_models::sequential::Module<T_CONTENT, T_NEXT_MODULE>;
using MODULE_CHAIN = Module<ACTOR>;
using ACTOR_NETWORK_TYPE = rlt::nn_models::sequential::Build<ACTOR_CAPABILITY, MODULE_CHAIN, ACTOR_INPUT_SHAPE>;
//using ACTOR_NETWORK_TYPE = rlt::nn_models::mlp::NeuralNetwork<ACTOR_NETWORK_SPEC, ACTOR_CAPABILITY, ACTOR_INPUT_SHAPE>;

//using ACTOR_TARGET_NETWORK_TYPE = rlt::nn_models::mlp::NeuralNetwork<ACTOR_NETWORK_SPEC, rlt::nn::capability::Forward<>, ACTOR_INPUT_SHAPE>;
using ACTOR_TARGET_NETWORK_TYPE = rlt::nn_models::sequential::Build<rlt::nn::capability::Forward<>, MODULE_CHAIN, ACTOR_INPUT_SHAPE>;

using CRITIC_CAPABILITY = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam>;
using CRITIC_NETWORK_TYPE = rlt::nn_models::mlp::NeuralNetwork<CRITIC_NETWORK_SPEC, CRITIC_CAPABILITY, CRITIC_INPUT_SHAPE>;

using CRITIC_TARGET_NETWORK_TYPE = rlt::nn_models::mlp::NeuralNetwork<CRITIC_NETWORK_SPEC, rlt::nn::capability::Forward<>, CRITIC_INPUT_SHAPE>;

using TD3_SPEC = rlt::rl::algorithms::td3::Specification<TYPE_POLICY, AC_DEVICE::index_t, ENVIRONMENT, ACTOR_NETWORK_TYPE, ACTOR_TARGET_NETWORK_TYPE, CRITIC_NETWORK_TYPE, CRITIC_TARGET_NETWORK_TYPE, OPTIMIZER, TD3_PARAMETERS>;
using ActorCriticType = rlt::rl::algorithms::td3::ActorCritic<TD3_SPEC>;


constexpr typename DEVICE::index_t REPLAY_BUFFER_CAP = 500000;
constexpr typename DEVICE::index_t EPISODE_STEP_LIMIT = 200;

using POLICIES = rl_tools::utils::Tuple<TI, ACTOR_NETWORK_TYPE>;

using OFF_POLICY_RUNNER_SPEC = rlt::rl::components::off_policy_runner::Specification<TYPE_POLICY, AC_DEVICE::index_t, ENVIRONMENT, POLICIES, rlt::rl::components::off_policy_runner::ParametersDefault<TYPE_POLICY, AC_DEVICE::index_t>>;
rlt::rl::components::OffPolicyRunner<OFF_POLICY_RUNNER_SPEC> off_policy_runner;
ActorCriticType actor_critic;
const T STATE_TOLERANCE = 0.00001;
constexpr int N_WARMUP_STEPS = ActorCriticType::SPEC::PARAMETERS::ACTOR_BATCH_SIZE;
static_assert(ActorCriticType::SPEC::PARAMETERS::ACTOR_BATCH_SIZE == ActorCriticType::SPEC::PARAMETERS::CRITIC_BATCH_SIZE);

int main() {
    AC_DEVICE::SPEC::LOGGING logger;
    AC_DEVICE device;
    NN_DEVICE nn_device;
    rlt::malloc(device, off_policy_runner);
    rlt::malloc(device, actor_critic);
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng);
    rlt::init(device, actor_critic, rng);

    rlt::rl::environments::DummyUI ui;

    using CRITIC_BATCH_SPEC = rlt::rl::components::off_policy_runner::SequentialBatchSpecification<decltype(off_policy_runner)::SPEC, 1, ActorCriticType::SPEC::PARAMETERS::CRITIC_BATCH_SIZE>;
    rlt::rl::components::off_policy_runner::SequentialBatch<CRITIC_BATCH_SPEC> critic_batch;
    rlt::rl::algorithms::td3::CriticTrainingBuffers<rlt::rl::algorithms::td3::CriticTrainingBuffersSpecification<ActorCriticType::SPEC>> critic_training_buffers;
    CRITIC_NETWORK_TYPE::Buffer<> critic_buffers[2];
    rlt::malloc(device, critic_batch);
    rlt::malloc(device, critic_training_buffers);
    rlt::malloc(device, critic_buffers[0]);
    rlt::malloc(device, critic_buffers[1]);

    using ACTOR_BATCH_SPEC = rlt::rl::components::off_policy_runner::SequentialBatchSpecification<decltype(off_policy_runner)::SPEC, 1, ActorCriticType::SPEC::PARAMETERS::ACTOR_BATCH_SIZE>;
    rlt::rl::components::off_policy_runner::SequentialBatch<ACTOR_BATCH_SPEC> actor_batch;
    rlt::rl::algorithms::td3::ActorTrainingBuffers<rlt::rl::algorithms::td3::ActorTrainingBuffersSpecification<ActorCriticType::SPEC>> actor_training_buffers;
    ACTOR_NETWORK_TYPE::Buffer<> actor_buffers[2];
    using ACTOR_EVAL_TYPE = typename ACTOR_NETWORK_TYPE::template CHANGE_BATCH_SIZE<DEVICE::index_t, OFF_POLICY_RUNNER_SPEC::PARAMETERS::N_ENVIRONMENTS>;
    ACTOR_EVAL_TYPE::template Buffer<> actor_buffers_eval;
    using ACTOR_EVALUATION_TYPE = typename ACTOR_NETWORK_TYPE::template CHANGE_BATCH_SIZE<DEVICE::index_t, 10>;
    ACTOR_EVALUATION_TYPE::template Buffer<> actor_buffers_evaluation;
    rlt::malloc(device, actor_batch);
    rlt::malloc(device, actor_training_buffers);
    rlt::malloc(device, actor_buffers_eval);
    rlt::malloc(device, actor_buffers_evaluation);
    rlt::malloc(device, actor_buffers[0]);
    rlt::malloc(device, actor_buffers[1]);



    rlt::init(device, off_policy_runner);

    for(int step_i = 0; step_i < 15000; step_i++){
        rlt::step<0>(device, off_policy_runner, actor_critic.actor, actor_buffers_eval, rng);

        if(step_i > N_WARMUP_STEPS){
            if(step_i % 1000 == 0){
                rlt::log(device, device.logger, "step_i: ", step_i);
            }
            for(int critic_i = 0; critic_i < 2; critic_i++){
                auto target_next_action_noise_matrix_view = rlt::matrix_view(device, critic_training_buffers.target_next_action_noise);
                rlt::target_action_noise(device, actor_critic, target_next_action_noise_matrix_view, rng);
                rlt::gather_batch(device, off_policy_runner, critic_batch, rng);
                rlt::train_critic(device, actor_critic, actor_critic.critics[critic_i], critic_batch, actor_critic.critic_optimizers[critic_i], actor_buffers[critic_i], actor_buffers[critic_i], critic_buffers[critic_i], critic_buffers[critic_i], critic_training_buffers, rng);
            }
            if(step_i % 2 == 0){
                rlt::gather_batch(device, off_policy_runner, actor_batch, rng);
                rlt::train_actor(device, actor_critic, actor_batch, actor_critic.actor_optimizer, actor_buffers[0], critic_buffers[0], actor_training_buffers, rng);
                rlt::update_critic_targets(device, actor_critic);
                rlt::update_actor_target(device, actor_critic);
            }
        }
        if(step_i % 1000 == 0){
            using RESULT_SPEC = rlt::rl::utils::evaluation::Specification<TYPE_POLICY, TI, ENVIRONMENT, 10, EPISODE_STEP_LIMIT>;
            rlt::rl::utils::evaluation::Result<RESULT_SPEC> result;
            rlt::evaluate(device, env, ui, actor_critic.actor, result, rng, rlt::Mode<rlt::mode::Evaluation<>>{});
            rlt::log(device, device.logger, "Mean return: ", result.returns_mean);
            if(result.returns_mean > -200000){
                return 0;
            }
        }
    }
    return -1;

}
