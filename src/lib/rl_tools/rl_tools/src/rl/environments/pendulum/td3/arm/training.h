//#define RL_TOOLS_DISABLE_DYNAMIC_MEMORY_ALLOCATIONS
#define RL_TOOLS_DEBUG_CONTAINER_COUNT_MALLOC
#include <rl_tools/operations/arm.h>

namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools;

#include <rl_tools/nn/optimizers/adam/instance/operations_generic.h>
#include <rl_tools/nn/layers/dense/operations_arm/opt.h>
#include <rl_tools/nn/layers/td3_sampling/operations_generic.h>
//#include <rl_tools/nn/layers/dense/operations_arm/dsp.h>
#include <rl_tools/nn/operations_generic.h>
using DEVICE = rlt::devices::arm::Generic<rlt::devices::DefaultARMSpecification>;
using TI = typename DEVICE::index_t;

#include <rl_tools/rl/environments/pendulum/operations_generic.h>
#include <rl_tools/nn_models/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>
#include <rl_tools/nn/optimizers/adam/operations_generic.h>
#include <rl_tools/rl/components/off_policy_runner/operations_generic.h>
#include <rl_tools/rl/algorithms/td3/operations_generic.h>

#include <rl_tools/rl/utils/evaluation/operations_generic.h>
#ifndef RL_TOOLS_DEPLOYMENT_ARDUINO
#include <chrono>
#include <iostream>
#endif

using T = float;
using TYPE_POLICY = rlt::numeric_types::Policy<T>;
//using CONTAINER_TYPE_TAG = rlt::MatrixDynamicTag;
//using CONTAINER_TYPE_TAG_CRITIC = rlt::MatrixStaticTag;
//using CONTAINER_TYPE_TAG_OFF_POLICY_RUNNER = rlt::MatrixStaticTag;
//using CONTAINER_TYPE_TAG_TRAINING_BUFFERS = rlt::MatrixDynamicTag;
constexpr bool DYNAMIC_ALLOCATION = true;

using PENDULUM_SPEC = rlt::rl::environments::pendulum::Specification<T, DEVICE::index_t, rlt::rl::environments::pendulum::DefaultParameters<T>>;
typedef rlt::rl::environments::Pendulum<PENDULUM_SPEC> ENVIRONMENT;

struct TD3PendulumParameters: rlt::rl::algorithms::td3::DefaultParameters<TYPE_POLICY, DEVICE::index_t>{
    constexpr static typename DEVICE::index_t CRITIC_BATCH_SIZE = 100;
    constexpr static typename DEVICE::index_t ACTOR_BATCH_SIZE = 100;
};

using TD3_PARAMETERS = TD3PendulumParameters;

template <typename T_CONTENT, typename T_NEXT_MODULE = rlt::nn_models::sequential::OutputModule>
using Module = typename rlt::nn_models::sequential::Module<T_CONTENT, T_NEXT_MODULE>;

using ACTOR_INPUT_SHAPE = rlt::tensor::Shape<TI, 1, TD3_PARAMETERS::ACTOR_BATCH_SIZE, ENVIRONMENT::Observation::DIM>;
using ACTOR_SPEC = rlt::nn_models::mlp::Configuration<TYPE_POLICY, DEVICE::index_t, ENVIRONMENT::ACTION_DIM, 3, 64, rlt::nn::activation_functions::RELU, rlt::nn::activation_functions::TANH, rlt::nn::layers::dense::DefaultInitializer<TYPE_POLICY, TI>>;
using CRITIC_INPUT_SHAPE = rlt::tensor::Shape<TI, 1, TD3_PARAMETERS::CRITIC_BATCH_SIZE, ENVIRONMENT::Observation::DIM + ENVIRONMENT::ACTION_DIM>;
using CRITIC_SPEC = rlt::nn_models::mlp::Configuration<TYPE_POLICY, DEVICE::index_t, 1, 3, 64, rlt::nn::activation_functions::RELU, rlt::nn::activation_functions::IDENTITY,  rlt::nn::layers::dense::DefaultInitializer<TYPE_POLICY, TI>>;


using OPTIMIZER_SPEC = typename rlt::nn::optimizers::adam::Specification<TYPE_POLICY, typename DEVICE::index_t>;
using OPTIMIZER = rlt::nn::optimizers::Adam<OPTIMIZER_SPEC>;
using ACTOR_CAPABILITY = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam>;
using ACTOR = rlt::nn_models::mlp::BindConfiguration<ACTOR_SPEC>;

using CRITIC_CAPABILITY = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam>;
using CRITIC = rlt::nn_models::mlp::BindConfiguration<CRITIC_SPEC>;

using ACTOR_MODULE_CHAIN = Module<ACTOR>;
using CRITIC_MODULE_CHAIN = Module<CRITIC>;

using ACTOR_NETWORK_TYPE = rlt::nn_models::sequential::Build<ACTOR_CAPABILITY, ACTOR_MODULE_CHAIN, ACTOR_INPUT_SHAPE>;
using CRITIC_NETWORK_TYPE = rlt::nn_models::sequential::Build<CRITIC_CAPABILITY, CRITIC_MODULE_CHAIN, CRITIC_INPUT_SHAPE>;
using ACTOR_TARGET_NETWORK_TYPE = rlt::nn_models::sequential::Build<rlt::nn::capability::Forward<>, ACTOR_MODULE_CHAIN, ACTOR_INPUT_SHAPE>;
using CRITIC_TARGET_NETWORK_TYPE = rlt::nn_models::sequential::Build<rlt::nn::capability::Forward<>, CRITIC_MODULE_CHAIN, CRITIC_INPUT_SHAPE>;

using TD3_SPEC = rlt::rl::algorithms::td3::Specification<TYPE_POLICY, DEVICE::index_t, ENVIRONMENT, ACTOR_NETWORK_TYPE, ACTOR_TARGET_NETWORK_TYPE, CRITIC_NETWORK_TYPE, CRITIC_TARGET_NETWORK_TYPE, OPTIMIZER, TD3_PARAMETERS>;
using ActorCriticType = rlt::rl::algorithms::td3::ActorCritic<TD3_SPEC>;



constexpr DEVICE::index_t N_STEPS = 10000;
constexpr DEVICE::index_t EVALUATION_INTERVAL = 1000;
constexpr DEVICE::index_t N_EVALUATIONS = N_STEPS / EVALUATION_INTERVAL;
#ifndef RL_TOOLS_DISABLE_EVALUATION
T evaluation_returns[N_EVALUATIONS];
#endif

struct OFF_POLICY_RUNNER_PARAMETERS: rlt::rl::components::off_policy_runner::ParametersDefault<TYPE_POLICY, DEVICE::index_t>{
    static constexpr TI REPLAY_BUFFER_CAPACITY = 10000;
    static constexpr TI EPISODE_STEP_LIMIT = 200;
    static constexpr bool STOCHASTIC_POLICY = false;
    static constexpr bool COLLECT_EPISODE_STATS = false;
    static constexpr TI EPISODE_STATS_BUFFER_SIZE = 0;
    static constexpr T EXPLORATION_NOISE = 0.1;
};
using POLICIES = rl_tools::utils::Tuple<TI, ACTOR_NETWORK_TYPE>;
using OFF_POLICY_RUNNER_SPEC = rlt::rl::components::off_policy_runner::Specification<TYPE_POLICY, DEVICE::index_t, ENVIRONMENT, POLICIES, OFF_POLICY_RUNNER_PARAMETERS, DYNAMIC_ALLOCATION>;
#ifdef RL_TOOLS_DEPLOYMENT_ARDUINO
EXTMEM rlt::rl::components::OffPolicyRunner<OFF_POLICY_RUNNER_SPEC> off_policy_runner;
#else
rlt::rl::components::OffPolicyRunner<OFF_POLICY_RUNNER_SPEC> off_policy_runner;
#endif
ActorCriticType actor_critic;

const T STATE_TOLERANCE = 0.00001;
constexpr int N_WARMUP_STEPS = ActorCriticType::SPEC::PARAMETERS::ACTOR_BATCH_SIZE;
static_assert(ActorCriticType::SPEC::PARAMETERS::ACTOR_BATCH_SIZE == ActorCriticType::SPEC::PARAMETERS::CRITIC_BATCH_SIZE);

ENVIRONMENT envs[decltype(off_policy_runner)::N_ENVIRONMENTS];
ENVIRONMENT::Parameters env_parameters[decltype(off_policy_runner)::N_ENVIRONMENTS];

rlt::rl::components::off_policy_runner::SequentialBatch<rlt::rl::components::off_policy_runner::SequentialBatchSpecification<decltype(off_policy_runner)::SPEC, 1, ActorCriticType::SPEC::PARAMETERS::CRITIC_BATCH_SIZE>> critic_batch;
rlt::rl::algorithms::td3::CriticTrainingBuffers<rlt::rl::algorithms::td3::CriticTrainingBuffersSpecification<ActorCriticType::SPEC, DYNAMIC_ALLOCATION>> critic_training_buffers;
CRITIC_NETWORK_TYPE::Buffer<> critic_buffers;

rlt::rl::components::off_policy_runner::SequentialBatch<rlt::rl::components::off_policy_runner::SequentialBatchSpecification<decltype(off_policy_runner)::SPEC, 1, ActorCriticType::SPEC::PARAMETERS::ACTOR_BATCH_SIZE>> actor_batch;
rlt::rl::algorithms::td3::ActorTrainingBuffers<rlt::rl::algorithms::td3::ActorTrainingBuffersSpecification<ActorCriticType::SPEC, DYNAMIC_ALLOCATION>> actor_training_buffers;
ACTOR_NETWORK_TYPE::Buffer<> actor_buffers;
using ACTOR_EVAL_TYPE = ACTOR_NETWORK_TYPE::template CHANGE_BATCH_SIZE<TI, OFF_POLICY_RUNNER_SPEC::PARAMETERS::N_ENVIRONMENTS>;
ACTOR_EVAL_TYPE::template Buffer<> actor_buffers_eval;

rlt::Matrix<rlt::matrix::Specification<T, DEVICE::index_t, 1, ENVIRONMENT::Observation::DIM, false>> observations_mean;
rlt::Matrix<rlt::matrix::Specification<T, DEVICE::index_t, 1, ENVIRONMENT::Observation::DIM, false>> observations_std;


void train(){

    DEVICE::SPEC::LOGGING logger;
    DEVICE device;
    device.logger = logger;

    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 2);

    rlt::rl::environments::DummyUI ui;

    rlt::malloc(device, actor_critic);
    rlt::malloc(device, off_policy_runner);
    rlt::malloc(device, critic_batch);
    rlt::malloc(device, critic_training_buffers);
    rlt::malloc(device, critic_buffers);
    rlt::malloc(device, actor_batch);
    rlt::malloc(device, actor_training_buffers);
    rlt::malloc(device, actor_buffers_eval);
    rlt::malloc(device, actor_buffers);

#ifndef RL_TOOLS_DEPLOYMENT_ARDUINO
#ifdef RL_TOOLS_DEBUG_CONTAINER_COUNT_MALLOC
    std::cout << "malloc counter: " << device.malloc_counter << std::endl;
#endif
#endif


    rlt::init(device, actor_critic, rng);
    rlt::init(device, off_policy_runner);
    rlt::set_all(device, observations_mean, 0);
    rlt::set_all(device, observations_std, 1);


#ifndef RL_TOOLS_DEPLOYMENT_ARDUINO
    auto start_time = std::chrono::high_resolution_clock::now();
    std::cout << "ActorCritic size: " << sizeof(actor_critic) << std::endl;
    std::cout << "ActorCritic.actor size: " << sizeof(actor_critic.actor) << std::endl;
    std::cout << "ActorCritic.actor_target size: " << sizeof(actor_critic.actor_target) << std::endl;
    std::cout << "ActorCritic.critics[0] size: " << sizeof(actor_critic.critics[0]) << std::endl;
    std::cout << "ActorCritic.critics[1] size: " << sizeof(actor_critic.critics[1]) << std::endl;
    std::cout << "ActorCritic.critics_target[0] size: " << sizeof(actor_critic.critics_target[0]) << std::endl;
    std::cout << "ActorCritic.critics_target[1] size: " << sizeof(actor_critic.critics_target[1]) << std::endl;
    std::cout << "OffPolicyRunner size: " << sizeof(off_policy_runner) << std::endl;
    std::cout << "OffPolicyRunner.replay_buffers size: " << sizeof(off_policy_runner.replay_buffers) << std::endl;
    std::cout << "CriticBatch size: " << sizeof(critic_batch) << std::endl;
    std::cout << "CriticTrainingBuffers size: " << sizeof(critic_training_buffers) << std::endl;
    std::cout << "CriticBuffers size: " << sizeof(critic_buffers) << std::endl;
    std::cout << "ActorBatch size: " << sizeof(actor_batch) << std::endl;
    std::cout << "ActorTrainingBuffers size: " << sizeof(actor_training_buffers) << std::endl;
    std::cout << "ActorBuffers size: " << sizeof(actor_buffers) << std::endl;
    std::cout << "Total: " << sizeof(actor_critic) + sizeof(off_policy_runner) + sizeof(critic_batch) + sizeof(critic_training_buffers) + sizeof(critic_buffers) + sizeof(actor_batch) + sizeof(actor_training_buffers) + sizeof(actor_buffers) << std::endl;
#endif

    for(typename DEVICE::index_t step_i = 0; step_i < N_STEPS; step_i+=OFF_POLICY_RUNNER_SPEC::PARAMETERS::N_ENVIRONMENTS){
        rlt::step<0>(device, off_policy_runner, actor_critic.actor, actor_buffers_eval, rng);
#ifdef RL_TOOLS_DEPLOYMENT_ARDUINO
        if(step_i % 100 == 0){
            Serial.printf("step: %d\n", step_i);
#else
        if(step_i % 1000 == 0){
            auto current_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed_seconds = current_time - start_time;
            std::cout << "step_i: " << step_i << " " << elapsed_seconds.count() << "s" << std::endl;
#endif

        }
        if(step_i > N_WARMUP_STEPS){

            for(int critic_i = 0; critic_i < 2; critic_i++){
                auto target_next_action_noise_matrix_view = rlt::matrix_view(device, critic_training_buffers.target_next_action_noise);
                rlt::target_action_noise(device, actor_critic, target_next_action_noise_matrix_view, rng);
                rlt::gather_batch(device, off_policy_runner, critic_batch, rng);
                rlt::train_critic(device, actor_critic, actor_critic.critics[critic_i], critic_batch, actor_critic.critic_optimizers[critic_i], actor_buffers, actor_buffers, critic_buffers, critic_buffers, critic_training_buffers, rng);
            }

            if(step_i % 2 == 0){
                {
                    rlt::gather_batch(device, off_policy_runner, actor_batch, rng);
                    rlt::train_actor(device, actor_critic, actor_batch, actor_critic.actor_optimizer, actor_buffers, critic_buffers, actor_training_buffers, rng);
                }

                rlt::update_critic_targets(device, actor_critic);
                rlt::update_actor_target(device, actor_critic);
            }
        }
#ifndef RL_TOOLS_DISABLE_EVALUATION
        if(step_i % EVALUATION_INTERVAL == 0){
            using RESULT_SPEC = rlt::rl::utils::evaluation::Specification<TYPE_POLICY, TI, ENVIRONMENT, 10, OFF_POLICY_RUNNER_PARAMETERS::EPISODE_STEP_LIMIT>;
            rlt::rl::utils::evaluation::Result<RESULT_SPEC> result;
            rlt::sample_initial_parameters(device, envs[0], env_parameters[0], rng);
            rlt::evaluate(device, envs[0], ui, actor_critic.actor, result, rng, rlt::Mode<rlt::mode::Evaluation<>>{});
            if(N_EVALUATIONS > 0){
                evaluation_returns[(step_i / EVALUATION_INTERVAL) % N_EVALUATIONS] = result.returns_mean;
            }
#ifdef RL_TOOLS_DEPLOYMENT_ARDUINO
            Serial.printf("mean return: %f\n", result.returns_mean);
#else
            std::cout << "Mean return: " << result.returns_mean << std::endl;
#endif
        }
#endif
    }
#ifndef RL_TOOLS_DEPLOYMENT_ARDUINO
    {
        auto current_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed_seconds = current_time - start_time;
        std::cout << "total time: " << elapsed_seconds.count() << "s" << std::endl;
    }
#endif
    rlt::free(device, actor_critic);
    rlt::free(device, off_policy_runner);
    rlt::free(device, critic_batch);
    rlt::free(device, critic_training_buffers);
    rlt::free(device, critic_buffers);
    rlt::free(device, actor_batch);
    rlt::free(device, actor_training_buffers);
    rlt::free(device, actor_buffers_eval);
    rlt::free(device, actor_buffers);
}

