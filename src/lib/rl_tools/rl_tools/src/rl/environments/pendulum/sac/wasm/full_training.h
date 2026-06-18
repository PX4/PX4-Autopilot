//#include <rl_tools/operations/cpu.h>
//
//#include <rl_tools/nn/optimizers/adam/instance/operations_generic.h>
//#include <rl_tools/nn/operations_generic.h>
//
//#include <rl_tools/rl/environments/pendulum/operations_generic.h>
//#include <rl_tools/nn_models/operations_generic.h>
//#include <rl_tools/nn/optimizers/adam/operations_generic.h>
//#include <rl_tools/rl/components/off_policy_runner/operations_generic.h>
//#include <rl_tools/rl/algorithms/sac/operations_generic.h>
//
//
//#ifndef RL_TOOLS_BENCHMARK
//#include <rl_tools/rl/utils/evaluation/operations_generic.h>
//#include <chrono>
//#endif
//
//namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools;
//
//using DEV_SPEC = rlt::devices::DefaultCPUSpecification;
//using DEVICE = rlt::devices::CPU<DEV_SPEC>;
//
//struct TrainingConfig{
//    using T = float;
//    using TI = typename DEVICE::index_t;
//    using RNG = DEVICE::SPEC::RANDOM::ENGINE<>;
//
//    using PENDULUM_SPEC = rlt::rl::environments::pendulum::Specification<T, TI, rlt::rl::environments::pendulum::DefaultParameters<T>>;
//    using ENVIRONMENT = rlt::rl::environments::Pendulum<PENDULUM_SPEC> ;
//
//    struct DEVICE_SPEC: rlt::devices::DefaultCPUSpecification {
//        using LOGGING = rlt::devices::logging::CPU;
//    };
//    struct SACPendulumParameters: rlt::rl::algorithms::sac::DefaultParameters<T, TI>{
//        constexpr static TI CRITIC_BATCH_SIZE = 100;
//        constexpr static TI ACTOR_BATCH_SIZE = 100;
//    };
//
//    using SAC_PARAMETERS = SACPendulumParameters;
//
//    using ACTOR_SPEC = rlt::nn_models::mlp::Specification<T, TI, ENVIRONMENT::Observation::DIM, 2*ENVIRONMENT::ACTION_DIM, 3, 64, rlt::nn::activation_functions::RELU, rlt::nn::activation_functions::TANH>;
//    using CRITIC_SPEC = rlt::nn_models::mlp::Specification<T, TI, ENVIRONMENT::Observation::DIM + ENVIRONMENT::ACTION_DIM, 1, 3, 64, rlt::nn::activation_functions::RELU, rlt::nn::activation_functions::IDENTITY>;
//
//
//    using OPTIMIZER_SPEC = typename rlt::nn::optimizers::adam::Specification<T, TI>;
//    using OPTIMIZER = rlt::nn::optimizers::Adam<OPTIMIZER_SPEC>;
//    using ACTOR_CAPABILITY = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam, SAC_PARAMETERS::ACTOR_BATCH_SIZE>;
//    using ACTOR_NETWORK_TYPE = rlt::nn_models::mlp::NeuralNetwork<ACTOR_CAPABILITY, ACTOR_SPEC>;
//    using ACTOR_TARGET_NETWORK_TYPE = rlt::nn_models::mlp::NeuralNetwork<rlt::nn::capability::Forward, ACTOR_SPEC>;
//
//    using CRITIC_CAPABILITY = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam, SAC_PARAMETERS::CRITIC_BATCH_SIZE>;
//    using CRITIC_NETWORK_TYPE = rlt::nn_models::mlp::NeuralNetwork<CRITIC_CAPABILITY, CRITIC_SPEC>;
//    using CRITIC_TARGET_NETWORK_TYPE = rlt::nn_models::mlp::NeuralNetwork<rlt::nn::capability::Forward, CRITIC_SPEC>;
//
//    using ALPHA_PARAMETER_TYPE = rlt::nn::parameters::Adam;
//    using ACTOR_CRITIC_SPEC = rlt::rl::algorithms::sac::Specification<T, TI, ENVIRONMENT, ACTOR_NETWORK_TYPE, CRITIC_NETWORK_TYPE, CRITIC_TARGET_NETWORK_TYPE, ALPHA_PARAMETER_TYPE, OPTIMIZER, OPTIMIZER, OPTIMIZER, SAC_PARAMETERS>;
//    using ACTOR_CRITIC_TYPE = rlt::rl::algorithms::sac::ActorCritic<ACTOR_CRITIC_SPEC>;
//
//
//    static constexpr int N_WARMUP_STEPS = ACTOR_CRITIC_TYPE::SPEC::PARAMETERS::ACTOR_BATCH_SIZE;
//#ifndef RL_TOOLS_STEP_LIMIT
//    static constexpr TI STEP_LIMIT = 100000; //2 * N_WARMUP_STEPS;
//#else
//    static constexpr TI STEP_LIMIT = RL_TOOLS_STEP_LIMIT;
//#endif
//    static constexpr TI EVALUATION_INTERVAL = 1000;
////    static constexpr TI REPLAY_BUFFER_CAP = STEP_LIMIT;
////    static constexpr TI EPISODE_STEP_LIMIT = 200;
////    static constexpr bool STOCHASTIC_POLICY = true;
//    struct OFF_POLICY_RUNNER_PARAMETERS: rlt::rl::components::off_policy_runner::ParametersDefault<T, TI>{
//        static constexpr TI REPLAY_BUFFER_CAPACITY = STEP_LIMIT;
//        static constexpr TI EPISODE_STEP_LIMIT = 200;
//        static constexpr bool STOCHASTIC_POLICY = true;
//        static constexpr bool COLLECT_EPISODE_STATS = true;
//        static constexpr TI EPISODE_STATS_BUFFER_SIZE = 1000;
//    };
//    using OFF_POLICY_RUNNER_SPEC = rlt::rl::components::off_policy_runner::Specification<
//            T,
//            TI,
//            ENVIRONMENT,
//            OFF_POLICY_RUNNER_PARAMETERS
//    >;
//    const T STATE_TOLERANCE = 0.00001;
//    static_assert(ACTOR_CRITIC_TYPE::SPEC::PARAMETERS::ACTOR_BATCH_SIZE == ACTOR_CRITIC_TYPE::SPEC::PARAMETERS::CRITIC_BATCH_SIZE);
//};
//
//template <typename T_TRAINING_CONFIG>
//struct CoreTrainingState{
//    using TRAINING_CONFIG = T_TRAINING_CONFIG;
//    using T = typename TRAINING_CONFIG::T;
//    using TI = typename TRAINING_CONFIG::TI;
//    typename TRAINING_CONFIG::OPTIMIZER actor_optimizer, critic_optimizers[2];
//    typename TRAINING_CONFIG::RNG rng;
//    rlt::rl::environments::DummyUI ui;
//    rlt::rl::components::OffPolicyRunner<typename TRAINING_CONFIG::OFF_POLICY_RUNNER_SPEC> off_policy_runner;
//    typename TRAINING_CONFIG::ENVIRONMENT envs[decltype(off_policy_runner)::N_ENVIRONMENTS];
//    typename TRAINING_CONFIG::ACTOR_CRITIC_TYPE actor_critic;
//    typename TRAINING_CONFIG::ACTOR_NETWORK_TYPE::template Buffer<1> actor_deterministic_evaluation_buffers;
//    rlt::rl::components::off_policy_runner::Batch<rlt::rl::components::off_policy_runner::BatchSpecification<typename decltype(off_policy_runner)::SPEC, TRAINING_CONFIG::ACTOR_CRITIC_TYPE::SPEC::PARAMETERS::CRITIC_BATCH_SIZE>> critic_batch;
//    rlt::rl::algorithms::sac::CriticTrainingBuffers<typename TRAINING_CONFIG::ACTOR_CRITIC_SPEC> critic_training_buffers;
//    rlt::Matrix<rlt::matrix::Specification<T, TI, TRAINING_CONFIG::SAC_PARAMETERS::CRITIC_BATCH_SIZE, TRAINING_CONFIG::ENVIRONMENT::ACTION_DIM>> action_noise_critic[2];
//    rlt::Matrix<rlt::matrix::Specification<typename TRAINING_CONFIG::T, TI, 1, TRAINING_CONFIG::ENVIRONMENT::Observation::DIM>> observations_mean, observations_std;
//    typename TRAINING_CONFIG::CRITIC_NETWORK_TYPE::template Buffer<TRAINING_CONFIG::ACTOR_CRITIC_TYPE::SPEC::PARAMETERS::CRITIC_BATCH_SIZE> critic_buffers[2];
//    rlt::rl::components::off_policy_runner::Batch<rlt::rl::components::off_policy_runner::BatchSpecification<typename decltype(off_policy_runner)::SPEC, TRAINING_CONFIG::ACTOR_CRITIC_TYPE::SPEC::PARAMETERS::ACTOR_BATCH_SIZE>> actor_batch;
//    rlt::Matrix<rlt::matrix::Specification<T, TI, TRAINING_CONFIG::SAC_PARAMETERS::CRITIC_BATCH_SIZE, TRAINING_CONFIG::ENVIRONMENT::ACTION_DIM>> action_noise_actor;
//    rlt::rl::algorithms::sac::ActorTrainingBuffers<typename TRAINING_CONFIG::ACTOR_CRITIC_TYPE::SPEC> actor_training_buffers;
//    typename TRAINING_CONFIG::ACTOR_NETWORK_TYPE::template Buffer<TRAINING_CONFIG::ACTOR_CRITIC_TYPE::SPEC::PARAMETERS::ACTOR_BATCH_SIZE> actor_buffers[2];
//    typename TRAINING_CONFIG::ACTOR_NETWORK_TYPE::template Buffer<TRAINING_CONFIG::OFF_POLICY_RUNNER_SPEC::PARAMETERS::N_ENVIRONMENTS> actor_buffers_eval;
//};
//
//template <typename TRAINING_CONFIG>
//struct TrainingState: CoreTrainingState<TRAINING_CONFIG>{
//    using T = typename TRAINING_CONFIG::T;
//    using TI = typename TRAINING_CONFIG::TI;
//#ifndef RL_TOOLS_BENCHMARK
//    std::chrono::high_resolution_clock::time_point start_time;
//#endif
//    TI step = 0;
//#ifndef RL_TOOLS_BENCHMARK
//    static constexpr TI N_EVALUATIONS = TRAINING_CONFIG::STEP_LIMIT / TRAINING_CONFIG::EVALUATION_INTERVAL;
//    static_assert(N_EVALUATIONS > 0 && N_EVALUATIONS < 1000000);
//    T evaluation_returns[N_EVALUATIONS];
//#endif
//};
//
//
//template <typename DEVICE, typename TRAINING_STATE>
//void training_init(DEVICE& device, TRAINING_STATE& ts, typename TRAINING_STATE::TRAINING_CONFIG::TI seed){
//    using TRAINING_CONFIG = typename TRAINING_STATE::TRAINING_CONFIG;
//
//
//    ts.rng = rlt::random::default_engine(DEVICE{}, seed);
//
//    rlt::malloc(device, ts.actor_critic);
//    rlt::init(device, ts.actor_critic, ts.rng);
//
//    rlt::malloc(device, ts.off_policy_runner);
//    rlt::init(device, ts.off_policy_runner, ts.envs);
//
//    rlt::malloc(device, ts.critic_batch);
//    rlt::malloc(device, ts.critic_training_buffers);
//    rlt::malloc(device, ts.action_noise_critic[0]);
//    rlt::malloc(device, ts.action_noise_critic[1]);
//    rlt::malloc(device, ts.critic_buffers[0]);
//    rlt::malloc(device, ts.critic_buffers[1]);
//
//    rlt::malloc(device, ts.actor_batch);
//    rlt::malloc(device, ts.actor_training_buffers);
//    rlt::malloc(device, ts.action_noise_actor);
//    rlt::malloc(device, ts.actor_buffers_eval);
//    rlt::malloc(device, ts.actor_buffers[0]);
//    rlt::malloc(device, ts.actor_buffers[1]);
//
//    rlt::malloc(device, ts.observations_mean);
//    rlt::malloc(device, ts.observations_std);
//
//    rlt::malloc(device, ts.actor_deterministic_evaluation_buffers);
//
//    rlt::set_all(device, ts.observations_mean, 0);
//    rlt::set_all(device, ts.observations_std, 1);
//
//
//#ifndef RL_TOOLS_BENCHMARK
//    ts.start_time = std::chrono::high_resolution_clock::now();
//#endif
//    ts.step = 0;
//}
//
//template <typename DEVICE, typename TRAINING_STATE>
//void training_destroy(DEVICE& device, TRAINING_STATE& ts){
//    rlt::free(device, ts.critic_batch);
//    rlt::free(device, ts.critic_training_buffers);
//    rlt::free(device, ts.actor_batch);
//    rlt::free(device, ts.actor_training_buffers);
//    rlt::free(device, ts.off_policy_runner);
//    rlt::free(device, ts.actor_critic);
//    rlt::free(device, ts.observations_mean);
//    rlt::free(device, ts.observations_std);
//}
//
//
//template <typename DEVICE, typename TRAINING_STATE>
//bool training_step(DEVICE& device, TRAINING_STATE& ts){
//    bool finished = false;
//    using TRAINING_CONFIG = typename TRAINING_STATE::TRAINING_CONFIG;
//    rlt::step(device, ts.off_policy_runner, ts.actor_critic.actor, ts.actor_buffers_eval, ts.rng);
//#ifndef RL_TOOLS_BENCHMARK
//    if(ts.step % 1000 == 0){
//        auto current_time = std::chrono::high_resolution_clock::now();
//        std::chrono::duration<double> elapsed_seconds = current_time - ts.start_time;
//        std::cout << "step_i: " << ts.step << " " << elapsed_seconds.count() << "s" << std::endl;
//    }
//#endif
//    if(ts.step > TRAINING_CONFIG::N_WARMUP_STEPS){
//
//        for(int critic_i = 0; critic_i < 2; critic_i++){
//            rlt::gather_batch(device, ts.off_policy_runner, ts.critic_batch, ts.rng);
//            rlt::randn(device, ts.action_noise_critic[critic_i], ts.rng);
//            rlt::train_critic(device, ts.actor_critic, critic_i == 0 ? ts.actor_critic.critic_1 : ts.actor_critic.critic_2, ts.critic_batch, ts.critic_optimizers[critic_i], ts.actor_buffers[critic_i], ts.critic_buffers[critic_i], ts.critic_training_buffers, ts.action_noise_critic[critic_i], ts.rng);
//        }
//
//        {
//            rlt::gather_batch(device, ts.off_policy_runner, ts.actor_batch, ts.rng);
//            rlt::randn(device, ts.action_noise_actor, ts.rng);
//            rlt::train_actor(device, ts.actor_critic, ts.actor_batch, ts.actor_optimizer, ts.actor_buffers[0], ts.critic_buffers[0], ts.actor_training_buffers, ts.action_noise_actor, ts.rng);
//        }
//
//        rlt::update_critic_targets(device, ts.actor_critic);
//    }
//#ifndef RL_TOOLS_BENCHMARK
//    if(ts.step % TRAINING_CONFIG::EVALUATION_INTERVAL == 0){
//        using T = typename TRAINING_CONFIG::T;
//        using TI = typename TRAINING_CONFIG::TI;
//        using RESULT_SPEC = rlt::rl::utils::evaluation::Specification<T, TI, typename TRAINING_CONFIG::ENVIRONMENT, 1, TRAINING_CONFIG::OFF_POLICY_RUNNER_PARAMETERS::EPISODE_STEP_LIMIT>;
//        rlt::rl::utils::evaluation::Result<RESULT_SPEC> result;
//        rlt::evaluate(device, ts.envs[0], ts.ui, ts.actor_critic.actor, result, ts.actor_deterministic_evaluation_buffers, ts.rng);
//        std::cout << "Mean return: " << result.returns_mean << std::endl;
//        ts.evaluation_returns[ts.step / TRAINING_CONFIG::EVALUATION_INTERVAL] = result.returns_mean;
//    }
//#endif
//    if(ts.step > 5000){
//        ts.off_policy_runner.parameters.exploration_noise = 0;
//    }
//    ts.step++;
//    if(ts.step > TRAINING_CONFIG::STEP_LIMIT){
//#ifndef RL_TOOLS_BENCHMARK
//        auto current_time = std::chrono::high_resolution_clock::now();
//        std::chrono::duration<double> elapsed_seconds = current_time - ts.start_time;
//        std::cout << "total time: " << elapsed_seconds.count() << "s" << std::endl;
//#endif
//        return true;
//    }
//    else{
//        return finished;
//    }
//}

#include <rl_tools/operations/cpu_mux.h>
#include <rl_tools/nn/optimizers/adam/instance/operations_generic.h>
#include <rl_tools/nn/operations_cpu_mux.h>
#include <rl_tools/nn/layers/sample_and_squash/operations_generic.h>
#include <rl_tools/rl/environments/pendulum/operations_cpu.h>
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

using DEVICE = rlt::devices::DEVICE_FACTORY<>;
using RNG = DEVICE::SPEC::RANDOM::ENGINE<>;
using T = float;
using TYPE_POLICY = rlt::numeric_types::Policy<T>;
using TI = typename DEVICE::index_t;

using PENDULUM_SPEC = rlt::rl::environments::pendulum::Specification<T, TI, rlt::rl::environments::pendulum::DefaultParameters<T>>;
using ENVIRONMENT = rlt::rl::environments::Pendulum<PENDULUM_SPEC>;
struct LOOP_CORE_PARAMETERS: rlt::rl::algorithms::sac::loop::core::DefaultParameters<TYPE_POLICY, TI, ENVIRONMENT>{
    struct SAC_PARAMETERS: rlt::rl::algorithms::sac::DefaultParameters<TYPE_POLICY, TI, ENVIRONMENT::ACTION_DIM>{
        static constexpr TI ACTOR_BATCH_SIZE = 100;
        static constexpr TI CRITIC_BATCH_SIZE = 100;
    };
    static constexpr TI STEP_LIMIT = 100000;
    static constexpr TI ACTOR_NUM_LAYERS = 3;
    static constexpr TI CRITIC_NUM_LAYERS = 3;
#ifdef BENCHMARK
    static constexpr TI ACTOR_HIDDEN_DIM = 64;
    static constexpr TI CRITIC_HIDDEN_DIM = 64;
#else
    static constexpr TI ACTOR_HIDDEN_DIM = 64;
    static constexpr TI CRITIC_HIDDEN_DIM = 64;
#endif
    static constexpr T ALPHA = 1.0;
    static constexpr TI N_ENVIRONMENTS = 1;
};
#ifdef BENCHMARK
using LOOP_CORE_CONFIG = rlt::rl::algorithms::sac::loop::core::Config<T, TI, RNG, ENVIRONMENT, LOOP_CORE_PARAMETERS>;
using LOOP_TIMING_CONFIG = rlt::rl::loop::steps::timing::Config<LOOP_CORE_CONFIG>;
using LOOP_CONFIG = LOOP_TIMING_CONFIG;
#else
using RNG = DEVICE::SPEC::RANDOM::ENGINE<>;
using LOOP_CORE_CONFIG = rlt::rl::algorithms::sac::loop::core::Config<TYPE_POLICY, TI, RNG, ENVIRONMENT, LOOP_CORE_PARAMETERS, rlt::rl::algorithms::sac::loop::core::ConfigApproximatorsMLP>;
struct LOOP_EVAL_PARAMETERS: rlt::rl::loop::steps::evaluation::Parameters<TYPE_POLICY, TI, LOOP_CORE_CONFIG>{
    static constexpr TI EVALUATION_EPISODES = 100;
};
using LOOP_EVAL_CONFIG = rlt::rl::loop::steps::evaluation::Config<LOOP_CORE_CONFIG, LOOP_EVAL_PARAMETERS>;
using LOOP_TIMING_CONFIG = rlt::rl::loop::steps::timing::Config<LOOP_EVAL_CONFIG>;
using LOOP_CONFIG = LOOP_TIMING_CONFIG;
#endif

using LOOP_STATE = LOOP_CONFIG::State<LOOP_CONFIG>;

//void run(TI seed = 0){
//    DEVICE device;
//    LOOP_STATE ts;
//    rlt::malloc(device, ts);
//    rlt::init(device, ts, 0);
//    while(!rlt::step(device, ts)){
//#ifndef BENCHMARK
//        if(ts.step == 5000){
//            std::cout << "steppin yourself > callbacks 'n' hooks: " << ts.step << std::endl;
//        }
//#endif
//#ifdef BENCHMARK_ABLATION_SIMULATOR
//        std::this_thread::sleep_for(std::chrono::duration<T>(8.072980403900147e-05)); // python gymnasium Pendulum-v1 step time
//#endif
//    }
//    rlt::free(device, ts);
//}


// benchmark training should take < 2s on P1, < 0.75 on M3
