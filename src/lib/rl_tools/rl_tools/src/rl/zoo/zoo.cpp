// #define RL_TOOLS_DISABLE_DYNAMIC_MEMORY_ALLOCATIONS
// #define RL_TOOLS_NN_DISABLE_GENERIC_FORWARD_BACKWARD

#ifdef RL_TOOLS_ENABLE_TRACY
#include "Tracy.hpp"
#endif

#ifdef RL_TOOLS_RL_ZOO_BENCHMARK
#undef RL_TOOLS_ENABLE_TENSORBOARD
#endif

#include <rl_tools/operations/cpu_mux.h>
#include <rl_tools/nn/optimizers/adam/instance/operations_generic.h>
#include <rl_tools/nn/operations_cpu_mux.h>
#include <rl_tools/nn/layers/sample_and_squash/operations_generic.h>
#include <rl_tools/nn/layers/td3_sampling/operations_generic.h>
#include <rl_tools/nn/layers/standardize/operations_generic.h>
#include <rl_tools/nn_models/mlp/operations_generic.h>
#include <rl_tools/nn_models/mlp_unconditional_stddev/operations_generic.h>
#include <rl_tools/nn_models/random_uniform/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>
#include <rl_tools/nn_models/multi_agent_wrapper/operations_generic.h>
#include <rl_tools/nn/optimizers/adam/operations_generic.h>

#include <rl_tools/persist/backends/tar/operations_cpu.h>
#include <rl_tools/persist/backends/tar/operations_generic.h>

#include <rl_tools/numeric_types/persist_code.h>
#include <rl_tools/nn/parameters/persist.h>
#include <rl_tools/nn/layers/sample_and_squash/persist.h>
#include <rl_tools/nn/layers/dense/persist.h>
#include <rl_tools/nn/layers/standardize/persist.h>
#include <rl_tools/nn/layers/gru/persist.h>
#include <rl_tools/nn/layers/td3_sampling/persist.h>
#include <rl_tools/nn_models/mlp/persist.h>
#include <rl_tools/nn_models/sequential/persist.h>
#include <rl_tools/nn_models/multi_agent_wrapper/persist.h>
#include <rl_tools/rl/components/replay_buffer/persist.h>

#include <rl_tools/containers/tensor/persist_code.h>
#include <rl_tools/nn/optimizers/adam/instance/persist_code.h>
#include <rl_tools/nn/layers/dense/persist_code.h>
#include <rl_tools/nn/layers/standardize/persist_code.h>
#include <rl_tools/nn/layers/gru/persist_code.h>
#include <rl_tools/nn/layers/sample_and_squash/persist_code.h>
#include <rl_tools/nn/layers/td3_sampling/persist_code.h>
#include <rl_tools/nn_models/mlp/persist_code.h>
#include <rl_tools/nn_models/sequential/persist_code.h>
#include <rl_tools/nn_models/multi_agent_wrapper/persist_code.h>

// Environment Configurations
#include "pendulum-v1/sac.h"
// #include "pendulum-v1/sac_state_estimation_dataset.h"
#include "pendulum-v1/td3.h"
#include "pendulum-v1/ppo.h"
#include "pendulum-velocity-v1/sac.h"
#include "pendulum-velocity-v1/td3.h"
#include "pendulum-multitask-v1/sac.h"
#include "pendulum-multitask-v1/td3.h"
#include "pendulum-multitask-v1/ppo.h"
#include "flag/sac.h"
#include "flag/td3.h"
#include "flag/ppo.h"
#include "acrobot-swingup-v0/sac.h"
#include "bottleneck-v0/ppo.h"
#ifdef RL_TOOLS_EXPERIMENTAL
#include "l2f/sac_tiny.h"
// #include "l2f/sac_big.h"
#else
#include "l2f/sac_tiny.h"
#endif
#include "l2f/td3.h"
#include "l2f/ppo.h"
#ifdef RL_TOOLS_RL_ZOO_ENVIRONMENT_ANT_V4
#include "ant-v4/ppo.h"
#include "ant-v4/td3.h"
#endif

// Algorithm Loops
#include <rl_tools/rl/algorithms/td3/loop/core/operations_generic.h>
#include <rl_tools/rl/algorithms/sac/loop/core/operations_generic.h>
#if defined(RL_TOOLS_BACKEND_ENABLE_MKL) && !defined(RL_TOOLS_BACKEND_DISABLE_BLAS)
#include <rl_tools/rl/components/on_policy_runner/operations_cpu_mkl.h>
#else
#if defined(RL_TOOLS_BACKEND_ENABLE_ACCELERATE) && !defined(RL_TOOLS_BACKEND_DISABLE_BLAS)
#include <rl_tools/rl/components/on_policy_runner/operations_cpu_accelerate.h>
#else
#include <rl_tools/rl/components/on_policy_runner/operations_cpu.h>
#endif
#endif
#include <rl_tools/rl/algorithms/ppo/loop/core/operations_generic.h>

// Additional Loop steps
#include <rl_tools/rl/loop/steps/timing/operations_cpu.h>
#include <rl_tools/rl/loop/steps/extrack/operations_cpu.h>
#include <rl_tools/rl/loop/steps/evaluation/operations_generic.h>
#include <rl_tools/rl/loop/steps/checkpoint/operations_cpu.h>
#include <rl_tools/rl/loop/steps/save_trajectories/operations_cpu.h>
#include <rl_tools/rl/loop/steps/nn_analytics/operations_cpu.h>

#include <rl_tools/rl/utils/evaluation/operations_cpu.h>

#if defined(__unix__) || defined(__APPLE__)
#include <signal.h>
#include <unistd.h>
#endif

namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools;

using SUPER_DEVICE = rlt::devices::DEVICE_FACTORY<>;
using TI = typename SUPER_DEVICE::index_t;

namespace execution_hints{
    struct HINTS: rlt::rl::components::on_policy_runner::ExecutionHints<TI, 16>{};
}
struct DEV_SPEC: rlt::devices::DEVICE_FACTORY<>::SPEC{
    using EXECUTION_HINTS = execution_hints::HINTS;
};

using DEVICE = rlt::devices::DEVICE_FACTORY<DEV_SPEC>;
using RNG = typename DEVICE::SPEC::RANDOM::ENGINE<>;
using PARAMETER_POLICY = rlt::numeric_types::UseCase<rlt::numeric_types::categories::Parameter, float>;
using TYPE_POLICY = rlt::numeric_types::Policy<float, PARAMETER_POLICY>;
constexpr TI BASE_SEED = 0;
constexpr bool DYNAMIC_ALLOCATION = true;

#ifdef RL_TOOLS_DISABLE_DYNAMIC_MEMORY_ALLOCATIONS
static_assert(!DYNAMIC_ALLOCATION, "Dynamic memory allocations are disabled, but DYNAMIC_ALLOCATION is set to true");
#endif

#if defined(RL_TOOLS_RL_ZOO_ALGORITHM_SAC)
#if defined(RL_TOOLS_RL_ZOO_ENVIRONMENT_PENDULUM_V1)
using LOOP_CORE_CONFIG = rlt::rl::zoo::pendulum_v1::sac::FACTORY<DEVICE, TYPE_POLICY, TI, RNG, DYNAMIC_ALLOCATION>::LOOP_CORE_CONFIG;
template <typename BASE>
struct LOOP_EVALUATION_PARAMETER_OVERWRITES: BASE{}; // no-op, this allows to have a different EPISODE_STEP_LIMIT for training and evaluation (on a per algorithm&environment baseis)
#elif defined(RL_TOOLS_RL_ZOO_ENVIRONMENT_PENDULUM_VELOCITY_V1)
using LOOP_CORE_CONFIG = rlt::rl::zoo::pendulum_velocity_v1::sac::FACTORY<DEVICE, TYPE_POLICY, TI, RNG, DYNAMIC_ALLOCATION>::LOOP_CORE_CONFIG;
template <typename BASE>
struct LOOP_EVALUATION_PARAMETER_OVERWRITES: BASE{}; // no-op, this allows to have a different EPISODE_STEP_LIMIT for training and evaluation (on a per algorithm&environment baseis)
#elif defined(RL_TOOLS_RL_ZOO_ENVIRONMENT_PENDULUM_MULTITASK_V1)
using LOOP_CORE_CONFIG = rlt::rl::zoo::pendulum_multitask_v1::sac::FACTORY<DEVICE, TYPE_POLICY, TI, RNG, DYNAMIC_ALLOCATION>::LOOP_CORE_CONFIG;
template <typename BASE>
struct LOOP_EVALUATION_PARAMETER_OVERWRITES: rlt::rl::zoo::pendulum_multitask_v1::sac::FACTORY<DEVICE, TYPE_POLICY, TI, RNG, DYNAMIC_ALLOCATION>::LOOP_EVALUATION_PARAMETER_OVERWRITES<BASE>{}; // no-op, this allows to have a different EPISODE_STEP_LIMIT for training and evaluation (on a per algorithm&environment baseis)
#elif defined(RL_TOOLS_RL_ZOO_ENVIRONMENT_FLAG)
using LOOP_CORE_CONFIG = rlt::rl::zoo::flag::sac::FACTORY<DEVICE, TYPE_POLICY, TI, RNG, DYNAMIC_ALLOCATION>::LOOP_CORE_CONFIG;
template <typename BASE>
struct LOOP_EVALUATION_PARAMETER_OVERWRITES: BASE{}; // no-op, this allows to have a different EPISODE_STEP_LIMIT for training and evaluation (on a per algorithm&environment baseis)
#elif defined(RL_TOOLS_RL_ZOO_ENVIRONMENT_ACROBOT_SWINGUP_V0)
using LOOP_CORE_CONFIG = rlt::rl::zoo::acrobot_swingup_v0::sac::FACTORY<DEVICE, TYPE_POLICY, TI, RNG, DYNAMIC_ALLOCATION>::LOOP_CORE_CONFIG;
template <typename BASE>
using LOOP_EVALUATION_PARAMETER_OVERWRITES = rlt::rl::zoo::acrobot_swingup_v0::sac::FACTORY<DEVICE, TYPE_POLICY, TI, RNG, DYNAMIC_ALLOCATION>::LOOP_EVALUATION_PARAMETER_OVERWRITES<BASE>;
#elif defined(RL_TOOLS_RL_ZOO_ENVIRONMENT_L2F)
using LOOP_CORE_CONFIG = rlt::rl::zoo::l2f::sac::FACTORY<DEVICE, TYPE_POLICY, TI, RNG, DYNAMIC_ALLOCATION>::LOOP_CORE_CONFIG;
template <typename BASE>
struct LOOP_EVALUATION_PARAMETER_OVERWRITES: BASE{};
#else
#error "RLtools Zoo SAC: Environment not defined"
#endif
#elif defined(RL_TOOLS_RL_ZOO_ALGORITHM_TD3)
#if defined(RL_TOOLS_RL_ZOO_ENVIRONMENT_PENDULUM_V1)
using LOOP_CORE_CONFIG = rlt::rl::zoo::pendulum_v1::td3::FACTORY<DEVICE, TYPE_POLICY, TI, RNG, DYNAMIC_ALLOCATION>::LOOP_CORE_CONFIG;
template <typename BASE>
struct LOOP_EVALUATION_PARAMETER_OVERWRITES: BASE{}; // no-op
#elif defined(RL_TOOLS_RL_ZOO_ENVIRONMENT_PENDULUM_VELOCITY_V1)
using LOOP_CORE_CONFIG = rlt::rl::zoo::pendulum_velocity_v1::td3::FACTORY<DEVICE, TYPE_POLICY, TI, RNG, DYNAMIC_ALLOCATION>::LOOP_CORE_CONFIG;
template <typename BASE>
struct LOOP_EVALUATION_PARAMETER_OVERWRITES: BASE{}; // no-op, this allows to have a different EPISODE_STEP_LIMIT for training and evaluation (on a per algorithm&environment baseis)
#elif defined(RL_TOOLS_RL_ZOO_ENVIRONMENT_PENDULUM_MULTITASK_V1)
using LOOP_CORE_CONFIG = rlt::rl::zoo::pendulum_multitask_v1::td3::FACTORY<DEVICE, TYPE_POLICY, TI, RNG, DYNAMIC_ALLOCATION>::LOOP_CORE_CONFIG;
template <typename BASE>
struct LOOP_EVALUATION_PARAMETER_OVERWRITES: BASE{}; // no-op, this allows to have a different EPISODE_STEP_LIMIT for training and evaluation (on a per algorithm&environment baseis)
#elif defined(RL_TOOLS_RL_ZOO_ENVIRONMENT_FLAG)
using LOOP_CORE_CONFIG = rlt::rl::zoo::flag::td3::FACTORY<DEVICE, TYPE_POLICY, TI, RNG, DYNAMIC_ALLOCATION>::LOOP_CORE_CONFIG;
template <typename BASE>
struct LOOP_EVALUATION_PARAMETER_OVERWRITES: BASE{}; // no-op
#elif defined(RL_TOOLS_RL_ZOO_ENVIRONMENT_L2F)
using LOOP_CORE_CONFIG = rlt::rl::zoo::l2f::td3::FACTORY<DEVICE, TYPE_POLICY, TI, RNG, DYNAMIC_ALLOCATION>::LOOP_CORE_CONFIG;
template <typename BASE>
struct LOOP_EVALUATION_PARAMETER_OVERWRITES: BASE{}; // no-op
#elif defined(RL_TOOLS_RL_ZOO_ENVIRONMENT_ANT_V4)
using LOOP_CORE_CONFIG = rlt::rl::zoo::ant_v4::td3::FACTORY<DEVICE, TYPE_POLICY, TI, RNG, DYNAMIC_ALLOCATION>::LOOP_CORE_CONFIG;
template <typename BASE>
struct LOOP_EVALUATION_PARAMETER_OVERWRITES: BASE{}; // no-op
#else
#error "RLtools Zoo TD3: Environment not defined"
#endif
#elif defined(RL_TOOLS_RL_ZOO_ALGORITHM_PPO)
#if defined(RL_TOOLS_RL_ZOO_ENVIRONMENT_PENDULUM_V1)
using LOOP_CORE_CONFIG = rlt::rl::zoo::pendulum_v1::ppo::FACTORY<DEVICE, TYPE_POLICY, TI, RNG, DYNAMIC_ALLOCATION>::LOOP_CORE_CONFIG;
template <typename BASE>
struct LOOP_EVALUATION_PARAMETER_OVERWRITES: BASE{}; // no-op
#elif defined(RL_TOOLS_RL_ZOO_ENVIRONMENT_PENDULUM_MULTITASK_V1)
using LOOP_CORE_CONFIG = rlt::rl::zoo::pendulum_multitask_v1::ppo::FACTORY<DEVICE, TYPE_POLICY, TI, RNG, DYNAMIC_ALLOCATION>::LOOP_CORE_CONFIG;
template <typename BASE>
struct LOOP_EVALUATION_PARAMETER_OVERWRITES: BASE{}; // no-op
#elif defined(RL_TOOLS_RL_ZOO_ENVIRONMENT_FLAG)
using LOOP_CORE_CONFIG = rlt::rl::zoo::flag::ppo::FACTORY<DEVICE, TYPE_POLICY, TI, RNG, DYNAMIC_ALLOCATION>::LOOP_CORE_CONFIG;
template <typename BASE>
struct LOOP_EVALUATION_PARAMETER_OVERWRITES: BASE{}; // no-op
#elif defined(RL_TOOLS_RL_ZOO_ENVIRONMENT_BOTTLENECK_V0)
using LOOP_CORE_CONFIG = rlt::rl::zoo::bottleneck_v0::ppo::FACTORY<DEVICE, TYPE_POLICY, TI, RNG, DYNAMIC_ALLOCATION>::LOOP_CORE_CONFIG;
template <typename BASE>
using LOOP_EVALUATION_PARAMETER_OVERWRITES = rlt::rl::zoo::bottleneck_v0::ppo::FACTORY<DEVICE, TYPE_POLICY, TI, RNG, DYNAMIC_ALLOCATION>::LOOP_EVALUATION_PARAMETER_OVERWRITES<BASE>;
#elif defined(RL_TOOLS_RL_ZOO_ENVIRONMENT_ANT_V4)
using LOOP_CORE_CONFIG = rlt::rl::zoo::ant_v4::ppo::FACTORY<DEVICE, TYPE_POLICY, TI, RNG, DYNAMIC_ALLOCATION>::LOOP_CORE_CONFIG;
template <typename BASE>
struct LOOP_EVALUATION_PARAMETER_OVERWRITES: BASE{}; // no-op
#elif defined(RL_TOOLS_RL_ZOO_ENVIRONMENT_L2F)
using LOOP_CORE_CONFIG = rlt::rl::zoo::l2f::ppo::FACTORY<DEVICE, TYPE_POLICY, TI, RNG, DYNAMIC_ALLOCATION>::LOOP_CORE_CONFIG;
template <typename BASE>
struct LOOP_EVALUATION_PARAMETER_OVERWRITES: BASE{}; // no-op
#else
#error "RLtools Zoo PPO: Environment not defined"
#endif
#else
#error "RLtools Zoo: Algorithm not defined"
#endif

constexpr TI NUM_CHECKPOINTS = 10;
constexpr TI NUM_EVALUATIONS = 100;
constexpr TI NUM_SAVE_TRAJECTORIES = 10;
#ifdef RL_TOOLS_RL_ZOO_ALGORITHM_PPO
static constexpr TI TIMING_INTERVAL = 10;
#else
static constexpr TI TIMING_INTERVAL = 10000;
#endif
using LOOP_TIMING_CONFIG = rlt::rl::loop::steps::timing::Config<LOOP_CORE_CONFIG, rlt::rl::loop::steps::timing::Parameters<TI, TIMING_INTERVAL>>;
using LOOP_EXTRACK_CONFIG = rlt::rl::loop::steps::extrack::Config<LOOP_TIMING_CONFIG>;
struct LOOP_EVALUATION_PARAMETERS: LOOP_EVALUATION_PARAMETER_OVERWRITES<rlt::rl::loop::steps::evaluation::Parameters<TYPE_POLICY, TI, LOOP_EXTRACK_CONFIG>>{
    static constexpr TI EVALUATION_INTERVAL_TEMP = LOOP_CORE_CONFIG::CORE_PARAMETERS::STEP_LIMIT / NUM_EVALUATIONS;
    static constexpr TI EVALUATION_INTERVAL = EVALUATION_INTERVAL_TEMP == 0 ? 1 : EVALUATION_INTERVAL_TEMP;
    static constexpr TI NUM_EVALUATION_EPISODES = 100;
    static constexpr TI N_EVALUATIONS = LOOP_CORE_CONFIG::CORE_PARAMETERS::STEP_LIMIT / EVALUATION_INTERVAL;
};
using LOOP_EVALUATION_CONFIG = rlt::rl::loop::steps::evaluation::Config<LOOP_EXTRACK_CONFIG, LOOP_EVALUATION_PARAMETERS>;
struct LOOP_CHECKPOINT_PARAMETERS: rlt::rl::loop::steps::checkpoint::Parameters<TYPE_POLICY, TI>{
    static constexpr TI CHECKPOINT_INTERVAL_TEMP = LOOP_CORE_CONFIG::CORE_PARAMETERS::STEP_LIMIT / NUM_CHECKPOINTS;
    static constexpr TI CHECKPOINT_INTERVAL = CHECKPOINT_INTERVAL_TEMP == 0 ? 1 : CHECKPOINT_INTERVAL_TEMP;
};
using LOOP_CHECKPOINT_CONFIG = rlt::rl::loop::steps::checkpoint::Config<LOOP_EVALUATION_CONFIG, LOOP_CHECKPOINT_PARAMETERS>;
struct LOOP_SAVE_TRAJECTORIES_PARAMETERS: LOOP_EVALUATION_PARAMETER_OVERWRITES<rlt::rl::loop::steps::save_trajectories::Parameters<TYPE_POLICY, TI, LOOP_CHECKPOINT_CONFIG>>{
    static constexpr TI INTERVAL_TEMP = LOOP_CORE_CONFIG::CORE_PARAMETERS::STEP_LIMIT / NUM_SAVE_TRAJECTORIES;
    static constexpr TI INTERVAL = INTERVAL_TEMP == 0 ? 1 : INTERVAL_TEMP;
    static constexpr TI NUM_EPISODES = 100;
};
using LOOP_SAVE_TRAJECTORIES_CONFIG = rlt::rl::loop::steps::save_trajectories::Config<LOOP_CHECKPOINT_CONFIG, LOOP_SAVE_TRAJECTORIES_PARAMETERS>;
struct LOOP_NN_ANALYTICS_PARAMETERS: LOOP_EVALUATION_PARAMETER_OVERWRITES<rlt::rl::loop::steps::nn_analytics::Parameters<TYPE_POLICY, TI, LOOP_CHECKPOINT_CONFIG>>{
    static constexpr TI INTERVAL_TEMP = LOOP_SAVE_TRAJECTORIES_PARAMETERS::INTERVAL;
    static constexpr TI INTERVAL = INTERVAL_TEMP == 0 ? 1 : INTERVAL_TEMP;
#if defined(RL_TOOLS_RL_ZOO_ALGORITHM_SAC) || defined(RL_TOOLS_RL_ZOO_ALGORITHM_TD3)
    static constexpr TI WARMUP_STEPS = LOOP_CORE_CONFIG::CORE_PARAMETERS::N_WARMUP_STEPS_ACTOR;
#else
    static constexpr TI WARMUP_STEPS = 0;
#endif
};
using LOOP_NN_ANALYTICS_CONFIG = rlt::rl::loop::steps::nn_analytics::Config<LOOP_SAVE_TRAJECTORIES_CONFIG, LOOP_NN_ANALYTICS_PARAMETERS>;
#ifdef RL_TOOLS_RL_ZOO_BENCHMARK
using LOOP_CONFIG = LOOP_EXTRACK_CONFIG;
#else
using LOOP_CONFIG = LOOP_NN_ANALYTICS_CONFIG;
#endif

#if defined(RL_TOOLS_RL_ZOO_ALGORITHM_SAC)
std::string algorithm = "sac";
#elif defined(RL_TOOLS_RL_ZOO_ALGORITHM_TD3)
std::string algorithm = "td3";
#elif defined(RL_TOOLS_RL_ZOO_ALGORITHM_PPO)
std::string algorithm = "ppo";
#else
#error "RLtools Zoo: Algorithm not defined"
#endif
#if defined(RL_TOOLS_RL_ZOO_ENVIRONMENT_PENDULUM_V1)
std::string environment = "pendulum-v1";
#elif defined(RL_TOOLS_RL_ZOO_ENVIRONMENT_PENDULUM_VELOCITY_V1)
std::string environment = "pendulum-velocity-v1";
#elif defined(RL_TOOLS_RL_ZOO_ENVIRONMENT_PENDULUM_MULTITASK_V1)
std::string environment = "pendulum-multitask-v1";
#elif defined(RL_TOOLS_RL_ZOO_ENVIRONMENT_FLAG)
std::string environment = "flag";
#elif defined(RL_TOOLS_RL_ZOO_ENVIRONMENT_ACROBOT_SWINGUP_V0)
std::string environment = "acrobot-swingup-v0";
#elif defined(RL_TOOLS_RL_ZOO_ENVIRONMENT_BOTTLENECK_V0)
std::string environment = "bottleneck-v0";
#elif defined(RL_TOOLS_RL_ZOO_ENVIRONMENT_ANT_V4)
std::string environment = "ant-v4";
#elif defined(RL_TOOLS_RL_ZOO_ENVIRONMENT_L2F)
std::string environment = "l2f";
#else
#error "RLtools Zoo: Environment not defined"
#endif
// ---------------------------------------------------------------------------------------

#if defined(__unix__) || defined(__APPLE__)
volatile sig_atomic_t signal_flag = false;
void signal_handler(int signal_number){
    signal_flag = true;
};
#else
bool signal_flag = false;
#endif

int zoo(int initial_seed, int num_seeds, std::string extrack_base_path, std::string extrack_experiment, std::string extrack_experiment_path, std::string config_path){
#if defined(__unix__) || defined(__APPLE__)
    std::cerr << "PID: " << getpid() << " (use kill -SIGUSR1 " << getpid() << " to create evaluate, create a checkpoint and save trajectories on demand)" << std::endl;
    if (signal(SIGUSR1, signal_handler) == SIG_ERR){
        perror("Error setting up signal handler");
        exit(EXIT_FAILURE);
    }
#endif
    using LOOP_STATE = LOOP_CONFIG::State<LOOP_CONFIG>;
    DEVICE device;
    static_assert(sizeof(LOOP_STATE) < 100000000);
    LOOP_STATE test_state;
//    rlt::utils::assert_exit(device, num_seeds > 0, "Number of seeds must be greater than 0.");
    for(TI seed = initial_seed; seed < (TI)num_seeds; seed++){
        LOOP_STATE ts;
        ts.extrack_config.name = "zoo";
        if(extrack_base_path != ""){
            ts.extrack_config.base_path = extrack_base_path;
        }
        if(extrack_experiment != ""){
            ts.extrack_config.experiment = extrack_experiment;
        }
        ts.extrack_config.population_variates = "environment_algorithm";
        ts.extrack_config.population_values = environment + "_" + algorithm;
        if(extrack_experiment_path != ""){
            ts.extrack_config.experiment = extrack_experiment_path;
        }
        rlt::malloc(device);
        rlt::init(device);
        rlt::malloc(device, ts);
        rlt::init(device, ts, seed);
#ifdef RL_TOOLS_ENABLE_TENSORBOARD
        rlt::init(device, device.logger, ts.extrack_paths.seed);
#endif
#ifndef RL_TOOLS_RL_ZOO_BENCHMARK
        std::cout << "Evaluation Interval: " << LOOP_CONFIG::EVALUATION_PARAMETERS::EVALUATION_INTERVAL << std::endl;
        std::cout << "Checkpoint Interval: " << LOOP_CONFIG::CHECKPOINT_PARAMETERS::CHECKPOINT_INTERVAL << std::endl;
        std::cout << "Save Trajectories Interval: " << LOOP_CONFIG::SAVE_TRAJECTORIES_PARAMETERS::INTERVAL << std::endl;
        using T = typename TYPE_POLICY::DEFAULT;
        T difficulty = 0; // [0, 1]
#if defined(RL_TOOLS_RL_ZOO_ENVIRONMENT_L2F) && defined(RL_TOOLS_RL_ZOO_ALGORITHM_SAC)
        const auto initial_parameters = rlt::get(ts.off_policy_runner.envs, 0, 0).parameters;
        auto set_difficulty = [&device, &initial_parameters](auto& env, T difficulty) {
            env.parameters.mdp.init.guidance = (T)0.5 + ((T)1-difficulty)/(T)2;
            env.parameters.mdp.init.max_angle = initial_parameters.mdp.init.max_angle * difficulty;   // orientation
            env.parameters.mdp.init.max_linear_velocity = initial_parameters.mdp.init.max_linear_velocity * difficulty; // velocity
            rlt::add_scalar(device, device.logger, "env/parameters/init/guidance", env.parameters.mdp.init.guidance);
            rlt::add_scalar(device, device.logger, "env/parameters/init/max_angle", env.parameters.mdp.init.max_angle);
            rlt::add_scalar(device, device.logger, "env/parameters/init/max_linear_velocity", env.parameters.mdp.init.max_linear_velocity);
        };
        auto update_parameters = [&device, &ts, set_difficulty](T difficulty) {
            rlt::log(device, device.logger, "Increasing difficulty to ", difficulty);
            set_difficulty(ts.env_eval, difficulty);
            for (TI env_i=0; env_i<decltype(ts.off_policy_runner)::SPEC::PARAMETERS::N_ENVIRONMENTS; env_i++) {
                auto& env = rlt::get(ts.off_policy_runner.envs, 0, env_i);
                set_difficulty(env, difficulty);
            }
        };
#endif
#endif
        while(!rlt::step(device, ts)){
#ifdef RL_TOOLS_ENABLE_TRACY
            FrameMark;
#endif

// #if defined(RL_TOOLS_RL_ZOO_ENVIRONMENT_L2F) && defined(RL_TOOLS_RL_ZOO_ALGORITHM_SAC)
//             // hacked L2F curriculum
//             TI previous_step = ts.step-1;
//             TI evaluation_index = previous_step / LOOP_CONFIG::EVALUATION_PARAMETERS::EVALUATION_INTERVAL;
//             bool evaluate_scheduled = previous_step % LOOP_CONFIG::EVALUATION_PARAMETERS::EVALUATION_INTERVAL == 0 && evaluation_index < LOOP_CONFIG::EVALUATION_PARAMETERS::N_EVALUATIONS;
//             if (evaluate_scheduled){
//                 auto& results = rlt::get(ts.evaluation_results, 0, evaluation_index);
//                 T share_terminated = results.share_terminated;
//                 if (share_terminated < 0.1) {
//                     difficulty = rlt::math::min(device.math, difficulty + (T)0.1, (T)1);
//                     update_parameters(difficulty);
//                 }
//             }
// #endif
#ifndef RL_TOOLS_RL_ZOO_BENCHMARK
            if(signal_flag){
                ts.evaluate_this_step = true;
                ts.checkpoint_this_step = true;
                ts.save_trajectories_this_step = true;
            }
            signal_flag = false;
#endif
            // if (ts.step == 500000){
            //     rlt::init_weights(device, ts.actor_critic.actor, ts.rng);
            //     rlt::init_weights(device, ts.actor_critic.critics[0], ts.rng);
            //     rlt::init_weights(device, ts.actor_critic.critics[1], ts.rng);
            //     rlt::reset_optimizer_state(device, ts.actor_critic.actor_optimizer, ts.actor_critic.actor);
            //     rlt::reset_optimizer_state(device, ts.actor_critic.critic_optimizers[0], ts.actor_critic.critics[0]);
            //     rlt::reset_optimizer_state(device, ts.actor_critic.critic_optimizers[1], ts.actor_critic.critics[1]);
            //     rlt::copy(device, device, ts.actor_critic.critics[0], ts.actor_critic.critics_target[0]);
            //     rlt::copy(device, device, ts.actor_critic.critics[1], ts.actor_critic.critics_target[1]);
            // }

        }
#ifndef RL_TOOLS_RL_ZOO_BENCHMARK
        std::filesystem::create_directories(ts.extrack_paths.seed);
        std::ofstream return_file(ts.extrack_paths.seed / "return.json");
        return_file << "[";
        for(TI evaluation_i = 0; evaluation_i < LOOP_CONFIG::EVALUATION_PARAMETERS::N_EVALUATIONS; evaluation_i++){
            auto& result = rlt::get(ts.evaluation_results, 0, evaluation_i);
            return_file << rlt::json(device, result, LOOP_CONFIG::EVALUATION_PARAMETERS::EVALUATION_INTERVAL * LOOP_CONFIG::ENVIRONMENT_STEPS_PER_LOOP_STEP * evaluation_i);
            if(evaluation_i < LOOP_CONFIG::EVALUATION_PARAMETERS::N_EVALUATIONS - 1){
                return_file << ", ";
            }
        }
        return_file << "]";
        std::ofstream return_file_confirmation(ts.extrack_paths.seed / "return.json.set");
        return_file_confirmation.close();
#else
        {
            constexpr TI NUM_EPISODES = 100;
            rlt::rl::utils::evaluation::Result<rlt::rl::utils::evaluation::Specification<TYPE_POLICY, TI, LOOP_CONFIG::ENVIRONMENT, NUM_EPISODES, LOOP_CONFIG::CORE_PARAMETERS::EPISODE_STEP_LIMIT>> result;
            using EVALUATION_ACTOR_TYPE_BATCH_SIZE = typename LOOP_CONFIG::NN::ACTOR_TYPE::template CHANGE_BATCH_SIZE<TI, NUM_EPISODES>;
            using EVALUATION_ACTOR_TYPE = typename EVALUATION_ACTOR_TYPE_BATCH_SIZE::template CHANGE_CAPABILITY<rlt::nn::capability::Forward<LOOP_CONFIG::DYNAMIC_ALLOCATION>>;
            rlt::rl::environments::DummyUI ui;
            EVALUATION_ACTOR_TYPE evaluation_actor;
            EVALUATION_ACTOR_TYPE::Buffer<LOOP_CONFIG::DYNAMIC_ALLOCATION> eval_buffer;
            rlt::malloc(device, evaluation_actor);
            rlt::malloc(device, eval_buffer);
            auto actor = rlt::get_actor(ts);
            rlt::copy(device, device, actor, evaluation_actor);
            typename LOOP_CONFIG::ENVIRONMENT_EVALUATION env_eval;
            typename LOOP_CONFIG::ENVIRONMENT_EVALUATION::Parameters env_eval_parameters;
            rlt::init(device, env_eval);
            rlt::initial_parameters(device, env_eval, env_eval_parameters);

            RNG rng;
            rlt::init(device, rng, seed);
            rlt::Mode<rlt::mode::Evaluation<>> evaluation_mode;
            rlt::evaluate(device, env_eval, ui, evaluation_actor, result, rng, evaluation_mode);
            rlt::free(device, evaluation_actor);
            rlt::log(device, device.logger, "Seed: ", seed, " Step: ", ts.step, "/", LOOP_CONFIG::CORE_PARAMETERS::STEP_LIMIT, " Mean return: ", result.returns_mean, " Mean episode length: ", result.episode_length_mean);
        }
        {
            auto& actor = rlt::get_actor(ts);
            static constexpr TI BATCH_SIZE = 13;
            using ORIGINAL_ACTOR = typename LOOP_CORE_CONFIG::NN::ACTOR_TYPE;
            using EVALUATION_ACTOR_TYPE_BATCH_SIZE = ORIGINAL_ACTOR::template CHANGE_BATCH_SIZE<TI, BATCH_SIZE>;
            using EVALUATION_ACTOR_TYPE = typename EVALUATION_ACTOR_TYPE_BATCH_SIZE::template CHANGE_CAPABILITY<rlt::nn::capability::Forward<DYNAMIC_ALLOCATION>>;
            EVALUATION_ACTOR_TYPE evaluation_actor;
            rlt::malloc(device, evaluation_actor);
            rlt::copy(device, device, actor, evaluation_actor);
            std::stringstream step_ss;
            step_ss << std::setw(15) << std::setfill('0') << ts.step;
            RNG rng;
            rlt::init(device, rng, seed);
            rlt::rl::loop::steps::checkpoint::save_code<DYNAMIC_ALLOCATION, typename LOOP_CONFIG::ENVIRONMENT_EVALUATION>(device, (ts.extrack_paths.seed / "steps" / step_ss.str()).string(), evaluation_actor, rng);
            rlt::free(device, evaluation_actor);
        }
#endif
// #if defined(RL_TOOLS_EXPERIMENTAL) && defined(RL_TOOLS_RL_ZOO_ALGORITHM_SAC) && defined(RL_TOOLS_RL_ZOO_ENVIRONMENT_PENDULUM_V1)
//         {
//             HighFive::File replay_buffer_file("replay_buffer.h5", HighFive::File::Overwrite);
//             for (TI rb_i = 0; rb_i < decltype(ts.off_policy_runner)::SPEC::PARAMETERS::N_ENVIRONMENTS; rb_i++){
//                 auto& rb = rlt::get(ts.off_policy_runner.replay_buffers, 0, rb_i);
//                 auto group = replay_buffer_file.createGroup(std::to_string(rb_i));
//                 rlt::save(device, rb, group);
//             }
//         }
// #endif
#ifdef RL_TOOLS_ENABLE_TENSORBOARD
        rlt::free(device, device.logger);
#endif
        rlt::free(device, ts);
        rlt::free(device);
    }
    return 0;
}
