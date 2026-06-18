#include <rl_tools/operations/cpu_mux.h>
#include <rl_tools/rl/components/off_policy_runner/off_policy_runner.h>
namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools;
#if defined(RL_TOOLS_ENABLE_TENSORBOARD) && !defined(RL_TOOLS_DISABLE_TENSORBOARD)
using LOGGER = rlt::devices::logging::CPU_TENSORBOARD<>;
#else
using LOGGER = rlt::devices::logging::CPU;
#endif


using DEV_SPEC_SUPER = rlt::devices::cpu::Specification<rlt::devices::math::CPU, rlt::devices::random::CPU, LOGGER>;
using TI = typename rlt::devices::DEVICE_FACTORY<DEV_SPEC_SUPER>::index_t;
namespace execution_hints{
    struct HINTS: rlt::rl::components::off_policy_runner::ExecutionHints<TI, 1>{};
}
struct DEV_SPEC: DEV_SPEC_SUPER{
    using EXECUTION_HINTS = execution_hints::HINTS;
};

using DEVICE = rlt::devices::DEVICE_FACTORY<DEV_SPEC>;
using RNG = typename DEVICE::SPEC::RANDOM::ENGINE<>;
using TI = typename DEVICE::index_t;

#include <rl_tools/nn/optimizers/adam/instance/operations_generic.h>
#include <rl_tools/nn/operations_cpu_mux.h>
#include <rl_tools/nn/layers/td3_sampling/operations_generic.h>

// generic nn_model operations use the specialized layer operations depending on the backend device
#include <rl_tools/nn_models/operations_generic.h>
#include <rl_tools/nn_models/mlp/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>
#include <rl_tools/nn/optimizers/adam/operations_generic.h>
// simulation is run on the cpu and the environments functions are required in the off_policy_runner operations included afterwards
#include <rl_tools/rl/environments/mujoco/ant/operations_cpu.h>

#include <rl_tools/rl/algorithms/td3/operations_cpu_mux.h>

// additional includes for the ui and persisting
#if defined(RL_TOOLS_ENABLE_HDF5) && !defined(RL_TOOLS_DISABLE_HDF5)
#include <rl_tools/nn_models/persist.h>
#include <rl_tools/nn_models/sequential/persist.h>
#include <rl_tools/rl/components/replay_buffer/persist.h>
#endif

#include <rl_tools/rl/utils/evaluation/operations_generic.h>

#include "parameters.h"

#include <iostream>
#if defined(RL_TOOLS_ENABLE_HDF5) && !defined(RL_TOOLS_DISABLE_HDF5)
#include <highfive/H5File.hpp>
#endif
#include <filesystem>
#include <thread>
#include <future>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <string>
#include <sstream>

using T = float;

namespace parameter_set = parameters_0;

using parameters_environment = parameter_set::environment<double, typename DEVICE::index_t>;
using ENVIRONMENT = typename parameters_environment::ENVIRONMENT;

using parameters_rl = parameter_set::rl<T, typename DEVICE::index_t, ENVIRONMENT>;
static_assert(parameters_rl::ActorCriticType::SPEC::PARAMETERS::ACTOR_BATCH_SIZE == parameters_rl::ActorCriticType::SPEC::PARAMETERS::CRITIC_BATCH_SIZE);

#if !defined(RL_TOOLS_RL_ENVIRONMENTS_MUJOCO_ANT_DISABLE_EVALUATION)
constexpr bool ENABLE_EVALUATION = false;
#else
constexpr bool ENABLE_EVALUATION = true;
#endif
constexpr DEVICE::index_t performance_logging_interval = 100;
constexpr DEVICE::index_t ACTOR_CRITIC_EVALUATION_SYNC_INTERVAL = 100;
constexpr DEVICE::index_t DETERMINISTIC_EVALUATION_INTERVAL = 10000;
constexpr bool ACTOR_ENABLE_CHECKPOINTS = true;
constexpr bool ACTOR_OVERWRITE_CHECKPOINTS = false;
constexpr DEVICE::index_t ACTOR_CHECKPOINT_INTERVAL = 10000;
const std::string ACTOR_CHECKPOINT_DIRECTORY = "checkpoints/td3_ant";
const std::string REPLAY_BUFFER_OUTPUT_PATH = "replay_buffer.h5";
constexpr bool RL_TOOLS_SAVE_REPLAY_BUFFER = false;

#ifdef RL_TOOLS_TEST_RL_ENVIRONMENTS_MULTIROTOR_TRAINING_DEBUG
constexpr DEVICE::index_t STEP_LIMIT = parameters_rl::N_WARMUP_STEPS_ACTOR + 5000;
#else
#ifdef RL_TOOLS_TEST_RL_ENVIRONMENTS_MUJOCO_ANT_TRAINING_TD3_TEST
constexpr DEVICE::index_t STEP_LIMIT = 30000;
#else
constexpr DEVICE::index_t STEP_LIMIT = parameters_rl::OFF_POLICY_RUNNER_PARAMETERS::REPLAY_BUFFER_CAPACITY * 100;
#endif
#endif
constexpr DEVICE::index_t NUM_RUNS = 1;

std::string sanitize_file_name(const std::string &input) {
    std::string output = input;

    const std::string invalid_chars = R"(<>:\"/\|?*)";

    std::replace_if(output.begin(), output.end(), [&invalid_chars](const char &c) {
        return invalid_chars.find(c) != std::string::npos;
    }, '_');

    return output;
}

void run(){
#if defined(RL_TOOLS_ENABLE_HDF5) && !defined(RL_TOOLS_DISABLE_HDF5)
    if(ACTOR_ENABLE_CHECKPOINTS){
        std::cout << "Saving checkpoints to: " << ACTOR_CHECKPOINT_DIRECTORY << std::endl;
    }
#endif
    std::string DATA_FILE_PATH = "learning_curves.h5";
    std::vector<std::vector<T>> episode_step;
    std::vector<std::vector<T>> episode_returns;
    std::vector<std::vector<T>> episode_steps;

    std::vector<std::vector<T>> eval_step;
    std::vector<std::vector<T>> eval_return;

    for(typename DEVICE::index_t run_i = 0; run_i < NUM_RUNS; run_i++){
        std::string run_name;
        {
            auto now = std::chrono::system_clock::now();
            auto local_time = std::chrono::system_clock::to_time_t(now);
            std::tm* tm = std::localtime(&local_time);

            std::ostringstream oss;
            oss << std::put_time(tm, "%FT%T%z");
            run_name = sanitize_file_name(oss.str());
        }

        episode_step.push_back({});
        episode_returns.push_back({});
        episode_steps.push_back({});

        eval_step.push_back({});
        eval_return.push_back({});

        auto& run_episode_step = episode_step.back();
        auto& run_episode_returns = episode_returns.back();
        auto& run_episode_steps = episode_steps.back();

        auto& run_eval_step = eval_step.back();
        auto& run_eval_return = eval_return.back();

        RNG rng, evaluation_rng;

        // device
        typename DEVICE::SPEC::LOGGING logger;
        DEVICE device;
        rlt::construct(device, device.logger);

        // optimizer
        parameters_rl::OPTIMIZER actor_optimizer;
        parameters_rl::OPTIMIZER critic_optimizers[2];
        typename parameters_rl::OPTIMIZER::SPEC::T alpha = 1e-3;

        // environment
        T ui_speed_factor = 1;
//        auto parameters = parameters_environment::parameters;
        rlt::rl::environments::DummyUI ui;

        // rl
        parameters_rl::ActorCriticType actor_critic;
        rlt::malloc(device, rng);
        rlt::malloc(device, evaluation_rng);
        rlt::init(device, rng, run_i + 1);
        rlt::init(device, evaluation_rng, run_i + 1000);
        rlt::malloc(device, actor_optimizer);
        rlt::malloc(device, critic_optimizers[0]);
        rlt::malloc(device, critic_optimizers[1]);
        rlt::init(device, actor_optimizer);
        rlt::init(device, critic_optimizers[0]);
        rlt::init(device, critic_optimizers[1]);
        rlt::get_ref(device, actor_optimizer.parameters, 0).alpha = alpha;
        rlt::get_ref(device, critic_optimizers[0].parameters, 0).alpha = alpha;
        rlt::get_ref(device, critic_optimizers[1].parameters, 0).alpha = alpha;
        rlt::malloc(device, actor_critic);
        rlt::init(device, actor_critic, rng);

        rlt::rl::components::OffPolicyRunner<parameters_rl::OFF_POLICY_RUNNER_SPEC> off_policy_runner;
        rlt::malloc(device, off_policy_runner);

        ENVIRONMENT envs[decltype(off_policy_runner)::N_ENVIRONMENTS], evaluation_env;
        ENVIRONMENT::Parameters env_parameters[decltype(off_policy_runner)::N_ENVIRONMENTS], evaluation_env_parameters;
        for (auto& env : envs) {
            rlt::malloc(device, env);
        }
        rlt::malloc(device, evaluation_env);

        rlt::init(device, off_policy_runner);

        constexpr TI SEQUENCE_LENGTH = 1;
        using CRITIC_BATCH_SPEC = rlt::rl::components::off_policy_runner::SequentialBatchSpecification<decltype(off_policy_runner)::SPEC, SEQUENCE_LENGTH, parameters_rl::ActorCriticType::SPEC::PARAMETERS::CRITIC_BATCH_SIZE>;
        rlt::rl::components::off_policy_runner::SequentialBatch<CRITIC_BATCH_SPEC> critic_batches[2];
        rlt::rl::algorithms::td3::CriticTrainingBuffers<rlt::rl::algorithms::td3::CriticTrainingBuffersSpecification<parameters_rl::ActorCriticType::SPEC, true>> critic_training_buffers[2];
        parameters_rl::CRITIC_TYPE::Buffer<true> critic_buffers[2];
        rlt::malloc(device, critic_batches[0]);
        rlt::malloc(device, critic_batches[1]);
        rlt::malloc(device, critic_training_buffers[0]);
        rlt::malloc(device, critic_training_buffers[1]);
        rlt::malloc(device, critic_buffers[0]);
        rlt::malloc(device, critic_buffers[1]);

        using ACTOR_BATCH_SPEC = rlt::rl::components::off_policy_runner::SequentialBatchSpecification<decltype(off_policy_runner)::SPEC, SEQUENCE_LENGTH, parameters_rl::ActorCriticType::SPEC::PARAMETERS::ACTOR_BATCH_SIZE>;
        rlt::rl::components::off_policy_runner::SequentialBatch<ACTOR_BATCH_SPEC> actor_batch;
        rlt::rl::algorithms::td3::ActorTrainingBuffers<rlt::rl::algorithms::td3::ActorTrainingBuffersSpecification<parameters_rl::ActorCriticType::SPEC, true>> actor_training_buffers[2];
        parameters_rl::ACTOR_TYPE::Buffer<> actor_buffers[2];
        using ACTOR_EVAL_TYPE = typename parameters_rl::ACTOR_TYPE::template CHANGE_BATCH_SIZE<TI, decltype(off_policy_runner)::N_ENVIRONMENTS>;
        ACTOR_EVAL_TYPE::Buffer<> actor_buffers_eval;
        using ACTOR_DETERMINISTIC_EVAL_TYPE = typename parameters_rl::ACTOR_TYPE::template CHANGE_BATCH_SIZE<TI, 10>;
        ACTOR_DETERMINISTIC_EVAL_TYPE::Buffer<> actor_buffers_deterministic_eval;
        rlt::malloc(device, actor_batch);
        rlt::malloc(device, actor_training_buffers[0]);
        rlt::malloc(device, actor_training_buffers[1]);
        rlt::malloc(device, actor_buffers[0]);
        rlt::malloc(device, actor_buffers[1]);
        rlt::malloc(device, actor_buffers_eval);
        rlt::malloc(device, actor_buffers_deterministic_eval);


        // training
        for(TI step_i = 0; step_i < STEP_LIMIT; step_i++){
            auto step_start = std::chrono::high_resolution_clock::now();
            rlt::set_step(device, device.logger, step_i);
            rlt::step<0>(device, off_policy_runner, actor_critic.actor, actor_buffers_eval, rng);
            if(step_i % 1000 == 0){
                std::cout << "run_i: " << run_i << " step_i: " << step_i << std::endl;
            }
            if(step_i > std::max(parameters_rl::ACTOR_CRITIC_PARAMETERS::ACTOR_BATCH_SIZE, parameters_rl::ACTOR_CRITIC_PARAMETERS::CRITIC_BATCH_SIZE)){
                if(step_i >= parameters_rl::N_WARMUP_STEPS_CRITIC){
                    if(step_i % parameters_rl::ActorCriticType::SPEC::PARAMETERS::CRITIC_TRAINING_INTERVAL == 0) {
                        auto train_critic = [&device, &actor_critic, &off_policy_runner](parameters_rl::CRITIC_TYPE& critic, decltype(critic_batches[0])& critic_batch, parameters_rl::OPTIMIZER& optimizer, decltype(actor_buffers[0])& actor_buffers, decltype(actor_training_buffers[0])& actor_training_buffers, decltype(critic_buffers[0])& critic_buffers, decltype(critic_training_buffers[0])& critic_training_buffers, decltype(rng)& rng){
                            auto gather_batch_start = std::chrono::high_resolution_clock::now();
                            auto target_action_noise_matrix_view = rlt::matrix_view(device, critic_training_buffers.target_next_action_noise);
                            rlt::target_action_noise(device, actor_critic, target_action_noise_matrix_view, rng);
                            rlt::gather_batch(device, off_policy_runner, critic_batch, rng);
                            auto gather_batch_end = std::chrono::high_resolution_clock::now();
                            rlt::add_scalar(device, device.logger, "performance/gather_batch_duration", std::chrono::duration_cast<std::chrono::microseconds>(gather_batch_end - gather_batch_start).count(), performance_logging_interval);
                            auto critic_training_start = std::chrono::high_resolution_clock::now();
                            rlt::train_critic(device, actor_critic, critic, critic_batch, optimizer, actor_buffers, actor_training_buffers, critic_buffers, critic_training_buffers, rng);
                            auto critic_training_end = std::chrono::high_resolution_clock::now();
                            rlt::add_scalar(device, device.logger, "performance/critic_training_duration", std::chrono::duration_cast<std::chrono::microseconds>(critic_training_end - critic_training_start).count(), performance_logging_interval);
                        };
                        RNG rng[2];
                        rlt::malloc(device, rng[0]);
                        rlt::malloc(device, rng[1]);
                        rlt::init(device, rng[0], step_i * 2 + 12345);
                        rlt::init(device, rng[1], step_i * 2 + 12346);


                        if(std::getenv("RL_TOOLS_TEST_RL_ENVIRONMENTS_MULTIROTOR_TRAINING_CONCURRENT") != nullptr){
                            auto critic_1_training = std::async([&](){return train_critic(actor_critic.critic_1, critic_batches[0], critic_optimizers[0], actor_buffers[0], actor_training_buffers[0], critic_buffers[0], critic_training_buffers[0], rng[0]);});
                            auto critic_2_training = std::async([&](){return train_critic(actor_critic.critic_2, critic_batches[1], critic_optimizers[1], actor_buffers[1], actor_training_buffers[1], critic_buffers[1], critic_training_buffers[1], rng[1]);});
                            critic_1_training.wait();
                            critic_2_training.wait();
                        }
                        else{
                            train_critic(actor_critic.critic_1, critic_batches[0], critic_optimizers[0], actor_buffers[0], critic_buffers[0], critic_training_buffers[0], rng1);
                            train_critic(actor_critic.critic_2, critic_batches[1], critic_optimizers[1], actor_buffers[1], critic_buffers[1], critic_training_buffers[1], rng2);
                        }
                    }
                    if(step_i % parameters_rl::ActorCriticType::SPEC::PARAMETERS::CRITIC_TARGET_UPDATE_INTERVAL == 0) {
                        auto update_critic_targets_start = std::chrono::high_resolution_clock::now();
                        rlt::update_critic_targets(device, actor_critic);
                        auto update_critic_targets_end = std::chrono::high_resolution_clock::now();
                        rlt::add_scalar(device, device.logger, "performance/update_critic_targets_duration", std::chrono::duration_cast<std::chrono::microseconds>(update_critic_targets_end - update_critic_targets_start).count(), performance_logging_interval);
                    }
                }
                if(step_i >= parameters_rl::N_WARMUP_STEPS_ACTOR){
                    if(step_i % parameters_rl::ActorCriticType::SPEC::PARAMETERS::ACTOR_TRAINING_INTERVAL == 0){
                        rlt::gather_batch(device, off_policy_runner, actor_batch, rng);
                        auto actor_training_start = std::chrono::high_resolution_clock::now();
                        rlt::train_actor(device, actor_critic, actor_batch, actor_optimizer, actor_buffers[0], critic_buffers[0], actor_training_buffers, rng);
                        auto actor_training_end = std::chrono::high_resolution_clock::now();
                        rlt::add_scalar(device, device.logger, "performance/actor_training_duration", std::chrono::duration_cast<std::chrono::microseconds>(actor_training_end - actor_training_start).count(), performance_logging_interval);
                    }
                    if(step_i % parameters_rl::ActorCriticType::SPEC::PARAMETERS::ACTOR_TARGET_UPDATE_INTERVAL == 0) {
                        rlt::update_actor_target(device, actor_critic);
                    }
                }
                if(step_i % ACTOR_CRITIC_EVALUATION_SYNC_INTERVAL == 0){
                    rlt::gather_batch(device, off_policy_runner, critic_batches[0], rng);
//                    T critic_1_loss = rlt::critic_loss(device, actor_critic, actor_critic.critic_1, critic_batches[0], actor_buffers[0], critic_buffers[0], critic_training_buffers[0], rng);
//                    rlt::add_scalar(device, device.logger, "critic_1_loss", critic_1_loss, 100);

//                    rlt::gather_batch(device, off_policy_runner, actor_batch, rng);
//                    T actor_value = rlt::mean(device, actor_training_buffers.state_action_value);
//                    rlt::add_scalar(device, device.logger, "actor_value", actor_value, 100);

                    {
                        typename DEVICE::index_t num_episodes = 0;
                        T mean_return = 0;
                        T mean_steps = 0;

                        for(typename DEVICE::index_t env_i = 0; env_i < parameters_rl::OFF_POLICY_RUNNER_SPEC::PARAMETERS::N_ENVIRONMENTS; env_i++){
                            auto& episode_stats = get(off_policy_runner.episode_stats, 0, env_i);
                            if(episode_stats.next_episode_i > 0){
                                for(typename DEVICE::index_t episode_i = 0; episode_i < episode_stats.next_episode_i - 1; episode_i++){
                                    mean_return += get(episode_stats.returns, episode_i, 0);
                                    mean_steps  += get(episode_stats.steps  , episode_i, 0);
                                    num_episodes++;
                                }
                                episode_stats.next_episode_i = 1;
                            }
                        }
                        if(num_episodes > 0){
                            mean_return /= num_episodes;
                            mean_steps /= num_episodes;

                            rlt::add_scalar(device, device.logger, "episode/return", mean_return);
                            rlt::add_scalar(device, device.logger, "episode/length", mean_steps);
                            run_episode_step.push_back(step_i);
                            run_episode_returns.push_back(mean_return);
                            run_episode_steps.push_back(mean_steps);
                        }
                    }
                }
            }
            auto step_end = std::chrono::high_resolution_clock::now();
            rlt::add_scalar(device, device.logger, "performance/step_duration", std::chrono::duration_cast<std::chrono::microseconds>(step_end - step_start).count(), performance_logging_interval);
            if(step_i % DETERMINISTIC_EVALUATION_INTERVAL == 0){
                using RESULT_SPEC = rlt::rl::utils::evaluation::Specification<T, TI, ENVIRONMENT, 10, parameters_environment::ENVIRONMENT::EPISODE_STEP_LIMIT>;
                rlt::rl::utils::evaluation::Result<RESULT_SPEC> result;
                rlt::evaluate(device, evaluation_env, evaluation_env_parameters, ui, actor_critic.actor, result, actor_buffers_deterministic_eval, evaluation_rng, rlt::Mode<rlt::mode::Evaluation<>>{});
                rlt::add_scalar(device, device.logger, "evaluation/return/mean", result.returns_mean);
                rlt::add_scalar(device, device.logger, "evaluation/return/std", result.returns_std);
                rlt::add_histogram(device, device.logger, "evaluation/return", result.returns, decltype(result)::N_EPISODES);
                std::cout << "Evaluation return mean: " << result.returns_mean << " (std: " << result.returns_std << ")" << std::endl;
                run_eval_step.push_back(step_i);
                run_eval_return.push_back(result.returns_mean);

//            if(step_i > 250000){
//                ASSERT_GT(mean_return, 1000);
//            }
            }
            if(ACTOR_ENABLE_CHECKPOINTS && step_i % ACTOR_CHECKPOINT_INTERVAL == 0){
                std::filesystem::path actor_output_dir = std::filesystem::path(ACTOR_CHECKPOINT_DIRECTORY) / run_name;
                try {
                    std::filesystem::create_directories(actor_output_dir);
                }
                catch (std::exception& e) {
                }
                std::string checkpoint_name = "latest.h5";
                if(!ACTOR_OVERWRITE_CHECKPOINTS){
                    std::stringstream checkpoint_name_ss;
                    checkpoint_name_ss << "actor_" << std::setw(15) << std::setfill('0') << (step_i / ACTOR_CHECKPOINT_INTERVAL) << "_" << std::setw(15) << std::setfill('0') << step_i << ".h5";
                    checkpoint_name = checkpoint_name_ss.str();
                }
                std::filesystem::path actor_output_path = actor_output_dir / checkpoint_name;
#if defined(RL_TOOLS_ENABLE_HDF5) && !defined(RL_TOOLS_DISABLE_HDF5)
                try{
                    auto actor_file = HighFive::File(actor_output_path.string(), HighFive::File::Overwrite);
                    rlt::save(device, actor_critic.actor, actor_file.createGroup("actor"));
                }
                catch(HighFive::Exception& e){
                    std::cout << "Error while saving actor: " << e.what() << std::endl;
                }
#endif
            }
//#if defined(RL_TOOLS_ENABLE_HDF5) && !defined(RL_TOOLS_DISABLE_HDF5)
//            if(step_i % ACTOR_CHECKPOINT_INTERVAL == 0){
//                std::filesystem::path actor_output_dir = std::filesystem::path(ACTOR_CHECKPOINT_DIRECTORY) / run_name;
//                try {
//                    std::filesystem::create_directories(actor_output_dir);
//                }
//                catch (std::exception& e) {
//                }
//                std::stringstream checkpoint_name;
//                checkpoint_name << "actor_" << std::setw(15) << std::setfill('0') << step_i << ".h5";
//                std::filesystem::path actor_output_path = actor_output_dir / checkpoint_name.str();
//                try{
//                    auto actor_file = HighFive::File(actor_output_path.string(), HighFive::File::Overwrite);
//                    rlt::save(device, actor_critic.actor, actor_file.createGroup("actor"));
//                }
//                catch(HighFive::Exception& e){
//                    std::cout << "Error while saving actor: " << e.what() << std::endl;
//                }
//            }
//#endif
        }
#if defined(RL_TOOLS_ENABLE_HDF5) && !defined(RL_TOOLS_DISABLE_HDF5)
        if constexpr(RL_TOOLS_SAVE_REPLAY_BUFFER){
            try{
                auto actor_file = HighFive::File(REPLAY_BUFFER_OUTPUT_PATH, HighFive::File::Overwrite);
                auto replay_buffer_group = actor_file.createGroup("replay_buffer");
                for(typename DEVICE::index_t env_i = 0; env_i < decltype(off_policy_runner)::N_ENVIRONMENTS; env_i++){
                    auto& replay_buffer = get(off_policy_runner.replay_buffers, 0, env_i);
                    rlt::save(device, replay_buffer, replay_buffer_group.createGroup(std::to_string(env_i)));
                }
            }
            catch(HighFive::Exception& e){
                std::cout << "Error while saving actor: " << e.what() << std::endl;
            }
        }
#endif
        rlt::free(device, device.logger);

        rlt::free(device, actor_critic);
        rlt::free(device, off_policy_runner);

        rlt::free(device, critic_batches[0]);
        rlt::free(device, critic_batches[1]);
        rlt::free(device, critic_training_buffers[0]);
        rlt::free(device, critic_training_buffers[1]);
        rlt::free(device, critic_buffers[0]);
        rlt::free(device, critic_buffers[1]);

        rlt::free(device, actor_batch);
        rlt::free(device, actor_training_buffers);
        rlt::free(device, actor_buffers[0]);
        rlt::free(device, actor_buffers[1]);
        rlt::free(device, actor_buffers_eval);
    }


#if defined(RL_TOOLS_ENABLE_HDF5) && !defined(RL_TOOLS_DISABLE_HDF5)
    auto data_file = HighFive::File(DATA_FILE_PATH, HighFive::File::Overwrite);
    for(typename DEVICE::index_t run_i = 0; run_i < episode_step.size(); run_i++){
        auto group = data_file.createGroup(std::to_string(run_i));
        group.createDataSet("episode_step", episode_step[run_i]);
        group.createDataSet("episode_returns", episode_returns[run_i]);
        group.createDataSet("episode_steps", episode_steps[run_i]);
        group.createDataSet("eval_step", eval_step[run_i]);
        group.createDataSet("eval_return", eval_return[run_i]);
    }
#endif
}
