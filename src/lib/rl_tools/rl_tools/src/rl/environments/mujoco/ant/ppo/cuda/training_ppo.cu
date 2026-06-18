#define RL_TOOLS_OPERATIONS_CPU_MUX_INCLUDE_CUDA
#include <rl_tools/operations/cpu_mux.h>
// -------------- added for cuda training ----------------
// -------------------------------------------------------
#include <rl_tools/nn/operations_cpu_mux.h>
#include <rl_tools/nn/optimizers/adam/instance/operations_generic.h>
#include <rl_tools/nn/optimizers/adam/instance/operations_cuda.h>
//#include <rl_tools/nn_models/operations_cpu.h>
#include <rl_tools/nn/layers/standardize/operations_generic.h>
#include <rl_tools/nn/layers/standardize/operations_cuda.h>
#include <rl_tools/nn_models/mlp_unconditional_stddev/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>
#include <rl_tools/nn_models/sequential/persist.h>
#if defined(RL_TOOLS_ENABLE_HDF5) && !defined(RL_TOOLS_DISABLE_HDF5)
#include <rl_tools/nn/layers/standardize/persist.h>
#include <rl_tools/nn_models/persist.h>
#include <rl_tools/nn_models/sequential/persist.h>
#endif
namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools;
// --------------- changed for cuda training -----------------
#include "../parameters.h"
#include <rl_tools/nn/optimizers/adam/operations_generic.h>
// -------------------------------------------------------
#if defined(RL_TOOLS_BACKEND_ENABLE_MKL) && !defined(RL_TOOLS_BACKEND_DISABLE_BLAS)
#include <rl_tools/rl/components/on_policy_runner/operations_cpu_mkl.h>
#else
#if defined(RL_TOOLS_BACKEND_ENABLE_ACCELERATE) && !defined(RL_TOOLS_BACKEND_DISABLE_BLAS)
#include <rl_tools/rl/components/on_policy_runner/operations_cpu_accelerate.h>
#else
#include <rl_tools/rl/components/on_policy_runner/operations_cpu.h>
#endif
#endif
// -------------- added for cuda training ----------------
#include <rl_tools/rl/components/on_policy_runner/operations_generic_extensions.h>
// -------------------------------------------------------
#include <rl_tools/rl/algorithms/ppo/operations_generic.h>
// -------------- added for cuda training ----------------
#include <rl_tools/rl/algorithms/ppo/operations_generic_extensions.h>
// -------------------------------------------------------
#include <rl_tools/rl/components/running_normalizer/operations_generic.h>
#if defined(RL_TOOLS_ENABLE_HDF5) && !defined(RL_TOOLS_DISABLE_HDF5)
#include <rl_tools/rl/components/running_normalizer/persist.h>
#endif
#include <rl_tools/rl/utils/evaluation/operations_generic.h>

#if defined(RL_TOOLS_ENABLE_HDF5) && !defined(RL_TOOLS_DISABLE_HDF5)
#include <highfive/H5File.hpp>
#endif

#if defined(RL_TOOLS_ENABLE_CLI11) && !defined(RL_TOOLS_DISABLE_CLI11)
#include <CLI/CLI.hpp>
#endif


#include <sstream>


namespace parameters = parameters_0;

#if defined(RL_TOOLS_ENABLE_TENSORBOARD) && !defined(RL_TOOLS_DISABLE_TENSORBOARD)
using LOGGER = rlt::devices::logging::CPU_TENSORBOARD<>;
#else
using LOGGER = rlt::devices::logging::CPU;
#endif

using DEV_SPEC_SUPER = rlt::devices::cpu::Specification<rlt::devices::math::CPU, rlt::devices::random::CPU, LOGGER>;
using TI = typename rlt::devices::DEVICE_FACTORY<DEV_SPEC_SUPER>::index_t;
constexpr TI NUM_RUNS = 1;
namespace execution_hints{
    struct HINTS: rlt::rl::components::on_policy_runner::ExecutionHints<TI, 16>{};
}
struct DEV_SPEC: DEV_SPEC_SUPER{
    using EXECUTION_HINTS = execution_hints::HINTS;
};

using DEVICE = rlt::devices::DEVICE_FACTORY<DEV_SPEC>;
using RNG = DEVICE::SPEC::RANDOM::ENGINE<>;
// -------------- added for cuda training ----------------
using DEVICE_GPU = rlt::devices::DEVICE_FACTORY_CUDA<rlt::devices::DefaultCUDASpecification>;
using RNG_GPU = DEVICE_GPU::SPEC::RANDOM::ENGINE<>;
// -------------------------------------------------------
using T = float;
using TI = typename DEVICE::index_t;


constexpr TI BASE_SEED = 600;
#if defined(RL_TOOLS_ENABLE_HDF5) && !defined(RL_TOOLS_DISABLE_HDF5)
constexpr bool ACTOR_ENABLE_CHECKPOINTS = true;
#else
constexpr bool ACTOR_ENABLE_CHECKPOINTS = false;
#endif
constexpr TI ACTOR_CHECKPOINT_INTERVAL = 100000;
#if !defined(RL_TOOLS_RL_ENVIRONMENTS_MUJOCO_ANT_DISABLE_EVALUATION)
constexpr bool ENABLE_EVALUATION = true;
#else
constexpr bool ENABLE_EVALUATION = false;
#endif
constexpr TI NUM_EVALUATION_EPISODES = 10;
constexpr TI EVALUATION_INTERVAL = 100000;
constexpr bool ACTOR_OVERWRITE_CHECKPOINTS = false;
std::string sanitize_file_name(const std::string &input) {
    std::string output = input;

    const std::string invalid_chars = R"(<>:\"/\|?*)";

    std::replace_if(output.begin(), output.end(), [&invalid_chars](const char &c) {
        return invalid_chars.find(c) != std::string::npos;
    }, '_');

    return output;
}

// --------------- changed for cuda training -----------------
int main(int argc, char** argv){
    std::string actor_checkpoints_dir_stub = "checkpoints";
    std::string logs_dir = "logs";
    TI job_seed = 0;
#if defined(RL_TOOLS_ENABLE_CLI11) && !defined(RL_TOOLS_DISABLE_CLI11)
    {
        CLI::App app;
        app.add_option("--checkpoints", actor_checkpoints_dir_stub, "path to the checkpoint directory");
        app.add_option("--logs", logs_dir, "path to the logs directory");
        app.add_option("--seed", job_seed, "seed for this job");
//        app.add_option("--runs", num_runs, "number of runs with different seeds");
        CLI11_PARSE(app, argc, argv);
    }
#endif
    std::string actor_checkpoints_dir = actor_checkpoints_dir_stub + "/ppo_ant";
    if (ACTOR_ENABLE_CHECKPOINTS){
        std::cout << "Saving actor checkpoints to: " << actor_checkpoints_dir << std::endl;
    }
// -------------------------------------------------------
    for(TI run_i = 0; run_i < NUM_RUNS; ++run_i){
        using penv = parameters::environment<double, TI>;
        using prl = parameters::rl<T, TI, penv::ENVIRONMENT>;
        // -------------- added for cuda training ----------------
        using ON_POLICY_RUNNER_COLLECTION_EVALUATION_BUFFER_TYPE = rlt::rl::components::on_policy_runner::CollectionEvaluationBuffer<prl::ON_POLICY_RUNNER_SPEC>;
        using PPO_TRAINING_HYBRID_BUFFER_TYPE = rlt::rl::algorithms::ppo::TrainingBuffersHybrid<prl::PPO_SPEC>;
        // -------------------------------------------------------
        TI seed = BASE_SEED + job_seed * NUM_RUNS + run_i;
        std::stringstream run_name_ss;
        run_name_ss << "ppo_ant_" + std::to_string(seed);
        if(prl::PPO_SPEC::PARAMETERS::LEARN_ACTION_STD){
            run_name_ss << "_learn_astd";
        }
        if(prl::PPO_SPEC::PARAMETERS::NORMALIZE_OBSERVATIONS){
            run_name_ss << "_normobs";
        }
        if(prl::PPO_SPEC::PARAMETERS::ADAPTIVE_LEARNING_RATE){
            run_name_ss << "_adapt_lr";
        }
        if(prl::PPO_SPEC::PARAMETERS::NORMALIZE_ADVANTAGE){
            run_name_ss << "_norm_adv";
        }
        std::string run_name = run_name_ss.str();
        {
            auto now = std::chrono::system_clock::now();
            auto local_time = std::chrono::system_clock::to_time_t(now);
            std::tm* tm = std::localtime(&local_time);

            std::ostringstream oss;
            oss << std::put_time(tm, "%FT%T%z");
            run_name = oss.str() + "_" + run_name;
            run_name = sanitize_file_name(run_name);
        }


        DEVICE::SPEC::LOGGING logger;
        DEVICE device;
        // -------------- added for cuda training ----------------
        DEVICE_GPU device_gpu;
        // -------------------------------------------------------
        prl::ACTOR_OPTIMIZER actor_optimizer;
        prl::CRITIC_OPTIMIZER critic_optimizer;
        RNG rng, evaluation_rng;;
        RNG_GPU rng_gpu;
        prl::PPO_TYPE ppo;
        // -------------- added for cuda training ----------------
        prl::PPO_TYPE ppo_gpu;
        // -------------------------------------------------------
        prl::PPO_BUFFERS_TYPE ppo_buffers;
        prl::ON_POLICY_RUNNER_TYPE on_policy_runner;
        prl::ON_POLICY_RUNNER_DATASET_TYPE on_policy_runner_dataset;
        // -------------- added for cuda training ----------------
        ON_POLICY_RUNNER_COLLECTION_EVALUATION_BUFFER_TYPE on_policy_runner_collection_eval_buffer_gpu, on_policy_runner_collection_eval_buffer_cpu;
        PPO_TRAINING_HYBRID_BUFFER_TYPE ppo_training_hybrid_buffer_cpu, ppo_training_hybrid_buffer_gpu;
        rlt::Matrix<rlt::matrix::Specification<T, TI, decltype(on_policy_runner_dataset.data)::ROWS, prl::PPO_SPEC::ENVIRONMENT::Observation::DIM>> gae_all_observations;
        rlt::Matrix<rlt::matrix::Specification<T, TI, decltype(on_policy_runner_dataset.data)::ROWS, 1>> gae_all_values;
        // -------------------------------------------------------
        // -------------- replaced for cuda training ----------------
        prl::ACTOR_EVAL_BUFFERS actor_eval_buffers, actor_eval_buffers_gpu;
        prl::PPO_TYPE::SPEC::ACTOR_TYPE::Buffer<1> actor_deterministic_eval_buffers;
        // ----------------------------------------------------------
        prl::ACTOR_BUFFERS actor_buffers;
        prl::CRITIC_BUFFERS critic_buffers;
        prl::CRITIC_BUFFERS_GAE critic_buffers_gae;
        rlt::rl::components::RunningNormalizer<rlt::rl::components::running_normalizer::Specification<T, TI, penv::ENVIRONMENT::Observation::DIM>> observation_normalizer;
        penv::ENVIRONMENT envs[prl::N_ENVIRONMENTS];
        penv::ENVIRONMENT::Parameters env_parameters[prl::N_ENVIRONMENTS];
        penv::ENVIRONMENT evaluation_env;
        rlt::rl::environments::DummyUI ui;
        TI next_checkpoint_id = 0;
        TI next_evaluation_id = 0;

        // -------------- added for cuda training ----------------
        rlt::init(device);
        rlt::init(device_gpu);
        // -------------------------------------------------------
        rlt::malloc(device, rng);
        rlt::malloc(device, evaluation_rng);
        rlt::init(device, rng, seed);
        rlt::init(device, evaluation_rng, seed);
        rlt::malloc(device, actor_optimizer);
        rlt::malloc(device, critic_optimizer);
        rlt::malloc(device, ppo);
        rlt::malloc(device, ppo_buffers);
        rlt::malloc(device, on_policy_runner_dataset);
        // -------------- added for cuda training ----------------
        rlt::malloc(device, on_policy_runner_collection_eval_buffer_cpu);
        rlt::malloc(device, ppo_training_hybrid_buffer_cpu);
        // -------------------------------------------------------
        rlt::malloc(device, on_policy_runner);
        rlt::malloc(device, actor_eval_buffers);
        rlt::malloc(device, actor_deterministic_eval_buffers);
        // ------------- removed for cuda training ---------------
//        rlt::malloc(device, actor_buffers);
//        rlt::malloc(device, critic_buffers);
//        rlt::malloc(device, critic_buffers_gae);
        // -------------------------------------------------------
        rlt::malloc(device, observation_normalizer);
        for(TI env_i = 0; env_i < prl::N_ENVIRONMENTS; env_i++){
            rlt::malloc(device, envs[env_i]);
        }
        rlt::malloc(device, evaluation_env);
        // -------------- added for cuda training ----------------
        rlt::malloc(device_gpu, rng_gpu);
        rlt::malloc(device_gpu, actor_buffers);
        rlt::malloc(device_gpu, critic_buffers);
        rlt::malloc(device_gpu, critic_buffers_gae);
        rlt::malloc(device_gpu, ppo_gpu);
        rlt::malloc(device_gpu, on_policy_runner_collection_eval_buffer_gpu);
        rlt::malloc(device_gpu, ppo_training_hybrid_buffer_gpu);
        rlt::malloc(device_gpu, actor_eval_buffers_gpu);
        rlt::malloc(device_gpu, gae_all_observations);
        rlt::malloc(device_gpu, gae_all_values);
        // -------------------------------------------------------

//        auto on_policy_runner_dataset_all_observations = prl::PPO_SPEC::PARAMETERS::NORMALIZE_OBSERVATIONS ? on_policy_runner_dataset.all_observations_normalized : on_policy_runner_dataset.all_observations;
//        auto on_policy_runner_dataset_observations = prl::PPO_SPEC::PARAMETERS::NORMALIZE_OBSERVATIONS ? on_policy_runner_dataset.observations_normalized : on_policy_runner_dataset.observations;

        rlt::init(device);
        rlt::init(device, on_policy_runner, envs, env_parameters, rng);
        rlt::init(device, observation_normalizer);
        rlt::init(device, ppo, actor_optimizer, critic_optimizer, rng);
        // -------------- added for cuda training ----------------
        rlt::copy(device, device_gpu, ppo, ppo_gpu);
        // -------------------------------------------------------
        rlt::init(device, device.logger);
        rlt::init(device_gpu, rng_gpu, seed);
        auto training_start = std::chrono::high_resolution_clock::now();
        if(prl::PPO_SPEC::PARAMETERS::NORMALIZE_OBSERVATIONS){
            for(TI observation_normalization_warmup_step_i = 0; observation_normalization_warmup_step_i < prl::OBSERVATION_NORMALIZATION_WARMUP_STEPS; observation_normalization_warmup_step_i++) {
                rlt::collect(device, on_policy_runner_dataset, on_policy_runner, ppo.actor, actor_eval_buffers, rng);
                rlt::update(device, observation_normalizer, on_policy_runner_dataset.observations);
            }
            std::cout << "Observation means: " << std::endl;
            rlt::print(device, observation_normalizer.mean);
            std::cout << "Observation std: " << std::endl;
            rlt::print(device, observation_normalizer.std);
            rlt::init(device, on_policy_runner, envs, env_parameters, rng); // reinitializing the on_policy_runner to reset the episode counters
            rlt::set_statistics(device, ppo.actor.content, observation_normalizer.mean, observation_normalizer.std);
            rlt::set_statistics(device, ppo.critic.content, observation_normalizer.mean, observation_normalizer.std);
            rlt::copy(device, device_gpu, ppo, ppo_gpu);
        }
        for(TI ppo_step_i = 0; ppo_step_i < 2500; ppo_step_i++) {
            // -------------- added for cuda training ----------------
            rlt::copy(device_gpu, device, ppo_gpu, ppo);
            // -------------------------------------------------------
#if defined(RL_TOOLS_ENABLE_HDF5) && !defined(RL_TOOLS_DISABLE_HDF5)
            if(ACTOR_ENABLE_CHECKPOINTS && (on_policy_runner.step / ACTOR_CHECKPOINT_INTERVAL == next_checkpoint_id)){
                std::filesystem::path actor_output_dir = std::filesystem::path(actor_checkpoints_dir) / run_name;
                try {
                    std::filesystem::create_directories(actor_output_dir);
                }
                catch (std::exception& e) {
                }
                std::string checkpoint_name = "latest.h5";
                if(!ACTOR_OVERWRITE_CHECKPOINTS){
                    std::stringstream checkpoint_name_ss;
                    checkpoint_name_ss << "actor_" << std::setw(15) << std::setfill('0') << next_checkpoint_id << "_" << std::setw(15) << std::setfill('0') << on_policy_runner.step << ".h5";
                    checkpoint_name = checkpoint_name_ss.str();
                }
                std::filesystem::path actor_output_path = actor_output_dir / checkpoint_name;
                try{
                    auto actor_file = HighFive::File(actor_output_path.string(), HighFive::File::Overwrite);
                    rlt::save(device, ppo.actor, actor_file.createGroup("actor"));
                    rlt::save(device, observation_normalizer, actor_file.createGroup("observation_normalizer"));
                }
                catch(HighFive::Exception& e){
                    std::cout << "Error while saving actor: " << e.what() << std::endl;
                }
                next_checkpoint_id++;
            }
#endif
            if(ENABLE_EVALUATION && (on_policy_runner.step / EVALUATION_INTERVAL == next_evaluation_id)){
                using RESULT_SPEC = rlt::rl::utils::evaluation::Specification<T, TI, penv::ENVIRONMENT, NUM_EVALUATION_EPISODES, prl::ON_POLICY_RUNNER_STEP_LIMIT>;
                rlt::rl::utils::evaluation::Result<RESULT_SPEC> result;
                rlt::evaluate(device, evaluation_env, ui, ppo.actor, result, evaluation_rng);
//                rlt::add_scalar(device, device.logger, "evaluation/return/mean", result.mean);
//                rlt::add_scalar(device, device.logger, "evaluation/return/std", result.std);
                rlt::add_histogram(device, device.logger, "evaluation/return", result.returns, decltype(result)::N_EPISODES);
                std::cout << "Evaluation return mean: " << result.returns_mean << " (std: " << result.returns_std << ")" << std::endl;
                next_evaluation_id++;
            }
            rlt::set_step(device, device.logger, on_policy_runner.step);

//            for (TI action_i = 0; action_i < penv::ENVIRONMENT::ACTION_DIM; action_i++) {
//                T action_log_std = rlt::get(ppo.actor.log_std.parameters, 0, action_i);
//                std::stringstream topic;
//                topic << "actor/action_std/" << action_i;
//                rlt::add_scalar(device, device.logger, topic.str(), rlt::math::exp(DEVICE::SPEC::MATH(), action_log_std));
//            }
//            auto start = std::chrono::high_resolution_clock::now();
            auto training_step_start = std::chrono::high_resolution_clock::now();
            {
//                auto start = std::chrono::high_resolution_clock::now();
                // -------------- replaced for cuda training ----------------
                rlt::collect_hybrid(device, device_gpu, on_policy_runner_dataset, on_policy_runner, ppo.actor, ppo_gpu.actor, actor_eval_buffers_gpu, on_policy_runner_collection_eval_buffer_cpu, on_policy_runner_collection_eval_buffer_gpu, rng, rng_gpu);
                // ----------------------------------------------------------
                if(prl::PPO_SPEC::PARAMETERS::NORMALIZE_OBSERVATIONS){
                    rlt::update(device, observation_normalizer, on_policy_runner_dataset.observations);
                    rlt::set_statistics(device, ppo.actor.content, observation_normalizer.mean, observation_normalizer.std);
                    rlt::set_statistics(device, ppo.critic.content, observation_normalizer.mean, observation_normalizer.std);
                    rlt::copy(device, device_gpu, ppo, ppo_gpu);
                    for(TI state_i = 0; state_i < penv::ENVIRONMENT::Observation::DIM; state_i++){
//                        rlt::add_scalar(device, device.logger, std::string("observation_normalizer/mean_") + std::to_string(state_i), get(observation_normalizer.mean, 0, state_i));
//                        rlt::add_scalar(device, device.logger, std::string("observation_normalizer/std") + std::to_string(state_i), get(observation_normalizer.std, 0, state_i));
                    }
                }
//                rlt::add_scalar(device, device.logger, "opr/observation/mean", rlt::mean(device, on_policy_runner_dataset.observations));
//                rlt::add_scalar(device, device.logger, "opr/observation/std", rlt::std(device, on_policy_runner_dataset.observations));
//                rlt::add_scalar(device, device.logger, "opr/action/mean", rlt::mean(device, on_policy_runner_dataset.actions));
//                rlt::add_scalar(device, device.logger, "opr/action/std", rlt::std(device, on_policy_runner_dataset.actions));
//                rlt::add_scalar(device, device.logger, "opr/rewards/mean", rlt::mean(device, on_policy_runner_dataset.rewards));
//                rlt::add_scalar(device, device.logger, "opr/rewards/std", rlt::std(device, on_policy_runner_dataset.rewards));
//                auto end = std::chrono::high_resolution_clock::now();
//                std::chrono::duration<T> elapsed = end - start;
//                std::cout << "Rollout: " << elapsed.count() << " s" << std::endl;
            }
            {
//                auto start = std::chrono::high_resolution_clock::now();
                // -------------- replaced for cuda training ----------------
                copy(device, device_gpu, on_policy_runner_dataset.all_observations_privileged, gae_all_observations);
                evaluate(device_gpu, ppo_gpu.critic, gae_all_observations, gae_all_values, critic_buffers_gae, rng);
                copy(device_gpu, device, gae_all_values, on_policy_runner_dataset.all_values);
                // ----------------------------------------------------------
                rlt::estimate_generalized_advantages(device, on_policy_runner_dataset, prl::PPO_TYPE::SPEC::PARAMETERS{});
//                auto end = std::chrono::high_resolution_clock::now();
//                std::chrono::duration<T> elapsed = end - start;
//                std::cout << "GAE: " << elapsed.count() << " s" << std::endl;
            }
            {
//                auto start = std::chrono::high_resolution_clock::now();
                // -------------- replaced for cuda training ----------------
                rlt::train_hybrid(device, device_gpu, ppo, ppo_gpu, on_policy_runner_dataset, actor_optimizer, critic_optimizer, ppo_buffers, ppo_training_hybrid_buffer_gpu, actor_buffers, critic_buffers, rng);
                // ----------------------------------------------------------
//                auto end = std::chrono::high_resolution_clock::now();
//                std::chrono::duration<T> elapsed = end - start;
//                std::cout << "Train: " << elapsed.count() << " s" << std::endl;
            }
//            auto end = std::chrono::high_resolution_clock::now();
//            std::chrono::duration<T> elapsed = end - start;
//            std::cout << "Total: " << elapsed.count() << " s" << std::endl;
            if(ppo_step_i % 1 == 0){
                auto now = std::chrono::high_resolution_clock::now();
                std::chrono::duration<T> training_elapsed = now - training_start;
                std::chrono::duration<T> step_elapsed = now - training_step_start;
                T steps_per_second_lifetime = on_policy_runner.step / training_elapsed.count();
                T steps_per_second_current = prl::ON_POLICY_RUNNER_SPEC::N_ENVIRONMENTS * prl::ON_POLICY_RUNNER_STEPS_PER_ENV / step_elapsed.count();
                std::cout << "PPO step: " << std::setw(10) << ppo_step_i << " environment step: " << std::setw(10) << on_policy_runner.step << " elapsed: " << std::setw(10) << std::setprecision(2) << training_elapsed.count() << "s (lifetime: " << std::setw(10) << std::setprecision(2) << steps_per_second_lifetime << " steps/s, current: " << std::setw(10) << std::setprecision(2) << steps_per_second_current << " steps/s)" << std::endl;
//                rlt::add_scalar(device, device.logger, "ppo/step", ppo_step_i);
//                rlt::add_scalar(device, device.logger, "ppo/actor_learning_rate", actor_optimizer.alpha);
//                rlt::add_scalar(device, device.logger, "ppo/critic_learning_rate", critic_optimizer.alpha);
            }
        }

        rlt::free(device, ppo);
        rlt::free(device, ppo_buffers);
        rlt::free(device, on_policy_runner_dataset);
        // -------------- added for cuda training ----------------
        rlt::free(device, on_policy_runner_collection_eval_buffer_cpu);
        rlt::free(device, ppo_training_hybrid_buffer_cpu);
        // -------------------------------------------------------
        rlt::free(device, on_policy_runner);
        rlt::free(device, actor_eval_buffers);
        // ------------- removed for cuda training ---------------
//        rlt::free(device, actor_buffers);
//        rlt::free(device, critic_buffers);
//        rlt::free(device, critic_buffers_gae);
        // -------------------------------------------------------
        rlt::free(device, observation_normalizer);
        for(auto& env : envs){
            rlt::free(device, env);
        }
        rlt::free(device, evaluation_env);
        // -------------- added for cuda training ----------------
        rlt::free(device_gpu, actor_buffers);
        rlt::free(device_gpu, critic_buffers);
        rlt::free(device_gpu, critic_buffers_gae);
        rlt::free(device_gpu, ppo_gpu);
        rlt::free(device_gpu, on_policy_runner_collection_eval_buffer_gpu);
        rlt::free(device_gpu, ppo_training_hybrid_buffer_gpu);
        rlt::free(device_gpu, actor_eval_buffers_gpu);
        rlt::free(device_gpu, gae_all_observations);
        rlt::free(device_gpu, gae_all_values);
        // -------------------------------------------------------
    }

    return 0;
}
