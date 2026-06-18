#include <rl_tools/operations/cpu.h>

#include <rl_tools/rl/environments/mujoco/ant/operations_cpu.h>
#include <rl_tools/rl/environments/mujoco/ant/ui.h>
#include <rl_tools/nn/optimizers/adam/instance/operations_generic.h>
#include <rl_tools/nn/optimizers/adam/instance/persist.h>
#include <rl_tools/nn/layers/standardize/operations_generic.h>
#include <rl_tools/nn/layers/standardize/persist.h>
#include <rl_tools/nn_models/operations_cpu.h>
#include <rl_tools/nn_models/mlp_unconditional_stddev/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>
#include <rl_tools/nn_models/persist.h>
#include <rl_tools/nn_models/mlp/persist.h>
#include <rl_tools/nn_models/sequential/persist.h>
#include <rl_tools/rl/components/running_normalizer/operations_generic.h>

namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools;

#ifdef RL_TOOLS_TEST_RL_ENVIRONMENTS_MUJOCO_ANT_EVALUATE_ACTOR_PPO
#include "ppo/parameters.h"
#else
#include "td3/parameters.h"
#endif

#include <chrono>
#include <iostream>
#include <filesystem>
#include <algorithm>
#include <thread>
#include <highfive/H5File.hpp>
#include <CLI/CLI.hpp>

namespace TEST_DEFINITIONS{
    using DEVICE = rlt::devices::DefaultCPU;
    using RNG = typename DEVICE::SPEC::RANDOM::ENGINE<>;
    using T = double;
    using TYPE_POLICY = rlt::numeric_types::Policy<T>;
    using TI = typename DEVICE::index_t;
    namespace parameter_set = parameters_0;

    using parameters_environment = parameter_set::environment<T, TI>;
    struct ENVIRONMENT_EVALUATION_PARAMETERS: parameters_environment::ENVIRONMENT_SPEC::PARAMETERS{
        constexpr static TI FRAME_SKIP = 5; // for smoother playback
    };
    using ENVIRONMENT_EVALUATION_SPEC = rlt::rl::environments::mujoco::ant::Specification<T, TI, ENVIRONMENT_EVALUATION_PARAMETERS>;
    using ENVIRONMENT = rlt::rl::environments::mujoco::Ant<ENVIRONMENT_EVALUATION_SPEC>;
    using UI = rlt::rl::environments::mujoco::ant::UI<ENVIRONMENT>;

    using parameters_rl = parameter_set::rl<TYPE_POLICY, TI, ENVIRONMENT>;
    constexpr TI MAX_EPISODE_LENGTH = 1000;
}


int main(int argc, char** argv) {
    using namespace TEST_DEFINITIONS;
    CLI::App app;
    std::string run = "", checkpoint = "";
    DEVICE::index_t startup_timeout = 0;
    app.add_option("--run", run, "path to the run's directory");
    app.add_option("--checkpoint", checkpoint, "path to the checkpoint");
    app.add_option("--timeout", startup_timeout, "time to wait after first render");

    CLI11_PARSE(app, argc, argv);
    DEVICE dev;
    ENVIRONMENT env;
    ENVIRONMENT::Parameters env_parameters;
    UI ui;
    parameters_rl::ACTOR_TYPE actor;
    parameters_rl::ACTOR_TYPE::Buffer<1> actor_buffer;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, ENVIRONMENT::ACTION_DIM>> action;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, ENVIRONMENT::Observation::DIM>> observation;
    typename ENVIRONMENT::State state, next_state;
    RNG rng;
    rlt::rl::components::RunningNormalizer<rlt::rl::components::running_normalizer::Specification<TYPE_POLICY, TI, ENVIRONMENT::Observation::DIM>> observation_normalizer;

    rlt::malloc(dev, rng);
    rlt::malloc(dev, env);
    rlt::malloc(dev, actor);
    rlt::malloc(dev, actor_buffer);
    rlt::malloc(dev, action);
    rlt::malloc(dev, observation);
    rlt::malloc(dev, observation_normalizer);

    rlt::init(dev, rng, 0);
    rlt::init(dev, env, env_parameters, ui);
    rlt::init(dev, observation_normalizer);
    DEVICE::index_t episode_i = 0;
    while(true){
        std::filesystem::path actor_run;
        if(run == "" && checkpoint == ""){
#ifdef RL_TOOLS_TEST_RL_ENVIRONMENTS_MUJOCO_ANT_EVALUATE_ACTOR_PPO
            std::filesystem::path actor_checkpoints_dir = std::filesystem::path("checkpoints") / "ppo_ant";
#else
            std::filesystem::path actor_checkpoints_dir = std::filesystem::path("checkpoints") / "td3_ant";
#endif
            std::vector<std::filesystem::path> actor_runs;

            for (const auto& run : std::filesystem::directory_iterator(actor_checkpoints_dir)) {
                if (run.is_directory()) {
                    actor_runs.push_back(run.path());
                }
            }
            std::sort(actor_runs.begin(), actor_runs.end());
            actor_run = actor_runs.back();
        }
        else{
            actor_run = run;
        }
        if(checkpoint == ""){
            std::vector<std::filesystem::path> actor_checkpoints;
            for (const auto& checkpoint : std::filesystem::directory_iterator(actor_run)) {
                if (checkpoint.is_regular_file()) {
                    actor_checkpoints.push_back(checkpoint.path());
                }
            }
            std::sort(actor_checkpoints.begin(), actor_checkpoints.end());
            checkpoint = actor_checkpoints.back().string();
        }

        std::cout << "Loading actor from " << checkpoint << std::endl;
        {
            try{
                auto data_file = HighFive::File(checkpoint, HighFive::File::ReadOnly);
                auto group = rlt::get_group(dev, data_file, "actor");
                rlt::load(dev, actor, group);
#ifdef RL_TOOLS_TEST_RL_ENVIRONMENTS_MUJOCO_ANT_EVALUATE_ACTOR_PPO
                auto obsnorm_group = rlt::get_group(dev, data_file, "observation_normalizer");
                rlt::load(dev, observation_normalizer.mean, obsnorm_group, "mean");
                rlt::load(dev, observation_normalizer.std, obsnorm_group, "std");
#endif
            }
            catch(HighFive::FileException& e){
                std::cout << "Failed to load actor from " << checkpoint << std::endl;
                std::cout << "Error: " << e.what() << std::endl;
                continue;
            }
        }

        rlt::sample_initial_parameters(dev, env, env_parameters, rng);
        rlt::sample_initial_state(dev, env, env_parameters, state, rng);
        T reward_acc = 0;
        decltype(actor)::State<true> actor_state;
        rlt::malloc(dev, actor_state);
        rlt::reset(dev, actor, actor_state, rng);
        for(TI step_i = 0; step_i < MAX_EPISODE_LENGTH; step_i++){
            auto start = std::chrono::high_resolution_clock::now();
            rlt::observe(dev, env, env_parameters, state, typename ENVIRONMENT::Observation{}, observation, rng);
            auto observation_tensor = rlt::to_tensor(dev, observation);
            auto action_tensor = rlt::to_tensor(dev, action);

            rlt::evaluate_step(dev, actor, observation_tensor, actor_state, action_tensor, actor_buffer, rng, rlt::Mode<rlt::mode::Evaluation<>>{});
            T dt = rlt::step(dev, env, env_parameters, state, action, next_state, rng);
            bool terminated_flag = rlt::terminated(dev, env, env_parameters, next_state, rng);
            reward_acc += rlt::reward(dev, env, env_parameters, state, action, next_state, rng);
            rlt::set_state(dev, env, env_parameters, ui, state);
            state = next_state;
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> diff = end-start;
            if(startup_timeout > 0 && episode_i == 0 && step_i == 0){
                for(TI timeout_step_i = 0; timeout_step_i < startup_timeout; timeout_step_i++){
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    if(timeout_step_i % 100 == 0){
                        rlt::set_state(dev, env, env_parameters, ui, state);
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds((TI)((dt - diff.count())*1000)));
            if(terminated_flag || step_i == (MAX_EPISODE_LENGTH - 1)){
                std::cout << "Episode terminated after " << step_i << " steps with reward " << reward_acc << std::endl;
                break;
            }
        }
        episode_i++;
    }
}

