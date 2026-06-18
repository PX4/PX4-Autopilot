#include <rl_tools/operations/cpu.h>
#include <rl_tools/nn/layers/dense/operations_cpu.h>
#include <rl_tools/nn/layers/gru/operations_generic.h>
#include <rl_tools/nn/layers/sample_and_squash/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>
#include <rl_tools/rl/environments/l2f/operations_cpu.h>
#include <rl_tools/rl/algorithms/sac/loop/core/config.h>
#include <rl_tools/rl/loop/steps/extrack/config.h>
#include <rl_tools/rl/loop/steps/evaluation/config.h>
#include <rl_tools/rl/loop/steps/checkpoint/config.h>
#include <rl_tools/rl/loop/steps/save_trajectories/config.h>
#include <rl_tools/rl/loop/steps/timing/config.h>



#include <rl_tools/nn/layers/dense/persist.h>
#include <rl_tools/nn/layers/gru/persist.h>
#include <rl_tools/nn/layers/sample_and_squash/persist.h>
#include <rl_tools/nn_models/sequential/persist.h>

#include <rl_tools/ui_server/client/operations_websocket.h>

namespace rlt = rl_tools;

//#include <CLI/CLI.hpp>
#include <regex>
#include <iostream>
#include <filesystem>


#include "approximators.h"


constexpr bool ORIGINAL_CONDITIONS = true;
constexpr bool AUTOMATIC_RESET = false;
constexpr bool ENV_ZERO_ORIENTATION_INIT = false;

using DEVICE = rlt::devices::DefaultCPU;
using RNG = DEVICE::SPEC::RANDOM::ENGINE<>;
using T = float;
using TI = typename DEVICE::index_t;

#include "parameters.h"

struct DEFAULT_CONFIG: rl_tools::rl::environments::l2f::parameters::DEFAULT_CONFIG{
    static constexpr bool ZERO_ORIENTATION_INIT = ORIGINAL_CONDITIONS || ZERO_ORIENTATION_INIT;
};

using EVALUATION_ENVIRONMENT = typename env_param_builder::ENVIRONMENT_PARAMETERS<DEFAULT_CONFIG>::ENVIRONMENT;


std::filesystem::path find_latest_checkpoint(std::filesystem::path experiments_path){
    std::vector<std::string> experiments;
    for (const auto & entry : std::filesystem::directory_iterator(experiments_path)){
        if (entry.is_directory()){
            experiments.push_back(entry.path().filename());
        }
    }
    std::sort(experiments.begin(), experiments.end());
    while(true){ // finding experiment
        if(experiments.empty()){
            std::cout << "No experiments found" << std::endl;
            return std::filesystem::path{};
        }
        std::filesystem::path latest_experiment = experiments_path / experiments.back();
        std::cout << "Latest experiment: " << latest_experiment << std::endl;
        std::vector<std::string> populations;
        for (const auto & entry : std::filesystem::directory_iterator(latest_experiment)){
            if (entry.is_directory()){
                populations.push_back(entry.path().filename());
            }
        }
        std::sort(populations.begin(), populations.end());
        while(true){ // finding population
            if(populations.empty()){
                std::cout << "No populations found" << std::endl;
                experiments.pop_back();
                break;
            }
            std::filesystem::path latest_population = latest_experiment / populations.back();
            std::cout << "Latest population: " << latest_population << std::endl;
            std::vector<std::string> configurations;
            for (const auto & entry : std::filesystem::directory_iterator(latest_population)){
                if (entry.is_directory()){
                    configurations.push_back(entry.path().filename());
                }
            }
            std::sort(configurations.begin(), configurations.end());
            while(true){ // finding configuration
                if(configurations.empty()){
                    std::cout << "No configurations found" << std::endl;
                    populations.pop_back();
                    break;
                }
                std::filesystem::path latest_configuration = latest_population / configurations.back();
                std::cout << "Latest configuration: " << latest_configuration << std::endl;
                std::vector<std::string> seeds;
                for (const auto & entry : std::filesystem::directory_iterator(latest_configuration)){
                    if (entry.is_directory()){
                        seeds.push_back(entry.path().filename());
                    }
                }
                std::sort(seeds.begin(), seeds.end());
                while(true){
                    if(seeds.empty()){
                        std::cout << "No seeds found" << std::endl;
                        configurations.pop_back();
                        break;
                    }
                    std::filesystem::path latest_seed = latest_configuration / seeds.back();
                    std::cout << "Latest seed: " << latest_seed << std::endl;
                    auto steps_directory = latest_seed / "steps";
                    if(!std::filesystem::exists(steps_directory)){
                        std::cout << "No steps directory found" << std::endl;
                        seeds.pop_back();
                        continue;
                    }
                    std::vector<std::string> steps;
                    for (const auto & entry : std::filesystem::directory_iterator(steps_directory)){
                        if (entry.is_directory()){
                            steps.push_back(entry.path().filename());
                        }
                    }
                    std::sort(steps.begin(), steps.end());
                    while(true){
                        if(steps.empty()){
                            std::cout << "No steps found" << std::endl;
                            seeds.pop_back();
                            break;
                        }
                        std::filesystem::path latest_step = steps_directory / steps.back();
                        std::cout << "Latest step: " << latest_step << std::endl;
                        auto hdf5_checkpoint = latest_step / "checkpoint.h5";
                        if(!std::filesystem::exists(hdf5_checkpoint)){
                            std::cout << "No checkpoint.h5 found" << std::endl;
                            steps.pop_back();
                            continue;
                        }
                        std::cout << "Found checkpoint.h5: " << hdf5_checkpoint << std::endl;
                        return hdf5_checkpoint;
                    }
                }
            }
        }
    }
}

int main(){
    std::filesystem::path experiments_path = "experiments";
    std::filesystem::path latest_checkpoint = find_latest_checkpoint(experiments_path);
    if(latest_checkpoint.empty()){
        std::cout << "No checkpoints found" << std::endl;
        return 1;
    }
    std::cout << "Latest checkpoint: " << latest_checkpoint << std::endl;

    using ACTOR = LOOP_CORE_CONFIG::ACTOR_CRITIC_TYPE::SPEC::ACTOR_NETWORK_TYPE::CHANGE_CAPABILITY<rlt::nn::capability::Forward<>>;
    ACTOR actor;
    ACTOR::Buffer<> buffer;
    ACTOR::State<> actor_state;


    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);

    rlt::malloc(device, actor);
    rlt::malloc(device, actor_state);
    rlt::malloc(device, buffer);

    HighFive::File file(latest_checkpoint, HighFive::File::ReadOnly);

    rlt::load(device, actor, file.getGroup("actor"));

    using ENVIRONMENT_UI = rlt::ui_server::client::UIWebSocket<EVALUATION_ENVIRONMENT>;
    ENVIRONMENT_UI ui;

    EVALUATION_ENVIRONMENT env;
    EVALUATION_ENVIRONMENT::Parameters parameters;
    EVALUATION_ENVIRONMENT::State state, next_state;


    rlt::malloc(device, env);
    rlt::sample_initial_parameters(device, env, parameters, rng);

    rlt::Tensor<rlt::tensor::Specification<T, TI, rlt::tensor::Shape<TI, 1, EVALUATION_ENVIRONMENT::Observation::DIM>>> observation;
    rlt::Tensor<rlt::tensor::Specification<T, TI, rlt::tensor::Shape<TI, 1, EVALUATION_ENVIRONMENT::ACTION_DIM>>> action;
    rlt::malloc(device, observation);
    rlt::malloc(device, action);

    auto observation_matrix_view = rlt::matrix_view(device, observation);
    auto action_matrix_view = rlt::matrix_view(device, action);




    rlt::init(device, env, parameters, ui);
    TI step = 0;
    bool truncated = true;
    T cumulative_rewards = 0;
    while(true){
        if(truncated){
            rlt::sample_initial_state(device, env, parameters, state, rng);
        }

        rlt::observe(device, env, parameters, state, EVALUATION_ENVIRONMENT::Observation{}, observation_matrix_view, rng);
        if(truncated || step % SEQUENCE_LENGTH == 0){
            rlt::reset(device, actor, actor_state, rng);
        }
        rlt::evaluate_step(device, actor, observation, actor_state, action, buffer, rng);
        T dt = rlt::step(device, env, parameters, state, action_matrix_view, next_state, rng);
        state = next_state;
        T reward = rlt::reward(device, env, parameters, state, action_matrix_view, next_state, rng);
        cumulative_rewards += reward;

        rlt::set_state(device, env, parameters, ui, state);
        std::this_thread::sleep_for(std::chrono::milliseconds((TI)(1000 * dt)));
        bool terminated = rlt::terminated(device, env, parameters, state, rng);
        truncated = terminated || step >= 500;
        if(truncated){
            std::cout << "Episode terminated after " << step << " steps with cumulative rewards: " << cumulative_rewards << std::endl;
            step = 0;
            cumulative_rewards = 0;
        }
        step++;
    }




    return 0;
}