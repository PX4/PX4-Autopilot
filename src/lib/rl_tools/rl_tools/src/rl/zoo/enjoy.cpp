#include <rl_tools/operations/cpu_mux.h>
#include <rl_tools/nn/operations_cpu_mux.h>
#include <rl_tools/nn/layers/standardize/operations_generic.h>
#include <rl_tools/nn_models/mlp_unconditional_stddev/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>

#include "td3/pendulum-v1.h"
#include "sac/pendulum-v1.h"
#include "ppo/pendulum-v1.h"

#include <rl_tools/rl/algorithms/td3/loop/core/operations_generic.h>
#include <rl_tools/rl/algorithms/sac/loop/core/operations_generic.h>
#include <rl_tools/rl/algorithms/ppo/loop/core/operations_generic.h>
#include <rl_tools/rl/loop/steps/checkpoint/operations_cpu.h>
#include <rl_tools/rl/loop/steps/extrack/operations_cpu.h>
#include <rl_tools/rl/loop/steps/evaluation/operations_generic.h>
#include <rl_tools/rl/loop/steps/timing/operations_cpu.h>

#include <rl_tools/rl/utils/evaluation/operations_generic.h>

#include <rl_tools/nn/layers/standardize/persist.h>
#include <rl_tools/nn_models/mlp_unconditional_stddev/persist.h>
#include <rl_tools/nn_models/sequential/persist.h>

#include <CLI/CLI.hpp>

#include <iostream>
#include <filesystem>
#include <regex>
#include <string>

namespace rlt = rl_tools;

using DEVICE = rlt::devices::DEVICE_FACTORY<>;
using RNG = DEVICE::SPEC::RANDOM::ENGINE<>;
using T = float;
using TI = typename DEVICE::index_t;

#include "config.h"

namespace fs = std::filesystem;

template <typename DEVICE>
fs::path findLastLexicographicMatch(DEVICE& device, const fs::path& directory, const std::string& regex_pattern, bool check_commit_hash) {
    std::regex r(regex_pattern);
    std::vector<fs::path> paths;
    for (const auto& entry : fs::recursive_directory_iterator(directory)) {
        if (entry.is_regular_file()) {
            std::string filePath = entry.path().string();
            std::cout << "Testing " << filePath << " with: " << regex_pattern << std::endl;
            std::smatch match;
            if(std::regex_search(filePath, match, r)){
                rlt::utils::assert_exit(device, match.size() == 6, "The regex pattern must have 5 capture groups.");
                if(check_commit_hash){
                    std::string current_commit_hash = RL_TOOLS_STRINGIFY(RL_TOOLS_COMMIT_HASH);
                    std::string entry_commit_hash = match[1];
                    if(current_commit_hash.substr(0, entry_commit_hash.length()) != entry_commit_hash){
                        std::cout << "Skipping " << filePath << " because the commit hash (" << entry_commit_hash << ") does not match the current commit hash (" << current_commit_hash << ")." << std::endl;
                        continue;
                    }
                }
                paths.push_back(entry.path());
            }
        }
    }
    std::sort(paths.begin(), paths.end());
    if (paths.empty()) {
        return fs::path();
    }
    else{
        return paths.back();
    }
}


template <typename DEVICE, typename ACTOR, typename ACTOR_BUFFER, typename RNG>
void evaluate(DEVICE& device, ACTOR& actor, ACTOR_BUFFER& actor_buffer, RNG& rng){
    using ENVIRONMENT = LOOP_CONFIG::ENVIRONMENT;
    ENVIRONMENT env;
    rlt::rl::environments::DummyUI ui;
    using RESULT_SPEC = rlt::rl::utils::evaluation::Specification<T, TI, ENVIRONMENT, 10000, ENVIRONMENT::EPISODE_STEP_LIMIT>;
    rlt::rl::utils::evaluation::Result<RESULT_SPEC> result;
    rlt::evaluate(device, env, ui, actor, result, actor_buffer, rng);
    std::cout << "Mean return: " << result.returns_mean << " +/- " << result.returns_std << std::endl;

}

int main(int argc, char** argv){
    using LOOP_STATE = LOOP_CONFIG::State<LOOP_CONFIG>;
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);
    TI seed = 0;
    LOOP_STATE ts;
    CLI::App app{"rl_zoo"};
    std::filesystem::path experiments_path, checkpoint_path;
    app.add_option("-e,--experiments", experiments_path, "experiments");
    app.add_option("-c,--checkpoint", checkpoint_path, "checkpoint");
    bool relax_commit_hash_checking = false;
    app.add_option("-r,--relax", relax_commit_hash_checking, "relax commit hash conformity between the version of this code and the version of the checkpoint");
    CLI11_PARSE(app, argc, argv);
    if(checkpoint_path.empty()){
        rlt::utils::assert_exit(device, !experiments_path.empty(), "Experiments path (-e,--experiments) must be set if the checkpoint path (-c,--checkpoint) is not set.");
        std::string regex_pattern = R"((\w\w\w\w\w\w\w)([^/]+)\/([^/]+)\/(\d+)\/steps\/(\d+)\/checkpoint.h5$)";
        checkpoint_path = findLastLexicographicMatch(device, experiments_path, regex_pattern, !relax_commit_hash_checking);
        if(checkpoint_path.empty()){
            std::cerr << "No suitable checkpoint found in: " << experiments_path << std::endl;
            return 1;
        }
    }
    std::cerr << "Checkpoint: " << checkpoint_path << std::endl;
//    using ACTOR = rlt::utils::typing::remove_reference<decltype(rlt::get_actor(std::declval<std::add_lvalue_reference_t<LOOP_STATE>>()))>;
    using ACTOR = LOOP_CONFIG::NN::ACTOR_TYPE::template CHANGE_CAPABILITY<rlt::nn::capability::Forward>;
    ACTOR actor;
    ACTOR::template Buffer<1> actor_buffer;
    rlt::malloc(device, actor);
    auto actor_file = HighFive::File(checkpoint_path.string(), HighFive::File::ReadOnly);
    rlt::load(device, actor, actor_file.getGroup("actor"));
    rlt::malloc(device, actor_buffer);
    evaluate(device, actor, actor_buffer, rng);
    rlt::free(device, actor);
    rlt::free(device, actor_buffer);


    return 0;
}
