#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_UTILS_EXTRACK_OPERATIONS_CPU_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_UTILS_EXTRACK_OPERATIONS_CPU_H


#include "extrack.h"

#include <iostream>
#include <chrono>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <filesystem>
#include <fstream>
#include <string>
#include <algorithm>

#include <cstdlib>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    // note usage examples in ./extrack.h
    template <typename DEVICE, typename TI>
    void init(DEVICE& device, utils::extrack::Config<TI>& config, utils::extrack::Paths& paths, typename DEVICE::index_t seed = 0){
        if(paths.experiment.empty()){
            utils::assert_exit(device, !config.base_path.empty(), "Extrack base path (-e,--extrack) must be set if the Extrack experiment path (--ee,--extrack-experiment) is not set.");
            if(config.experiment.empty()){

                auto environment_extrack_experiment = std::getenv("RL_TOOLS_EXTRACK_EXPERIMENT");
                if(environment_extrack_experiment != nullptr){
                    config.experiment = std::string(environment_extrack_experiment);
                } else {
                    config.experiment = utils::extrack::get_timestamp_string();
                }
            }
            paths.experiment = config.base_path / config.experiment;
        }
        {
#ifdef RL_TOOLS_COMMIT_HASH_SET
            std::string commit_hash = RL_TOOLS_STRINGIFY(RL_TOOLS_COMMIT_HASH);
            std::string short_hash = commit_hash.substr(0, 7);
#else
            std::string short_hash = "no-hash";
#endif

            std::string setup_name = short_hash + "_" + config.name;
            if(!config.population_variates.empty()){
                setup_name = setup_name + "_" + config.population_variates;
            }
            paths.setup = paths.experiment / setup_name;
            paths.config = paths.setup / config.population_values;
        }
        {
            std::stringstream padded_seed_ss;
            padded_seed_ss << std::setw(4) << std::setfill('0') << seed;
            paths.seed = paths.config / padded_seed_ss.str();
        }
        std::cerr << "Seed: " << seed << std::endl;
        std::cerr << "Extrack Experiment: " << paths.seed << std::endl;
#ifdef RL_TOOLS_EXTRACK_GIT_DIFF
        // Save rl-tools library git info
        std::filesystem::path git_library_path = paths.seed / "git" / "rl_tools";
        std::filesystem::create_directories(git_library_path);
        {
            std::ofstream diff_file(git_library_path / "commit.txt");
            diff_file << rl_tools::utils::extrack::git::rl_tools::commit;
            diff_file.close();
        }
        {
            std::ofstream diff_file(git_library_path / "diff.txt");
            diff_file << rl_tools::utils::extrack::git::rl_tools::diff;
            diff_file.close();
        }
        {
            std::ofstream diff_file(git_library_path / "diff_color.txt");
            diff_file << rl_tools::utils::extrack::git::rl_tools::diff_color;
            diff_file.close();
        }
        {
            std::ofstream diff_file(git_library_path / "word_diff.txt");
            diff_file << rl_tools::utils::extrack::git::rl_tools::word_diff;
            diff_file.close();
        }
        {
            std::ofstream diff_file(git_library_path / "word_diff_color.txt");
            diff_file << rl_tools::utils::extrack::git::rl_tools::word_diff_color;
            diff_file.close();
        }
        {
            std::ofstream diff_file(git_library_path / "diff_staged.txt");
            diff_file << rl_tools::utils::extrack::git::rl_tools::diff_staged;
            diff_file.close();
        }
        {
            std::ofstream diff_file(git_library_path / "diff_staged_color.txt");
            diff_file << rl_tools::utils::extrack::git::rl_tools::diff_staged_color;
            diff_file.close();
        }
        {
            std::ofstream diff_file(git_library_path / "word_diff_staged.txt");
            diff_file << rl_tools::utils::extrack::git::rl_tools::word_diff_staged;
            diff_file.close();
        }
        {
            std::ofstream diff_file(git_library_path / "word_diff_staged_color.txt");
            diff_file << rl_tools::utils::extrack::git::rl_tools::word_diff_staged_color;
            diff_file.close();
        }
        
        // Save parent project git info if available
        if(rl_tools::utils::extrack::git::project::available){
            std::filesystem::path git_project_path = paths.seed / "git" / "project";
            std::filesystem::create_directories(git_project_path);
            {
                std::ofstream diff_file(git_project_path / "commit.txt");
                diff_file << rl_tools::utils::extrack::git::project::commit;
                diff_file.close();
            }
            {
                std::ofstream diff_file(git_project_path / "diff.txt");
                diff_file << rl_tools::utils::extrack::git::project::diff;
                diff_file.close();
            }
            {
                std::ofstream diff_file(git_project_path / "diff_color.txt");
                diff_file << rl_tools::utils::extrack::git::project::diff_color;
                diff_file.close();
            }
            {
                std::ofstream diff_file(git_project_path / "word_diff.txt");
                diff_file << rl_tools::utils::extrack::git::project::word_diff;
                diff_file.close();
            }
            {
                std::ofstream diff_file(git_project_path / "word_diff_color.txt");
                diff_file << rl_tools::utils::extrack::git::project::word_diff_color;
                diff_file.close();
            }
            {
                std::ofstream diff_file(git_project_path / "diff_staged.txt");
                diff_file << rl_tools::utils::extrack::git::project::diff_staged;
                diff_file.close();
            }
            {
                std::ofstream diff_file(git_project_path / "diff_staged_color.txt");
                diff_file << rl_tools::utils::extrack::git::project::diff_staged_color;
                diff_file.close();
            }
            {
                std::ofstream diff_file(git_project_path / "word_diff_staged.txt");
                diff_file << rl_tools::utils::extrack::git::project::word_diff_staged;
                diff_file.close();
            }
            {
                std::ofstream diff_file(git_project_path / "word_diff_staged_color.txt");
                diff_file << rl_tools::utils::extrack::git::project::word_diff_staged_color;
                diff_file.close();
            }
        }
#endif

    }
    template <typename DEVICE, typename TI>
    std::filesystem::path get_step_folder(DEVICE& device, utils::extrack::Config<TI>& config, utils::extrack::Paths& paths, typename DEVICE::index_t step){
        std::stringstream step_ss;
        step_ss << std::setw(config.step_width) << std::setfill('0') << step;
        std::filesystem::path step_folder = paths.seed / "steps" / step_ss.str();
        std::filesystem::create_directories(step_folder);
        return step_folder;
    }
    template <typename DEVICE>
    void parse_setup(DEVICE& device, std::string setup, utils::extrack::Path& path){
        using TI = typename DEVICE::index_t;
        std::vector<std::string> parts;
        std::string part;
        std::stringstream ss(setup);
        while(std::getline(ss, part, '_')){
            parts.push_back(part);
        }
        utils::assert_exit(device, parts.size() >= 3, "Invalid setup: " + setup);
        path.commit = parts[0];
        path.name = parts[1];
        path.population_variates.clear();
        for(TI i = 2; i < parts.size(); i++){
            path.population_variates.push_back(parts[i]);
        }
    }
    template <typename DEVICE>
    void parse_config(DEVICE& device, std::string config, utils::extrack::Path& path){
        using TI = typename DEVICE::index_t;
        std::vector<std::string> parts;
        std::string part;
        std::stringstream ss(config);
        while(std::getline(ss, part, '_')){
            parts.push_back(part);
        }
        utils::assert_exit(device, parts.size() >= 1, "Invalid config: " + config);
        utils::assert_exit(device, parts.size() == path.population_variates.size(), "Population variates <-> values mismatch" + config);
        path.population_values.clear();
        path.attributes.clear();
        for(TI i = 0; i < parts.size(); i++){
            path.population_values.push_back(parts[i]);
            path.attributes[path.population_variates[i]] = parts[i];
        }
    }
    template <typename DEVICE>
    bool find_latest_run(DEVICE& device, std::filesystem::path experiments_path, utils::extrack::Path& p_query){
        std::vector<std::filesystem::directory_entry> experiments{std::filesystem::directory_iterator(experiments_path), std::filesystem::directory_iterator()};
        std::sort(experiments.begin(), experiments.end());
        std::vector<std::string> exclude_filenames = {"index.txt", "index_directories.txt", "index_files.txt"};
        auto in = [](const std::vector<std::string>& exclude_filenames, const std::filesystem::path& path){
            return std::find(exclude_filenames.begin(), exclude_filenames.end(), path.filename()) != exclude_filenames.end();
        };
        bool found = false;
        utils::extrack::Path query = p_query;
        utils::extrack::Path current_path;
        for (auto& experiment : experiments){
            if(!in(exclude_filenames, experiment.path().filename()) && experiment.is_directory() && (query.experiment.empty() || experiment.path().filename() == query.experiment)){
                current_path.experiment = experiment.path().filename().string();
                std::vector<std::filesystem::directory_entry> setups{std::filesystem::directory_iterator(experiment), std::filesystem::directory_iterator()};
                std::sort(setups.begin(), setups.end());
                for (auto& setup : setups){
                    if(setup.is_directory()){
                        parse_setup(device, setup.path().filename().string(), current_path);
                        if((query.commit.empty() || query.commit == current_path.commit) && (query.name.empty() || query.name == current_path.name)){
                            std::vector<std::filesystem::directory_entry> configs{std::filesystem::directory_iterator(setup), std::filesystem::directory_iterator()};
                            std::sort(configs.begin(), configs.end());
                            for (auto& config : configs){
                                parse_config(device, config.path().filename().string(), current_path);
                                bool matches_query = true;
                                for (const auto& [key, value] : query.attributes){
                                    auto it = current_path.attributes.find(key);
                                    if (it == current_path.attributes.end() || it->second != value) {
                                        matches_query = false;
                                        break;
                                    }
                                }
                                if(config.is_directory() && matches_query){
                                    std::vector<std::filesystem::directory_entry> seeds{std::filesystem::directory_iterator(config), std::filesystem::directory_iterator()};
                                    std::sort(seeds.begin(), seeds.end());
                                    for (auto& seed : seeds){
                                        if(seed.is_directory()){
                                            if(query.seed.empty() || std::stoull(query.seed) == std::stoull(seed.path().filename())){
                                                bool has_checkpoint = false;
                                                auto steps_path = seed.path() / "steps";
                                                if (std::filesystem::exists(steps_path) && std::filesystem::is_directory(steps_path)){
                                                    std::vector<std::filesystem::directory_entry> steps{std::filesystem::directory_iterator(steps_path), std::filesystem::directory_iterator()};
                                                    std::sort(steps.begin(), steps.end());
                                                    for (auto& step : steps){
                                                        if(step.is_directory()){
                                                            if(query.step.empty() || std::stoull(query.step) == std::stoull(step.path().filename())){
                                                                if(std::filesystem::exists(step.path() / "checkpoint.h5")){
                                                                    has_checkpoint = true;
                                                                    current_path.checkpoint_path = step.path() / "checkpoint.h5";
                                                                }
                                                            }
                                                        }
                                                    }
                                                }
                                                current_path.seed = seed.path().filename().string();
                                                current_path.path = seed.path();
                                                if(!query.require_checkpoint || has_checkpoint){
                                                    p_query = current_path;
                                                    found = true;
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        if(!found){
            std::cout << "Could not find run for query:" << std::endl;
            if(!query.experiment.empty()){
                std::cout << "\tExperiment: " << query.experiment << std::endl;
            }
            if(!query.commit.empty()){
                std::cout << "\tCommit: " << query.commit << std::endl;
            }
            if(!query.name.empty()){
                std::cout << "\tName: " << query.name << std::endl;
            }
            if (!query.attributes.empty()){
                std::cout << "\tAttributes: " << std::endl;
                for (const auto& [key, value] : query.attributes){
                    std::cout << "\t\t" << key << ": " << value << std::endl;
                }
            }
            if(!query.seed.empty()){
                std::cout << "\tSeed: " << query.seed << std::endl;
            }
            if(!query.step.empty()){
                std::cout << "\tStep: " << query.step << std::endl;
            }
            if(query.require_checkpoint){
                std::cout << "\tRequire checkpoint: true" << std::endl;
            }
        }
        return found;
    }
    template <typename DEVICE>
    bool find_latest_checkpoint(DEVICE& device, utils::extrack::Path& p_query){
        utils::extrack::Path query = p_query;
        std::vector<std::filesystem::directory_entry> steps{std::filesystem::directory_iterator(query.path / "steps"), std::filesystem::directory_iterator()};
        std::sort(steps.begin(), steps.end());
        bool found = false;
        utils::extrack::Path current_path = query;
        for (auto& step : steps){
            if(step.is_directory()){
                if(std::filesystem::exists(step.path() / "checkpoint.h5")){
                    current_path.step = step.path().filename();
                    current_path.checkpoint_path = step.path() / "checkpoint.h5";
                    p_query = current_path;
                    found = true;
                }
            }
        }
        return found;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
