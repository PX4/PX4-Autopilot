#include <rl_tools/rl_tools.h>

#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_UTILS_EXTRACK_EXTRACK_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_UTILS_EXTRACK_EXTRACK_H

#include <iostream>
#include <chrono>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <filesystem>
#include <map>
#include <vector>

#include <cstdlib>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    namespace utils::extrack{
#ifdef RL_TOOLS_EXTRACK_GIT_DIFF
        namespace git{
            // rl-tools library repository
            namespace rl_tools{
                extern const char* const commit;
                extern const char* const diff;
                extern const char* const diff_color;
                extern const char* const word_diff;
                extern const char* const word_diff_color;
                extern const char* const diff_staged;
                extern const char* const diff_staged_color;
                extern const char* const word_diff_staged;
                extern const char* const word_diff_staged_color;
            }
            // Parent project repository (if used via add_subdirectory)
            namespace project{
                extern const bool available; // true if parent project exists
                extern const char* const commit;
                extern const char* const diff;
                extern const char* const diff_color;
                extern const char* const word_diff;
                extern const char* const word_diff_color;
                extern const char* const diff_staged;
                extern const char* const diff_staged_color;
                extern const char* const word_diff_staged;
                extern const char* const word_diff_staged_color;
            }
        }
#endif

        // How does this work:
        /*
        // Example training setup (using the Loop step):
            rlt::malloc(device, ts);
            ts.extrack_config.name = "foundation-policy-pre-training";
            ts.extrack_config.population_variates = "motor-mapping_thrust-curves_rng-warmup";
            ts.extrack_config.population_values = std::string(OPTIONS::RANDOMIZE_MOTOR_MAPPING ? "true" : "false") + "_" + (OPTIONS::RANDOMIZE_THRUST_CURVES ? "true" : "false") + "_" + std::to_string(FACTORY::RNG_PARAMS_WARMUP_STEPS);
            rlt::init(device, ts, seed);

        // Example query:
            rlt::utils::extrack::Path checkpoint_path;
            checkpoint_path.experiment = "2025-02-13_11-30-07";
            checkpoint_path.name = "foundation-policy-pre-training";
            rlt::find_latest_run(device, "experiments", checkpoint_path);
            if (!rlt::find_latest_checkpoint(device, checkpoint_path)){
                std::cerr << "No checkpoint found for " << checkpoint_path.experiment << std::endl;
                return 1;
            }

         */
        template <typename TI>
        struct Config{
            std::filesystem::path base_path = "experiments";
            std::string experiment;
            std::string name = "default";
            std::string population_variates = "default";
            std::string population_values = "default";
            TI step_width = 15;
        };
        struct Paths{
            std::filesystem::path experiment;
            std::filesystem::path setup;
            std::filesystem::path config;
            std::filesystem::path seed;
        };
        struct Path{
            std::filesystem::path path;
            std::string experiment;
            std::string commit;
            std::string name;
            std::vector<std::string> population_variates;
            std::vector<std::string> population_values;
            std::map<std::string, std::string> attributes;
            std::string seed;
            std::string step;
            bool require_checkpoint = false;
            std::filesystem::path checkpoint_path;
        };
        template <typename DUMMY = bool>
        std::string get_timestamp_string(){
            // equivalent to date '+%Y-%m-%d_%H-%M-%S'
            auto now = std::chrono::system_clock::now();
            std::time_t now_c = std::chrono::system_clock::to_time_t(now);
            std::tm now_local;

#if defined(_WIN32) || defined(_WIN64)
            localtime_s(&now_local, &now_c);
#else
            localtime_r(&now_c, &now_local);
#endif
            std::stringstream ss;
            ss << std::put_time(&now_local, "%Y-%m-%d_%H-%M-%S");

            return ss.str();
        }
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif

