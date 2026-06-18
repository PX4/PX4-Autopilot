#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_UTILS_VALIDATION_ANALYSIS_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_UTILS_VALIDATION_ANALYSIS_H

#include <string>
#include <vector>
#include <map>
#include "../../utils/generic/typing.h"
#ifdef RL_TOOLS_ENABLE_JSON
#include <nlohmann/json.hpp>
#endif
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    namespace rl::utils::validation{
        template <typename DEVICE, typename SPEC, typename CONTENT, typename NEXT_COMPONENT>
        void analyse_step_log(DEVICE& device, Task<SPEC>& task, rl::utils::validation::set::Component<CONTENT, NEXT_COMPONENT>){
            using COMPONENT = rl::utils::validation::set::Component<CONTENT, NEXT_COMPONENT>;
            using T = typename SPEC::T;
            std::string n = name(CONTENT{});
            T value = evaluate(device, CONTENT{}, task);
            add_scalar(device, device.logger, std::string("validation/") + n, value);
            if constexpr (!rl_tools::utils::typing::is_same_v<NEXT_COMPONENT, rl::utils::validation::set::FinalComponent>){
                analyse_step_log(device, task, typename COMPONENT::NEXT_COMPONENT{});
            }
        }
        template <typename DEVICE, typename SPEC, typename CONTENT, typename NEXT_COMPONENT>
        void analyse_step_string(DEVICE& device, Task<SPEC>& task, std::vector<std::string>& metric_names, std::vector<typename SPEC::T>& metric_values, rl::utils::validation::set::Component<CONTENT, NEXT_COMPONENT>){
            using COMPONENT = rl::utils::validation::set::Component<CONTENT, NEXT_COMPONENT>;
            using T = typename SPEC::T;
            std::string n = name(CONTENT{});
            T value = evaluate(device, CONTENT{}, task);
            add_scalar(device, device.logger, std::string("validation/") + n, value);
            metric_names.push_back(n);
            metric_values.push_back(value);
            if constexpr (!rl_tools::utils::typing::is_same_v<NEXT_COMPONENT, rl::utils::validation::set::FinalComponent>){
                analyse_step_string(device, task, metric_names, metric_values, typename COMPONENT::NEXT_COMPONENT{});
            }
        }
    }
    template <typename DEVICE, typename SPEC, typename METRICS>
    void analyse_stdout(DEVICE& device, rl::utils::validation::Task<SPEC>& task, METRICS){
        using TI = typename SPEC::TI;
        std::vector<std::string> metric_names;
        std::vector<typename SPEC::T> metric_values;
        analyse_step_string(device, task, metric_names, metric_values, METRICS{});
        for(typename SPEC::TI metric_i = 0; metric_i < metric_names.size(); metric_i++){
            std::string name = metric_names[metric_i];
            constexpr TI n = 40;
            std::string padded_name = name + std::string(n > name.length() ? n - name.length() : 0, ' ');
            std::cout << padded_name << ": " << metric_values[metric_i] << std::endl;
        }
    }
    template <typename DEVICE, typename SPEC, typename METRICS>
    void analyse_log(DEVICE& device, rl::utils::validation::Task<SPEC>& task, METRICS){
        using TI = typename SPEC::TI;
        analyse_step_log(device, task, METRICS{});
    }
    template <typename DEVICE, typename SPEC, typename METRICS>
    std::map<std::string, typename SPEC::T> analyse_dict(DEVICE& device, rl::utils::validation::Task<SPEC>& task, METRICS){
        using TI = typename SPEC::TI;
        std::vector<std::string> metric_names;
        std::vector<typename SPEC::T> metric_values;
        analyse_step_string(device, task, metric_names, metric_values, METRICS{});
        std::map<std::string, typename SPEC::T> metrics;
        for(typename SPEC::TI metric_i = 0; metric_i < metric_names.size(); metric_i++){
            std::string name = metric_names[metric_i];
            metrics[name] = metric_values[metric_i];
        }
        return metrics;
    }
#ifdef RL_TOOLS_ENABLE_JSON
    template <typename DEVICE, typename SPEC, typename METRICS>
    nlohmann::json analyse_json(DEVICE& device, rl::utils::validation::Task<SPEC>& task, METRICS){
        auto metrics = analyse_dict(device, task, METRICS{});
        std::map<std::string, double> metrics_double;
        for(auto& [name, value]: metrics){
            metrics_double[name] = value;
        }
        return metrics_double;
    }
#endif
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif