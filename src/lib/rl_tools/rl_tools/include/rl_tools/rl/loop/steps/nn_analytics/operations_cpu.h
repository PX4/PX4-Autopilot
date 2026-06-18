#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_LOOP_STEPS_NN_ANALYTICS_OPERATIONS_CPU_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_LOOP_STEPS_NN_ANALYTICS_OPERATIONS_CPU_H

#include "../../../../rl/algorithms/sac/operations_generic.h"
#include "../../../../rl/components/off_policy_runner/operations_generic.h"

#include "../../../../rl/environments/operations_generic.h"


#include "../../../../rl/utils/evaluation/operations_generic.h"

#include "../../../../utils/zlib/operations_cpu.h"

#include "config.h"
#include <string>
#include <sstream>
#include <fstream>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template <typename DEVICE, typename T_CONFIG>
    void malloc(DEVICE& device, rl::loop::steps::nn_analytics::State<T_CONFIG>& ts){
        using STATE = rl::loop::steps::nn_analytics::State<T_CONFIG>;
        malloc(device, static_cast<typename STATE::NEXT&>(ts));
    }
    template <typename DEVICE, typename T_CONFIG>
    void init(DEVICE& device, rl::loop::steps::nn_analytics::State<T_CONFIG>& ts, typename T_CONFIG::TI seed = 0){
        using STATE = rl::loop::steps::nn_analytics::State<T_CONFIG>;
        init(device, static_cast<typename STATE::NEXT&>(ts), seed);
    }

    template <typename DEVICE, typename T_CONFIG>
    void free(DEVICE& device, rl::loop::steps::nn_analytics::State<T_CONFIG>& ts){
        using STATE = rl::loop::steps::nn_analytics::State<T_CONFIG>;
        free(device, static_cast<typename STATE::NEXT&>(ts));
    }

    namespace rl::loop::steps::nn_analytics{
        template <auto INDEX, typename DEVICE, typename TS>
        std::string accumulate_nns(DEVICE& device, TS& ts){
            using TI = typename DEVICE::index_t;
            constexpr TI N = TS::CONFIG::NUM_NNS;
            static_assert(N >= 1);
            std::string data;
            if(INDEX == 0){
                data += "{";
            }
            if(INDEX < N){
                auto& nn = get_nn<INDEX>(device, ts);
                std::string name = get_nn_name<INDEX>(device, ts);
                data += "\"" + name + "\": " + rl_tools::nn_analytics(device, nn);
            }
            if constexpr(INDEX < N - 1){
                data += ",";
                data += accumulate_nns<INDEX + 1>(device, ts);
            }
            if (INDEX == 0){
                data += "}";
            }
            return data;
        }
    }

    template <typename DEVICE, typename CONFIG>
    bool step(DEVICE& device, rl::loop::steps::nn_analytics::State<CONFIG>& ts){
        using TS = rl::loop::steps::nn_analytics::State<CONFIG>;
        using TI = typename CONFIG::TI;
        using PARAMETERS = typename CONFIG::NN_ANALYTICS_PARAMETERS;
        using STATE = rl::loop::steps::nn_analytics::State<CONFIG>;
        if(ts.step % PARAMETERS::INTERVAL == 0 && (ts.step == 0 || ts.step >= PARAMETERS::WARMUP_STEPS)){
            const char * env_var = environment_variable(device, "RL_TOOLS_NN_ANALYTICS");
            if(env_var != nullptr && std::string(env_var) != "1"){
                std::cout << "RL_TOOLS_NN_ANALYTICS=" << env_var << " is set. Skipping NN Analytics step." << std::endl;
            }
            else
            {
                std::stringstream step_ss;
                step_ss << std::setw(15) << std::setfill('0') << ts.step;
                std::filesystem::path step_path = ts.extrack_paths.seed / "steps" / step_ss.str();
                std::filesystem::create_directories(step_path);
                auto data = rl::loop::steps::nn_analytics::accumulate_nns<0>(device, ts);
#ifndef RL_TOOLS_ENABLE_ZLIB
                std::string file_extension = "json";
                std::string data_output = data;
#else
                std::string file_extension = "json.gz";
                std::vector<uint8_t> data_output;
                if(!compress_zlib(data, data_output)){
                    std::cerr << "Error while compressing trajectories." << std::endl;
                    return true;
                }
#endif
                std::ofstream file(step_path / ("nn_analytics." + file_extension), std::ios::binary);
                file.write(reinterpret_cast<const char*>(data_output.data()), data_output.size());
                file.close();
            }
        }
        bool finished = step(device, static_cast<typename STATE::NEXT&>(ts));
        return finished;
    }
    // to log the configuration
    template <typename DEVICE, typename PARAMETERS, typename utils::typing::enable_if<utils::typing::is_same_v<typename PARAMETERS::TAG, rl::loop::steps::nn_analytics::ParametersTag>>::type* = nullptr>
    void log(DEVICE& device, PARAMETERS){}
    template <typename DEVICE, typename CONFIG, typename utils::typing::enable_if<utils::typing::is_same_v<typename CONFIG::TAG, rl::loop::steps::nn_analytics::ConfigTag>>::type* = nullptr>
    void log(DEVICE& device, CONFIG){
        log(device, typename CONFIG::NEXT{});
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END


#endif
