//#define RL_TOOLS_BACKEND_DISABLE_BLAS
#ifdef RL_TOOLS_STATIC_MEM
#define RL_TOOLS_DISABLE_DYNAMIC_MEMORY_ALLOCATIONS
#endif

#include <rl_tools/operations/cpu_mux.h>

#include <rl_tools/rl/environments/pendulum/operations_generic.h>
#include <rl_tools/rl/environment_wrappers/scale_observations/operations_generic.h>

#include <rl_tools/nn/optimizers/adam/instance/operations_generic.h>
#include <rl_tools/nn/operations_cpu_mux.h>
#include <rl_tools/nn/layers/standardize/operations_generic.h>
#include <rl_tools/nn_models/mlp_unconditional_stddev/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>
#include <rl_tools/nn/optimizers/adam/operations_generic.h>



#include <rl_tools/rl/algorithms/ppo/loop/core/config.h>
#include <rl_tools/rl/loop/steps/evaluation/config.h>
#include <rl_tools/rl/loop/steps/timing/config.h>
#include <rl_tools/rl/algorithms/ppo/loop/core/operations_generic.h>
#include <rl_tools/rl/loop/steps/evaluation/operations_generic.h>
#include <rl_tools/rl/loop/steps/timing/operations_cpu.h>


#include <utility> // for std::declval (which is needed to prevent MSVC from erroring out on too deeply nested struct init)

#ifdef RL_TOOLS_ENABLE_JSON
#include <nlohmann/json.hpp>
#include <fstream>
#endif

namespace rlt = rl_tools;

#include "config.h"

using DEVICE = rlt::devices::DEVICE_FACTORY<>;
using T = float;
using TYPE_POLICY = rlt::numeric_types::Policy<float>;
using TI = typename DEVICE::index_t;
#ifdef RL_TOOLS_STATIC_MEM
static constexpr bool DYNAMIC_ALLOCATION = false;
#else
static constexpr bool DYNAMIC_ALLOCATION = true;
#endif

using CONFIG = CONFIG_FACTORY<DEVICE, TYPE_POLICY, DYNAMIC_ALLOCATION>;

using LOOP_CONFIG = CONFIG::LOOP_TIMING_CONFIG;
using LOOP_STATE = typename LOOP_CONFIG::template State<LOOP_CONFIG>;

static constexpr TI NUM_EPISODES_FINAL_EVAL = 1000;
using EVAL_SPEC = rlt::rl::utils::evaluation::Specification<TYPE_POLICY, TI, typename LOOP_CONFIG::ENVIRONMENT_EVALUATION, NUM_EPISODES_FINAL_EVAL, CONFIG::ENVIRONMENT::EPISODE_STEP_LIMIT>;

using POLICY = rlt::utils::typing::remove_reference_t<decltype(rlt::get_actor(std::declval<LOOP_STATE>()))>;
using EVAL_BUFFER = rlt::rl::utils::evaluation::PolicyBuffer<rlt::rl::utils::evaluation::PolicyBufferSpecification<EVAL_SPEC, POLICY, DYNAMIC_ALLOCATION>>;

EVAL_BUFFER eval_buffer;

auto run(TI seed, bool verbose){
    DEVICE device;
    if(verbose){
        rlt::log(device, CONFIG::LOOP_TIMING_CONFIG{});
    }
    LOOP_STATE ts;
    rlt::malloc(device, ts);
    rlt::malloc(device, eval_buffer);
    rlt::init(device, ts, seed);
    while(!rlt::step(device, ts)){
    }
    rlt::log(device, device.logger, "PPO steps: ", ts.step);
    rlt::rl::utils::evaluation::Result<EVAL_SPEC> result;
    auto actor = rlt::get_actor(ts);
    evaluate(device, ts.envs[0], ts.ui, actor, eval_buffer, result, ts.rng, rlt::Mode<rlt::mode::Evaluation<>>{});
    rlt::log(device, device.logger, "Final return: ", result.returns_mean);
    rlt::log(device, device.logger, "              mean: ", result.returns_mean);
    rlt::log(device, device.logger, "              std : ", result.returns_std);
    rlt::free(device, ts);
    rlt::free(device, eval_buffer);
    return result;
}


int main(int argc, char** argv) {
#ifdef RL_TOOLS_STATIC_MEM
    std::cout << "Training state size: " << sizeof(LOOP_STATE) << std::endl;
#endif
    bool verbose = true;
    std::vector<decltype(run(0, verbose))> returns;
    for (TI seed=0; seed < 1; seed++){
        auto return_stats = run(seed, verbose);
        returns.push_back(return_stats);
    }
    T sum = 0;
    T sum_squared = 0;
    for(auto& return_stats: returns){
        sum += return_stats.returns_mean;
        sum_squared += return_stats.returns_mean * return_stats.returns_mean;
    }
    T mean = sum / returns.size();
    T std = std::sqrt(sum_squared / returns.size() - mean * mean);
    // median
    std::sort(returns.begin(), returns.end(), [](auto& a, auto& b){
        return a.returns_mean < b.returns_mean;
    });
    T median = returns[returns.size() / 2].returns_mean;
    std::cout << "Mean return: " << mean << std::endl;
    std::cout << "Std return: " << (returns.size() > 1 ? std::to_string(std) : "-")  << std::endl;
    std::cout << "Median return: " << median << std::endl;
#ifdef RL_TOOLS_ENABLE_JSON
    nlohmann::json j;
    for(auto& return_stats: returns){
        j.push_back(return_stats.returns);
    }
    std::ofstream file("pendulum_ppo_returns.json");
    file << j.dump(4);
#endif
    return 0;
}

// Should take ~ 0.3s on M3 Pro in BECHMARK mode
// - tested @ 1118e19f904a26a9619fac7b1680643a0afcb695)
// - tested @ 361c2f5e9b14d52ee497139a3b82867fce0404a7