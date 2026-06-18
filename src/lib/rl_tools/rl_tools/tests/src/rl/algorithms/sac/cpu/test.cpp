#include "../../../../../../src/rl/environments/pendulum/sac/cpu/training.h"

#include <gtest/gtest.h>

#include <vector>
#include <chrono>


TEST(RL_TOOLS_RL_ALGORITHMS_SAC, FULL_TRAINING){
    static constexpr TI NUM_EVALUATION_EPISODES = 1000;
    DEVICE device;
    LOOP_STATE ts;
    rlt::malloc(device, ts);
    rlt::init(device, ts, 0);
    using T = float;
    std::vector<T> evaluation_returns;
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time = std::chrono::high_resolution_clock::now();
    while(!rlt::step(device, ts)){
        if(ts.step % 1000 == 0){
            rlt::rl::utils::evaluation::Result<typename decltype(ts)::CONFIG::EVALUATION_RESULT_SPEC> result;
            rlt::evaluate(device, ts.env_eval, ts.ui, rlt::get_actor(ts), result, ts.rng_eval, ts.evaluation_mode);
            evaluation_returns.push_back(result.returns_mean);
            std::sort(evaluation_returns.begin(), evaluation_returns.end());
            std::cout << "perc60: " << evaluation_returns[evaluation_returns.size()*0.6] << std::endl;
            for(auto r: evaluation_returns){
                std::cout << r << " ";
            }
            std::cout << std::endl;
        }
#ifdef RL_TOOLS_TESTS_CODE_COVERAGE
        std::cout << "step: " << ts.step << std::endl;
        if (ts.step >= LOOP_CORE_PARAMETERS::N_WARMUP_STEPS*2){
            break;
        }
#endif
    }
    std::chrono::time_point<std::chrono::high_resolution_clock> end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<T> elapsed_seconds = end_time - start_time;
    std::cout << "Elapsed time: " << elapsed_seconds.count() << "s\n";
#ifndef RL_TOOLS_TESTS_CODE_COVERAGE
    T perc_80 = evaluation_returns[evaluation_returns.size()*0.8];
    EXPECT_GT(perc_80, -200);
#endif
    // sort returns
    rlt::free(device, ts);
}

