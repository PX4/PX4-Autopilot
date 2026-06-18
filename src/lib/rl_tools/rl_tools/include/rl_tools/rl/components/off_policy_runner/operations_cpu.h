#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_COMPONENTS_OFF_POLICY_RUNNER_OPERATIONS_CPU_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_COMPONENTS_OFF_POLICY_RUNNER_OPERATIONS_CPU_H

#include <thread>

#include "operations_generic_per_env.h"
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::components::off_policy_runner{
    constexpr auto get_num_threads(devices::ExecutionHints hints) {
        return 1;
    }
    template<typename TI, TI NUM_THREADS>
    constexpr TI get_num_threads(rl::components::off_policy_runner::ExecutionHints<TI, NUM_THREADS> hints) {
        return NUM_THREADS;
    }

    template<typename DEV_SPEC, typename SPEC, typename RNG>
    void prologue(devices::CPU<DEV_SPEC>& device, rl::components::OffPolicyRunner<SPEC>& runner, RNG &rng) {
        using DEVICE = devices::CPU<DEV_SPEC>;
        using TI = typename DEVICE::index_t;
        std::vector<std::thread> threads;

        constexpr TI NUM_THREADS = get_num_threads(typename DEVICE::EXECUTION_HINTS());

        if(NUM_THREADS > 1){
            RNG rngs[SPEC::PARAMETERS::N_ENVIRONMENTS];
            auto base = random::uniform_int_distribution(typename DEV_SPEC::RANDOM(), 0, 1000000, rng);
            for (TI env_i = 0; env_i < SPEC::PARAMETERS::N_ENVIRONMENTS; env_i++) {
                init(device, rngs[env_i], base + env_i);
            }

            for (TI thread_i = 0; thread_i < NUM_THREADS; thread_i++) {
                threads.emplace_back([NUM_THREADS, &device, thread_i, &runner, &rngs](){
                    for (TI env_i = thread_i; env_i < SPEC::PARAMETERS::N_ENVIRONMENTS; env_i += NUM_THREADS) {
                        prologue_per_env(device, runner, rngs[env_i], env_i);
                    }
                });
            }

            for (auto& thread : threads) {
                thread.join();
            }
        }
        else{
            for (TI env_i = 0; env_i < SPEC::PARAMETERS::N_ENVIRONMENTS; env_i++) {
                prologue_per_env(device, runner, rng, env_i);
            }
        }

    }

    template<typename DEV_SPEC, typename SPEC, typename POLICY, typename RNG>
    void epilogue(devices::CPU<DEV_SPEC>& device, rl::components::OffPolicyRunner<SPEC>& runner, const POLICY& policy, RNG& rng){
        using DEVICE = devices::CPU<DEV_SPEC>;
        using TI = typename DEVICE::index_t;
        using OPR = rl::components::OffPolicyRunner<SPEC>;
        std::vector<std::thread> threads;

        constexpr TI NUM_THREADS = get_num_threads(typename DEVICE::EXECUTION_HINTS());

        if(NUM_THREADS > 1){
            RNG rngs[OPR::N_ENVIRONMENTS];
            auto base = random::uniform_int_distribution(typename DEV_SPEC::RANDOM(), 0, 1000000, rng);
            for (TI env_i = 0; env_i < OPR::N_ENVIRONMENTS; env_i++) {
                init(device, rngs[env_i], base + env_i);
            }

            for (TI thread_i = 0; thread_i < NUM_THREADS; thread_i++) {
                threads.emplace_back([NUM_THREADS, &device, thread_i, &runner, &policy, &rngs](){
                    for (TI env_i = thread_i; env_i < OPR::N_ENVIRONMENTS; env_i += NUM_THREADS) {
                        epilogue_per_env(device, runner, policy, rngs[env_i], env_i);
                    }
                });
            }

            for (auto& thread : threads) {
                thread.join();
            }
        }
        else{
            for (TI env_i = 0; env_i < OPR::N_ENVIRONMENTS; env_i++) {
                epilogue_per_env(device, runner, policy, rng, env_i);
            }
        }

    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#ifndef RL_TOOLS_RL_COMPONENTS_OFF_POLICY_RUNNER_OPERATIONS_CPU_DELAY_OPERATIONS_GENERIC_INCLUDE
#include "operations_generic.h"
#endif

#endif
