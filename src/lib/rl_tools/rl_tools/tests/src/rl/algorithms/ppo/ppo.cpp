#include <rl_tools/operations/cpu_mux.h>
#include <rl_tools/nn/optimizers/adam/instance/operations_generic.h>
#include <rl_tools/nn/operations_cpu_mux.h>
#include <rl_tools/nn_models/operations_cpu.h>
namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools;
#include "parameters_rl.h"
#include <rl_tools/nn/optimizers/adam/operations_generic.h>
#include <rl_tools/rl/components/on_policy_runner/operations_generic.h>
#include <rl_tools/rl/algorithms/ppo/operations_generic.h>

#include <gtest/gtest.h>

namespace parameters = parameters_0;

using LOGGER = rlt::devices::logging::CPU_TENSORBOARD<>;
using DEV_SPEC = rlt::devices::cpu::Specification<rlt::devices::math::CPU, rlt::devices::random::CPU, LOGGER>;

using DEVICE = rlt::devices::DEVICE_FACTORY<DEV_SPEC>;
using T = float;
using TYPE_POLICY = rlt::numeric_types::Policy<T>;
using TI = typename DEVICE::index_t;




TEST(RL_TOOLS_RL_ALGORITHMS_PPO, TEST){
    using penv = parameters::environment<T, TI>;
    using prl = parameters::rl<TYPE_POLICY, TI, penv::ENVIRONMENT>;

    DEVICE::SPEC::LOGGING logger;
    DEVICE device;
    prl::OPTIMIZER actor_optimizer, critic_optimizer;
    {
        static constexpr T ALPHA = 0.001;
    };
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 10);

    prl::PPO_TYPE ppo;
    prl::PPO_BUFFERS_TYPE ppo_buffers;
    prl::ON_POLICY_RUNNER_TYPE on_policy_runner;
    prl::ON_POLICY_RUNNER_DATASET_TYPE on_policy_runner_dataset;
    prl::ACTOR_EVAL_BUFFERS actor_eval_buffers;
    prl::ACTOR_BUFFERS actor_buffers;
    prl::CRITIC_BUFFERS critic_buffers;
    prl::CRITIC_BUFFERS_ALL critic_buffers_all;

    rlt::malloc(device, actor_optimizer);
    rlt::malloc(device, critic_optimizer);
    rlt::malloc(device, ppo);
    rlt::malloc(device, ppo_buffers);
    rlt::malloc(device, on_policy_runner_dataset);
    rlt::malloc(device, on_policy_runner);
    rlt::malloc(device, actor_eval_buffers);
    rlt::malloc(device, actor_buffers);
    rlt::malloc(device, critic_buffers);
    rlt::malloc(device, critic_buffers_all);

    penv::ENVIRONMENT envs[prl::N_ENVIRONMENTS];
    penv::ENVIRONMENT::Parameters env_parameters[prl::N_ENVIRONMENTS];
    rlt::init(device, on_policy_runner, envs, env_parameters, rng);
    rlt::init(device, ppo, actor_optimizer, critic_optimizer, rng);
    rlt::construct(device, device.logger);
    auto training_start = std::chrono::high_resolution_clock::now();
    for(TI ppo_step_i = 0; ppo_step_i < 1000; ppo_step_i++) {
        rlt::set_step(device, device.logger, on_policy_runner.step);

        if(ppo_step_i % 100 == 0){
            std::chrono::duration<T> training_elapsed = std::chrono::high_resolution_clock::now() - training_start;
            std::cout << "PPO step: " << ppo_step_i << " elapsed: " << training_elapsed.count() << "s" << std::endl;
            rlt::add_scalar(device, device.logger, "ppo/step", ppo_step_i);
        }
        for (TI action_i = 0; action_i < penv::ENVIRONMENT::ACTION_DIM; action_i++) {
            auto& last_layer = rlt::get_last_layer(ppo.actor);
            T action_log_std = rlt::get(device, last_layer.log_std.parameters, action_i);
            std::stringstream topic;
            topic << "actor/action_std/" << action_i;
            rlt::add_scalar(device, device.logger, topic.str(), rlt::math::exp(DEVICE::SPEC::MATH(), action_log_std));
        }
        auto start = std::chrono::high_resolution_clock::now();
        {
            auto start = std::chrono::high_resolution_clock::now();
            rlt::collect(device, on_policy_runner_dataset, on_policy_runner, ppo.actor, actor_eval_buffers, rng);
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<T> elapsed = end - start;
//            std::cout << "Rollout: " << elapsed.count() << " s" << std::endl;
        }
        {
            auto start = std::chrono::high_resolution_clock::now();
            evaluate(device, ppo.critic, on_policy_runner_dataset.all_observations_privileged, on_policy_runner_dataset.all_values, critic_buffers_all, rng);
            rlt::estimate_generalized_advantages(device, on_policy_runner_dataset, prl::PPO_SPEC::PARAMETERS{});
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<T> elapsed = end - start;
//            std::cout << "GAE: " << elapsed.count() << " s" << std::endl;
        }
        {
            auto start = std::chrono::high_resolution_clock::now();
            rlt::train(device, ppo, on_policy_runner_dataset, actor_optimizer, critic_optimizer, ppo_buffers, actor_buffers, critic_buffers, rng);
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<T> elapsed = end - start;
//            std::cout << "Train: " << elapsed.count() << " s" << std::endl;
        }
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<T> elapsed = end - start;
//        std::cout << "Total: " << elapsed.count() << " s" << std::endl;
#ifdef RL_TOOLS_TESTS_CODE_COVERAGE
        std::cout << "step: " << ppo_step_i << std::endl;
        if (ppo_step_i >= 2){
            break;
        }
#endif
    }

    rlt::free(device, ppo);
    rlt::free(device, ppo_buffers);
    rlt::free(device, on_policy_runner_dataset);
    rlt::free(device, on_policy_runner);
    rlt::free(device, actor_eval_buffers);
    rlt::free(device, actor_buffers);
    rlt::free(device, critic_buffers);
    rlt::free(device, critic_buffers_all);

}