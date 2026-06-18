#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_COMPONENTS_ON_POLICY_RUNNER_OPERATIONS_GENERIC_PER_ENV_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_COMPONENTS_ON_POLICY_RUNNER_OPERATIONS_GENERIC_PER_ENV_H

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::components::on_policy_runner::per_env{
    template <typename DEVICE, typename OBSERVATIONS_PRIVILEGED_SPEC, typename OBSERVATIONS_SPEC, typename SPEC, typename RNG> // todo: make this not PPO but general policy with output distribution
    RL_TOOLS_FUNCTION_PLACEMENT void prologue(DEVICE& device, Matrix<OBSERVATIONS_PRIVILEGED_SPEC>& observations_privileged, Matrix<OBSERVATIONS_SPEC>& observations, rl::components::OnPolicyRunner<SPEC>& runner, RNG& rng, typename DEVICE::index_t env_i){
        static_assert(OBSERVATIONS_SPEC::ROWS == SPEC::N_ENVIRONMENTS);
        static_assert(OBSERVATIONS_SPEC::COLS == SPEC::ENVIRONMENT::Observation::DIM);
        static_assert(OBSERVATIONS_PRIVILEGED_SPEC::ROWS == SPEC::N_ENVIRONMENTS);
        static_assert(OBSERVATIONS_PRIVILEGED_SPEC::COLS == SPEC::ENVIRONMENT::ObservationPrivileged::DIM);
        auto& env = get(runner.environments, 0, env_i);
        auto& state = get(runner.states, 0, env_i);
        auto& parameters = get(runner.env_parameters, 0, env_i);
        if(get(runner.truncated, 0, env_i)){
            add_scalar(device, device.logger, "episode/length", get(runner.episode_step, 0, env_i));
            add_scalar(device, device.logger, "episode/return", get(runner.episode_return, 0, env_i));
            set(runner.truncated, 0, env_i, false);
            set(runner.episode_step, 0, env_i, 0);
            set(runner.episode_return, 0, env_i, 0);
            sample_initial_parameters(device, env, parameters, rng);
            sample_initial_state(device, env, parameters, state, rng);
        }
        auto observation = row(device, observations, env_i);
        observe(device, env, parameters, state, typename SPEC::ENVIRONMENT::Observation{}, observation, rng);
        if(SPEC::ASYMMETRIC_OBSERVATIONS){
            auto observation_privileged = row(device, observations_privileged, env_i);
            observe(device, env, parameters, state, typename SPEC::ENVIRONMENT::ObservationPrivileged{}, observation_privileged, rng);
        }
    }
    template <typename DEVICE, typename DATASET_SPEC, typename ACTIONS_MEAN_SPEC, typename ACTIONS_SPEC, typename ACTION_LOG_STD_SPEC, typename RNG> // todo: make this not PPO but general policy with output distribution
    RL_TOOLS_FUNCTION_PLACEMENT void epilogue(DEVICE& device, rl::components::on_policy_runner::Dataset<DATASET_SPEC>& dataset, rl::components::OnPolicyRunner<typename DATASET_SPEC::SPEC>& runner, Matrix<ACTIONS_MEAN_SPEC>& actions_mean, Matrix<ACTIONS_SPEC>& actions, Matrix<ACTION_LOG_STD_SPEC>& action_log_std, RNG& rng, typename DEVICE::index_t pos, typename DEVICE::index_t env_i){
        using SPEC = typename DATASET_SPEC::SPEC;
        using T = typename SPEC::TYPE_POLICY::DEFAULT;
        using TI = typename SPEC::TI;
        constexpr TI N_AGENTS = SPEC::ENVIRONMENT::N_AGENTS;
        constexpr TI ACTION_DIM = SPEC::ENVIRONMENT::ACTION_DIM;
        static_assert(ACTION_DIM % N_AGENTS == 0);
        constexpr TI PER_AGENT_ACTION_DIM = SPEC::ENVIRONMENT::ACTION_DIM/N_AGENTS;

        T action_log_prob = 0;
        for(TI action_i = 0; action_i < SPEC::ENVIRONMENT::ACTION_DIM; action_i++) {
            T action_mean = get(actions_mean, env_i, action_i);
//                    std::stringstream topic;
//                    topic << "action/" << action_i;
//                    add_scalar(device, device.logger, topic.str(), action_mu);

            static_assert(ACTION_DIM == ACTION_LOG_STD_SPEC::COLS * N_AGENTS);
            static_assert(ACTION_LOG_STD_SPEC::ROWS == 1);
            T current_action_log_std = get(action_log_std, 0, action_i % PER_AGENT_ACTION_DIM);
            T action_std = math::exp(device.math, current_action_log_std);
            T action_noisy = random::normal_distribution::sample(device.random, action_mean, action_std, rng);
            action_log_prob += random::normal_distribution::log_prob(device.random, action_mean, current_action_log_std, action_noisy);
            set(actions, env_i, action_i, action_noisy);
        }
        set(dataset.action_log_probs, pos, 0, action_log_prob);
        auto& env = get(runner.environments, 0, env_i);
        auto& state = get(runner.states, 0, env_i);
        auto& parameters = get(runner.env_parameters, 0, env_i);
        typename SPEC::ENVIRONMENT::State next_state;
        auto action = row(device, actions, env_i);
        step(device, env, parameters, state, action, next_state, rng);
        bool terminated_flag = terminated(device, env, parameters, next_state, rng);
        set(dataset.terminated, pos, 0, terminated_flag);
        T reward_value = reward(device, env, parameters, state, action, next_state, rng);
        increment(runner.episode_return, 0, env_i, reward_value);
        set(dataset.rewards, pos, 0, reward_value);
        increment(runner.episode_step, 0, env_i, 1);
        bool truncated = terminated_flag || (SPEC::STEP_LIMIT > 0 && get(runner.episode_step, 0, env_i) >= SPEC::STEP_LIMIT);
        set(dataset.truncated, pos, 0, truncated);
        set(runner.truncated, 0, env_i, truncated);
        state = next_state;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif
