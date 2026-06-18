#include "../../../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ALGORITHMS_PPO_LOOP_CORE_OPERATIONS_GENERIC_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ALGORITHMS_PPO_LOOP_CORE_OPERATIONS_GENERIC_H

#include "../../../../../nn/optimizers/adam/instance/operations_generic.h"
#include "../../../../../nn/layers/standardize/operations_generic.h"
#include "../../../../../nn_models/mlp_unconditional_stddev/operations_generic.h"
#include "../../../../../nn_models/sequential/operations_generic.h"
#include "../../../../../nn/optimizers/adam/operations_generic.h"
#include "../../../../../rl/algorithms/ppo/operations_generic.h"
#include "../../../../../rl/components/on_policy_runner/operations_generic.h"
#include "../../../../../rl/components/running_normalizer/operations_generic.h"

#include "config.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template <typename DEVICE, typename T_CONFIG>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, rl::algorithms::ppo::loop::core::State<T_CONFIG>& ts){
        malloc(device, ts.ppo);
        malloc(device, ts.ppo_buffers);
        malloc(device, ts.on_policy_runner_dataset);
        malloc(device, ts.on_policy_runner);
        malloc(device, ts.actor_eval_buffers);
        malloc(device, ts.actor_buffers);
        malloc(device, ts.critic_buffers);
        malloc(device, ts.critic_buffers_gae);
        malloc(device, ts.actor_optimizer);
        malloc(device, ts.critic_optimizer);
        malloc(device, ts.observations_dense);
        malloc(device, ts.observation_normalizer);
        malloc(device, ts.observation_privileged_normalizer);
        for(auto& env: ts.envs){
            malloc(device, env);
        }

    }
    template <typename DEVICE, typename T_CONFIG>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, rl::algorithms::ppo::loop::core::State<T_CONFIG>& ts){
        free(device, ts.ppo);
        free(device, ts.ppo_buffers);
        free(device, ts.on_policy_runner_dataset);
        free(device, ts.on_policy_runner);
        free(device, ts.actor_eval_buffers);
        free(device, ts.actor_buffers);
        free(device, ts.critic_buffers);
        free(device, ts.critic_buffers_gae);
        free(device, ts.observations_dense);
        free(device, ts.actor_optimizer);
        free(device, ts.critic_optimizer);
        free(device, ts.observation_normalizer);
        free(device, ts.observation_privileged_normalizer);
        for(auto& env: ts.envs){
            free(device, env);
        }
    }
    template <typename DEVICE, typename T_CONFIG>
    RL_TOOLS_FUNCTION_PLACEMENT void init(DEVICE& device, rl::algorithms::ppo::loop::core::State<T_CONFIG>& ts, typename T_CONFIG::TI seed = 0){
        using CONFIG = T_CONFIG;
        using T = typename CONFIG::T;
        using TI = typename DEVICE::index_t;

        init(device, ts.rng, seed);

        for(TI env_i=0; env_i < CONFIG::CORE_PARAMETERS::N_ENVIRONMENTS; env_i++){
            init(device, ts.envs[env_i]);
        }

        init(device, ts.on_policy_runner, ts.envs, ts.env_parameters, ts.rng);
        init(device, ts.observation_normalizer);
        init(device, ts.observation_privileged_normalizer);
        init(device, ts.ppo, ts.actor_optimizer, ts.critic_optimizer, ts.rng);

        ts.step = 0;
    }


    template <typename DEVICE, typename T_CONFIG>
    RL_TOOLS_FUNCTION_PLACEMENT bool step(DEVICE& device, rl::algorithms::ppo::loop::core::State<T_CONFIG>& ts){
        using CONFIG = T_CONFIG;
        using TI = typename DEVICE::index_t;
        using OBS_SPEC = decltype(ts.on_policy_runner_dataset.observations);
        constexpr TI N_AGENTS = T_CONFIG::ENVIRONMENT::N_AGENTS;
        set_step(device, device.logger, ts.step * CONFIG::CORE_PARAMETERS::N_ENVIRONMENTS * CONFIG::CORE_PARAMETERS::ON_POLICY_RUNNER_STEPS_PER_ENV);
        bool finished = false;

        auto per_agent_observations = reshape<OBS_SPEC::ROWS*N_AGENTS, OBS_SPEC::COLS/N_AGENTS>(device, ts.observations_dense);
        if(T_CONFIG::CORE_PARAMETERS::NORMALIZE_OBSERVATIONS && ts.step == 0){
            for(TI observation_normalization_warmup_step_i = 0; observation_normalization_warmup_step_i < T_CONFIG::OBSERVATION_NORMALIZATION_WARMUP_STEPS; observation_normalization_warmup_step_i++) {
                collect(device, ts.on_policy_runner_dataset, ts.on_policy_runner, ts.ppo.actor, ts.actor_eval_buffers, ts.rng);
                copy(device, device, ts.on_policy_runner_dataset.observations, ts.observations_dense);
                update(device, ts.observation_normalizer, per_agent_observations);
                update(device, ts.observation_privileged_normalizer, ts.on_policy_runner_dataset.all_observations_privileged);
            }
            init(device, ts.on_policy_runner, ts.envs, ts.env_parameters, ts.rng); // reinitializing the on_policy_runner to reset the episode counters
            set_statistics(device, get_first_layer(ts.ppo.actor), ts.observation_normalizer.mean, ts.observation_normalizer.std);
            set_statistics(device, ts.ppo.critic.content, ts.observation_privileged_normalizer.mean, ts.observation_privileged_normalizer.std);
        }
        collect(device, ts.on_policy_runner_dataset, ts.on_policy_runner, ts.ppo.actor, ts.actor_eval_buffers, ts.rng);
        if(T_CONFIG::CORE_PARAMETERS::NORMALIZE_OBSERVATIONS && T_CONFIG::CORE_PARAMETERS::NORMALIZE_OBSERVATIONS_CONTINUOUSLY){
            copy(device, device, ts.on_policy_runner_dataset.observations, ts.observations_dense);
            update(device, ts.observation_normalizer, per_agent_observations);
            set_statistics(device, get_first_layer(ts.ppo.actor), ts.observation_normalizer.mean, ts.observation_normalizer.std);
            update(device, ts.observation_privileged_normalizer, ts.on_policy_runner_dataset.all_observations_privileged);
            set_statistics(device, ts.ppo.critic.content, ts.observation_privileged_normalizer.mean, ts.observation_privileged_normalizer.std);
        }
        auto all_observations_privileged_tensor = to_tensor(device, ts.on_policy_runner_dataset.all_observations_privileged);
        auto all_observations_privileged_tensor_unsqueezed = unsqueeze(device, all_observations_privileged_tensor);
        auto all_values_tensor = to_tensor(device, ts.on_policy_runner_dataset.all_values);
        auto all_values_tensor_unsqueezed = unsqueeze(device, all_values_tensor);
        evaluate(device, ts.ppo.critic, all_observations_privileged_tensor_unsqueezed, all_values_tensor_unsqueezed, ts.critic_buffers_gae, ts.rng);
        estimate_generalized_advantages(device, ts.on_policy_runner_dataset, typename CONFIG::PPO_TYPE::SPEC::PARAMETERS{});
        train(device, ts.ppo, ts.on_policy_runner_dataset, ts.actor_optimizer, ts.critic_optimizer, ts.ppo_buffers, ts.actor_buffers, ts.critic_buffers, ts.rng);

        {
            // logging actor std
            auto& last_layer = get_last_layer(ts.ppo.actor);
            using T = typename CONFIG::T;
            constexpr TI PER_AGENT_ACTION_DIM = T_CONFIG::ENVIRONMENT::ACTION_DIM/N_AGENTS;
            for(TI action_i = 0; action_i < PER_AGENT_ACTION_DIM; action_i++){
                T current_action_log_std = get(device, last_layer.log_std.parameters, action_i % PER_AGENT_ACTION_DIM);
                add_scalar(device, device.logger, "actor/log_std", current_action_log_std);
            }
        }

//        log(device, device.logger, "log_std: ", get(ts.ppo.actor.log_std.parameters, 0, 0));

        ts.step++;
        if(ts.step > CONFIG::CORE_PARAMETERS::STEP_LIMIT){
            return true;
        }
        else{
            return finished;
        }
    }
    template <typename DEVICE, typename PARAMETERS, typename utils::typing::enable_if<utils::typing::is_same_v<typename PARAMETERS::TAG, rl::algorithms::ppo::loop::core::ParametersTag>>::type* = nullptr>
    RL_TOOLS_FUNCTION_PLACEMENT void log(DEVICE& device, PARAMETERS){
        log(device, device.logger, "STEP_LIMIT: ", PARAMETERS::STEP_LIMIT);
        log(device, device.logger, "ACTOR_HIDDEN_DIM: ", PARAMETERS::ACTOR_HIDDEN_DIM);
        log(device, device.logger, "ACTOR_NUM_LAYERS: ", PARAMETERS::ACTOR_NUM_LAYERS);
        log(device, device.logger, "ACTOR_ACTIVATION_FUNCTION: ", PARAMETERS::ACTOR_ACTIVATION_FUNCTION);
        log(device, device.logger, "CRITIC_HIDDEN_DIM: ", PARAMETERS::CRITIC_HIDDEN_DIM);
        log(device, device.logger, "CRITIC_NUM_LAYERS: ", PARAMETERS::CRITIC_NUM_LAYERS);
        log(device, device.logger, "CRITIC_ACTIVATION_FUNCTION: ", PARAMETERS::CRITIC_ACTIVATION_FUNCTION);
        log(device, device.logger, "EPISODE_STEP_LIMIT: ", PARAMETERS::EPISODE_STEP_LIMIT);
        log(device, device.logger, "N_ENVIRONMENTS: ", PARAMETERS::N_ENVIRONMENTS);
        log(device, device.logger, "ON_POLICY_RUNNER_STEPS_PER_ENV: ", PARAMETERS::ON_POLICY_RUNNER_STEPS_PER_ENV);
        log(device, device.logger, "BATCH_SIZE: ", PARAMETERS::BATCH_SIZE);
    }
    template <typename DEVICE, typename CONFIG, typename utils::typing::enable_if<utils::typing::is_same_v<typename CONFIG::TAG, rl::algorithms::ppo::loop::core::ConfigTag>>::type* = nullptr>
    RL_TOOLS_FUNCTION_PLACEMENT void log(DEVICE& device, CONFIG){
        log(device, typename CONFIG::CORE_PARAMETERS{});
//        log(device, typename CONFIG::NEXT{});
    }

    // the following operations are for nn_analytics iterating the neural networks
    template <auto INDEX, typename DEVICE, typename T_CONFIG>
    constexpr auto& get_nn(DEVICE& device, rl::algorithms::ppo::loop::core::State<T_CONFIG>& ts){
        static_assert(INDEX < T_CONFIG::NUM_NNS, "Index out of bounds, there are only 2 neural networks in the PPO");
        if constexpr(INDEX == 0){
            return ts.ppo.actor;
        }
        else{
            return ts.ppo.critic;
        }
    }
    template <auto INDEX, typename DEVICE, typename T_CONFIG>
    constexpr auto& get_nn_name(DEVICE& device, rl::algorithms::ppo::loop::core::State<T_CONFIG>& ts){
        static_assert(INDEX < T_CONFIG::NUM_NNS, "Index out of bounds, there are only 2 neural networks in the PPO");
        if constexpr(INDEX == 0){
            return "actor";
        }
        else{
            return "critic";
        }
    }


}
RL_TOOLS_NAMESPACE_WRAPPER_END


#endif
