#include "../../../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ALGORITHMS_TD3_LOOP_CORE_OPERATIONS_GENERIC_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ALGORITHMS_TD3_LOOP_CORE_OPERATIONS_GENERIC_H

#include "../../../../../nn/optimizers/adam/instance/operations_generic.h"
#include "../../../../../nn/layers/td3_sampling/operations_generic.h"
#include "../../../../../nn_models/operations_generic.h"
#include "../../../../../nn_models/sequential/operations_generic.h"
#include "../../../../../rl/algorithms/td3/operations_generic.h"
#include "../../../../../nn_models/random_uniform/operations_generic.h"
#include "../../../../../rl/components/off_policy_runner/operations_generic.h"

#include "config.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template <typename DEVICE, typename T_CONFIG>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, rl::algorithms::td3::loop::core::State<T_CONFIG>& ts){
        malloc(device, ts.actor_critic);
        malloc(device, ts.off_policy_runner);
        malloc(device, ts.critic_batch);
        malloc(device, ts.critic_training_buffers);
        malloc(device, ts.critic_buffers[0]);
        malloc(device, ts.critic_buffers[1]);
        malloc(device, ts.critic_target_buffers[0]);
        malloc(device, ts.critic_target_buffers[1]);
        malloc(device, ts.actor_batch);
        malloc(device, ts.actor_training_buffers);
        malloc(device, ts.actor_buffers_eval);
        malloc(device, ts.actor_buffers[0]);
        malloc(device, ts.actor_buffers[1]);
        malloc(device, ts.actor_target_buffers[0]);
        malloc(device, ts.actor_target_buffers[1]);
        for(auto& env: ts.envs){
            rl_tools::malloc(device, env);
        }
    }
    template <typename DEVICE, typename T_CONFIG>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, rl::algorithms::td3::loop::core::State<T_CONFIG>& ts){
        free(device, ts.actor_critic);
        free(device, ts.off_policy_runner);
        free(device, ts.critic_batch);
        free(device, ts.critic_training_buffers);
        free(device, ts.critic_buffers[0]);
        free(device, ts.critic_buffers[1]);
        free(device, ts.critic_target_buffers[0]);
        free(device, ts.critic_target_buffers[1]);
        free(device, ts.actor_batch);
        free(device, ts.actor_training_buffers);
        free(device, ts.actor_buffers_eval);
        free(device, ts.actor_buffers[0]);
        free(device, ts.actor_buffers[1]);
        free(device, ts.actor_target_buffers[0]);
        free(device, ts.actor_target_buffers[1]);
        for(auto& env: ts.envs){
            rl_tools::free(device, env);
        }
    }
    template <typename DEVICE, typename T_CONFIG>
    RL_TOOLS_FUNCTION_PLACEMENT void init(DEVICE& device, rl::algorithms::td3::loop::core::State<T_CONFIG>& ts, typename T_CONFIG::TI seed = 0){
        using CONFIG = T_CONFIG;
        using TYPE_POLICY = typename CONFIG::TYPE_POLICY;
        using TI = typename DEVICE::index_t;

        init(device, ts.rng, seed);

        init(device, ts.actor_critic, ts.rng);

        for(TI env_i = 0; env_i < CONFIG::CORE_PARAMETERS::N_ENVIRONMENTS; env_i ++){
            rl_tools::init(device, ts.envs[env_i]);
        }
        init(device, ts.off_policy_runner);

        ts.step = 0;
    }


    template <typename DEVICE, typename T_CONFIG>
    RL_TOOLS_FUNCTION_PLACEMENT bool step(DEVICE& device, rl::algorithms::td3::loop::core::State<T_CONFIG>& ts){
        using CONFIG = T_CONFIG;
        set_step(device, device.logger, ts.step);
        bool finished = false;
        if(ts.step >= CONFIG::CORE_PARAMETERS::N_WARMUP_STEPS){
            step<1>(device, ts.off_policy_runner, get_actor(ts), ts.actor_buffers_eval, ts.rng);
        }
        else{
            typename CONFIG::EXPLORATION_POLICY exploration_policy;
            typename CONFIG::EXPLORATION_POLICY::template Buffer<> exploration_policy_buffer;
            step<0>(device, ts.off_policy_runner, exploration_policy, exploration_policy_buffer, ts.rng);
        }

        bool train_critic_flag = ts.step >= CONFIG::CORE_PARAMETERS::N_WARMUP_STEPS_CRITIC && ts.step % CONFIG::CORE_PARAMETERS::TD3_PARAMETERS::CRITIC_TRAINING_INTERVAL == 0;
        bool update_critic_targets_flag = ts.step >= CONFIG::CORE_PARAMETERS::N_WARMUP_STEPS_CRITIC && ts.step % CONFIG::CORE_PARAMETERS::TD3_PARAMETERS::CRITIC_TARGET_UPDATE_INTERVAL == 0;
        bool train_actor_flag = ts.step >= CONFIG::CORE_PARAMETERS::N_WARMUP_STEPS_ACTOR && ts.step % CONFIG::CORE_PARAMETERS::TD3_PARAMETERS::ACTOR_TRAINING_INTERVAL == 0;
        bool update_actor_targets_flag = ts.step >= CONFIG::CORE_PARAMETERS::N_WARMUP_STEPS_ACTOR && ts.step % CONFIG::CORE_PARAMETERS::TD3_PARAMETERS::ACTOR_TARGET_UPDATE_INTERVAL == 0;

        if(CONFIG::CORE_PARAMETERS::SHARED_BATCH && (train_critic_flag || train_actor_flag)){
            gather_batch(device, ts.off_policy_runner, ts.critic_batch, ts.rng);
            auto action_noise_matrix_view = matrix_view(device, ts.critic_training_buffers.target_next_action_noise);
            target_action_noise(device, ts.actor_critic, action_noise_matrix_view, ts.rng);
        }
        if(train_critic_flag){
            for(int critic_i = 0; critic_i < 2; critic_i++){
                if constexpr(!CONFIG::CORE_PARAMETERS::SHARED_BATCH) {
                    gather_batch(device, ts.off_policy_runner, ts.critic_batch, ts.rng);
                    auto action_noise_matrix_view = matrix_view(device, ts.critic_training_buffers.target_next_action_noise);
                    target_action_noise(device, ts.actor_critic, action_noise_matrix_view, ts.rng);
                }
                train_critic(device, ts.actor_critic, ts.actor_critic.critics[critic_i], ts.critic_batch, ts.actor_critic.critic_optimizers[critic_i], ts.actor_buffers[critic_i], ts.actor_target_buffers[critic_i], ts.critic_buffers[critic_i], ts.critic_target_buffers[critic_i], ts.critic_training_buffers, ts.rng);
            }
        }
        if(update_critic_targets_flag){
            update_critic_targets(device, ts.actor_critic);
        }
        if(train_actor_flag){
            if constexpr(CONFIG::CORE_PARAMETERS::SHARED_BATCH) {
                train_actor(device, ts.actor_critic, ts.critic_batch, ts.actor_critic.actor_optimizer, ts.actor_buffers[0], ts.critic_buffers[0], ts.actor_training_buffers, ts.rng);
            }
            else{
                gather_batch(device, ts.off_policy_runner, ts.actor_batch, ts.rng);
                train_actor(device, ts.actor_critic, ts.actor_batch, ts.actor_critic.actor_optimizer, ts.actor_buffers[0], ts.critic_buffers[0], ts.actor_training_buffers, ts.rng);
            }
        }
        if(update_actor_targets_flag){
            update_actor_target(device, ts.actor_critic);
        }
        ts.step++;
        if(ts.step > CONFIG::CORE_PARAMETERS::STEP_LIMIT){
            return true;
        }
        else{
            return finished;
        }
    }
    // the following operations are for nn_analytics iterating the neural networks
    template <auto INDEX, typename DEVICE, typename T_CONFIG>
    constexpr auto& get_nn(DEVICE& device, rl::algorithms::td3::loop::core::State<T_CONFIG>& ts){
        static_assert(INDEX < T_CONFIG::NUM_NNS, "Index out of bounds, there are only 3 neural networks in the TD3");
        if constexpr(INDEX == 0){
            return ts.actor_critic.actor;
        }
        else{
            if constexpr(INDEX == 1){
                return ts.actor_critic.critics[0];
            }
            else{
                return ts.actor_critic.critics[1];
            }
        }
    }
    template <auto INDEX, typename DEVICE, typename T_CONFIG>
    constexpr auto& get_nn_name(DEVICE& device, rl::algorithms::td3::loop::core::State<T_CONFIG>& ts){
        static_assert(INDEX < T_CONFIG::NUM_NNS, "Index out of bounds, there are only 3 neural networks in the TD3");
        if constexpr(INDEX == 0){
            return "actor";
        }
        else{
            if constexpr(INDEX == 1){
                return "critic[0]";
            }
            else{
                return "critic[1]";
            }
        }
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END


#endif
