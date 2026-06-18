#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ALGORITHMS_PPO_OPERATIONS_GENERIC_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ALGORITHMS_PPO_OPERATIONS_GENERIC_H

#include "ppo.h"
#include "../../../nn/loss_functions/mse/operations_generic.h"
#include "../../../rl/components/on_policy_runner/on_policy_runner.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, rl::algorithms::ppo::Buffers<SPEC>& buffers){
        malloc(device, buffers.current_batch_actions);
        malloc(device, buffers.d_critic_output);
        malloc(device, buffers.d_action_log_prob_d_action);
        malloc(device, buffers.d_action_log_prob_d_action_log_std);
        malloc(device, buffers.rollout_log_std);
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, rl::algorithms::ppo::Buffers<SPEC>& buffers){
        free(device, buffers.current_batch_actions);
        free(device, buffers.d_critic_output);
        free(device, buffers.d_action_log_prob_d_action);
        free(device, buffers.d_action_log_prob_d_action_log_std);
        free(device, buffers.rollout_log_std);
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, rl::algorithms::PPO<SPEC>& ppo){
        malloc(device, ppo.actor);
        malloc(device, ppo.critic);
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, rl::algorithms::PPO<SPEC>& ppo){
        free(device, ppo.actor);
        free(device, ppo.critic);
    }
    template <typename DEVICE, typename SPEC, typename ACTOR_OPTIMIZER, typename CRITIC_OPTIMIZER, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT void init(DEVICE& device, rl::algorithms::PPO<SPEC>& ppo, ACTOR_OPTIMIZER& actor_optimizer, CRITIC_OPTIMIZER& critic_optimizer, RNG& rng){
#ifdef RL_TOOLS_DEBUG_RL_ALGORITHMS_PPO_CHECK_INIT
        ppo.initialized = true;
#endif
        init_weights(device, ppo.actor, rng);
        init(device, actor_optimizer);
        reset_optimizer_state(device, actor_optimizer, ppo.actor);
        auto& last_layer = get_last_layer(ppo.actor);
        set_all(device, last_layer.log_std.parameters, math::log(device.math, SPEC::PARAMETERS::INITIAL_ACTION_STD));
        init_weights(device, ppo.critic, rng);
        init(device, critic_optimizer);
        reset_optimizer_state(device, critic_optimizer, ppo.critic);
//        set_all(device, ppo.actor.input_layer.biases.parameters, 0);
//        set_all(device, ppo.actor.hidden_layers[0].biases.parameters, 0);
//        set_all(device, ppo.actor.output_layer.biases.parameters, 0);
    }
    template <typename DEVICE, typename DATASET_SPEC, typename PPO_PARAMETERS>
    RL_TOOLS_FUNCTION_PLACEMENT void estimate_generalized_advantages(DEVICE& device, rl::components::on_policy_runner::Dataset<DATASET_SPEC>& dataset, PPO_PARAMETERS ppo_parameters_tag){
        using OPR_SPEC = typename DATASET_SPEC::SPEC;
        using BUFFER = decltype(dataset);
        using T = typename DATASET_SPEC::SPEC::TYPE_POLICY::DEFAULT;
        using TI = typename DEVICE::index_t;
        constexpr TI STEPS_PER_ENV = DATASET_SPEC::STEPS_PER_ENV;
        for(TI env_i = 0; env_i < OPR_SPEC::N_ENVIRONMENTS; env_i++){
            T previous_value = get(dataset.all_values, STEPS_PER_ENV * OPR_SPEC::N_ENVIRONMENTS + env_i, 0);
            T previous_advantage = 0;
            for(TI step_forward_i = 0; step_forward_i < STEPS_PER_ENV; step_forward_i++){
                TI step_backward_i = (STEPS_PER_ENV - 1 - step_forward_i);
                TI pos = step_backward_i * OPR_SPEC::N_ENVIRONMENTS + env_i;
                bool terminated = get(dataset.terminated, pos, 0);
                bool truncated = get(dataset.truncated, pos, 0);
#ifdef RL_TOOLS_DEBUG_RL_ALGORITHMS_PPO_GAE_CHECK_TERMINATED_TRUNCATED
                utils::assert_exit(device, !terminated || (terminated && truncated), "terminationn should imply truncation");
#endif
                T current_step_value = get(dataset.values, pos, 0);
                bool terminated_actual = terminated && !PPO_PARAMETERS::IGNORE_TERMINATION;
                T next_step_value = terminated_actual ? 0 : previous_value;

                T td_error = get(dataset.rewards, pos, 0) + PPO_PARAMETERS::GAMMA * next_step_value - current_step_value;
                if(truncated){
                    if(!terminated){ // e.g. time limited or random truncation
                        td_error = 0;
                    }
                    previous_advantage = 0;
                }
                T advantage = PPO_PARAMETERS::LAMBDA * PPO_PARAMETERS::GAMMA * previous_advantage + td_error;
                set(dataset.advantages, pos, 0, advantage);
                set(dataset.target_values, pos, 0, advantage + current_step_value);
                previous_advantage = advantage;
                previous_value = current_step_value;
            }
        }
    }
    template <typename DEVICE, typename PPO_SPEC, typename DATASET_SPEC, typename ACTOR_OPTIMIZER, typename CRITIC_OPTIMIZER, typename BUFFERS_SPEC, typename ACTOR_BUFFER, typename CRITIC_BUFFER, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT void train(DEVICE& device, rl::algorithms::PPO<PPO_SPEC>& ppo, rl::components::on_policy_runner::Dataset<DATASET_SPEC>& dataset, ACTOR_OPTIMIZER& actor_optimizer, CRITIC_OPTIMIZER& critic_optimizer, rl::algorithms::ppo::Buffers<BUFFERS_SPEC>& ppo_buffers, ACTOR_BUFFER& actor_buffers, CRITIC_BUFFER& critic_buffers, RNG& rng){
#ifdef RL_TOOLS_DEBUG_RL_ALGORITHMS_PPO_CHECK_INIT
        utils::assert_exit(device, ppo.initialized, "PPO not initialized");
#endif
        using T = typename PPO_SPEC::TYPE_POLICY::DEFAULT;
        using TI = typename PPO_SPEC::TI;
        static_assert(utils::typing::is_same_v<typename PPO_SPEC::ENVIRONMENT, typename DATASET_SPEC::SPEC::ENVIRONMENT>, "environment mismatch");
        using ENVIRONMENT = typename PPO_SPEC::ENVIRONMENT;
        using DATASET = rl::components::on_policy_runner::Dataset<DATASET_SPEC>;
        static_assert(DATASET::STEPS_TOTAL > 1);
        constexpr TI N_EPOCHS = PPO_SPEC::PARAMETERS::N_EPOCHS;
        constexpr TI BATCH_SIZE = PPO_SPEC::PARAMETERS::BATCH_SIZE;
        constexpr TI N_BATCHES = DATASET::STEPS_TOTAL/BATCH_SIZE;
        static_assert(N_BATCHES > 0);
        constexpr TI ACTION_DIM = ENVIRONMENT::ACTION_DIM;
        constexpr TI OBSERVATION_DIM = ENVIRONMENT::Observation::DIM;
        constexpr TI OBSERVATION_PRIVILEGED_DIM = ENVIRONMENT::ObservationPrivileged::DIM;
        constexpr TI N_AGENTS = PPO_SPEC::ENVIRONMENT::N_AGENTS;
        static_assert(ACTION_DIM % N_AGENTS == 0);
        constexpr TI PER_AGENT_ACTION_DIM = PPO_SPEC::ENVIRONMENT::ACTION_DIM/N_AGENTS;
        // batch needs observations, original log-probs, advantages
        T policy_kl_divergence = 0; // KL( current || old ) todo: make hyperparameter that swaps the order
        if(PPO_SPEC::PARAMETERS::ADAPTIVE_LEARNING_RATE) {
            auto& last_layer = get_last_layer(ppo.actor);
            auto log_std = matrix_view(device, last_layer.log_std.parameters);
            copy(device, device, log_std, ppo_buffers.rollout_log_std);
        }
        for(TI epoch_i = 0; epoch_i < N_EPOCHS; epoch_i++){
            // shuffling
            for(TI dataset_i = 0; dataset_i < DATASET::STEPS_TOTAL; dataset_i++){
                TI sample_index = random::uniform_int_distribution(device.random, dataset_i, DATASET::STEPS_TOTAL-1, rng);
                {
                    auto target_row = row(device, dataset.observations, dataset_i);
                    auto source_row = row(device, dataset.observations, sample_index);
                    swap(device, target_row, source_row);
                }
                if(PPO_SPEC::ASYMMETRIC_OBSERVATIONS){
                    auto target_row = row(device, dataset.all_observations_privileged, dataset_i);
                    auto source_row = row(device, dataset.all_observations_privileged, sample_index);
                    swap(device, target_row, source_row);
                }
                if(PPO_SPEC::PARAMETERS::ADAPTIVE_LEARNING_RATE){
                    auto target_row = row(device, dataset.actions_mean, dataset_i);
                    auto source_row = row(device, dataset.actions_mean, sample_index);
                    swap(device, target_row, source_row);
                }
                {
                    auto target_row = row(device, dataset.actions, dataset_i);
                    auto source_row = row(device, dataset.actions, sample_index);
                    swap(device, target_row, source_row);
                }
                swap(device, dataset.advantages      , dataset.advantages      , dataset_i, 0, sample_index, 0);
                swap(device, dataset.action_log_probs, dataset.action_log_probs, dataset_i, 0, sample_index, 0);
                swap(device, dataset.target_values   , dataset.target_values   , dataset_i, 0, sample_index, 0);
            }
            static_assert(N_BATCHES > 0);
            for(TI batch_i = 0; batch_i < N_BATCHES; batch_i++){
                T batch_policy_kl_divergence = 0; // KL( current || old ) todo: make hyperparameter that swaps the order
                zero_gradient(device, ppo.critic);
                zero_gradient(device, ppo.actor); // has to be reset before accumulating the action-log-std gradient

                auto batch_offset = batch_i * BATCH_SIZE;
                auto batch_observations            = view(device, dataset.observations               , matrix::ViewSpec<BATCH_SIZE, OBSERVATION_DIM           >(), batch_offset, 0);
                auto batch_observations_privileged = view(device, dataset.all_observations_privileged, matrix::ViewSpec<BATCH_SIZE, OBSERVATION_PRIVILEGED_DIM>(), batch_offset, 0);
                auto batch_actions_mean            = view(device, dataset.actions_mean               , matrix::ViewSpec<BATCH_SIZE, ACTION_DIM                >(), batch_offset, 0);
                auto batch_actions                 = view(device, dataset.actions                    , matrix::ViewSpec<BATCH_SIZE, ACTION_DIM                >(), batch_offset, 0);
                auto batch_action_log_probs        = view(device, dataset.action_log_probs           , matrix::ViewSpec<BATCH_SIZE, 1                         >(), batch_offset, 0);
                auto batch_advantages              = view(device, dataset.advantages                 , matrix::ViewSpec<BATCH_SIZE, 1                         >(), batch_offset, 0);
                auto batch_target_values           = view(device, dataset.target_values              , matrix::ViewSpec<BATCH_SIZE, 1                         >(), batch_offset, 0);

                T advantage_mean = 0;
                T advantage_std = 0;
                if(PPO_SPEC::PARAMETERS::NORMALIZE_ADVANTAGE) {
                    for (TI batch_step_i = 0; batch_step_i < BATCH_SIZE; batch_step_i++) {
                        T advantage = get(batch_advantages, batch_step_i, 0);
                        advantage_mean += advantage;
                        advantage_std += advantage * advantage;
                    }
                    advantage_mean /= BATCH_SIZE;
                    advantage_std /= BATCH_SIZE;
                    advantage_std = math::sqrt(device.math, math::max(device.math, (T)0, advantage_std - advantage_mean * advantage_mean));
                }
//                add_scalar(device, device.logger, "ppo/advantage/mean", advantage_mean);
//                add_scalar(device, device.logger, "ppo/advantage/std", advantage_std);

                auto batch_observations_tensor = to_tensor(device, batch_observations);
                auto batch_observations_tensor_unsqueezed = unsqueeze(device, batch_observations_tensor);
                auto current_batch_actions_tensor = to_tensor(device, ppo_buffers.current_batch_actions);
                auto current_batch_actions_tensor_unsqueezed = unsqueeze(device, current_batch_actions_tensor);
                forward(device, ppo.actor, batch_observations_tensor_unsqueezed, current_batch_actions_tensor_unsqueezed, actor_buffers, rng);
//                auto abs_diff = abs_diff(device, batch_actions, dataset.actions);

                for(TI batch_step_i = 0; batch_step_i < BATCH_SIZE; batch_step_i++){
                    T action_log_prob = 0;
                    for(TI action_i = 0; action_i < ACTION_DIM; action_i++){

                        T current_action = get(ppo_buffers.current_batch_actions, batch_step_i, action_i);
                        T rollout_action = get(batch_actions, batch_step_i, action_i);
                        auto& last_layer = get_last_layer(ppo.actor);
                        T current_action_log_std = get(device, last_layer.log_std.parameters, action_i % PER_AGENT_ACTION_DIM);
                        T current_action_std = math::exp(device.math, current_action_log_std);
                        if(PPO_SPEC::PARAMETERS::ADAPTIVE_LEARNING_RATE){
                            T rollout_action_log_std = get(ppo_buffers.rollout_log_std, 0, action_i);
                            T rollout_action_std = math::exp(device.math, rollout_action_log_std);
                            T rollout_action_mean = get(batch_actions_mean, batch_step_i, action_i);
                            T action_mean_diff = rollout_action_mean - current_action;
                            T kl = rollout_action_log_std - current_action_log_std;
                            kl += (current_action_std * current_action_std + action_mean_diff * action_mean_diff)/(2 * rollout_action_std * rollout_action_std + PPO_SPEC::PARAMETERS::POLICY_KL_EPSILON);
                            kl += (T)-0.5;
                            kl = math::max(device.math, kl, (T)0);
                            policy_kl_divergence += kl;
                            batch_policy_kl_divergence += kl;
                        }

//                        T action_diff_by_action_std = (current_action - rollout_action) / current_action_std;
//                        action_log_prob += -0.5 * action_diff_by_action_std * action_diff_by_action_std - current_action_log_std - 0.5 * math::log(device.math, 2 * math::PI<T>);
                        // probability of the old actions under the new policy
                        action_log_prob += random::normal_distribution::log_prob(device.random, current_action, current_action_log_std, rollout_action);
//                        set(ppo_buffers.d_action_log_prob_d_action, batch_step_i, action_i, - action_diff_by_action_std / current_action_std);
                        set(ppo_buffers.d_action_log_prob_d_action, batch_step_i, action_i, random::normal_distribution::d_log_prob_d_mean(device.random, current_action, current_action_log_std, rollout_action));

                        T current_entropy = current_action_log_std + math::log(device.math, 2 * math::PI<T>)/(T)2 + (T)1/(T)2;
                        T current_entropy_loss = -(T)1/BATCH_SIZE * PPO_SPEC::PARAMETERS::ACTION_ENTROPY_COEFFICIENT * current_entropy;
                        // todo: think about possible implementation detail: clipping entropy bonus as well (because it changes the distribution)
                        if(PPO_SPEC::PARAMETERS::LEARN_ACTION_STD){
                            T d_entropy_loss_d_current_action_log_std = -(T)1/BATCH_SIZE * PPO_SPEC::PARAMETERS::ACTION_ENTROPY_COEFFICIENT;
                            auto& last_layer = get_last_layer(ppo.actor);
                            increment(device, last_layer.log_std.gradient, d_entropy_loss_d_current_action_log_std, action_i % PER_AGENT_ACTION_DIM);
//                          derivation: d_current_action_log_prob_d_action_log_std
//                          d_current_action_log_prob_d_action_std =  (-action_diff_by_action_std) * (-action_diff_by_action_std)      / action_std - 1 / action_std)
//                          d_current_action_log_prob_d_action_std = ((-action_diff_by_action_std) * (-action_diff_by_action_std) - 1) / action_std)
//                          d_current_action_log_prob_d_action_std = (action_diff_by_action_std * action_diff_by_action_std - 1) / action_std
//                          d_current_action_log_prob_d_action_log_std = (action_diff_by_action_std * action_diff_by_action_std - 1) / action_std * exp(action_log_std)
//                          d_current_action_log_prob_d_action_log_std = (action_diff_by_action_std * action_diff_by_action_std - 1) / action_std * action_std
//                          d_current_action_log_prob_d_action_log_std =  action_diff_by_action_std * action_diff_by_action_std - 1
//                            T d_current_action_log_prob_d_action_log_std = action_diff_by_action_std * action_diff_by_action_std - 1;
                            T d_action_log_prob_d_current_action_log_std = random::normal_distribution::d_log_prob_d_log_std(device.random, current_action, current_action_log_std, rollout_action);
                            set(ppo_buffers.d_action_log_prob_d_action_log_std, batch_step_i, action_i, d_action_log_prob_d_current_action_log_std);
                        }
                    }
                    T rollout_action_log_prob = get(batch_action_log_probs, batch_step_i, 0);
                    T advantage = get(batch_advantages, batch_step_i, 0);
                    if(PPO_SPEC::PARAMETERS::NORMALIZE_ADVANTAGE){
                        advantage = (advantage - advantage_mean) / (advantage_std + PPO_SPEC::PARAMETERS::ADVANTAGE_EPSILON);
                    }
                    T log_ratio = action_log_prob - rollout_action_log_prob;
                    T ratio = math::exp(device.math, log_ratio);
                    // todo: test relative clipping (clipping in log space makes more sense thatn clipping in exp space)
                    T clipped_ratio = math::clamp(device.math, ratio, 1 - PPO_SPEC::PARAMETERS::EPSILON_CLIP, 1 + PPO_SPEC::PARAMETERS::EPSILON_CLIP);
                    bool clipped = ratio != clipped_ratio;
                    T normal_advantage = ratio * advantage;
                    T clipped_advantage = clipped_ratio * advantage;
                    T slippage = 0.0;
                    bool ratio_min_switch = normal_advantage - clipped_advantage <= slippage;
                    T pessimistic_surrogate = ratio_min_switch ? normal_advantage : clipped_advantage;

                    T d_loss_d_pessimistic_surrogate = -(T)1/BATCH_SIZE;
                    T d_pessimistic_surrogate_d_normal_advantage = ratio_min_switch ? 1 : 0;
                    T d_pessimistic_surrogate_d_clipped_advantage = ratio_min_switch ? 0 : 1;
                    T d_normal_advantage_d_ratio = advantage;
                    T d_clipped_advantage_d_clipped_ratio = advantage;
                    T d_clipped_ratio_d_ratio = clipped ? 0 : 1;
                    T d_pessimistic_surrogate_d_ratio = d_pessimistic_surrogate_d_normal_advantage * d_normal_advantage_d_ratio + d_pessimistic_surrogate_d_clipped_advantage * d_clipped_advantage_d_clipped_ratio * d_clipped_ratio_d_ratio;
                    T d_loss_d_ratio = d_loss_d_pessimistic_surrogate * d_pessimistic_surrogate_d_ratio;
                    T d_ratio_d_action_log_prob = ratio;
                    T d_loss_d_action_log_prob = d_loss_d_ratio * d_ratio_d_action_log_prob;
                    for(TI action_i = 0; action_i < ACTION_DIM; action_i++){
                        multiply(ppo_buffers.d_action_log_prob_d_action, batch_step_i, action_i, d_loss_d_action_log_prob);
                        if(PPO_SPEC::PARAMETERS::LEARN_ACTION_STD){
                            T current_d_action_log_prob_d_action_log_std = get(ppo_buffers.d_action_log_prob_d_action_log_std, batch_step_i, action_i);
                            auto& last_layer = get_last_layer(ppo.actor);
                            increment(device, last_layer.log_std.gradient, d_loss_d_action_log_prob * current_d_action_log_prob_d_action_log_std, action_i % PER_AGENT_ACTION_DIM);
                        }
                    }
                }
                if(PPO_SPEC::PARAMETERS::ADAPTIVE_LEARNING_RATE){
                    batch_policy_kl_divergence /= BATCH_SIZE;
                    auto& actor_optimizer_parameters = get_ref(device, actor_optimizer.parameters, 0);
                    if(batch_policy_kl_divergence > 2 * PPO_SPEC::PARAMETERS::ADAPTIVE_LEARNING_RATE_POLICY_KL_THRESHOLD){
                        actor_optimizer_parameters.alpha = math::max(device.math, actor_optimizer_parameters.alpha * PPO_SPEC::PARAMETERS::ADAPTIVE_LEARNING_RATE_DECAY, PPO_SPEC::PARAMETERS::ADAPTIVE_LEARNING_RATE_MIN);
                    }
                    if(batch_policy_kl_divergence < 0.5 * PPO_SPEC::PARAMETERS::ADAPTIVE_LEARNING_RATE_POLICY_KL_THRESHOLD){
                        actor_optimizer_parameters.alpha = math::min(device.math, actor_optimizer_parameters.alpha / PPO_SPEC::PARAMETERS::ADAPTIVE_LEARNING_RATE_DECAY, PPO_SPEC::PARAMETERS::ADAPTIVE_LEARNING_RATE_MAX);
                    }
                }
                auto d_action_d_log_prob_action_tensor = to_tensor(device, ppo_buffers.d_action_log_prob_d_action);
                auto d_action_d_log_prob_action_tensor_unsqueezed = unsqueeze(device, d_action_d_log_prob_action_tensor);
                backward(device, ppo.actor, batch_observations_tensor_unsqueezed, d_action_d_log_prob_action_tensor_unsqueezed, actor_buffers);
//                forward_backward_mse(device, ppo.critic, batch_observations, batch_target_values, critic_buffers);

                auto batch_observations_privileged_tensor = to_tensor(device, batch_observations_privileged);
                auto batch_observations_privileged_tensor_unsqueezed = unsqueeze(device, batch_observations_privileged_tensor);
                {
                    forward(device, ppo.critic, batch_observations_privileged_tensor_unsqueezed, critic_buffers, rng);
                    auto output_tensor = output(device, ppo.critic);
                    static_assert(sizeof(output_tensor) <= sizeof(void*));
                    auto output_matrix_view = matrix_view(device, output_tensor);
                    nn::loss_functions::mse::gradient(device, output_matrix_view, batch_target_values, ppo_buffers.d_critic_output, 0.5);
                    auto d_critic_output_tensor = to_tensor(device, ppo_buffers.d_critic_output);
                    auto d_critic_output_tensor_unsqueezed = unsqueeze(device, d_critic_output_tensor);
                    backward(device, ppo.critic, batch_observations_privileged_tensor_unsqueezed, d_critic_output_tensor_unsqueezed, critic_buffers);
                }
                auto output_tensor = output(device, ppo.critic);
                auto output_matrix_view = matrix_view(device, output_tensor);
                T critic_loss = nn::loss_functions::mse::evaluate(device, output_matrix_view, batch_target_values);
                add_scalar(device, device.logger, "ppo/critic_loss", critic_loss);
                step(device, actor_optimizer, ppo.actor);
                step(device, actor_optimizer, ppo.critic); // todo: evaluate switch to critic_optimizer
            }
        }
        if(PPO_SPEC::PARAMETERS::ADAPTIVE_LEARNING_RATE) {
            policy_kl_divergence /= N_EPOCHS * N_BATCHES * BATCH_SIZE;
            add_scalar(device, device.logger, "ppo/policy_kl", policy_kl_divergence);
        }
    }

    template <typename DEVICE_SOURCE, typename DEVICE_TARGET, typename PPO_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(DEVICE_SOURCE& device_source, DEVICE_TARGET& device_target, const rl::algorithms::PPO<PPO_SPEC>& source, rl::algorithms::PPO<PPO_SPEC>& target){
        copy(device_source, device_target, source.actor, target.actor);
        copy(device_source, device_target, source.critic, target.critic);
#ifdef RL_TOOLS_DEBUG_RL_ALGORITHMS_PPO_CHECK_INIT
        target.initialized = source.initialized;
#endif
    }

}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
