#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ALGORITHMS_PPO_OPERATIONS_GENERIC_EXTENSIONS_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ALGORITHMS_PPO_OPERATIONS_GENERIC_EXTENSIONS_H

#include "ppo.h"
#include "../../../rl/components/on_policy_runner/on_policy_runner.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    namespace rl::algorithms::ppo{

        template <typename PPO_SPEC>
        struct TrainingBuffersHybrid{
            using SPEC = PPO_SPEC;
            using T = typename SPEC::T;
            using TI = typename SPEC::TI;
            static constexpr TI BATCH_SIZE = SPEC::BATCH_SIZE;
            static constexpr TI ACTION_DIM = SPEC::ENVIRONMENT::ACTION_DIM;
            static constexpr TI OBSERVATION_DIM = SPEC::ENVIRONMENT::Observation::DIM;
            typename SPEC::CONTAINER_TYPE_TAG::template type<matrix::Specification<T, TI, BATCH_SIZE, ACTION_DIM>> actions;
            typename SPEC::CONTAINER_TYPE_TAG::template type<matrix::Specification<T, TI, BATCH_SIZE, OBSERVATION_DIM>> observations;
            typename SPEC::CONTAINER_TYPE_TAG::template type<matrix::Specification<T, TI, BATCH_SIZE, ACTION_DIM>> d_action_log_prob_d_action;
            typename SPEC::CONTAINER_TYPE_TAG::template type<matrix::Specification<T, TI, BATCH_SIZE, 1>> target_values;
            typename SPEC::CONTAINER_TYPE_TAG::template type<matrix::Specification<T, TI, BATCH_SIZE, 1>> d_critic_output;
        };
    }
    template <typename DEVICE, typename SPEC>
    void malloc(DEVICE& device, rl::algorithms::ppo::TrainingBuffersHybrid<SPEC>& buffers){
        malloc(device, buffers.actions);
        malloc(device, buffers.observations);
        malloc(device, buffers.d_action_log_prob_d_action);
        malloc(device, buffers.target_values);
        malloc(device, buffers.d_critic_output);
    }
    template <typename DEVICE, typename SPEC>
    void free(DEVICE& device, rl::algorithms::ppo::TrainingBuffersHybrid<SPEC>& buffers){
        free(device, buffers.actions);
        free(device, buffers.observations);
        free(device, buffers.d_action_log_prob_d_action);
        free(device, buffers.target_values);
        free(device, buffers.d_critic_output);
    }
    template <typename DEVICE, typename DEVICE_EVALUATION, typename PPO_SPEC, typename OPR_SPEC, auto STEPS_PER_ENV, typename ACTOR_OPTIMIZER, typename CRITIC_OPTIMIZER, typename RNG>
    void train_hybrid(DEVICE& device,
        DEVICE_EVALUATION& device_evaluation,
        rl::algorithms::PPO<PPO_SPEC>& ppo,
        rl::algorithms::PPO<PPO_SPEC>& ppo_evaluation,
        rl::components::on_policy_runner::Dataset<rl::components::on_policy_runner::DatasetSpecification<OPR_SPEC, STEPS_PER_ENV>>& dataset,
        ACTOR_OPTIMIZER& actor_optimizer,
        CRITIC_OPTIMIZER& critic_optimizer,
        rl::algorithms::ppo::Buffers<PPO_SPEC>& ppo_buffers,
        rl::algorithms::ppo::TrainingBuffersHybrid<PPO_SPEC>& hybrid_buffers,
        typename PPO_SPEC::ACTOR_TYPE::template Buffer<PPO_SPEC::BATCH_SIZE>& actor_buffers,
        typename PPO_SPEC::CRITIC_TYPE::template Buffer<PPO_SPEC::BATCH_SIZE>& critic_buffers,
        RNG& rng){
#ifdef RL_TOOLS_DEBUG_RL_ALGORITHMS_PPO_CHECK_INIT
        utils::assert_exit(device, ppo.initialized, "PPO not initialized");
#endif
        using T = typename PPO_SPEC::T;
        using TI = typename PPO_SPEC::TI;
        static_assert(utils::typing::is_same_v<typename PPO_SPEC::ENVIRONMENT, typename OPR_SPEC::ENVIRONMENT>, "environment mismatch");
        using BUFFER = rl::components::on_policy_runner::Dataset<rl::components::on_policy_runner::DatasetSpecification<OPR_SPEC, STEPS_PER_ENV>>;
        static_assert(BUFFER::STEPS_TOTAL > 0);
        constexpr TI N_EPOCHS = PPO_SPEC::PARAMETERS::N_EPOCHS;
        constexpr TI BATCH_SIZE = PPO_SPEC::BATCH_SIZE;
        constexpr TI N_BATCHES = BUFFER::STEPS_TOTAL/BATCH_SIZE;
        static_assert(N_BATCHES > 0);
        constexpr TI ACTION_DIM = OPR_SPEC::ENVIRONMENT::ACTION_DIM;
        constexpr TI OBSERVATION_DIM = OPR_SPEC::ENVIRONMENT::Observation::DIM;
        constexpr bool NORMALIZE_OBSERVATIONS = PPO_SPEC::PARAMETERS::NORMALIZE_OBSERVATIONS;
//        auto all_observations = NORMALIZE_OBSERVATIONS ? dataset.all_observations_normalized : dataset.all_observations;
//        auto observations = NORMALIZE_OBSERVATIONS ? dataset.observations_normalized : dataset.observations;
        // batch needs observations, original log-probs, advantages
        T policy_kl_divergence = 0; // KL( current || old ) todo: make hyperparameter that swaps the order
        if(PPO_SPEC::PARAMETERS::ADAPTIVE_LEARNING_RATE) {
            auto& last_layer = get_last_layer(ppo.actor);
            copy(device, device, last_layer.log_std.parameters, ppo_buffers.rollout_log_std);
        }
        for(TI epoch_i = 0; epoch_i < N_EPOCHS; epoch_i++){
            // shuffling
            for(TI dataset_i = 0; dataset_i < BUFFER::STEPS_TOTAL; dataset_i++){
                TI sample_index = random::uniform_int_distribution(typename DEVICE::SPEC::RANDOM(), dataset_i, BUFFER::STEPS_TOTAL-1, rng);
                {
                    auto target_row = row(device, dataset.observations, dataset_i);
                    auto source_row = row(device, dataset.observations, sample_index);
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
            for(TI batch_i = 0; batch_i < N_BATCHES; batch_i++){
                T batch_policy_kl_divergence = 0; // KL( current || old ) todo: make hyperparameter that swaps the order
                zero_gradient(device_evaluation, ppo_evaluation.critic);
                zero_gradient(device_evaluation, ppo_evaluation.actor); // has to be reset before accumulating the action-log-std gradient

                auto batch_offset = batch_i * BATCH_SIZE;
                auto batch_observations     = view(device, dataset.observations    , matrix::ViewSpec<BATCH_SIZE, OBSERVATION_DIM>(), batch_offset, 0);
                auto batch_actions_mean     = view(device, dataset.actions_mean    , matrix::ViewSpec<BATCH_SIZE, ACTION_DIM     >(), batch_offset, 0);
                auto batch_actions          = view(device, dataset.actions         , matrix::ViewSpec<BATCH_SIZE, ACTION_DIM     >(), batch_offset, 0);
                auto batch_action_log_probs = view(device, dataset.action_log_probs, matrix::ViewSpec<BATCH_SIZE, 1              >(), batch_offset, 0);
                auto batch_advantages       = view(device, dataset.advantages      , matrix::ViewSpec<BATCH_SIZE, 1              >(), batch_offset, 0);
                auto batch_target_values    = view(device, dataset.target_values   , matrix::ViewSpec<BATCH_SIZE, 1              >(), batch_offset, 0);

                T advantage_mean = 0;
                T advantage_std = 0;
                for(TI batch_step_i = 0; batch_step_i < BATCH_SIZE; batch_step_i++){
                    T advantage = get(batch_advantages, batch_step_i, 0);
                    advantage_mean += advantage;
                    advantage_std += advantage * advantage;
                }
                advantage_mean /= BATCH_SIZE;
                advantage_std /= BATCH_SIZE;

                advantage_std = math::sqrt(device.math, advantage_std - advantage_mean * advantage_mean);
//                add_scalar(device, device.logger, "ppo/advantage/mean", advantage_mean);
//                add_scalar(device, device.logger, "ppo/advantage/std", advantage_std);

                copy(device, device_evaluation, batch_observations, hybrid_buffers.observations);
                forward(device_evaluation, ppo_evaluation.actor, hybrid_buffers.observations, hybrid_buffers.actions, actor_buffers, rng);
                copy(device_evaluation, device, hybrid_buffers.actions, ppo_buffers.current_batch_actions);
//                auto abs_diff = abs_diff(device, batch_actions, buffer.actions);

                auto& last_layer = get_last_layer(ppo.actor);
                auto& last_layer_eval = get_last_layer(ppo_evaluation.actor);
                copy(device_evaluation, device, last_layer_eval.log_std.parameters, last_layer.log_std.parameters);
                copy(device_evaluation, device, last_layer_eval.log_std.gradient, last_layer.log_std.gradient);
                for(TI batch_step_i = 0; batch_step_i < BATCH_SIZE; batch_step_i++){
                    T action_log_prob = 0;
                    for(TI action_i = 0; action_i < ACTION_DIM; action_i++){

                        T current_action = get(ppo_buffers.current_batch_actions, batch_step_i, action_i);
                        T rollout_action = get(batch_actions, batch_step_i, action_i);
                        auto& last_layer = get_last_layer(ppo.actor);
                        T current_action_log_std = get(last_layer.log_std.parameters, 0, action_i);
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

                        T action_diff_by_action_std = (current_action - rollout_action) / current_action_std;
                        action_log_prob += -0.5 * action_diff_by_action_std * action_diff_by_action_std - current_action_log_std - 0.5 * math::log(device.math, 2 * math::PI<T>);
                        set(ppo_buffers.d_action_log_prob_d_action, batch_step_i, action_i, - action_diff_by_action_std / current_action_std);
                        T current_entropy = current_action_log_std + math::log(device.math, 2 * math::PI<T>)/(T)2 + (T)1/(T)2;
                        T current_entropy_loss = -(T)1/BATCH_SIZE * PPO_SPEC::PARAMETERS::ACTION_ENTROPY_COEFFICIENT * current_entropy;
                        // todo: think about possible implementation detail: clipping entropy bonus as well (because it changes the distribution)
                        if(PPO_SPEC::PARAMETERS::LEARN_ACTION_STD){
                            T d_entropy_loss_d_current_action_log_std = -(T)1/BATCH_SIZE * PPO_SPEC::PARAMETERS::ACTION_ENTROPY_COEFFICIENT;
                            auto& last_layer = get_last_layer(ppo.actor);
                            increment(last_layer.log_std.gradient, 0, action_i, d_entropy_loss_d_current_action_log_std);
//                          derivation: d_current_action_log_prob_d_action_log_std
//                          d_current_action_log_prob_d_action_std =  (-action_diff_by_action_std) * (-action_diff_by_action_std)      / action_std - 1 / action_std)
//                          d_current_action_log_prob_d_action_std = ((-action_diff_by_action_std) * (-action_diff_by_action_std) - 1) / action_std)
//                          d_current_action_log_prob_d_action_std = (action_diff_by_action_std * action_diff_by_action_std - 1) / action_std
//                          d_current_action_log_prob_d_action_log_std = (action_diff_by_action_std * action_diff_by_action_std - 1) / action_std * exp(action_log_std)
//                          d_current_action_log_prob_d_action_log_std = (action_diff_by_action_std * action_diff_by_action_std - 1) / action_std * action_std
//                          d_current_action_log_prob_d_action_log_std =  action_diff_by_action_std * action_diff_by_action_std - 1
                            T d_current_action_log_prob_d_action_log_std = action_diff_by_action_std * action_diff_by_action_std - 1;
                            set(ppo_buffers.d_action_log_prob_d_action_log_std, batch_step_i, action_i, d_current_action_log_prob_d_action_log_std);
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
                    T normal_advantage = ratio * advantage;
                    T clipped_advantage = clipped_ratio * advantage;
                    T slippage = 0.0;
                    bool ratio_min_switch = normal_advantage - clipped_advantage <= slippage;
                    T clipped_surrogate = ratio_min_switch ? normal_advantage : clipped_advantage;

                    T d_loss_d_clipped_surrogate = -(T)1/BATCH_SIZE;
                    T d_clipped_surrogate_d_ratio = ratio_min_switch ? advantage : 0;
                    T d_ratio_d_action_log_prob = ratio;
                    T d_loss_d_action_log_prob = d_loss_d_clipped_surrogate * d_clipped_surrogate_d_ratio * d_ratio_d_action_log_prob;
                    for(TI action_i = 0; action_i < ACTION_DIM; action_i++){
                        multiply(ppo_buffers.d_action_log_prob_d_action, batch_step_i, action_i, d_loss_d_action_log_prob);
                        if(PPO_SPEC::PARAMETERS::LEARN_ACTION_STD){
                            T current_d_action_log_prob_d_action_log_std = get(ppo_buffers.d_action_log_prob_d_action_log_std, batch_step_i, action_i);
                            auto& last_layer = get_last_layer(ppo.actor);
                            increment(last_layer.log_std.gradient, 0, action_i, d_loss_d_action_log_prob * current_d_action_log_prob_d_action_log_std);
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
                copy(device, device_evaluation, last_layer.log_std.parameters, last_layer_eval.log_std.parameters);
                copy(device, device_evaluation, last_layer.log_std.gradient, last_layer_eval.log_std.gradient);

                copy(device, device_evaluation, ppo_buffers.d_action_log_prob_d_action, hybrid_buffers.d_action_log_prob_d_action);
                backward(device_evaluation, ppo_evaluation.actor, hybrid_buffers.observations, hybrid_buffers.d_action_log_prob_d_action, actor_buffers);
                copy(device, device_evaluation, batch_target_values, hybrid_buffers.target_values);
//                forward_backward_mse(device_evaluation, ppo_evaluation.critic, hybrid_buffers.observations, hybrid_buffers.target_values, critic_buffers);
                {
                    forward(device_evaluation, ppo_evaluation.critic, hybrid_buffers.observations, critic_buffers, rng);
                    nn::loss_functions::mse::gradient(device_evaluation, output(ppo_evaluation.critic), hybrid_buffers.target_values, hybrid_buffers.d_critic_output);
                    backward(device_evaluation, ppo_evaluation.critic, hybrid_buffers.observations, hybrid_buffers.d_critic_output, critic_buffers);
                }
                step(device_evaluation, actor_optimizer, ppo_evaluation.actor);
                step(device_evaluation, critic_optimizer, ppo_evaluation.critic);
            }
        }
        if(PPO_SPEC::PARAMETERS::ADAPTIVE_LEARNING_RATE) {
            policy_kl_divergence /= N_EPOCHS * N_BATCHES * BATCH_SIZE;
            add_scalar(device, device.logger, "ppo/policy_kl", policy_kl_divergence);
        }
    }

}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
