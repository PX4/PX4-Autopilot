#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ALGORITHMS_TD3_OPERATIONS_GENERIC_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ALGORITHMS_TD3_OPERATIONS_GENERIC_H

#include "td3.h"

#include "../../../rl/components/replay_buffer/replay_buffer.h"
#include "../../../rl/components/off_policy_runner/off_policy_runner.h"
#include "../../../nn/nn.h"
#include "../../../nn/layers/td3_sampling/operations_generic.h"
#include "../../../nn_models/sequential/model.h"
#include "../../../utils/polyak/operations_generic.h"
#include "../../../math/operations_generic.h"
#include "../../../utils/generic/memcpy.h"
#include "../../../rl/algorithms/sac/operations_generic.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, rl::algorithms::td3::ActorCritic<SPEC>& actor_critic){
        malloc(device, actor_critic.actor);
        malloc(device, actor_critic.actor_target);
        malloc(device, actor_critic.critics[0]);
        malloc(device, actor_critic.critics[1]);
        malloc(device, actor_critic.critics_target[0]);
        malloc(device, actor_critic.critics_target[1]);
        malloc(device, actor_critic.actor_optimizer);
        malloc(device, actor_critic.critic_optimizers[0]);
        malloc(device, actor_critic.critic_optimizers[1]);
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, rl::algorithms::td3::ActorCritic<SPEC>& actor_critic){
        free(device, actor_critic.actor);
        free(device, actor_critic.actor_target);
        free(device, actor_critic.critics[0]);
        free(device, actor_critic.critics[1]);
        free(device, actor_critic.critics_target[0]);
        free(device, actor_critic.critics_target[1]);
        free(device, actor_critic.actor_optimizer);
        free(device, actor_critic.critic_optimizers[0]);
        free(device, actor_critic.critic_optimizers[1]);
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, rl::algorithms::td3::ActorTrainingBuffers<SPEC>& actor_training_buffers){
        using BUFFERS = rl::algorithms::td3::ActorTrainingBuffers<SPEC>;
        malloc(device, actor_training_buffers.state_action_value_input);
        actor_training_buffers.observations = view_range(device, actor_training_buffers.state_action_value_input,                               0, tensor::ViewSpec<2, BUFFERS::CRITIC_OBSERVATION_DIM>{});
        actor_training_buffers.actions      = view_range(device, actor_training_buffers.state_action_value_input, BUFFERS::CRITIC_OBSERVATION_DIM, tensor::ViewSpec<2, BUFFERS::ACTION_DIM>{});
        malloc(device, actor_training_buffers.state_action_value);
        malloc(device, actor_training_buffers.d_output);
        malloc(device, actor_training_buffers.d_critic_input);
        malloc(device, actor_training_buffers.d_actor_output);
        malloc(device, actor_training_buffers.d_actor_input);
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, rl::algorithms::td3::ActorTrainingBuffers<SPEC>& actor_training_buffers){
        free(device, actor_training_buffers.state_action_value_input);
        actor_training_buffers.observations._data = nullptr;
        actor_training_buffers.actions._data      = nullptr;
        free(device, actor_training_buffers.state_action_value);
        free(device, actor_training_buffers.d_output);
        free(device, actor_training_buffers.d_critic_input);
        free(device, actor_training_buffers.d_actor_output);
        free(device, actor_training_buffers.d_actor_input);
    }

    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, rl::algorithms::td3::CriticTrainingBuffers<SPEC>& critic_training_buffers){
        using BUFFERS = rl::algorithms::td3::CriticTrainingBuffers<SPEC>;
        malloc(device, critic_training_buffers.target_next_action_noise);
        malloc(device, critic_training_buffers.next_state_action_value_input);
        critic_training_buffers.next_observations = view_range(device, critic_training_buffers.next_state_action_value_input,                               0, tensor::ViewSpec<2, BUFFERS::CRITIC_OBSERVATION_DIM>{});
        critic_training_buffers.next_actions      = view_range(device, critic_training_buffers.next_state_action_value_input, BUFFERS::CRITIC_OBSERVATION_DIM, tensor::ViewSpec<2, BUFFERS::ACTION_DIM>{});
        malloc(device, critic_training_buffers.action_value);
        malloc(device, critic_training_buffers.target_action_value);
        malloc(device, critic_training_buffers.next_state_action_value_critic_1);
        malloc(device, critic_training_buffers.next_state_action_value_critic_2);
        malloc(device, critic_training_buffers.d_output);
        malloc(device, critic_training_buffers.d_input);
    }

    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, rl::algorithms::td3::CriticTrainingBuffers<SPEC>& critic_training_buffers){
        free(device, critic_training_buffers.target_next_action_noise);
        free(device, critic_training_buffers.next_state_action_value_input);
        critic_training_buffers.next_observations._data = nullptr;
        critic_training_buffers.next_actions._data = nullptr;
        free(device, critic_training_buffers.action_value);
        free(device, critic_training_buffers.target_action_value);
        free(device, critic_training_buffers.next_state_action_value_critic_1);
        free(device, critic_training_buffers.next_state_action_value_critic_2);
        free(device, critic_training_buffers.d_output);
        free(device, critic_training_buffers.d_input);
    }

    template <typename DEVICE, typename SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT void init(DEVICE& device, rl::algorithms::td3::ActorCritic<SPEC>& actor_critic, RNG& rng){
        init_weights(device, actor_critic.actor   , rng);
        init_weights(device, actor_critic.critics[0], rng);
        init_weights(device, actor_critic.critics[1], rng);
        zero_gradient(device, actor_critic.actor);
        zero_gradient(device, actor_critic.critics[0]);
        zero_gradient(device, actor_critic.critics[1]);
        init(device, actor_critic.actor_optimizer);
        init(device, actor_critic.critic_optimizers[0]);
        init(device, actor_critic.critic_optimizers[1]);
        reset_optimizer_state(device, actor_critic.actor_optimizer, actor_critic.actor);
        reset_optimizer_state(device, actor_critic.critic_optimizers[0], actor_critic.critics[0]);
        reset_optimizer_state(device, actor_critic.critic_optimizers[1], actor_critic.critics[1]);

        copy(device, device, actor_critic.actor, actor_critic.actor_target);
        copy(device, device, actor_critic.critics[0], actor_critic.critics_target[0]);
        copy(device, device, actor_critic.critics[1], actor_critic.critics_target[1]);
    }
    template <typename DEVICE, typename SPEC, typename OUTPUT_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT void target_action_noise(DEVICE& device, const rl::algorithms::td3::ActorCritic<SPEC>& actor_critic, Matrix<OUTPUT_SPEC>& target_action_noise, RNG& rng ) {
        static_assert(OUTPUT_SPEC::COLS == SPEC::ENVIRONMENT::ACTION_DIM);
        for(typename DEVICE::index_t batch_sample_i=0; batch_sample_i < OUTPUT_SPEC::ROWS; batch_sample_i++){
            for(typename DEVICE::index_t action_i=0; action_i < SPEC::ENVIRONMENT::ACTION_DIM; action_i++){
                set(target_action_noise, batch_sample_i, action_i, math::clamp(device.math,
                        random::normal_distribution::sample(typename DEVICE::SPEC::RANDOM(), static_cast<decltype(actor_critic.target_next_action_noise_std)>(0), actor_critic.target_next_action_noise_std, rng),
                        -actor_critic.target_next_action_noise_clip,
                        actor_critic.target_next_action_noise_clip
                ));
            }
        }
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void noisy_next_actions(DEVICE& device, rl::algorithms::td3::CriticTrainingBuffers<SPEC>& training_buffers) {
        using T = typename SPEC::TYPE_POLICY::DEFAULT;
        using TI = typename DEVICE::index_t;
        using BUFFERS = rl::algorithms::td3::CriticTrainingBuffers<SPEC>;
        constexpr TI BATCH_SIZE = BUFFERS::BATCH_SIZE;
        constexpr TI SEQUENCE_LENGTH = BUFFERS::NEXT_SEQUENCE_LENGTH;
        for(TI seq_step_i = 0; seq_step_i < SEQUENCE_LENGTH; seq_step_i++){
            for(TI batch_step_i = 0; batch_step_i < BATCH_SIZE; batch_step_i++){
                for(TI action_i=0; action_i < SPEC::SPEC::ENVIRONMENT::ACTION_DIM; action_i++){
                    T noisy_next_action = get(device, training_buffers.next_actions, seq_step_i, batch_step_i, action_i) + get(device, training_buffers.target_next_action_noise, seq_step_i, batch_step_i, action_i);
                    noisy_next_action = math::clamp<T>(device.math, noisy_next_action, -1, 1);
                    set(device, training_buffers.next_actions, noisy_next_action, seq_step_i, batch_step_i, action_i);
                }
            }
        }
    }
    template <typename DEVICE, typename SPEC, typename BATCH_SPEC, typename TRAINING_BUFFERS_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void target_action_values(DEVICE& device, const rl::algorithms::td3::ActorCritic<SPEC>& actor_critic, rl::components::off_policy_runner::SequentialBatch<BATCH_SPEC>& batch, rl::algorithms::td3::CriticTrainingBuffers<TRAINING_BUFFERS_SPEC>& training_buffers) {
        using T = typename SPEC::TYPE_POLICY::DEFAULT;
        using TI = typename DEVICE::index_t;
        constexpr TI BATCH_SIZE = BATCH_SPEC::BATCH_SIZE;
        constexpr TI SEQUENCE_LENGTH = BATCH_SPEC::SEQUENCE_LENGTHH;
        using BUFFERS = rl::algorithms::td3::CriticTrainingBuffers<TRAINING_BUFFERS_SPEC>;
        static_assert(BATCH_SIZE == BUFFERS::BATCH_SIZE);
        constexpr auto OBSERVATION_DIM = SPEC::ENVIRONMENT::Observation::DIM;
        constexpr auto ACTION_DIM = SPEC::ENVIRONMENT::ACTION_DIM;
        constexpr TI TARGET_OFFSET = BATCH_SPEC::PARAMETERS::INCLUDE_FIRST_STEP_IN_TARGETS ? 1 : 0;
        for(TI seq_step_i = 0; seq_step_i < SEQUENCE_LENGTH; seq_step_i++){
            for(TI batch_step_i = 0; batch_step_i < BATCH_SIZE; batch_step_i++){
                T value_critic_1 = get(device, training_buffers.next_state_action_value_critic_1, seq_step_i + TARGET_OFFSET, batch_step_i, 0);
                T value_critic_2 = get(device, training_buffers.next_state_action_value_critic_2, seq_step_i + TARGET_OFFSET, batch_step_i, 0);
                T min_next_state_action_value = math::min(device.math, value_critic_1, value_critic_2);
                T reward = get(device, batch.rewards, seq_step_i, batch_step_i, 0);
                bool terminated = get(device, batch.terminated, seq_step_i, batch_step_i, 0);
                T future_value = SPEC::PARAMETERS::IGNORE_TERMINATION || !terminated ? actor_critic.gamma * min_next_state_action_value : 0;
                T current_target_action_value = reward + future_value;
                set(device, training_buffers.target_action_value, current_target_action_value, seq_step_i, batch_step_i, 0); // todo: improve pitch of target action values etc. (by transformig it into row vectors instead of column vectors)
            }
        }
    }
    template <typename DEVICE, typename SPEC, typename CRITIC_TYPE, typename BATCH_SPEC, typename OPTIMIZER, typename ACTOR_BUFFERS, typename ACTOR_TARGET_BUFFERS, typename CRITIC_BUFFERS, typename CRITIC_TARGET_BUFFERS, typename TRAINING_BUFFERS_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT void train_critic(DEVICE& device, const rl::algorithms::td3::ActorCritic<SPEC>& actor_critic, CRITIC_TYPE& critic, rl::components::off_policy_runner::SequentialBatch<BATCH_SPEC>& batch, OPTIMIZER& optimizer, ACTOR_BUFFERS& actor_buffers, ACTOR_TARGET_BUFFERS& actor_target_buffers, CRITIC_BUFFERS& critic_buffers, CRITIC_TARGET_BUFFERS& critic_target_buffers, rl::algorithms::td3::CriticTrainingBuffers<TRAINING_BUFFERS_SPEC>& training_buffers, RNG& rng) {
        // requires training_buffers.target_next_action_noise to be populated
        using T = typename SPEC::TYPE_POLICY::DEFAULT;
        using TI = typename DEVICE::index_t;
        constexpr TI SEQUENCE_LENGTH = BATCH_SPEC::SEQUENCE_LENGTHH;
        constexpr TI BATCH_SIZE = BATCH_SPEC::BATCH_SIZE;
        static_assert(SPEC::PARAMETERS::SEQUENCE_LENGTH == SEQUENCE_LENGTH, "Specification SEQUENCE_LENGTH should be equal to the batch sequence length");
//        static_assert(BATCH_SIZE == SPEC::PARAMETERS::CRITIC_BATCH_SIZE);
//        static_assert(BATCH_SIZE == CRITIC_BUFFERS::BATCH_SIZE);
//        static_assert(BATCH_SIZE == ACTOR_BUFFERS::BATCH_SIZE);

        constexpr TI TARGET_SEQUENCE_LENGTH = SPEC::PARAMETERS::SEQUENCE_LENGTH + (BATCH_SPEC::PARAMETERS::INCLUDE_FIRST_STEP_IN_TARGETS ? 1 : 0);
        static_assert(SPEC::PARAMETERS::MASK_NON_TERMINAL, "We currently assume that training is only performed on final steps. Otherwise there might be areas of the batch that are undefined memory (the current step observation is not set at the end of a sequence (in that step only the next observation is set). Additionally, the calculation of the target values assumes that there is no training on the last step because the target values are moved one step backwards to match the current MSBE");

        zero_gradient(device, critic);

        using NEXT_RESET_MODE_SPEC = nn::layers::gru::ResetModeSpecification<TI, decltype(batch.next_reset)>;
        using NEXT_RESET_MODE = nn::layers::gru::ResetMode<mode::Default<>, NEXT_RESET_MODE_SPEC>;
        Mode<NEXT_RESET_MODE> next_reset_mode;
        next_reset_mode.reset_container = batch.next_reset;
        evaluate(device, actor_critic.actor_target, batch.observations_next, training_buffers.next_actions, actor_target_buffers, rng, next_reset_mode);
        noisy_next_actions(device, training_buffers);
        if constexpr(SPEC::PARAMETERS::MASK_NON_TERMINAL){
            mask_actions(device, batch.actions_next, training_buffers.next_actions, batch.next_final_step_mask, true);
        }
        copy(device, device, batch.observations_privileged_next, training_buffers.next_observations);
        evaluate(device, actor_critic.critics_target[0], training_buffers.next_state_action_value_input, training_buffers.next_state_action_value_critic_1, critic_target_buffers, rng, next_reset_mode);
        evaluate(device, actor_critic.critics_target[1], training_buffers.next_state_action_value_input, training_buffers.next_state_action_value_critic_2, critic_target_buffers, rng, next_reset_mode);

        target_action_values(device, actor_critic, batch, training_buffers);
        using RESET_MODE_SPEC = nn::layers::gru::ResetModeSpecification<TI, decltype(batch.reset)>;
        using RESET_MODE = nn::layers::gru::ResetMode<mode::Default<>, RESET_MODE_SPEC>;
        Mode<RESET_MODE> reset_mode;
        reset_mode.reset_container = batch.reset;
        forward(device, critic, batch.observations_and_actions_current, critic_buffers, rng, reset_mode);
        {
            T loss_weight = 1;
            if constexpr(SPEC::PARAMETERS::MASK_NON_TERMINAL){
                T num_final_steps = cast_reduce_sum<T>(device, batch.final_step_mask);
                utils::assert_exit(device, num_final_steps > 0, "No reset in critic training");
                loss_weight *= SEQUENCE_LENGTH * BATCH_SIZE / num_final_steps; // reweight the loss by the number of non-masked outputs
            }
            auto critic_output = output(device, critic);
            auto critic_output_matrix_view = matrix_view(device, critic_output);
            auto target_action_value_matrix_view = matrix_view(device, training_buffers.target_action_value);
            auto d_output_matrix_view = matrix_view(device, training_buffers.d_output);
            nn::loss_functions::mse::gradient(device, critic_output_matrix_view, target_action_value_matrix_view, d_output_matrix_view, loss_weight);
            if constexpr(SPEC::PARAMETERS::MASK_NON_TERMINAL){
                mask_gradient(device, training_buffers.d_output, batch.final_step_mask, true);
            }
            { // logging
                if constexpr(SPEC::PARAMETERS::MASK_NON_TERMINAL){
                    // for the loss and average value calculation
                    auto output_temp = output(device, critic);
                    mask_gradient(device, output_temp, batch.final_step_mask, true);
                    mask_gradient(device, training_buffers.target_action_value, batch.final_step_mask, true);
                }
                T loss = nn::loss_functions::mse::evaluate(device, critic_output_matrix_view, target_action_value_matrix_view, loss_weight);
                add_scalar(device, device.logger, "critic_loss", loss, 1000);
                add_scalar(device, device.logger, "critic_value", get(critic_output_matrix_view, 0, 0), 1000);
            }
        }
        backward(device, critic, batch.observations_and_actions_current, training_buffers.d_output, critic_buffers, reset_mode);
        step(device, optimizer, critic);
    }
    template <typename DEVICE, typename SPEC, typename BATCH_SPEC, typename OPTIMIZER, typename ACTOR_BUFFERS, typename CRITIC_BUFFERS, typename TRAINING_BUFFER_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT void train_actor(DEVICE& device, rl::algorithms::td3::ActorCritic<SPEC>& actor_critic, rl::components::off_policy_runner::SequentialBatch<BATCH_SPEC>& batch, OPTIMIZER& optimizer, ACTOR_BUFFERS& actor_buffers, CRITIC_BUFFERS& critic_buffers, rl::algorithms::td3::ActorTrainingBuffers<TRAINING_BUFFER_SPEC>& training_buffers, RNG& rng) {
        using T = typename SPEC::TYPE_POLICY::DEFAULT;
        using TI = typename DEVICE::index_t;
        constexpr TI BATCH_SIZE = BATCH_SPEC::BATCH_SIZE;
        static_assert(BATCH_SIZE == SPEC::PARAMETERS::ACTOR_BATCH_SIZE);
//        static_assert(BATCH_SIZE == SPEC::PARAMETERS::ACTOR_BATCH_SIZE);
//        static_assert(BATCH_SIZE == CRITIC_BUFFERS::BATCH_SIZE);
//        static_assert(BATCH_SIZE == ACTOR_BUFFERS::BATCH_SIZE);
        constexpr TI ACTION_DIM = SPEC::ENVIRONMENT::ACTION_DIM;
        static_assert(get_last(typename SPEC::ACTOR_TYPE::OUTPUT_SHAPE{}) == ACTION_DIM);

        constexpr TI CRITIC_INPUT_DIM = get_last(typename SPEC::CRITIC_TYPE::INPUT_SHAPE{});

        using RESET_MODE_SPEC = nn::layers::gru::ResetModeSpecification<TI, decltype(batch.reset)>;
        using RESET_MODE = nn::layers::gru::ResetMode<mode::Default<>, RESET_MODE_SPEC>;
        Mode<RESET_MODE> reset_mode;
        reset_mode.reset_container = batch.reset;

        forward(device, actor_critic.actor, batch.observations_current, training_buffers.actions, actor_buffers, rng, reset_mode);
        if constexpr(SPEC::PARAMETERS::MASK_NON_TERMINAL){
            mask_actions(device, batch.actions_current, training_buffers.actions, batch.final_step_mask, true);
        }
        copy(device, device, batch.observations_privileged_current, training_buffers.observations);
        auto& critic = actor_critic.critics[0];
        forward(device, critic, training_buffers.state_action_value_input, training_buffers.state_action_value, critic_buffers, rng, reset_mode);
        if constexpr(SPEC::PARAMETERS::MASK_NON_TERMINAL) {
            T num_final_steps = cast_reduce_sum<T>(device, batch.final_step_mask);
            utils::assert_exit(device, num_final_steps > 0, "No reset in critic training");
            set_all(device, training_buffers.d_output, (T)-1/num_final_steps); // we only take the mean over the non-masked outputs
            mask_gradient(device, training_buffers.d_output, batch.final_step_mask, true);
        }
        else{
            set_all(device, training_buffers.d_output, (T)-1/(BATCH_SIZE*SPEC::PARAMETERS::SEQUENCE_LENGTH)); // we take the mean over the batch size and sequence length
        }
        backward_input(device, critic, training_buffers.d_output, training_buffers.d_critic_input, critic_buffers, reset_mode);
        auto d_actor_output = view_range(device, training_buffers.d_critic_input, CRITIC_INPUT_DIM - ACTION_DIM, tensor::ViewSpec<2, ACTION_DIM>{});
        if constexpr(SPEC::PARAMETERS::MASK_NON_TERMINAL) {
            mask_gradient(device, d_actor_output, batch.final_step_mask, true);
        }

        zero_gradient(device, actor_critic.actor);
        backward(device, actor_critic.actor, batch.observations_current, d_actor_output, actor_buffers, reset_mode);

        step(device, optimizer, actor_critic.actor);
    }

    namespace rl::algorithms::td3{
        template<typename DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC>
        RL_TOOLS_FUNCTION_PLACEMENT void update_target_module(DEVICE& device, const  nn::layers::dense::LayerForward<SOURCE_SPEC>& source, nn::layers::dense::LayerForward<TARGET_SPEC>& target, typename SOURCE_SPEC::TYPE_POLICY::DEFAULT polyak) {
            rl_tools::utils::polyak::update(device, source.weights.parameters, target.weights.parameters, polyak);
            rl_tools::utils::polyak::update(device, source.biases.parameters , target.biases.parameters , polyak);
        }
        template<typename DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC>
        RL_TOOLS_FUNCTION_PLACEMENT void update_target_module(DEVICE& device, const  nn::layers::td3_sampling::LayerForward<SOURCE_SPEC>& source, nn::layers::td3_sampling::LayerForward<TARGET_SPEC>& target, typename SOURCE_SPEC::TYPE_POLICY::DEFAULT polyak) { }
        template<typename DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC>
        RL_TOOLS_FUNCTION_PLACEMENT void update_target_module(DEVICE& device, const  nn::layers::gru::LayerForward<SOURCE_SPEC>& source, nn::layers::gru::LayerForward<TARGET_SPEC>& target, typename SOURCE_SPEC::TYPE_POLICY::DEFAULT polyak) {
            rl_tools::utils::polyak::update(device, source.weights_input.parameters, target.weights_input.parameters, polyak);
            rl_tools::utils::polyak::update(device, source.biases_input.parameters, target.biases_input.parameters, polyak);
            rl_tools::utils::polyak::update(device, source.weights_hidden.parameters, target.weights_hidden.parameters, polyak);
            rl_tools::utils::polyak::update(device, source.biases_hidden.parameters, target.biases_hidden.parameters, polyak);
            rl_tools::utils::polyak::update(device, source.initial_hidden_state.parameters, target.initial_hidden_state.parameters, polyak);
        }
        template<typename T, typename DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC>
        RL_TOOLS_FUNCTION_PLACEMENT void update_target_module(DEVICE& device, const  nn_models::mlp::NeuralNetworkForward<SOURCE_SPEC>& source, nn_models::mlp::NeuralNetworkForward<TARGET_SPEC>& target, T polyak) {
            using TargetNetworkType = nn_models::mlp::NeuralNetworkForward<TARGET_SPEC>;
            update_target_module(device, source.input_layer, target.input_layer, polyak);
            for(typename DEVICE::index_t layer_i=0; layer_i < TargetNetworkType::NUM_HIDDEN_LAYERS; layer_i++){
                update_target_module(device, source.hidden_layers[layer_i], target.hidden_layers[layer_i], polyak);
            }
            update_target_module(device, source.output_layer, target.output_layer, polyak);
        }
        template<typename T, typename DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC>
        RL_TOOLS_FUNCTION_PLACEMENT void update_target_module(DEVICE& device, const  nn_models::sequential::ModuleForward<SOURCE_SPEC>& source, nn_models::sequential::ModuleForward<TARGET_SPEC>& target, T polyak) {
            update_target_module(device, source.content, target.content, polyak);
            if constexpr(!rl_tools::utils::typing::is_same_v<typename SOURCE_SPEC::NEXT_MODULE, nn_models::sequential::OutputModule>){
                update_target_module(device, source.next_module, target.next_module, polyak);
            }
        }
    }

    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void update_critic_targets(DEVICE& device, rl::algorithms::td3::ActorCritic<SPEC>& actor_critic) {
        rl::algorithms::td3::update_target_module(device, actor_critic.critics[0], actor_critic.critics_target[0], SPEC::PARAMETERS::CRITIC_POLYAK);
        rl::algorithms::td3::update_target_module(device, actor_critic.critics[1], actor_critic.critics_target[1], SPEC::PARAMETERS::CRITIC_POLYAK);
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void update_actor_target(DEVICE& device, rl::algorithms::td3::ActorCritic<SPEC>& actor_critic) {
        rl::algorithms::td3::update_target_module(device, actor_critic.actor, actor_critic.actor_target, SPEC::PARAMETERS::ACTOR_POLYAK);
    }

    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT bool is_nan(DEVICE& device, rl::algorithms::td3::ActorCritic<SPEC>& ac) {
        bool found_nan = false;
        found_nan = found_nan || is_nan(device, ac.actor);
        found_nan = found_nan || is_nan(device, ac.critic_1);
        found_nan = found_nan || is_nan(device, ac.critic_2);
        found_nan = found_nan || is_nan(device, ac.actor_target);
        found_nan = found_nan || is_nan(device, ac.critic_target_1);
        found_nan = found_nan || is_nan(device, ac.critic_target_2);
        return found_nan;
    }
    template <typename SOURCE_DEVICE, typename TARGET_DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, rl::algorithms::td3::ActorCritic<SOURCE_SPEC>& source, rl::algorithms::td3::ActorCritic<TARGET_SPEC>& target){
        copy(source_device, target_device, source.actor   , target.actor);
        copy(source_device, target_device, source.critic_1, target.critic_1);
        copy(source_device, target_device, source.critic_2, target.critic_2);

        copy(source_device, target_device, source.actor_target   , target.actor_target);
        copy(source_device, target_device, source.critic_target_1, target.critic_target_1);
        copy(source_device, target_device, source.critic_target_2, target.critic_target_2);
    }
    template <typename SOURCE_DEVICE, typename TARGET_DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, rl::algorithms::td3::CriticTrainingBuffers<SOURCE_SPEC>& source, rl::algorithms::td3::CriticTrainingBuffers<TARGET_SPEC>& target){
        copy(source_device, target_device, source.target_next_action_noise, target.target_next_action_noise);
        copy(source_device, target_device, source.next_state_action_value_input, target.next_state_action_value_input);
        copy(source_device, target_device, source.target_action_value, target.target_action_value);
        copy(source_device, target_device, source.next_state_action_value_critic_1, target.next_state_action_value_critic_1);
        copy(source_device, target_device, source.next_state_action_value_critic_2, target.next_state_action_value_critic_2);
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif
