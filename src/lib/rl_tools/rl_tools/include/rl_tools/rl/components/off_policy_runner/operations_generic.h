#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_COMPONENTS_OFF_POLICY_RUNNER_OPERATIONS_GENERIC_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_COMPONENTS_OFF_POLICY_RUNNER_OPERATIONS_GENERIC_H

#include "../../../math/operations_generic.h"
#include "off_policy_runner.h"

#include "../../../rl/components/replay_buffer/operations_generic.h"
#include "../../../utils/generic/tuple/operations_generic.h"

#include "operations_generic_per_env.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, rl::components::off_policy_runner::Buffers<SPEC>& buffers) {
        malloc(device, buffers.observations);
        malloc(device, buffers.actions);
        malloc(device, buffers.next_observations);

        if constexpr(SPEC::PARAMETERS::ASYMMETRIC_OBSERVATIONS){
            malloc(device, buffers.observations_privileged);
            malloc(device, buffers.next_observations_privileged);
        }
        else{
            buffers.observations_privileged = view(device, buffers.observations);
            buffers.next_observations_privileged = view(device, buffers.next_observations);
        }
    }
    template <typename DEVICE, typename T_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void init_views(DEVICE& device, rl::components::off_policy_runner::EpisodeStats<T_SPEC>& episode_stats){
        episode_stats.returns = view<DEVICE, typename decltype(episode_stats.data)::SPEC, T_SPEC::BUFFER_SIZE, 1>(device, episode_stats.data, 0, 0);
        episode_stats.steps   = view<DEVICE, typename decltype(episode_stats.data)::SPEC, T_SPEC::BUFFER_SIZE, 1>(device, episode_stats.data, 0, 1);
    }
    template <typename DEVICE, typename T_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, rl::components::off_policy_runner::EpisodeStats<T_SPEC>& episode_stats) {
        malloc(device, episode_stats.data);
        init_views(device, episode_stats);
    }
    template <typename DEVICE, typename T_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void init(DEVICE& device, rl::components::off_policy_runner::EpisodeStats<T_SPEC>& episode_stats) {
        init_views(device, episode_stats);
        episode_stats.next_episode_i = 0;
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, rl::components::OffPolicyRunner<SPEC> &runner) {
        malloc(device, runner.buffers);
        malloc(device, runner.envs);
        malloc(device, runner.states);
        malloc(device, runner.env_parameters);
        malloc(device, runner.episode_return);
        malloc(device, runner.episode_step);
        malloc(device, runner.truncated);
        malloc(device, runner.replay_buffers);
        malloc(device, runner.episode_stats);
        for (typename DEVICE::index_t env_i = 0; env_i < SPEC::PARAMETERS::N_ENVIRONMENTS; env_i++){
            auto& replay_buffer = get(runner.replay_buffers, 0, env_i);
            malloc(device, replay_buffer);
            init(device, replay_buffer);
            auto& episode_stats = get(runner.episode_stats, 0, env_i);
            malloc(device, episode_stats);
            init(device, episode_stats);
            auto& env = get(runner.envs, 0, env_i);
            malloc(device, env);
            init(device, env);
        }
        malloc(device, runner.policy_states);
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void update_views(DEVICE& device, rl::components::off_policy_runner::SequentialBatch<SPEC>& batch) {
        using BATCH = rl::components::off_policy_runner::SequentialBatch<SPEC>;
        using TI = typename DEVICE::index_t;
        TI offset = 0;
        batch.observations_base                  = view_range(device, batch.observations_actions_base, offset, tensor::ViewSpec<2, BATCH::OBSERVATION_DIM>{}); offset += BATCH::ASYMMETRIC_OBSERVATIONS ? BATCH::OBSERVATION_DIM : 0;
        batch.observations_and_actions_base      = view_range(device, batch.observations_actions_base, offset, tensor::ViewSpec<2, BATCH::OBSERVATION_DIM_PRIVILEGED + BATCH::ACTION_DIM>{});
        batch.observations_privileged_base       = view_range(device, batch.observations_actions_base, offset, tensor::ViewSpec<2, BATCH::OBSERVATION_DIM_PRIVILEGED                    >{}); offset += BATCH::OBSERVATION_DIM_PRIVILEGED;
        batch.actions_base                       = view_range(device, batch.observations_actions_base, offset, tensor::ViewSpec<2, BATCH::     ACTION_DIM                               >{}); offset += BATCH::ACTION_DIM;

        batch.observations_current             = view_range(device, batch.observations_base, 0, tensor::ViewSpec<0, SPEC::SEQUENCE_LENGTHH>{});
        batch.observations_privileged_current  = view_range(device, batch.observations_privileged_base, 0, tensor::ViewSpec<0, SPEC::SEQUENCE_LENGTHH>{});
        batch.observations_and_actions_current = view_range(device, batch.observations_and_actions_base, 0, tensor::ViewSpec<0, SPEC::SEQUENCE_LENGTHH>{});
        batch.actions_current                  = view_range(device, batch.actions_base, 0, tensor::ViewSpec<0, SPEC::SEQUENCE_LENGTHH>{});

        constexpr TI NEXT_OFFSET = SPEC::PARAMETERS::INCLUDE_FIRST_STEP_IN_TARGETS ? 0 : 1;
        constexpr TI NEXT_SIZE = SPEC::PARAMETERS::INCLUDE_FIRST_STEP_IN_TARGETS ? SPEC::PADDED_SEQUENCE_LENGTH : SPEC::SEQUENCE_LENGTHH;
        batch.observations_next._data = view_range(device, batch.observations_base, NEXT_OFFSET, tensor::ViewSpec<0, NEXT_SIZE>{})._data;
        batch.observations_privileged_next._data = view_range(device, batch.observations_privileged_base, NEXT_OFFSET, tensor::ViewSpec<0, NEXT_SIZE>{})._data;
        batch.actions_next._data = view_range(device, batch.actions_base, NEXT_OFFSET, tensor::ViewSpec<0, NEXT_SIZE>{})._data;

        batch.next_reset._data = view_range(device, batch.next_reset_base, NEXT_OFFSET, tensor::ViewSpec<0, NEXT_SIZE>{})._data;
        batch.next_final_step_mask._data = view_range(device, batch.next_final_step_mask_base, NEXT_OFFSET, tensor::ViewSpec<0, NEXT_SIZE>{})._data;
    }
    template <typename DEVICE, typename BATCH_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, rl::components::off_policy_runner::SequentialBatch<BATCH_SPEC>& batch) {
        constexpr typename DEVICE::index_t BATCH_SIZE = BATCH_SPEC::BATCH_SIZE;
        malloc(device, batch.observations_actions_base);
        malloc(device, batch.rewards);
        malloc(device, batch.terminated);
        // malloc(device, batch.truncated);
        malloc(device, batch.reset);
        malloc(device, batch.next_reset_base);
        malloc(device, batch.final_step_mask);
        malloc(device, batch.next_final_step_mask_base);
        update_views(device, batch);
    }
    template <typename DEVICE, typename BATCH_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, rl::components::off_policy_runner::SequentialBatch<BATCH_SPEC>& batch) {
        constexpr typename DEVICE::index_t BATCH_SIZE = BATCH_SPEC::BATCH_SIZE;
        free(device, batch.observations_actions_base);
        free(device, batch.rewards);
        free(device, batch.terminated);
        // free(device, batch.truncated);
        free(device, batch.reset);
        free(device, batch.next_reset_base);
        free(device, batch.final_step_mask);
        free(device, batch.next_final_step_mask_base);
        update_views(device, batch);
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, rl::components::off_policy_runner::Buffers<SPEC>& buffers) {
        free(device, buffers.observations);
        free(device, buffers.actions);
        free(device, buffers.next_observations);
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, rl::components::off_policy_runner::EpisodeStats<SPEC>& episode_stats) {
        free(device, episode_stats.data);
        episode_stats.returns._data = nullptr;
        episode_stats.steps._data = nullptr;
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, rl::components::OffPolicyRunner<SPEC> &runner) {
        free(device, runner.buffers);
        free(device, runner.envs);
        free(device, runner.states);
        free(device, runner.env_parameters);
        free(device, runner.episode_return);
        free(device, runner.episode_step);
        free(device, runner.truncated);
        for (typename DEVICE::index_t env_i = 0; env_i < SPEC::PARAMETERS::N_ENVIRONMENTS; env_i++){
            auto& replay_buffer = get(runner.replay_buffers, 0, env_i);
            free(device, replay_buffer);
            auto& episode_stats = get(runner.episode_stats, 0, env_i);
            free(device, episode_stats);
        }
        free(device, runner.replay_buffers);
        free(device, runner.episode_stats);
        free(device, runner.policy_states);
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void truncate_all(DEVICE& device, rl::components::OffPolicyRunner<SPEC> &runner){
        set_all(device, runner.truncated, true);
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void set_parameters(DEVICE& device, rl::components::OffPolicyRunner<SPEC> &runner, typename SPEC::ENVIRONMENT::Parameters &parameters){
        for (typename DEVICE::index_t env_i = 0; env_i < SPEC::PARAMETERS::N_ENVIRONMENTS; env_i++) {
            auto& parameters_target = get(runner.env_parameters, 0, env_i);
            parameters_target = parameters;
        }
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void init(DEVICE& device, rl::components::OffPolicyRunner<SPEC> &runner){
        truncate_all(device, runner);
        for (typename DEVICE::index_t env_i = 0; env_i < SPEC::PARAMETERS::N_ENVIRONMENTS; env_i++){
            auto& replay_buffer = get(runner.replay_buffers, 0, env_i);
            init(device, replay_buffer);
            auto& episode_stats = get(runner.episode_stats, 0, env_i);
            init(device, episode_stats);
            auto& env = get(runner.envs, 0, env_i);
            init(device, env);
            auto& parameters = get(runner.env_parameters, 0, env_i);
            initial_parameters(device, env, parameters);
        }
        runner.previous_policy_set = false;
        runner.previous_policy = 0;
#ifdef RL_TOOLS_DEBUG_RL_COMPONENTS_OFF_POLICY_RUNNER_CHECK_INIT
        runner.initialized = true;
#endif
    }
    namespace rl::components::off_policy_runner{
        template<typename DEVICE, typename SPEC, typename RNG>
        RL_TOOLS_FUNCTION_PLACEMENT void prologue(DEVICE& device, rl::components::OffPolicyRunner<SPEC>& runner, RNG &rng) {
            using TI = typename DEVICE::index_t;
            for (TI env_i = 0; env_i < SPEC::PARAMETERS::N_ENVIRONMENTS; env_i++) {
                prologue_per_env(device, runner, rng, env_i);
            }
        }
        template<auto POLICY_INDEX, typename DEVICE, typename SPEC, typename POLICY, typename POLICY_BUFFERS, typename RNG>
        RL_TOOLS_FUNCTION_PLACEMENT void interlude(DEVICE& device, rl::components::OffPolicyRunner<SPEC>& runner, POLICY &policy, POLICY_BUFFERS& policy_eval_buffers, RNG& rng){
            using TI = typename DEVICE::index_t;
            constexpr TI BATCH_SIZE = decltype(runner.buffers.actions)::ROWS;
            constexpr TI POLICY_OUTPUT_DIM = get_last(typename POLICY::OUTPUT_SHAPE{});
            auto action_view = view(device, runner.buffers.actions, matrix::ViewSpec<BATCH_SIZE, POLICY_OUTPUT_DIM>{});
            auto observation_view_tensor = to_tensor(device, runner.buffers.observations);
            auto action_view_tensor = to_tensor(device, action_view);
            // static_assert(SPEC::PARAMETERS::N_ENVIRONMENTS == 1); // we assume only one environment here for now, so we can reset the hidden state of the whole batch
            auto& policy_state = get<POLICY_INDEX>(runner.policy_states);
            Mode<mode::sequential::ResetMask<mode::Default<>, mode::sequential::ResetMaskSpecification<decltype(runner.truncated)>>> mode_reset_mask;
            mode_reset_mask.mask = runner.truncated;
            reset(device, policy, policy_state, rng, mode_reset_mask);
            Mode<mode::Rollout<>> mode; // we want stochasticity for exploration
            evaluate_step(device, policy, observation_view_tensor, policy_state, action_view_tensor, policy_eval_buffers, rng, mode);
        }

        template<typename DEVICE, typename SPEC, typename POLICY, typename RNG>
        RL_TOOLS_FUNCTION_PLACEMENT void epilogue(DEVICE& device, rl::components::OffPolicyRunner<SPEC>& runner, const POLICY& policy, RNG& rng){
            using TI = typename DEVICE::index_t;
            for (TI env_i = 0; env_i < SPEC::PARAMETERS::N_ENVIRONMENTS; env_i++){
                epilogue_per_env(device, runner, policy, rng, env_i);
            }
        }
    }
    template<auto POLICY_INDEX, typename DEVICE, typename SPEC, typename POLICY, typename POLICY_BUFFERS, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT void step(DEVICE& device, rl::components::OffPolicyRunner<SPEC>& runner, POLICY& policy, POLICY_BUFFERS& policy_eval_buffers, RNG &rng){
#ifdef RL_TOOLS_DEBUG_RL_COMPONENTS_OFF_POLICY_RUNNER_CHECK_INIT
        utils::assert_exit(device, runner.initialized, "OffPolicyRunner not initialized");
#endif
        using TI = typename DEVICE::index_t;
        constexpr TI POLICY_INPUT_DIM = get_last(typename POLICY::INPUT_SHAPE{});
        constexpr TI POLICY_OUTPUT_DIM = get_last(typename POLICY::OUTPUT_SHAPE{});
        static_assert(POLICY_INPUT_DIM == SPEC::ENVIRONMENT::Observation::DIM, "The policy's input dimension must match the environment's observation dimension.");
//        static_assert(POLICY::OUTPUT_DIM == (SPEC::ENVIRONMENT::ACTION_DIM * (SPEC::STOCHASTIC_POLICY ? 2 : 1)), "The policy's output dimension must match the environment's action dimension.");
        static_assert(POLICY_OUTPUT_DIM == SPEC::ENVIRONMENT::ACTION_DIM || POLICY_OUTPUT_DIM == 2*SPEC::ENVIRONMENT::ACTION_DIM, "The policy's output dimension must match the environment's action dimension.");
        // todo: increase efficiency by removing the double observation of each state
        bool policy_switch = runner.previous_policy_set && (POLICY_INDEX != runner.previous_policy);
        if(policy_switch){
            truncate_all(device, runner);
        }

        rl::components::off_policy_runner::prologue(device, runner, rng);
        rl::components::off_policy_runner::interlude<POLICY_INDEX>(device, runner, policy, policy_eval_buffers, rng);
        rl::components::off_policy_runner::epilogue(device, runner, policy, rng);

        runner.previous_policy = POLICY_INDEX;
        runner.previous_policy_set = true;
    }
    template <bool DETERMINISTIC, typename DEVICE, typename RUNNER_SPEC, typename SPEC, typename BATCH_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT void gather_batch_step(DEVICE& device, rl::components::OffPolicyRunner<RUNNER_SPEC>& runner, rl::components::ReplayBuffer<SPEC>& replay_buffer, rl::components::off_policy_runner::SequentialBatch<BATCH_SPEC>& batch, typename DEVICE::index_t batch_step_i, RNG& rng){
        // note: make sure that the replay_buffer has at least one transition in it;
        using TI = typename DEVICE::index_t;
        constexpr TI PADDED_SEQUENCE_LENGTH = BATCH_SPEC::PADDED_SEQUENCE_LENGTH;
        constexpr TI SEQUENCE_LENGTH = BATCH_SPEC::SEQUENCE_LENGTHH;
        static_assert(PADDED_SEQUENCE_LENGTH >= 2, "PADDED_SEQUENCE_LENGTHSEQUENCE_LENGTH must be at least 2, such that the observation and next observation can be accommodated");
        using PARAMETERS = typename BATCH_SPEC::PARAMETERS;
        // when the replay buffer is wrapping around it starts overwriting elements of some episode
        // hence the beginning of that episode might not be available anymore
        // hence we sample from position + MAX_EPISODE_LENGTH => wrap => position
#if defined(RL_TOOLS_DEBUG) && !defined(__CUDA_ARCH__)
        utils::assert_exit(device, replay_buffer.full || replay_buffer.position > 0, "Replay buffer requires at least one element");
        if constexpr(PARAMETERS::ALWAYS_SAMPLE_FROM_INITIAL_STATE){
            utils::assert_exit(device, replay_buffer.full || replay_buffer.position > RUNNER_SPEC::MAX_EPISODE_LENGTH, "Replay buffer requires at least RUNNER_SPEC::MAX_EPISODE_LENGTH elements");
        }
#endif
        static_assert(!PARAMETERS::ALWAYS_SAMPLE_FROM_INITIAL_STATE || SPEC::CAPACITY >= RUNNER_SPEC::MAX_EPISODE_LENGTH);
        TI eligible_sample_count = replay_buffer.full ? (PARAMETERS::ALWAYS_SAMPLE_FROM_INITIAL_STATE ? (SPEC::CAPACITY - RUNNER_SPEC::MAX_EPISODE_LENGTH) : SPEC::CAPACITY) : replay_buffer.position; // if we sample from the initial state we need to invalidate the steps possibly remain after replay_buffer.position has overwritten the initial steps
        TI sample_index_max = eligible_sample_count - 1;
        TI sample_index = 0;
        // [-----------{current_position}[MAX_EPISODE_LENGTH-----]--------------------------]
        TI current_seq_length = SEQUENCE_LENGTH;
        if constexpr(PARAMETERS::RANDOM_SEQ_LENGTH){
            if(PARAMETERS::ENABLE_NOMINAL_SEQUENCE_LENGTH_PROBABILITY && random::uniform_real_distribution(device.random, 0.0, 1.0, rng) < PARAMETERS::NOMINAL_SEQUENCE_LENGTH_PROBABILITY){
                current_seq_length = SEQUENCE_LENGTH;
            }
            else{
                if constexpr(SEQUENCE_LENGTH > 1) {
                    current_seq_length = random::uniform_int_distribution(device.random, (TI) 1, SEQUENCE_LENGTH-1, rng);
                }
            }
        }
        TI current_seq_step = 0;

        bool previous_step_truncated = false;
        bool previous_padding_step = true;
        auto batch_sample_final_step_mask = view<1>(device, batch.final_step_mask, batch_step_i);
        set_all(device, batch_sample_final_step_mask, false);
        auto batch_sample_next_final_step_mask = view<1>(device, batch.next_final_step_mask_base, batch_step_i);
        set_all(device, batch_sample_next_final_step_mask, false);
        auto batch_sample_reset = view<1>(device, batch.reset, batch_step_i);
        set_all(device, batch_sample_reset, false);
        auto batch_sample_next_reset = view<1>(device, batch.next_reset_base, batch_step_i);
        set_all(device, batch_sample_next_reset, false);
        set(device, batch.reset, true, 0, batch_step_i, 0);
        set(device, batch.next_reset_base, true, 0, batch_step_i, 0);
        set(device, batch.next_reset, true, 0, batch_step_i, 0);
        for(TI seq_step_i=0; seq_step_i < PADDED_SEQUENCE_LENGTH; seq_step_i++){
            if(previous_step_truncated){
                previous_step_truncated = false;
                previous_padding_step = true;

                set(device, batch.next_final_step_mask_base, true, seq_step_i, batch_step_i, 0);
                if(seq_step_i < PADDED_SEQUENCE_LENGTH - 1){
                    set(device, batch.reset, true, seq_step_i, batch_step_i, 0);
                }
                continue;
            }
            if(previous_padding_step){
                if(seq_step_i < PADDED_SEQUENCE_LENGTH - 1){
                    set(device, batch.reset, true, seq_step_i, batch_step_i, 0);
                }
                set(device, batch.next_reset_base, true, seq_step_i, batch_step_i, 0);
                // if previous step was truncated we find a new sequence start
                TI sample_offset = DETERMINISTIC ? batch_step_i : random::uniform_int_distribution(device.random, (TI) 0, sample_index_max, rng);
                if(replay_buffer.full){
                    sample_index = replay_buffer.position + RUNNER_SPEC::MAX_EPISODE_LENGTH + sample_offset;
                    sample_index = sample_index % SPEC::CAPACITY;
                }
                else{
                    sample_index = sample_offset;
                }
                if constexpr(PARAMETERS::ALWAYS_SAMPLE_FROM_INITIAL_STATE){
                    TI new_sample_index = get(device, replay_buffer.episode_start, sample_index);
                    //                    log(device, device.logger, "sample_index: ", sample_index, " => ", new_sample_index);
                    sample_index = new_sample_index;
                }
            }


            auto observation_target_sequence = view<0>(device, batch.observations_base, seq_step_i);
            auto observation_target = view<0>(device, observation_target_sequence, batch_step_i);
            auto observation_source = row(device, replay_buffer.observations, sample_index);
            auto observation_source_tensor = to_tensor(device, observation_source);
            auto observation_source_tensor_squeezed = squeeze(device, observation_source_tensor);
            copy(device, device, observation_source_tensor_squeezed, observation_target);

            if constexpr(SPEC::ASYMMETRIC_OBSERVATIONS){
                auto observation_privileged_target_sequence = view<0>(device, batch.observations_privileged_base, seq_step_i);
                auto observation_privileged_target = view<0>(device, observation_privileged_target_sequence, batch_step_i);
                auto observation_privileged_source = row(device, replay_buffer.observations_privileged, sample_index);
                auto observation_privileged_source_tensor = to_tensor(device, observation_privileged_source);
                auto observation_privileged_source_squeezed = squeeze(device, observation_privileged_source_tensor);
                copy(device, device, observation_privileged_source_squeezed, observation_privileged_target);
            }

            auto action_target_sequence = view<0>(device, batch.actions_base, seq_step_i);
            auto action_target = view<0>(device, action_target_sequence, batch_step_i);
            auto action_source = row(device, replay_buffer.actions, sample_index);
            auto action_source_tensor = to_tensor(device, action_source);
            auto action_source_tensor_squeezed = squeeze(device, action_source_tensor);
            copy(device, device, action_source_tensor_squeezed, action_target);

            if(seq_step_i < PADDED_SEQUENCE_LENGTH - 1){
                set(device, batch.rewards, get(replay_buffer.rewards, sample_index, 0), seq_step_i, batch_step_i, 0);
                set(device, batch.terminated, get(replay_buffer.terminated, sample_index, 0), seq_step_i, batch_step_i, 0);
            }

            bool truncated = get(replay_buffer.truncated, sample_index, 0);

            TI next_sample_index = sample_index + 1;
            if(replay_buffer.full){
                next_sample_index = next_sample_index % SPEC::CAPACITY;
            }
            if(next_sample_index == replay_buffer.position){
                truncated = true;
            }
            if(seq_step_i == PADDED_SEQUENCE_LENGTH-2){
                truncated = true;
            }
            if constexpr(PARAMETERS::RANDOM_SEQ_LENGTH){
                if(current_seq_step == current_seq_length - 1){
                    truncated = true;
                    if(SEQUENCE_LENGTH > 1){
                        if(PARAMETERS::ENABLE_NOMINAL_SEQUENCE_LENGTH_PROBABILITY && random::uniform_real_distribution(device.random, 0.0, 1.0, rng) < PARAMETERS::NOMINAL_SEQUENCE_LENGTH_PROBABILITY){
                            current_seq_length = SEQUENCE_LENGTH;
                        }
                        else{
                            if constexpr(SEQUENCE_LENGTH > 1){
                                current_seq_length = random::uniform_int_distribution(device.random, (TI) 1, SEQUENCE_LENGTH-1, rng);
                            }
                        }
                    }
                    else{
                        current_seq_length = SEQUENCE_LENGTH;
                    }
                }
                if(truncated){
                    current_seq_step = 0;
                }
                else{
                    current_seq_step += 1;
                }
            }

            if(seq_step_i < PADDED_SEQUENCE_LENGTH - 1){
                // set(device, batch.truncated, truncated, seq_step_i, batch_step_i, 0);
            }
            if(truncated){
                if(seq_step_i < PADDED_SEQUENCE_LENGTH - 1){
                    set(device, batch.final_step_mask, true, seq_step_i, batch_step_i, 0);
                }
                if(seq_step_i < PADDED_SEQUENCE_LENGTH-1){
                    auto next_observation_target_sequence = view<0>(device, batch.observations_base, seq_step_i + 1);
                    auto next_observation_target = view<0>(device, next_observation_target_sequence, batch_step_i);
                    auto next_observation_source = row(device, replay_buffer.next_observations, sample_index);
                    auto next_observation_source_tensor = to_tensor(device, next_observation_source);
                    auto next_observation_source_tensor_squeezed = squeeze(device, next_observation_source_tensor);
                    copy(device, device, next_observation_source_tensor_squeezed, next_observation_target);

                    if constexpr(SPEC::ASYMMETRIC_OBSERVATIONS){
                        auto next_observation_privileged_target_sequence = view<0>(device, batch.observations_privileged_base, seq_step_i + 1);
                        auto next_observation_privileged_target = view<0>(device, next_observation_privileged_target_sequence, batch_step_i);
                        auto next_observation_privileged_source = row(device, replay_buffer.next_observations_privileged, sample_index);
                        auto next_observation_privileged_source_tensor = to_tensor(device, next_observation_privileged_source);
                        auto next_observation_privileged_source_squeezed = squeeze(device, next_observation_privileged_source_tensor);
                        copy(device, device, next_observation_privileged_source_squeezed, next_observation_privileged_target);
                    }

                    auto next_action_target_sequence = view<0>(device, batch.actions_base, seq_step_i + 1);
                    auto next_action_target = view<0>(device, next_action_target_sequence, batch_step_i);
                    set_all(device, next_action_target, 0);
                }
            }

            sample_index = next_sample_index;
            previous_padding_step = false;
            previous_step_truncated = truncated;
        }
    }
    template <typename DEVICE, typename SPEC, typename BATCH_SPEC, typename RNG, bool DETERMINISTIC=false>
    RL_TOOLS_FUNCTION_PLACEMENT void gather_batch(DEVICE& device, rl::components::OffPolicyRunner<SPEC>& runner, rl::components::off_policy_runner::SequentialBatch<BATCH_SPEC>& batch, RNG& rng){
        static_assert(utils::typing::is_same_v<SPEC, typename BATCH_SPEC::SPEC>);
        using TI = typename SPEC::TI;
        constexpr TI BATCH_SIZE = BATCH_SPEC::BATCH_SIZE;
        for(TI batch_step_i=0; batch_step_i < BATCH_SIZE; batch_step_i++) {
            TI env_i = DETERMINISTIC ? 0 : random::uniform_int_distribution(typename DEVICE::SPEC::RANDOM(), (TI) 0, (TI)(SPEC::PARAMETERS::N_ENVIRONMENTS - 1), rng);
            auto& replay_buffer = get(runner.replay_buffers, 0, env_i);
            gather_batch_step<DETERMINISTIC>(device, runner, replay_buffer, batch, batch_step_i, rng);
        }
    }
//    template<typename SOURCE_DEVICE, typename TARGET_DEVICE,  typename SOURCE_SPEC, typename TARGET_SPEC>
//    void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, const  rl::components::off_policy_runner::Batch<SOURCE_SPEC>& source, nn_models::mlp::NeuralNetworkBuffersForwardBackward<TARGET_SPEC>& target){
//        copy(source_device, target_device, (nn_models::mlp::NeuralNetworkBuffers<SOURCE_SPEC>&)source, (nn_models::mlp::NeuralNetworkBuffers<TARGET_SPEC>&)target);
//        copy(source_device, target_device, source.d_input, target.d_input);
//        copy(source_device, target_device, source.d_output, target.d_output);
//    }
    template <typename SOURCE_DEVICE, typename TARGET_DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, rl::components::off_policy_runner::SequentialBatch<SOURCE_SPEC>& source, rl::components::off_policy_runner::SequentialBatch<TARGET_SPEC>& target){
        copy(source_device, target_device, source.observations_actions_base, target.observations_actions_base);
        copy(source_device, target_device, source.rewards, target.rewards);
        copy(source_device, target_device, source.terminated, target.terminated);
        // copy(source_device, target_device, source.truncated, target.truncated);
        copy(source_device, target_device, source.reset, target.reset);
        copy(source_device, target_device, source.next_reset_base, target.next_reset_base);
        copy(source_device, target_device, source.final_step_mask, target.final_step_mask);
        copy(source_device, target_device, source.next_final_step_mask_base, target.next_final_step_mask_base);
    }
    template <typename DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT typename SOURCE_SPEC::SPEC::T abs_diff(DEVICE& device, rl::components::off_policy_runner::SequentialBatch<SOURCE_SPEC>& source, rl::components::off_policy_runner::SequentialBatch<TARGET_SPEC>& target) {
        using T = typename SOURCE_SPEC::SPEC::T;
        T acc = 0;
        acc += abs_diff(device, source.observations_actions, target.observations_actions);
        acc += abs_diff(device, source.rewards, target.rewards);
        acc += abs_diff(device, source.terminated, target.terminated);
        acc += abs_diff(device, source.truncated, target.truncated);
        acc += abs_diff(device, source.reset, target.reset);
        acc += abs_diff(device, source.final_step_mask, target.final_step_mask);
        return acc;
    }
    template <typename SOURCE_DEVICE, typename TARGET_DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, rl::components::off_policy_runner::Buffers<SOURCE_SPEC>& source, rl::components::off_policy_runner::Buffers<TARGET_SPEC>& target){
        copy(source_device, target_device, source.observations, target.observations);
        copy(source_device, target_device, source.actions, target.actions);
        copy(source_device, target_device, source.next_observations, target.next_observations);
    }
    template <typename SOURCE_DEVICE, typename TARGET_DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, rl::components::off_policy_runner::EpisodeStats<SOURCE_SPEC>& source, rl::components::off_policy_runner::EpisodeStats<TARGET_SPEC>& target){
        copy(source_device, target_device, source.data, target.data);
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void update_views(DEVICE& device, Matrix<SPEC>& replay_buffers) {
        static_assert(SPEC::ROWS == 1);
        using TI = typename DEVICE::index_t;
        for(TI rb_i = 0; rb_i < SPEC::COLS; rb_i++) {
            auto& rb = get(replay_buffers, 0, rb_i);
            update_views(device, rb);
        }
    }

    template <typename SOURCE_DEVICE, typename TARGET_DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, rl::components::OffPolicyRunner<SOURCE_SPEC>& source, rl::components::OffPolicyRunner<TARGET_SPEC>& target){
        copy(source_device, target_device, source.buffers, target.buffers);
        copy(source_device, target_device, source.states, target.states);
        copy(source_device, target_device, source.episode_return, target.episode_return);
        copy(source_device, target_device, source.episode_step, target.episode_step);
        copy(source_device, target_device, source.truncated, target.truncated);
        copy(source_device, target_device, source.replay_buffers, target.replay_buffers);
        update_views(target_device, target.replay_buffers);
        copy(source_device, target_device, source.episode_stats, target.episode_stats);
        copy(source_device, target_device, source.envs, target.envs);
#ifdef RL_TOOLS_DEBUG_RL_COMPONENTS_OFF_POLICY_RUNNER_CHECK_INIT
        target.initialized = source.initialized;
#endif
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif
