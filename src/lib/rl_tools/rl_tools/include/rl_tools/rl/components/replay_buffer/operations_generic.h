#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_COMPONENTS_REPLAY_BUFFER_OPERATIONS_GENERIC_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_COMPONENTS_REPLAY_BUFFER_OPERATIONS_GENERIC_H

#include "replay_buffer.h"
#include "../../../utils/generic/memcpy.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools {
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void update_views(DEVICE& device, rl::components::ReplayBuffer<SPEC>& rb){
        typename DEVICE::index_t offset = 0;
        rb.observations                 = view(device, rb.data, matrix::ViewSpec<SPEC::CAPACITY, SPEC::OBSERVATION_DIM           >{}, 0, offset); offset += SPEC::ASYMMETRIC_OBSERVATIONS ? SPEC::OBSERVATION_DIM : 0;
        rb.observations_privileged      = view(device, rb.data, matrix::ViewSpec<SPEC::CAPACITY, SPEC::OBSERVATION_DIM_PRIVILEGED>{}, 0, offset); offset += SPEC::OBSERVATION_DIM_PRIVILEGED;
        rb.actions                      = view(device, rb.data, matrix::ViewSpec<SPEC::CAPACITY, SPEC::ACTION_DIM                >{}, 0, offset); offset += SPEC::ACTION_DIM;
        rb.rewards                      = view(device, rb.data, matrix::ViewSpec<SPEC::CAPACITY, 1                               >{}, 0, offset); offset += 1;
        rb.next_observations            = view(device, rb.data, matrix::ViewSpec<SPEC::CAPACITY, SPEC::OBSERVATION_DIM           >{}, 0, offset); offset += SPEC::ASYMMETRIC_OBSERVATIONS ? SPEC::OBSERVATION_DIM : 0;
        rb.next_observations_privileged = view(device, rb.data, matrix::ViewSpec<SPEC::CAPACITY, SPEC::OBSERVATION_DIM_PRIVILEGED>{}, 0, offset); offset += SPEC::OBSERVATION_DIM_PRIVILEGED;
        rb.terminated                   = view(device, rb.data, matrix::ViewSpec<SPEC::CAPACITY, 1                               >{}, 0, offset); offset += 1;
        rb.truncated                    = view(device, rb.data, matrix::ViewSpec<SPEC::CAPACITY, 1                               >{}, 0, offset);
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, rl::components::ReplayBuffer<SPEC>& rb) {
        malloc(device, rb.data);
        malloc(device, rb.episode_start);
        update_views(device, rb);
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, rl::components::ReplayBuffer<SPEC>& rb) {
        free(device, rb.data);
        free(device, rb.episode_start);
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, rl::components::ReplayBufferWithStates<SPEC>& rb) {
        malloc(device, (rl::components::ReplayBuffer<typename SPEC::BASE_SPEC>&)rb);
        malloc(device, rb.states);
        malloc(device, rb.next_states);
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, rl::components::ReplayBufferWithStates<SPEC>& rb) {
        free(device, (rl::components::ReplayBuffer<typename SPEC::BASE_SPEC>&)rb);
        free(device, rb.states);
        free(device, rb.next_states);
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void init(DEVICE& device, rl::components::ReplayBuffer<SPEC>& rb) {
        update_views(device, rb);
        rb.full = false;
        rb.position = 0;
        rb.current_episode_start = 0;
    }
    template <typename DEVICE, typename SPEC, typename STATE, typename OBSERVATION_SPEC, typename OBSERVATION_PRIVILEGED_SPEC, typename ACTION_SPEC, typename NEXT_OBSERVATION_SPEC, typename NEXT_OBSERVATION_PRIVILEGED_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void add(DEVICE& device, rl::components::ReplayBuffer<SPEC>& buffer, const STATE& state, const Matrix<OBSERVATION_SPEC>& observation, const Matrix<OBSERVATION_PRIVILEGED_SPEC>& observation_privileged, const Matrix<ACTION_SPEC>& action, typename SPEC::TYPE_POLICY::DEFAULT reward, const STATE& next_state, const Matrix<NEXT_OBSERVATION_SPEC>& next_observation, const Matrix<NEXT_OBSERVATION_PRIVILEGED_SPEC>& next_observation_privileged, const bool terminated, const bool truncated) {
        // todo: change to memcpy?
        for(typename DEVICE::index_t i = 0; i < SPEC::OBSERVATION_DIM; i++) {
            set(buffer.observations, buffer.position, i, get(observation, 0, i));
            set(buffer.next_observations, buffer.position, i, get(next_observation, 0, i));
        }
        for(typename DEVICE::index_t i = 0; i < SPEC::OBSERVATION_DIM_PRIVILEGED; i++) {
            set(buffer.observations_privileged, buffer.position, i, get(observation_privileged, 0, i));
            set(buffer.next_observations_privileged, buffer.position, i, get(next_observation_privileged, 0, i));
        }
        for(typename DEVICE::index_t i = 0; i < SPEC::ACTION_DIM; i++) {
            set(buffer.actions, buffer.position, i, get(action, 0, i));
        }
        set(buffer.rewards, buffer.position, 0, reward);
        set(buffer.terminated, buffer.position, 0, terminated);
        set(buffer.truncated, buffer.position, 0, truncated);
        set(device, buffer.episode_start, buffer.current_episode_start, buffer.position);
        buffer.position = (buffer.position + 1) % SPEC::CAPACITY;
        if(truncated){
            buffer.current_episode_start = buffer.position;
        }
        if(buffer.position == 0 && !buffer.full) {
            buffer.full = true;
        }
//        add_scalar(device, device.logger, "replay_buffer/position", (typename SPEC::T)(buffer.full ? SPEC::CAPACITY : buffer.position), 1000);
    }
    template <typename DEVICE, typename SPEC, typename STATE, typename OBSERVATION_SPEC, typename OBSERVATION_PRIVILEGED_SPEC, typename ACTION_SPEC, typename NEXT_OBSERVATION_SPEC, typename NEXT_OBSERVATION_PRIVILEGED_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void add(DEVICE& device, rl::components::ReplayBufferWithStates<SPEC>& buffer, const STATE& state, const Matrix<OBSERVATION_SPEC>& observation, const Matrix<OBSERVATION_PRIVILEGED_SPEC>& observation_privileged, const Matrix<ACTION_SPEC>& action, const typename SPEC::T reward, const STATE& next_state, const Matrix<NEXT_OBSERVATION_SPEC>& next_observation, const Matrix<NEXT_OBSERVATION_PRIVILEGED_SPEC>& next_observation_privileged, const bool terminated, const bool truncated) {
        set(buffer.states, buffer.position, 0, state);
        set(buffer.next_states, buffer.position, 0, next_state);
        add(device, (rl::components::ReplayBuffer<typename SPEC::BASE_SPEC>&) buffer, state, observation, observation_privileged, action, reward, next_state, next_observation, next_observation_privileged, terminated, truncated);
    }
    template <typename SOURCE_DEVICE, typename TARGET_DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, rl::components::ReplayBuffer<SOURCE_SPEC>& source, rl::components::ReplayBuffer<TARGET_SPEC>& target) {
        copy(source_device, target_device, source.data, target.data);
        copy(source_device, target_device, source.episode_start, target.episode_start);
        target.full = source.full;
        target.position = source.position;
    }

    template <typename DEVICE, typename SPEC_1, typename SPEC_2>
    RL_TOOLS_FUNCTION_PLACEMENT typename SPEC_1::TYPE_POLICY::DEFAULT abs_diff(DEVICE& device, rl::components::ReplayBuffer<SPEC_1>& b1, rl::components::ReplayBuffer<SPEC_2>& b2) {
        typename SPEC_1::TYPE_POLICY::DEFAULT acc = 0;
        acc += abs_diff(device, b1.observations, b2.observations);
        acc += abs_diff(device, b1.observations_privileged, b2.observations_privileged);
        acc += abs_diff(device, b1.actions, b2.actions);
        acc += abs_diff(device, b1.rewards, b2.rewards);
        acc += abs_diff(device, b1.next_observations, b2.next_observations);
        acc += abs_diff(device, b1.next_observations_privileged, b2.next_observations_privileged);
        acc += abs_diff(device, b1.terminated, b2.terminated);
        acc += abs_diff(device, b1.truncated, b2.truncated);
        return acc;
    }
    template <typename DEVICE, typename SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT void recalculate_rewards(DEVICE& device, rl::components::ReplayBufferWithStates<SPEC>& buffer, const typename SPEC::ENVIRONMENT& env, RNG& rng) {
        using TI = typename DEVICE::index_t;
        using ENVIRONMENT = typename SPEC::ENVIRONMENT;
        using STATE = typename ENVIRONMENT::State;
        for(TI position_i = 0; position_i < SPEC::BASE_SPEC::CAPACITY; position_i++){
            auto prev_r = get(buffer.rewards, position_i, 0);
            STATE& state = get(buffer.states, position_i, 0);
            STATE& next_state = get(buffer.next_states, position_i, 0);
            auto action = row(device, buffer.actions, position_i);
            auto r = reward(device, env, state, action, next_state, rng);
            set(buffer.rewards, position_i, 0, r);
        }
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
