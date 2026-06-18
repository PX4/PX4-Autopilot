#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_UTILS_EVALUATION_OPERATIONS_GENERIC_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_UTILS_EVALUATION_OPERATIONS_GENERIC_H
/*
 * This file relies on the environments methods hence it should be included after the operations of the environments that it will be used with
 */

#include "../../../math/operations_generic.h"
#include "../../environments/operations_generic.h"

#include "evaluation.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, rl::utils::evaluation::NoData<SPEC>& data){ }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, rl::utils::evaluation::NoData<SPEC>& data){ }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, rl::utils::evaluation::Data<SPEC>& data){
        malloc(device, data.parameters);
        malloc(device, data.terminated);
        malloc(device, data.rewards);
        malloc(device, data.states);
        malloc(device, data.actions);
        malloc(device, data.dt);
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, rl::utils::evaluation::Data<SPEC>& data){
        free(device, data.parameters);
        free(device, data.terminated);
        free(device, data.rewards);
        free(device, data.states);
        free(device, data.actions);
        free(device, data.dt);
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, rl::utils::evaluation::Buffer<SPEC>& buffer){
        malloc(device, buffer.actions);
        malloc(device, buffer.observations);
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, rl::utils::evaluation::Buffer<SPEC>& buffer){
        free(device, buffer.actions);
        free(device, buffer.observations);
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, rl::utils::evaluation::PolicyBuffer<SPEC>& buffer){
        malloc(device, buffer.buffer);
        malloc(device, buffer.policy);
        malloc(device, buffer.policy_state);
        malloc(device, buffer.policy_evaluation_buffers);
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, rl::utils::evaluation::PolicyBuffer<SPEC>& buffer){
        free(device, buffer.buffer);
        free(device, buffer.policy);
        free(device, buffer.policy_state);
        free(device, buffer.policy_evaluation_buffers);
    }
    namespace rl::utils::evaluation{
        template<typename DEVICE, typename TI, typename SPEC>
        RL_TOOLS_FUNCTION_PLACEMENT void set_state(DEVICE& device, rl::utils::evaluation::NoData<SPEC>& data, TI episode_i, TI step_i, const typename SPEC::SPEC::ENVIRONMENT::State& state){}
        template<typename DEVICE, typename TI, typename SPEC>
        RL_TOOLS_FUNCTION_PLACEMENT void set_state(DEVICE& device, rl::utils::evaluation::Data<SPEC>& data, TI episode_i, TI step_i, const typename SPEC::SPEC::ENVIRONMENT::State& state){
            set(device, data.states, state, episode_i, step_i);
        }
        template<typename DEVICE, typename TI, typename SPEC>
        RL_TOOLS_FUNCTION_PLACEMENT void set_parameters(DEVICE& device, rl::utils::evaluation::NoData<SPEC>& data, TI episode_i, const typename SPEC::SPEC::ENVIRONMENT::Parameters& parameters){ }
        template<typename DEVICE, typename TI, typename SPEC>
        RL_TOOLS_FUNCTION_PLACEMENT void set_parameters(DEVICE& device, rl::utils::evaluation::Data<SPEC>& data, TI episode_i, const typename SPEC::SPEC::ENVIRONMENT::Parameters& parameters){
            set(device, data.parameters, parameters, episode_i);
        }
        template<typename DEVICE, typename TI, typename SPEC, typename ACTION_SPEC>
        RL_TOOLS_FUNCTION_PLACEMENT void set_action(DEVICE& device, rl::utils::evaluation::NoData<SPEC>& data, TI step_i, const Matrix<ACTION_SPEC>& actions){}
        template<typename DEVICE, typename TI, typename DATA_SPEC, typename ACTION_SPEC>
        RL_TOOLS_FUNCTION_PLACEMENT void set_action(DEVICE& device, rl::utils::evaluation::Data<DATA_SPEC>& data, TI step_i, const Matrix<ACTION_SPEC>& actions){
            using SPEC = typename DATA_SPEC::SPEC;
            static_assert(ACTION_SPEC::ROWS == SPEC::N_EPISODES);
            static_assert(ACTION_SPEC::COLS == SPEC::ENVIRONMENT::ACTION_DIM);
            for (TI episode_i = 0; episode_i < SPEC::N_EPISODES; episode_i++){
                for (TI action_i = 0; action_i < SPEC::ENVIRONMENT::ACTION_DIM; action_i++) {
                    set(device, data.actions, get(actions, episode_i, action_i), episode_i, step_i, action_i);
                }
            }
        }
        template<typename DEVICE, typename TI, typename SPEC>
        RL_TOOLS_FUNCTION_PLACEMENT void set_dt(DEVICE& device, rl::utils::evaluation::NoData<SPEC>& data, TI episode_i, TI step_i, typename SPEC::SPEC::T dt){ }
        template<typename DEVICE, typename TI, typename SPEC>
        RL_TOOLS_FUNCTION_PLACEMENT void set_dt(DEVICE& device, rl::utils::evaluation::Data<SPEC>& data, TI episode_i, TI step_i, typename SPEC::SPEC::T dt){
            set(device, data.dt, dt, episode_i, step_i);
        }
        template<typename DEVICE, typename TI, typename SPEC>
        RL_TOOLS_FUNCTION_PLACEMENT void set_reward(DEVICE& device, rl::utils::evaluation::NoData<SPEC>& data, TI episode_i, TI step_i, typename SPEC::SPEC::T reward){}
        template<typename DEVICE, typename TI, typename SPEC>
        RL_TOOLS_FUNCTION_PLACEMENT void set_reward(DEVICE& device, rl::utils::evaluation::Data<SPEC>& data, TI episode_i, TI step_i, typename SPEC::SPEC::T reward){
            set(device, data.rewards, reward, episode_i, step_i);
        }
        template<typename DEVICE, typename TI, typename SPEC>
        RL_TOOLS_FUNCTION_PLACEMENT void set_terminated(DEVICE& device, rl::utils::evaluation::NoData<SPEC>& data, TI episode_i, TI step_i, bool terminated){}
        template<typename DEVICE, typename TI, typename SPEC>
        RL_TOOLS_FUNCTION_PLACEMENT void set_terminated(DEVICE& device, rl::utils::evaluation::Data<SPEC>& data, TI episode_i, TI step_i, bool terminated){
            set(device, data.terminated, terminated, episode_i, step_i);
        }
    }
    template<typename DEVICE, typename ENVIRONMENT, typename UI, typename POLICY, typename POLICY_STATE, typename POLICY_EVALUATION_BUFFERS, typename BUFFER_SPEC, typename DATA_SPEC, typename RNG, typename SPEC, template <typename> typename DATA, typename MODE>
    RL_TOOLS_FUNCTION_PLACEMENT void evaluate(DEVICE& device, ENVIRONMENT& env_init, UI& ui, const POLICY& policy, POLICY_STATE& policy_state, POLICY_EVALUATION_BUFFERS& policy_evaluation_buffers, rl::utils::evaluation::Buffer<BUFFER_SPEC>& evaluation_buffers, rl::utils::evaluation::Result<SPEC>& results, DATA<DATA_SPEC>& data, RNG &rng, const Mode<MODE>& mode){
        static_assert(utils::typing::is_same_v<SPEC, typename BUFFER_SPEC::SPEC>);
        static_assert(utils::typing::is_same_v<SPEC, typename DATA_SPEC::SPEC>);
        using T = typename POLICY::TYPE_POLICY::DEFAULT;
        using TI = typename DEVICE::index_t;
        constexpr TI INPUT_DIM = get_last(typename POLICY::INPUT_SHAPE{});
        constexpr TI OUTPUT_DIM = get_last(typename POLICY::OUTPUT_SHAPE{});
        static_assert(ENVIRONMENT::Observation::DIM == INPUT_DIM, "Observation and policy input dimensions must match");
        static_assert(ENVIRONMENT::ACTION_DIM == OUTPUT_DIM || (2*ENVIRONMENT::ACTION_DIM == OUTPUT_DIM), "Action and policy output dimensions must match");
        results.returns_mean = 0;
        results.returns_std = 0;
        results.episode_length_mean = 0;
        results.episode_length_std = 0;
        results.num_terminated = 0;

        auto actions_buffer = view(device, evaluation_buffers.actions, matrix::ViewSpec<SPEC::N_EPISODES, ENVIRONMENT::ACTION_DIM>{});

        ENVIRONMENT envs[SPEC::N_EPISODES];
        typename ENVIRONMENT::State states[SPEC::N_EPISODES];
        typename ENVIRONMENT::Parameters parameters[SPEC::N_EPISODES];
        bool terminated[SPEC::N_EPISODES];
        // using ADJUSTED_POLICY = typename POLICY::template CHANGE_BATCH_SIZE<TI, SPEC::N_EPISODES>;
        // typename ADJUSTED_POLICY::template State<true> policy_state;

        reset(device, policy, policy_state, rng);
        for(TI env_i = 0; env_i < SPEC::N_EPISODES; env_i++){
            auto& env = envs[env_i];
            env = env_init;
            malloc(device, env);
            results.returns[env_i] = 0;
            results.episode_length[env_i] = 0;
            terminated[env_i] = false;
            auto& state = states[env_i];
            auto& current_parameters = parameters[env_i];
            if constexpr(SPEC::DETERMINISTIC_INITIAL_STATE) {
                rl_tools::initial_parameters(device, env, current_parameters);
                rl_tools::initial_state(device, env, current_parameters, state);
            }
            else{
                sample_initial_parameters(device, env, current_parameters, rng);
                sample_initial_state(device, env, current_parameters, state, rng);
            }
            rl::utils::evaluation::set_parameters(device, data, env_i, current_parameters);
        }
        for(TI step_i = 0; step_i < SPEC::STEP_LIMIT; step_i++){
            for(TI env_i = 0; env_i < SPEC::N_EPISODES; env_i++){
                auto observation = row(device, evaluation_buffers.observations, env_i);
                auto& state = states[env_i];
                auto& env_parameters = parameters[env_i];
                rl::utils::evaluation::set_state(device, data, env_i, step_i, states[env_i]);
                auto& env = envs[env_i];
                observe(device, env, env_parameters, state, typename ENVIRONMENT::Observation{}, observation, rng);
            }
            auto observations_chunk = view(device, evaluation_buffers.observations, matrix::ViewSpec<SPEC::N_EPISODES, ENVIRONMENT::Observation::DIM>{}, 0, 0);
            auto actions_buffer_chunk = view(device, evaluation_buffers.actions, matrix::ViewSpec<SPEC::N_EPISODES, ENVIRONMENT::ACTION_DIM>{}, 0, 0);
            auto input_tensor = to_tensor(device, observations_chunk);
            auto output_tensor = to_tensor(device, actions_buffer_chunk);

            evaluate_step(device, policy, input_tensor, policy_state, output_tensor, policy_evaluation_buffers, rng, mode);
            rl::utils::evaluation::set_action(device, data, step_i, actions_buffer);
            for(TI env_i = 0; env_i < SPEC::N_EPISODES; env_i++) {
                if(step_i > 0){
                    if(terminated[env_i]){
                        continue;
                    }
                }
                auto& env = envs[env_i];
                typename ENVIRONMENT::State next_state;
                auto& state = states[env_i];
                auto& env_parameters = parameters[env_i];
                auto action = row(device, actions_buffer, env_i);
                T dt = step(device, env, env_parameters, state, action, next_state, rng);
                if(env_i == 0 && !terminated[env_i]){ // only render the first environment
                    set_state(device, env, env_parameters, ui, state, action);
                    render(device, env, env_parameters, ui);
                }
                rl::utils::evaluation::set_dt(device, data, env_i, step_i, dt);
                T r = reward(device, env, env_parameters, state, action, next_state, rng);
                rl::utils::evaluation::set_reward(device, data, env_i, step_i, r);
                bool terminated_flag = rl_tools::terminated(device, env, env_parameters, next_state, rng);
                if(!terminated[env_i]){
                    // count the final step as well (e.g. termination penalty)
                    results.returns[env_i] += r;
                    results.episode_length[env_i] += 1;
                    results.num_terminated += terminated_flag ? 1 : 0;
                }
                terminated_flag = terminated_flag || terminated[env_i];
                terminated[env_i] = terminated_flag;
                rl::utils::evaluation::set_terminated(device, data, env_i, step_i, terminated_flag);
                if(terminated_flag){
                    set_truncated(device, env, env_parameters, ui, next_state); // this is to sed the terminated flag to the car env
                    render(device, env, env_parameters, ui);
                }
                states[env_i] = next_state;
            }
        }
        for(TI env_i = 0; env_i < SPEC::N_EPISODES; env_i++) {
            auto& env_parameters = parameters[env_i];
            auto& env = envs[env_i];
            auto& state = states[env_i];
            set_truncated(device, env, env_parameters, ui, state); // this is to sed the terminated flag to the car env
            render(device, env, env_parameters, ui);
        }
        for(TI env_i = 0; env_i < SPEC::N_EPISODES; env_i++) {
            auto &env = envs[env_i];
            free(device, env);
        }
        for(TI env_i = 0; env_i < SPEC::N_EPISODES; env_i++){
            results.returns[env_i] = results.returns[env_i];
            results.returns_mean += results.returns[env_i];
            results.returns_std += results.returns[env_i]*results.returns[env_i];
            results.episode_length[env_i] = results.episode_length[env_i];
            results.episode_length_mean += results.episode_length[env_i];
            results.episode_length_std += results.episode_length[env_i]*results.episode_length[env_i];
        }
        results.returns_mean /= SPEC::N_EPISODES;
        results.returns_std = math::sqrt(device.math, math::max(device.math, (T)0, results.returns_std/SPEC::N_EPISODES - results.returns_mean*results.returns_mean));
        results.episode_length_mean /= SPEC::N_EPISODES;
        results.episode_length_std = math::sqrt(device.math, math::max(device.math, (T)0, results.episode_length_std/SPEC::N_EPISODES - results.episode_length_mean*results.episode_length_mean));
        results.share_terminated = results.num_terminated / (T)SPEC::N_EPISODES;
    }
    template<typename DEVICE, typename ENVIRONMENT, typename UI, typename POLICY, typename POLICY_BUFFER_SPEC, typename RESULT_SPEC, template <typename> typename DATA, typename DATA_SPEC, typename RNG, typename MODE>
    RL_TOOLS_FUNCTION_PLACEMENT void evaluate(DEVICE& device, ENVIRONMENT& env_init, UI& ui, const POLICY& policy, rl::utils::evaluation::PolicyBuffer<POLICY_BUFFER_SPEC>& buffer, rl::utils::evaluation::Result<RESULT_SPEC>& results, DATA<DATA_SPEC>& data, RNG &rng, const Mode<MODE>& mode){
        evaluate(device, env_init, ui, policy, buffer.policy_state, buffer.policy_evaluation_buffers, buffer.buffer, results, data, rng, mode);
    }
    template<typename DEVICE, typename ENVIRONMENT, typename UI, typename POLICY, typename POLICY_BUFFER_SPEC, typename RESULT_SPEC, typename RNG, typename MODE>
    RL_TOOLS_FUNCTION_PLACEMENT void evaluate(DEVICE& device, ENVIRONMENT& env_init, UI& ui, const POLICY& policy, rl::utils::evaluation::PolicyBuffer<POLICY_BUFFER_SPEC>& buffer, rl::utils::evaluation::Result<RESULT_SPEC>& results, RNG &rng, const Mode<MODE>& mode){
        rl::utils::evaluation::NoData<rl::utils::evaluation::DataSpecification<RESULT_SPEC>> data;
        evaluate(device, env_init, ui, policy, buffer, results, data, rng, mode);
    }
    template<typename DEVICE, typename ENVIRONMENT, typename UI, typename POLICY, typename RNG, typename SPEC, template <typename> typename DATA, typename DATA_SPEC, typename MODE>
    RL_TOOLS_FUNCTION_PLACEMENT void evaluate(DEVICE& device, ENVIRONMENT& env_init, UI& ui, const POLICY& policy, rl::utils::evaluation::Result<SPEC>& results, DATA<DATA_SPEC>& data, RNG &rng, const Mode<MODE>& mode){
        using TI = typename DEVICE::index_t;
        static constexpr bool DYNAMIC_ALLOCATION = true;
        using ADJUSTED_POLICY = typename POLICY::template CHANGE_BATCH_SIZE<TI, SPEC::N_EPISODES>;
        typename ADJUSTED_POLICY::template State<DYNAMIC_ALLOCATION> policy_state;
        typename ADJUSTED_POLICY::template Buffer<DYNAMIC_ALLOCATION> policy_evaluation_buffers;
        rl::utils::evaluation::Buffer<rl::utils::evaluation::BufferSpecification<SPEC, DYNAMIC_ALLOCATION>> evaluation_buffers;
        malloc(device, policy_state);
        malloc(device, policy_evaluation_buffers);
        malloc(device, evaluation_buffers);
        evaluate(device, env_init, ui, policy, policy_state, policy_evaluation_buffers, evaluation_buffers, results, data, rng, mode);
        free(device, policy_state);
        free(device, policy_evaluation_buffers);
        free(device, evaluation_buffers);
    }
    // proxies without data collection
    template<typename DEVICE, typename ENVIRONMENT, typename UI, typename POLICY, typename POLICY_STATE, typename POLICY_EVALUATION_BUFFERS, typename BUFFER_SPEC, typename RNG, typename SPEC, typename MODE>
    RL_TOOLS_FUNCTION_PLACEMENT void evaluate(DEVICE& device, ENVIRONMENT& env_init, UI& ui, const POLICY& policy, POLICY_STATE& policy_state, POLICY_EVALUATION_BUFFERS& policy_evaluation_buffers, rl::utils::evaluation::Buffer<BUFFER_SPEC>& evaluation_buffers, rl::utils::evaluation::Result<SPEC>& results, RNG &rng, const Mode<MODE>& mode){
        rl::utils::evaluation::NoData<rl::utils::evaluation::DataSpecification<SPEC>> data;
        evaluate(device, env_init, ui, policy, policy_state, policy_evaluation_buffers, evaluation_buffers, results, data, rng, mode);
    }
    template<typename DEVICE, typename ENVIRONMENT, typename UI, typename POLICY, typename RNG, typename SPEC, typename MODE>
    RL_TOOLS_FUNCTION_PLACEMENT void evaluate(DEVICE& device, ENVIRONMENT& env_init, UI& ui, const POLICY& policy, rl::utils::evaluation::Result<SPEC>& results, RNG &rng, const Mode<MODE>& mode){
        rl::utils::evaluation::NoData<rl::utils::evaluation::DataSpecification<SPEC>> data;
        evaluate(device, env_init, ui, policy, results, data, rng, mode);
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif
