#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_UTILS_VALIDATION_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_UTILS_VALIDATION_H

#include "../../math/operations_generic.h"
#include "../../utils/generic/typing.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::utils::validation{
    template <typename T_T, typename T_TI, typename T_ENVIRONMENT>
    struct Specification{
        using T = T_T;
        using TI = T_TI;
        using ENVIRONMENT = T_ENVIRONMENT;
    };

    template <typename T_SPEC, typename T_SPEC::TI SIZE>
    struct EpisodeBuffer{
        using SPEC = T_SPEC;
        MatrixDynamic<matrix::Specification<typename SPEC::ENVIRONMENT::State, typename SPEC::TI, SIZE, 1>> states;
        MatrixDynamic<matrix::Specification<typename SPEC::T, typename SPEC::TI, SIZE, SPEC::ENVIRONMENT::ACTION_DIM>> actions;
        MatrixDynamic<matrix::Specification<typename SPEC::ENVIRONMENT::State, typename SPEC::TI, SIZE, 1>> next_states;
        MatrixDynamic<matrix::Specification<typename SPEC::T, typename SPEC::TI, SIZE, 1>> rewards;
        MatrixDynamic<matrix::Specification<bool, typename SPEC::TI, SIZE, 1>> terminated;
    };

    template <typename T_SPEC, typename T_SPEC::TI T_N_EPISODES, typename T_SPEC::TI T_MAX_EPISODE_LENGTH>
    struct TaskSpecification{
        using SPEC = T_SPEC;
        using T = typename SPEC::T;
        using TI = typename SPEC::TI;
        static constexpr auto N_EPISODES = T_N_EPISODES;
        static constexpr auto MAX_EPISODE_LENGTH = T_MAX_EPISODE_LENGTH;
    };
    template <typename T_SPEC>
    struct Task{
        using SPEC = T_SPEC;
        using T = typename SPEC::T;
        using TI = typename SPEC::TI;
        using ENVIRONMENT = typename SPEC::SPEC::ENVIRONMENT;
        TI step = 0;
        EpisodeBuffer<typename SPEC::SPEC, SPEC::MAX_EPISODE_LENGTH> episode_buffer[SPEC::N_EPISODES];
        MatrixDynamic<matrix::Specification<T, TI, SPEC::N_EPISODES, ENVIRONMENT::Observation::DIM>> observation_buffer;
        MatrixDynamic<matrix::Specification<T, TI, SPEC::N_EPISODES, ENVIRONMENT::ACTION_DIM>> action_buffer;
        TI episode_length[SPEC::N_EPISODES];
        bool terminated[SPEC::N_EPISODES];
        ENVIRONMENT environment[SPEC::N_EPISODES];
        typename ENVIRONMENT::State state[SPEC::N_EPISODES];
        bool completed = false;
    };
    struct Metric{ };
    namespace tasks{
        template <typename T_SPEC>
        struct Default: Task<T_SPEC>{ };
    };
    namespace metrics{
        struct ReturnMean: Metric{};
        struct ReturnStd: Metric{};
        struct EpisodeLengthMean: Metric{};
        struct EpisodeLengthStd: Metric{};
        struct TerminatedFraction: Metric{};
    };
    namespace set{
        template <typename T_CONTENT, typename T_NEXT_COMPONENT>
        struct Component{
            using CONTENT = T_CONTENT;
            using NEXT_COMPONENT = T_NEXT_COMPONENT;
        };
        struct FinalComponent{};
    };
    template <typename NEXT_COMPONENT = set::FinalComponent>
    using DefaultMetrics = set::Component<
            metrics::ReturnMean,
            set::Component<metrics::ReturnStd,
            set::Component<metrics::EpisodeLengthMean,
            set::Component<metrics::EpisodeLengthStd,
            set::Component<metrics::TerminatedFraction,
            NEXT_COMPONENT>>>>>;
}
namespace rl_tools{
    template <typename DEVICE, typename SPEC, typename SPEC::TI SIZE>
    void malloc(DEVICE& device, rl::utils::validation::EpisodeBuffer<SPEC, SIZE>& eb){
        malloc(device, eb.states);
        malloc(device, eb.actions);
        malloc(device, eb.next_states);
        malloc(device, eb.rewards);
        malloc(device, eb.terminated);
    }
    template <typename DEVICE, typename SPEC, typename SPEC::TI SIZE>
    void free(DEVICE& device, rl::utils::validation::EpisodeBuffer<SPEC, SIZE>& eb){
        free(device, eb.states);
        free(device, eb.actions);
        free(device, eb.next_states);
        free(device, eb.rewards);
        free(device, eb.terminated);
    }
    template <typename DEVICE, typename SPEC, typename RNG>
    void reset(DEVICE& device, rl::utils::validation::Task<SPEC>& task, RNG& rng){
        using TI = typename DEVICE::index_t;
        task.completed = false;
        task.step = 0;
        for(TI i = 0; i < SPEC::N_EPISODES; i++){
            sample_initial_state(device, task.environment[i], task.state[i], rng);
            task.episode_length[i] = 0;
            task.terminated[i] = false;
        }
    }
    template <typename DEVICE, typename SPEC, typename RNG>
    void init(DEVICE& device, rl::utils::validation::Task<SPEC>& task, typename SPEC::SPEC::ENVIRONMENT envs[SPEC::N_EPISODES], RNG& rng){
        using TI = typename SPEC::TI;
        for(TI i = 0; i < SPEC::N_EPISODES; i++){
            task.environment[i] = envs[i];
            malloc(device, task.episode_buffer[i]);
        }
        malloc(device, task.observation_buffer);
        malloc(device, task.action_buffer);
        reset(device, task, rng);
    }
    template <typename DEVICE, typename SPEC, typename POLICY, typename POLICY_BUFFERS, typename RNG>
    bool step(DEVICE& device, rl::utils::validation::Task<SPEC>& task, POLICY& policy, POLICY_BUFFERS& buffers, RNG& rng){
        using TI = typename SPEC::TI;
        using T = typename SPEC::T;
        if(!task.completed){
            for(TI episode_i = 0; episode_i < SPEC::N_EPISODES; episode_i++){
                auto observation = row(device, task.observation_buffer, episode_i);
                observe(device, task.environment[episode_i], task.state[episode_i], observation, rng);
            }
            evaluate(device, policy, task.observation_buffer, task.action_buffer, buffers);

            for(TI episode_i = 0; episode_i < SPEC::N_EPISODES; episode_i++){
                auto& eb = task.episode_buffer[episode_i];
                bool terminated_flag = true;
                if(task.step == 0 || !get(eb.terminated, task.step - 1, 0)){
                    auto action = row(device, task.action_buffer, episode_i);
                    auto action_buffer = row(device, eb.actions, task.step);
                    copy(device, device, action, action_buffer);
                    set(eb.states, task.step, 0, task.state[episode_i]);
                    step(device, task.environment[episode_i], task.state[episode_i], action, get(eb.next_states, task.step, 0), rng);
                    T step_reward = reward(device, task.environment[episode_i], task.state[episode_i], action, get(eb.next_states, task.step, 0), rng);
                    set(eb.rewards, task.step, 0, step_reward);

                    task.state[episode_i] = get(eb.next_states, task.step, 0);
                    terminated_flag = terminated(device, task.environment[episode_i], task.state[episode_i], rng);
                    if(!terminated_flag){
                        task.episode_length[episode_i] = task.step + 1;
                    }
                }
                task.terminated[episode_i] = terminated_flag;
                set(eb.terminated, task.step, 0, terminated_flag);
            }
            if(task.step == SPEC::MAX_EPISODE_LENGTH - 1){
                task.completed = true;
            }
            else{
                task.step++;
            }
        }
        return task.completed;
    }
    template <typename DEVICE, typename SPEC>
    void destroy(DEVICE& device, rl::utils::validation::Task<SPEC>& task){
        using TI = typename SPEC::TI;
        for(TI i = 0; i < SPEC::N_EPISODES; i++){
            free(device, task.episode_buffer[i]);
        }
        free(device, task.observation_buffer);
        free(device, task.action_buffer);
    }
    template <typename DEVICE, typename CONTENT, typename NEXT_COMPONENT>
    constexpr typename DEVICE::index_t length(DEVICE& device, rl::utils::validation::set::Component<CONTENT, NEXT_COMPONENT>){
        if constexpr (utils::typing::is_same_v<NEXT_COMPONENT, rl::utils::validation::set::FinalComponent>){
            return 0;
        }
        else{
            return 1 + length(device, NEXT_COMPONENT{});
        }
    }
    template <typename DEVICE, typename CONTENT, typename NEXT_COMPONENT, auto I>
    constexpr auto get(DEVICE& device, rl::utils::validation::set::Component<CONTENT, NEXT_COMPONENT>, Constant<I>){
        using COMPONENT = rl::utils::validation::set::Component<CONTENT, NEXT_COMPONENT>;
        static_assert(I < length(device, COMPONENT{}), "Index out of bounds");
        if constexpr (I == 0){
            return CONTENT{};
        }
        else{
            return get(device, NEXT_COMPONENT{}, Constant<I-1>{});
        }
    }
    auto constexpr name(rl::utils::validation::metrics::ReturnMean){return "ReturnMean";}
    auto constexpr name(rl::utils::validation::metrics::ReturnStd){return "ReturnStd";}
    auto constexpr name(rl::utils::validation::metrics::EpisodeLengthMean){return "EpisodeLengthMean";}
    auto constexpr name(rl::utils::validation::metrics::EpisodeLengthStd){return "EpisodeLengthStd";}
    auto constexpr name(rl::utils::validation::metrics::TerminatedFraction){return "TerminatedFraction";}
    template <typename DEVICE, typename SPEC>
    typename SPEC::T evaluate(DEVICE& device, rl::utils::validation::metrics::ReturnMean, rl::utils::validation::Task<SPEC>& task){
        utils::assert_exit(device, task.completed, "Task is not completed");
        using TI = typename SPEC::TI;
        using T = typename SPEC::T;
        T return_sum = 0;
        for(TI episode_i = 0; episode_i < SPEC::N_EPISODES; episode_i++){
            auto& eb = task.episode_buffer[episode_i];
            for(TI step_i = 0; step_i < task.episode_length[episode_i]; step_i++){
                return_sum += get(eb.rewards, step_i, 0);
                if(step_i == task.episode_length[episode_i] - 1){
                    break;
                }
            }
        }
        return return_sum / SPEC::N_EPISODES;
    }
    template <typename DEVICE, typename SPEC>
    typename SPEC::T evaluate(DEVICE& device, rl::utils::validation::metrics::ReturnStd, rl::utils::validation::Task<SPEC>& task){
        utils::assert_exit(device, task.completed, "Task is not completed");
        using TI = typename SPEC::TI;
        using T = typename SPEC::T;
        T mean = evaluate(device, rl::utils::validation::metrics::ReturnMean{}, task);
        T variance_sum = 0;
        for(TI episode_i = 0; episode_i < SPEC::N_EPISODES; episode_i++){
            T return_sum = 0;
            auto& eb = task.episode_buffer[episode_i];
            for(TI step_i = 0; step_i < task.episode_length[episode_i]; step_i++){
                return_sum += get(eb.rewards, step_i, 0);
            }
            variance_sum += (return_sum - mean)*(return_sum - mean);
        }
        return math::sqrt(device.math, variance_sum / SPEC::N_EPISODES);
    }
    template <typename DEVICE, typename SPEC>
    typename SPEC::T evaluate(DEVICE& device, rl::utils::validation::metrics::EpisodeLengthMean, rl::utils::validation::Task<SPEC>& task){
        utils::assert_exit(device, task.completed, "Task is not completed");
        using TI = typename SPEC::TI;
        using T = typename SPEC::T;
        T return_sum = 0;
        for(TI episode_i = 0; episode_i < SPEC::N_EPISODES; episode_i++){
            return_sum += task.episode_length[episode_i];
        }
        return return_sum / SPEC::N_EPISODES;
    }
    template <typename DEVICE, typename SPEC>
    typename SPEC::T evaluate(DEVICE& device, rl::utils::validation::metrics::EpisodeLengthStd, rl::utils::validation::Task<SPEC>& task){
        utils::assert_exit(device, task.completed, "Task is not completed");
        using TI = typename SPEC::TI;
        using T = typename SPEC::T;
        T mean = evaluate(device, rl::utils::validation::metrics::EpisodeLengthMean{}, task);
        T return_sum = 0;
        for(TI episode_i = 0; episode_i < SPEC::N_EPISODES; episode_i++){
            T diff = task.episode_length[episode_i] - mean;
            return_sum += diff*diff;
        }
        return math::sqrt(device.math, return_sum / SPEC::N_EPISODES);
    }
    template <typename DEVICE, typename SPEC>
    typename SPEC::T evaluate(DEVICE& device, rl::utils::validation::metrics::TerminatedFraction, rl::utils::validation::Task<SPEC>& task){
        utils::assert_exit(device, task.completed, "Task is not completed");
        using TI = typename SPEC::TI;
        using T = typename SPEC::T;
        T terminated = 0;
        for(TI episode_i = 0; episode_i < SPEC::N_EPISODES; episode_i++){
            if(task.episode_length[episode_i] != SPEC::MAX_EPISODE_LENGTH){
                terminated += 1;
            }
        }
        return terminated / SPEC::N_EPISODES;
    }

};
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
