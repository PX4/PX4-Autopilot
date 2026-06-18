#include "../../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ENVIRONMENTS_L2F_METRICS_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENTS_L2F_METRICS_H

#include <rl_tools/rl/utils/validation.h>

#include <string>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::utils::validation{
    namespace metrics{
        template <typename T_TI, T_TI T_DISTANCE_MM>
        struct SettlingFractionPosition: Metric{
            using TI = T_TI;
            static constexpr TI DISTANCE_MM = T_DISTANCE_MM;
        };
        namespace multirotor{
            enum MultirotorQuantity{
                POSITION,
                ANGLE,
                LINEAR_VELOCITY,
                ANGULAR_VELOCITY,
                ANGULAR_ACCELERATION,
                ACTION,
                ACTION_RELATIVE,
            };
        }
        template <enum multirotor::MultirotorQuantity T_QUANTITY, typename T_TI, T_TI T_START_STEP = 0>
        struct MeanErrorMean: Metric{
            using TI = T_TI;
            static constexpr enum multirotor::MultirotorQuantity QUANTITY = T_QUANTITY;
            static constexpr TI START_STEP = T_START_STEP;
        };
        template <enum multirotor::MultirotorQuantity T_QUANTITY, typename T_TI, T_TI T_START_STEP = 0>
        struct MaxErrorMean: Metric{
            using TI = T_TI;
            static constexpr enum multirotor::MultirotorQuantity QUANTITY = T_QUANTITY;
            static constexpr TI START_STEP = T_START_STEP;
        };
        template <enum multirotor::MultirotorQuantity T_QUANTITY, typename T_TI, T_TI T_START_STEP = 0>
        struct MaxErrorStd: Metric{
            using TI = T_TI;
            static constexpr enum multirotor::MultirotorQuantity QUANTITY = T_QUANTITY;
            static constexpr TI START_STEP = T_START_STEP;
        };
    };
}
namespace rl_tools{
    template <typename T_TI, T_TI T_DISTANCE_MM>
    auto constexpr name(rl::utils::validation::metrics::SettlingFractionPosition<T_TI, T_DISTANCE_MM>){
        return std::string("SettlingFractionPosition(") + std::to_string(T_DISTANCE_MM) + std::string("mm)");
    }
    namespace rl::utils::validation::metrics::multirotor {
        auto constexpr name(rl::utils::validation::metrics::multirotor::MultirotorQuantity QUANTITY) {
            switch (QUANTITY) {
                case rl::utils::validation::metrics::multirotor::POSITION:
                    return "Position";
                case rl::utils::validation::metrics::multirotor::ANGLE:
                    return "Angle";
                case rl::utils::validation::metrics::multirotor::LINEAR_VELOCITY:
                    return "LinearVelocity";
                case rl::utils::validation::metrics::multirotor::ANGULAR_VELOCITY:
                    return "AngularVelocity";
                case rl::utils::validation::metrics::multirotor::ANGULAR_ACCELERATION:
                    return "AngularAcceleration";
                case rl::utils::validation::metrics::multirotor::ACTION:
                    return "Action";
                case rl::utils::validation::metrics::multirotor::ACTION_RELATIVE:
                    return "ActionRelative";
                default:
                    return "Unknown";
            }
        }
    }

    template <enum rl::utils::validation::metrics::multirotor::MultirotorQuantity T_QUANTITY, typename T_TI, T_TI T_START_STEP>
    auto constexpr name(rl::utils::validation::metrics::MeanErrorMean<T_QUANTITY, T_TI, T_START_STEP>){
        return std::string("MeanErrorMean(") + rl::utils::validation::metrics::multirotor::name(T_QUANTITY) + std::string(", after ") + std::to_string(T_START_STEP) + std::string(" steps)");
    }
    template <enum rl::utils::validation::metrics::multirotor::MultirotorQuantity T_QUANTITY, typename T_TI, T_TI T_START_STEP>
    auto constexpr name(rl::utils::validation::metrics::MaxErrorMean<T_QUANTITY, T_TI, T_START_STEP>){
        return std::string("MaxErrorMean(") + rl::utils::validation::metrics::multirotor::name(T_QUANTITY) + std::string(", after ") + std::to_string(T_START_STEP) + std::string(" steps)");
    }
    template <enum rl::utils::validation::metrics::multirotor::MultirotorQuantity T_QUANTITY, typename T_TI, T_TI T_START_STEP>
    auto constexpr name(rl::utils::validation::metrics::MaxErrorStd<T_QUANTITY, T_TI, T_START_STEP>){
        return std::string("MaxErrorStd(") + rl::utils::validation::metrics::multirotor::name(T_QUANTITY) + std::string(", after ") + std::to_string(T_START_STEP) + std::string(" steps)");
    }
    template <typename DEVICE, typename SPEC, typename T_TI, T_TI T_DISTANCE_MM>
    typename SPEC::T evaluate(DEVICE& device, rl::utils::validation::metrics::SettlingFractionPosition<T_TI, T_DISTANCE_MM>, rl::utils::validation::Task<SPEC>& task){
        utils::assert_exit(device, task.completed, "Task is not completed");
        using TI = typename SPEC::TI;
        using T = typename SPEC::T;
        T settled = 0;
        for(TI episode_i = 0; episode_i < SPEC::N_EPISODES; episode_i++){
            auto& eb = task.episode_buffer[episode_i];
            if(task.episode_length[episode_i] == SPEC::MAX_EPISODE_LENGTH){
                auto& last_state = get(eb.next_states, task.episode_length[episode_i] - 1, 0);
                T distance = 0;
                distance += last_state.position[0] * last_state.position[0];
                distance += last_state.position[1] * last_state.position[1];
                distance += last_state.position[2] * last_state.position[2];
                distance = math::sqrt(device.math, distance);
                if(distance < ((T)T_DISTANCE_MM / (T)1000.0)){
                    settled += 1;
                }
            }
        }
        return settled / SPEC::N_EPISODES;
    }
    namespace rl::utils::validation::metrics::multirotor{
        template <typename DEVICE, typename ENV_SPEC, typename STATE, typename ACTION_SPEC>
        typename STATE::T get_quantity(DEVICE& device, environments::Multirotor<ENV_SPEC>& env, rl::utils::validation::metrics::multirotor::MultirotorQuantity quantity, STATE& state, Matrix<ACTION_SPEC>& action, STATE& next_state, typename STATE::T dt){
            using T = typename STATE::T;
            using TI = typename DEVICE::index_t;
            T distance = 0;
            switch (quantity){
                case rl::utils::validation::metrics::multirotor::POSITION:
                    distance += state.position[0] * state.position[0];
                    distance += state.position[1] * state.position[1];
                    distance += state.position[2] * state.position[2];
                    distance = math::sqrt(device.math, distance);
                    break;
                case rl::utils::validation::metrics::multirotor::ANGLE:
                    distance = math::abs(device.math, 2 * math::acos(device.math, state.orientation[0]));
                    break;
                case rl::utils::validation::metrics::multirotor::LINEAR_VELOCITY:
                    distance += state.linear_velocity[0] * state.linear_velocity[0];
                    distance += state.linear_velocity[1] * state.linear_velocity[1];
                    distance += state.linear_velocity[2] * state.linear_velocity[2];
                    distance = math::sqrt(device.math, distance);
                    break;
                case rl::utils::validation::metrics::multirotor::ANGULAR_VELOCITY:
                    distance += state.angular_velocity[0] * state.angular_velocity[0];
                    distance += state.angular_velocity[1] * state.angular_velocity[1];
                    distance += state.angular_velocity[2] * state.angular_velocity[2];
                    distance = math::sqrt(device.math, distance);
                    break;
                case rl::utils::validation::metrics::multirotor::ANGULAR_ACCELERATION:
                    T angular_acceleration[3];
                    angular_acceleration[0] = (next_state.angular_velocity[0] - state.angular_velocity[0])/dt;
                    angular_acceleration[1] = (next_state.angular_velocity[1] - state.angular_velocity[1])/dt;
                    angular_acceleration[2] = (next_state.angular_velocity[2] - state.angular_velocity[2])/dt;

                    distance += angular_acceleration[0] * angular_acceleration[0];
                    distance += angular_acceleration[1] * angular_acceleration[1];
                    distance += angular_acceleration[2] * angular_acceleration[2];
                    distance = math::sqrt(device.math, distance);
                    break;
                case rl::utils::validation::metrics::multirotor::ACTION:
                    for(TI action_i=0; action_i < ACTION_SPEC::COLS; action_i++){
                        T half_range = (env.parameters.dynamics.action_limit.max - env.parameters.dynamics.action_limit.min) / 2;
                        T action_value = get(action, 0, action_i) * half_range + env.parameters.dynamics.action_limit.min + half_range;
                        T hovering_throttle_value = env.parameters.dynamics.hovering_throttle_relative * (env.parameters.dynamics.action_limit.max - env.parameters.dynamics.action_limit.min) + env.parameters.dynamics.action_limit.min;
                        T baseline_diff = action_value - hovering_throttle_value;
                        distance += math::abs(device.math, baseline_diff) / ACTION_SPEC::COLS;
                    }
                    break;
                case rl::utils::validation::metrics::multirotor::ACTION_RELATIVE:
                    for(TI action_i=0; action_i < ACTION_SPEC::COLS; action_i++){
                        T action_throttle_relative = (get(action, 0, action_i) + (T)1.0)/(T)2.0;
                        T baseline_diff = action_throttle_relative - env.parameters.dynamics.hovering_throttle_relative;
                        distance += math::abs(device.math, baseline_diff) / ACTION_SPEC::COLS;
                    }
                    break;
            }
            return distance;
        }
    }
    template <typename DEVICE, typename SPEC, enum rl::utils::validation::metrics::multirotor::MultirotorQuantity T_QUANTITY, typename T_TI, T_TI T_START_STEP>
    typename SPEC::T evaluate(DEVICE& device, rl::utils::validation::metrics::MeanErrorMean<T_QUANTITY, T_TI, T_START_STEP>, rl::utils::validation::Task<SPEC>& task){
        utils::assert_exit(device, task.completed, "Task is not completed");
        using TI = typename SPEC::TI;
        using T = typename SPEC::T;
        T acc = 0;
        TI count = 0;
        for(TI episode_i = 0; episode_i < SPEC::N_EPISODES; episode_i++){
            auto& eb = task.episode_buffer[episode_i];
            auto& env = task.environment[episode_i];
            if(task.episode_length[episode_i] == SPEC::MAX_EPISODE_LENGTH){
                for(TI step_i = T_START_STEP; step_i < task.episode_length[episode_i]; step_i++){
                    auto& state = get(eb.states, step_i, 0);
                    auto action = row(device, eb.actions, step_i);
                    auto& next_state = get(eb.next_states, step_i, 0);
                    T error = rl::utils::validation::metrics::multirotor::get_quantity(device, env, T_QUANTITY, state, action, next_state, task.environment[0].parameters.integration.dt);
                    acc += error;
                    count += 1;
                }
            }
        }
        if(count == 0){
            return math::nan<T>(device.math);
        }
        else{
            return acc / count;
        }
    }
    template <typename DEVICE, typename SPEC, enum rl::utils::validation::metrics::multirotor::MultirotorQuantity T_QUANTITY, typename T_TI, T_TI T_START_STEP>
    typename SPEC::T evaluate(DEVICE& device, rl::utils::validation::metrics::MaxErrorMean<T_QUANTITY, T_TI, T_START_STEP>, rl::utils::validation::Task<SPEC>& task){
        utils::assert_exit(device, task.completed, "Task is not completed");
        using TI = typename SPEC::TI;
        using T = typename SPEC::T;
        T max_pos_error_sum = 0;
        TI max_pos_error_count = 0;
        for(TI episode_i = 0; episode_i < SPEC::N_EPISODES; episode_i++){
            auto& eb = task.episode_buffer[episode_i];
            auto& env = task.environment[episode_i];
            if(task.episode_length[episode_i] == SPEC::MAX_EPISODE_LENGTH){
                T max_error = 0;
                for(TI step_i = T_START_STEP; step_i < task.episode_length[episode_i]; step_i++){
                    auto& state = get(eb.states, step_i, 0);
                    auto action = row(device, eb.actions, step_i);
                    auto& next_state = get(eb.next_states, step_i, 0);
                    T error = rl::utils::validation::metrics::multirotor::get_quantity(device, env, T_QUANTITY, state, action, next_state, task.environment[0].parameters.integration.dt);
                    if(error > max_error){
                        max_error = error;
                    }
                }
                max_pos_error_sum += max_error;
                max_pos_error_count += 1;
            }
        }
        if(max_pos_error_count == 0){
            return math::nan<T>(device.math);
        }
        else{
            return max_pos_error_sum / max_pos_error_count;
        }
    }
    template <typename DEVICE, typename SPEC, enum rl::utils::validation::metrics::multirotor::MultirotorQuantity T_QUANTITY, typename T_TI, T_TI T_START_STEP>
    typename SPEC::T evaluate(DEVICE& device, rl::utils::validation::metrics::MaxErrorStd<T_QUANTITY, T_TI, T_START_STEP>, rl::utils::validation::Task<SPEC>& task){
        utils::assert_exit(device, task.completed, "Task is not completed");
        using TI = typename SPEC::TI;
        using T = typename SPEC::T;
        T max_pos_error_variance_sum = 0;
        TI max_pos_error_count = 0;
        T mean = evaluate(device, rl::utils::validation::metrics::MaxErrorMean<T_QUANTITY, T_TI, T_START_STEP>{}, task);
        for(TI episode_i = 0; episode_i < SPEC::N_EPISODES; episode_i++){
            auto& eb = task.episode_buffer[episode_i];
            auto& env = task.environment[episode_i];
            if(task.episode_length[episode_i] == SPEC::MAX_EPISODE_LENGTH){
                T max_error = 0;
                for(TI step_i = T_START_STEP; step_i < task.episode_length[episode_i]; step_i++){
                    auto& state = get(eb.states, step_i, 0);
                    auto action = row(device, eb.actions, step_i);
                    auto& next_state = get(eb.next_states, step_i, 0);
                    T error = rl::utils::validation::metrics::multirotor::get_quantity(device, env, T_QUANTITY, state, action, next_state, task.environment[0].parameters.integration.dt);
                    if(error > max_error){
                        max_error = error;
                    }
                }
                T diff = max_error - mean;
                max_pos_error_variance_sum += diff * diff;
                max_pos_error_count += 1;
            }
        }
        if(max_pos_error_count == 0){
            return math::nan<T>(device.math);
        }
        else{
            return math::sqrt(device.math, max_pos_error_variance_sum / max_pos_error_count);
        }
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
